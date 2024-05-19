#!/usr/bin/env python3
#!coding=utf-8
import rospy
from xian_msg_pkg.msg import xian_crop_image_msg, xian_keypoints

# from cuda import cudart
import sys
import numpy as np
import cv2, time, datetime, os, json, logging
import tensorrt as trt
sys.path.append('/root/TensorRT-8.4.3.1/samples/python') 
import common
from cv_bridge import CvBridge, CvBridgeError

import pycuda.driver as cuda
cfx = cuda.Device(0).make_context()

def centernet_correct_boxes(box_xy, input_shape, image_shape, letterbox_image):
    #-----------------------------------------------------------------#
    #   把y轴放前面是因为方便预测框和图像的宽高进行相乘
    #-----------------------------------------------------------------#
    box_yx = box_xy[..., ::-1]
    # box_hw = box_wh[..., ::-1]
    input_shape = np.array(input_shape)
    image_shape = np.array(image_shape)

    if letterbox_image:
        #-----------------------------------------------------------------#
        #   这里求出来的offset是图像有效区域相对于图像左上角的偏移情况
        #   new_shape指的是宽高缩放情况
        #-----------------------------------------------------------------#
        new_shape = np.round(image_shape * np.min(input_shape/image_shape))
        offset  = (input_shape - new_shape)/2./input_shape
        scale   = input_shape/new_shape

        box_yx  = (box_yx - offset) * scale
        # box_hw *= scale

    # box_mins    = box_yx - (box_hw / 2.)
    # box_maxes   = box_yx + (box_hw / 2.)
    # boxes  = np.concatenate([box_mins[..., 0:1], box_mins[..., 1:2], box_maxes[..., 0:1], box_maxes[..., 1:2]], axis=-1)
    boxes  = np.concatenate([box_yx[..., 0:1], box_yx[..., 1:2]], axis=-1)
    # boxes *= np.concatenate([image_shape, image_shape], axis=-1)
    boxes *= np.concatenate([image_shape], axis=-1)
    return boxes

def postprocess(prediction, image_shape, input_shape, letterbox_image, nms_thres=0.4):
    output = [None for _ in range(len(prediction))]
    
    #----------------------------------------------------------#
    #   预测只用一张图片，只会进行一次
    #----------------------------------------------------------#
    for i, image_pred in enumerate(prediction):
        detections      = prediction[i]
        if len(detections) == 0:
            continue
        #------------------------------------------#
        #   获得预测结果中包含的所有种类
        #------------------------------------------#
        # unique_labels   = detections[:, -1].cpu().unique()
        unique_labels   = np.unique(detections[:, -1])


        # if detections.is_cuda:
        #     unique_labels = unique_labels.cuda()
        #     detections = detections.cuda()

        for c in unique_labels:
            #------------------------------------------#
            #   获得某一类得分筛选后全部的预测结果
            #------------------------------------------#
            detections_class = detections[detections[:, -1] == c]
            
            max_detections  = detections_class
            
            # output[i] = max_detections if output[i] is None else torch.cat((output[i], max_detections))
            output[i] = max_detections if output[i] is None else np.concatenate((output[i], max_detections))

        if output[i] is not None:
            output[i]           = output[i]
            # box_xy, box_wh      = (output[i][:, 0:2] + output[i][:, 2:4])/2, output[i][:, 2:4] - output[i][:, 0:2]
            box_xy = output[i][:, 0:2]
            output[i][:, :2]    = centernet_correct_boxes(box_xy, input_shape, image_shape, letterbox_image)
    return output

def max_pool2d(input_array, kernel_size, stride=None, padding=0):
    batch_size, channels, height, width = input_array.shape
    kernel_size = kernel_size
    stride = stride if stride is not None else kernel_size

    # 计算输出特征图的尺寸
    out_height = (height - kernel_size + 2 * padding) // stride + 1
    out_width = (width - kernel_size + 2 * padding) // stride + 1

    # 使用零填充（padding）扩展输入数组
    input_padded = np.pad(input_array, ((0, 0), (0, 0), (padding, padding), (padding, padding)), mode='constant')

    # 初始化输出数组
    output = np.zeros((batch_size, channels, out_height, out_width))

    # 对每个像素位置进行最大池化
    for i in range(0, out_height, stride):
        for j in range(0, out_width, stride):
            # 选取当前窗口内的最大值
            window = input_padded[:, :, i:i+kernel_size, j:j+kernel_size]
            max_val = np.max(window, axis=(2, 3))
            output[:, :, i//stride, j//stride] = max_val

    return output

def pool_nms(heat, kernel = 3):
    pad = (kernel - 1) // 2

    hmax = max_pool2d(heat, kernel_size=kernel, stride=1, padding=1)
    # keep = (hmax == heat).float()
    keep = (hmax == heat).astype(np.float32)
    
    return heat * keep

def decode_bbox(pred_hms, pred_offsets, confidence, cuda):
    #-------------------------------------------------------------------------#
    #   当利用512x512x3图片进行coco数据集预测的时候
    #   h = w = 128 num_classes = 80
    #   Hot map热力图 -> b, 80, 128, 128, 
    #   进行热力图的非极大抑制，利用3x3的卷积对热力图进行最大值筛选
    #   找出一定区域内，得分最大的特征点。
    #-------------------------------------------------------------------------#
    # pred_hms = pool_nms(pred_hms)
    
    b, c, output_h, output_w = pred_hms.shape
    detects = []
    #-------------------------------------------------------------------------#
    #   只传入一张图片，循环只进行一次
    #-------------------------------------------------------------------------#
    for batch in range(b):
        #-------------------------------------------------------------------------#
        #   heat_map        128*128, num_classes    热力图
        #   pred_wh         128*128, 2              特征点的预测宽高
        #                                           在预测过程的前处理以及后处理视频中讲的有点小问题，不是调整参数，就是宽高
        #   pred_offset     128*128, 2              特征点的xy轴偏移情况
        #-------------------------------------------------------------------------#
        # heat_map    = pred_hms[batch].permute(1, 2, 0).view([-1, c])
        heat_map    = pred_hms[batch].transpose(1, 2, 0).reshape([-1, c])
        # pred_wh     = pred_whs[batch].permute(1, 2, 0).view([-1, 2])
        # pred_offset = pred_offsets[batch].permute(1, 2, 0).view([-1, 2])
        pred_offset = pred_offsets[batch].transpose(1, 2, 0).reshape([-1, 2])

        # yv, xv      = torch.meshgrid(torch.arange(0, output_h), torch.arange(0, output_w))
        xv, yv       = np.meshgrid(np.arange(0, output_h), np.arange(0, output_w))
        #-------------------------------------------------------------------------#
        #   xv              128*128,    特征点的x轴坐标
        #   yv              128*128,    特征点的y轴坐标
        #-------------------------------------------------------------------------#
        # xv, yv      = xv.flatten().float(), yv.flatten().float()
        xv, yv      = xv.flatten().astype(np.float32), yv.flatten().astype(np.float32)
        # if cuda:
        #     xv      = xv.cuda()
        #     yv      = yv.cuda()

        #-------------------------------------------------------------------------#
        #   class_conf      128*128,    特征点的种类置信度
        #   class_pred      128*128,    特征点的种类
        #-------------------------------------------------------------------------#
        # class_conf, class_pred  = torch.max(heat_map, dim = -1)
        class_conf  = np.max(heat_map, axis = -1)
        class_pred = np.argmax(heat_map, axis = -1)
        mask                    = class_conf > confidence

        #-----------------------------------------#
        #   取出得分筛选后对应的结果
        #-----------------------------------------#
        # pred_wh_mask        = pred_wh[mask]
        pred_offset_mask    = pred_offset[mask]
        # if len(pred_wh_mask) == 0:
        #     detects.append([])
        #     continue     

        #----------------------------------------#
        #   计算调整后预测框的中心
        #----------------------------------------#
        # xv_mask = torch.unsqueeze(xv[mask] + pred_offset_mask[..., 0], -1)
        # yv_mask = torch.unsqueeze(yv[mask] + pred_offset_mask[..., 1], -1)
        xv_mask = np.expand_dims(xv[mask] + pred_offset_mask[..., 0], -1)
        yv_mask = np.expand_dims(yv[mask] + pred_offset_mask[..., 1], -1)
        #----------------------------------------#
        #   计算预测框的宽高
        #----------------------------------------#
        # half_w, half_h = pred_wh_mask[..., 0:1] / 2, pred_wh_mask[..., 1:2] / 2
        #----------------------------------------#
        #   获得预测框的左上角和右下角
        #----------------------------------------#
        # bboxes = torch.cat([xv_mask - half_w, yv_mask - half_h, xv_mask + half_w, yv_mask + half_h], dim=1)
        # bboxes = torch.cat([xv_mask, yv_mask], dim=1)
        bboxes = np.concatenate([xv_mask, yv_mask], axis=1)
        bboxes[:, [0]] /= output_w
        bboxes[:, [1]] /= output_h
        # detect = torch.cat([bboxes, torch.unsqueeze(class_conf[mask],-1), torch.unsqueeze(class_pred[mask],-1).float()], dim=-1)
        detect = np.concatenate([bboxes, np.expand_dims(class_conf[mask],-1), np.expand_dims(class_pred[mask],-1).astype(np.float32)], axis=-1)
        detects.append(detect)

    return detects

def get_keypoints(results, src_image_size):
    h = src_image_size[0]
    w = src_image_size[1]
    top_label   = np.array(results[:, 3], dtype = 'int32')
    top_conf    = results[:, 2]
    top_boxes   = results[:, :2]
    x_y_c_list = []
    for i, c in list(enumerate(top_label)):
        predicted_class = int(c)
        box             = top_boxes[i]
        score           = top_conf[i]
        y_c, x_c= box
        
        x_c     = max(0, np.floor(x_c).astype('int32'))
        y_c    = max(0, np.floor(y_c).astype('int32'))
        # y_c  = min(image.size[1], np.floor(y_c).astype('int32'))
        # x_c   = min(image.size[0], np.floor(x_c).astype('int32'))
        y_c  = min(h, np.floor(y_c).astype('int32'))
        x_c  = min(w, np.floor(x_c).astype('int32'))

        x_y_c_list.append([x_c, y_c, predicted_class, score])
    return x_y_c_list

def xian_display(final_results, image, color=(0,0,255)):
    for final_result in final_results:
        x_c = final_result[0]
        y_c = final_result[1]
        cls = final_result[2]
        scores = final_result[3]
        cv2.circle(image, (x_c, y_c), 4, color, -1)
    return image

def get_container_corner_keypoint(final_results, cls_idx, x, y, x0, y0, distance_threshold):
    final_results_np = np.array(final_results)
    cls = final_results_np[:, 2]
    target_idx = (cls == cls_idx)
    target_cls = final_results_np[target_idx]
    if target_cls.size == 0:
        return int(x), int(y)
    max_idx = np.argmax(target_cls[:, 3])
    target_point = target_cls[max_idx]
    x_c = target_point[0]
    y_c = target_point[1]
    x_decode = x0+x_c/2
    y_decode = y0+y_c/2
    distance = pow(pow(x_decode-x, 2) + pow(y_decode-y, 2), 0.5)
    if distance > distance_threshold:
        return int(x), int(y)
    else:
        return int(x_decode), int(y_decode)
    
def get_cell_guide_keypoint(final_results, cls_idx, x0, y0):
    final_results_np = np.array(final_results)
    cls = final_results_np[:, 2]
    target_idx = (cls == cls_idx)
    target_cls = final_results_np[target_idx]
    if target_cls.size == 0:
        return -1, -1
    max_idx = np.argmax(target_cls[:, 3])
    target_point = target_cls[max_idx]
    x_c = target_point[0]
    y_c = target_point[1]
    x_decode = x0+x_c/2
    y_decode = y0+y_c/2
    return int(x_decode), int(y_decode)

def get_final_container_corner_keypoint(results, image_shape, cls_idx,
                                        calibrate_container_corner_x, calibrate_container_corner_y,
                                        container_corner_tl_x0, container_corner_tl_y0, distance_threshold):
    if results is None:
        identifed_container_corner_x = calibrate_container_corner_x
        identifed_container_corner_y = calibrate_container_corner_y
    else:
        final_results = get_keypoints(results, image_shape) 
        container_corner = get_container_corner_keypoint(final_results, cls_idx=cls_idx, 
                                                         x=calibrate_container_corner_x, 
                                                         y=calibrate_container_corner_y, 
                                                         x0=container_corner_tl_x0,
                                                         y0=container_corner_tl_y0,
                                                         distance_threshold=distance_threshold) # cls_idx:0-cell_guide_point; 1-container corner
        identifed_container_corner_x = container_corner[0]
        identifed_container_corner_y = container_corner[1]
    return identifed_container_corner_x, identifed_container_corner_y
    

def get_final_cell_guide_keypoint():
    pass

def zpmc_onnx2trt(onnxFile, trtFile_save_dir, trtFile_save_name, FPMode):
    input_shape = [512, 512]
    logger = trt.Logger(trt.Logger.ERROR)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    profile = builder.create_optimization_profile()
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 3 << 30)

    if FPMode == 'FP32':
        print('Set FPMode = FP32!')
    elif FPMode == 'FP16':
        config.set_flag(trt.BuilderFlag.FP16)
        print('Set FPMode = FP16!')
    else:
        print('Please set FPMode = FP32 or FP16')
        exit()

    parser = trt.OnnxParser(network, logger)

    if not os.path.exists(onnxFile):
        print("Failed finding {}!".format(onnxFile))
        exit()
    print("Succeeded finding {}!".format(onnxFile))

    with open(onnxFile, "rb") as model:
        if not parser.parse(model.read()):
            print("Failed parsing {}!".format(onnxFile))
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            exit()
        print("Succeeded parsing {}!".format(onnxFile))

    inputTensor = network.get_input(0)
    outputTensor0 = network.get_output(0)
    outputTensor1 = network.get_output(1)

    print('inputTensor:', inputTensor.name, inputTensor.shape)
    print('outputTensor0:', outputTensor0.name, outputTensor0.shape)
    print('outputTensor1:', outputTensor1.name, outputTensor1.shape)


    batch_size, nHeight, nWidth, _  = inputTensor.shape
    profile.set_shape(inputTensor.name, (12, nHeight, nWidth, 3), (12, nHeight, nWidth, 3), (12, nHeight, nWidth, 3)) # 最小batch，常见batch，最大batch
    config.add_optimization_profile(profile)

    trtFile = os.path.join(trtFile_save_dir, trtFile_save_name)
    if not os.path.exists(trtFile):
        engineString = builder.build_serialized_network(network, config)
        if engineString == None:
            print("Failed building engine!")
            exit()
        print("Succeeded building engine!")

        if not os.path.exists(trtFile_save_dir):  # os模块判断并创建
            os.mkdir(trtFile_save_dir)
        with open(trtFile, "wb") as f:
            f.write(engineString)
        print("Generate {}!".format(trtFile))
        engine = trt.Runtime(logger).deserialize_cuda_engine(engineString)
        print('Loaded {}!'.format(trtFile))
    else:
         with open(trtFile, "rb") as f, trt.Runtime(logger) as runtime:
            engine =  runtime.deserialize_cuda_engine(f.read())
         print('Loaded {}!'.format(trtFile))

    context = engine.create_execution_context()
    inputs, outputs, bindings, stream = common.allocate_buffers(engine)
    return context, inputs, outputs, bindings, stream


# 编译调用onnx
onnxFile = '/root/code/xian_aqc_ws/xian_project_file/onnx/keypoints.onnx'
trtFile_save_dir = '/root/code/xian_aqc_ws/xian_project_file/trt'
trtFile_save_name = 'keypoints16.trt'
FPMode = 'FP16'
class_names = ['cell_guide', 'container_corner_point']
context, inputs, outputs, bindings, stream = zpmc_onnx2trt(onnxFile, trtFile_save_dir, trtFile_save_name, FPMode)


class xian_aqc_keypoints_recognition:
    def __init__(self):
        
        print("start xian_aqc_keypoints_recognition!")
        self.keypoints_publisher = rospy.Publisher('xian_aqc_keypoints', xian_keypoints, queue_size=1)  # 创建消息发布者
        # self.rate = rospy.Rate(1)  # 设置消息发布频率为1Hz
        rospy.Subscriber('xian_crop_images', xian_crop_image_msg, self.callback)

        self.pre_time = datetime.datetime.now()
        self.cur_time = datetime.datetime.now()
        self.timediff = 1
        self.counter = 0

        self.xian_keypoints_msg = xian_keypoints() # 定义消息

        rospy.spin()


    def callback(self, data):
        self.pre_time = self.cur_time
        self.cur_time = datetime.datetime.now()
        input_shape = [512, 512] # 由于预处理节点已经缩放到[512, 512]，因此输入尺寸就是[512, 512]
        
        xian_tl_container_point_x = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_tl_container_point_x")
        xian_tl_container_point_y = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_tl_container_point_y")
        xian_tr_container_point_x = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_tr_container_point_x")
        xian_tr_container_point_y = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_tr_container_point_y")
        xian_bl_container_point_x = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_bl_container_point_x")
        xian_bl_container_point_y = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_bl_container_point_y")
        xian_br_container_point_x = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_br_container_point_x")
        xian_br_container_point_y = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_br_container_point_y")
        identifed_container_corner_distance = rospy.get_param("/xian_aqc_dynamic_parameters_server/identifed_container_corner_distance")
        
        tl_container_corner_crop_image = CvBridge().imgmsg_to_cv2(data.tl_container_corner_crop_image, "bgr8") 
        tr_container_corner_crop_image = CvBridge().imgmsg_to_cv2(data.tr_container_corner_crop_image, "bgr8") 
        bl_container_corner_crop_image = CvBridge().imgmsg_to_cv2(data.bl_container_corner_crop_image, "bgr8") 
        br_container_corner_crop_image = CvBridge().imgmsg_to_cv2(data.br_container_corner_crop_image, "bgr8") 
        image_shape = br_container_corner_crop_image.shape[0:2]
        
        tl_cell_guide_crop_image1 = CvBridge().imgmsg_to_cv2(data.tl_cell_guide_crop_image1, "bgr8") 
        tr_cell_guide_crop_image1 = CvBridge().imgmsg_to_cv2(data.tr_cell_guide_crop_image1, "bgr8") 
        bl_cell_guide_crop_image1 = CvBridge().imgmsg_to_cv2(data.bl_cell_guide_crop_image1, "bgr8") 
        br_cell_guide_crop_image1 = CvBridge().imgmsg_to_cv2(data.br_cell_guide_crop_image1, "bgr8") 
        
        tl_cell_guide_crop_image2 = CvBridge().imgmsg_to_cv2(data.tl_cell_guide_crop_image2, "bgr8") 
        tr_cell_guide_crop_image2 = CvBridge().imgmsg_to_cv2(data.tr_cell_guide_crop_image2, "bgr8") 
        bl_cell_guide_crop_image2 = CvBridge().imgmsg_to_cv2(data.bl_cell_guide_crop_image2, "bgr8") 
        br_cell_guide_crop_image2 = CvBridge().imgmsg_to_cv2(data.br_cell_guide_crop_image2, "bgr8") 
        
        images_list = [tl_container_corner_crop_image, 
                       tr_container_corner_crop_image,
                       bl_container_corner_crop_image,
                       br_container_corner_crop_image,
                       tl_cell_guide_crop_image1,
                       tr_cell_guide_crop_image1,
                       bl_cell_guide_crop_image1,
                       br_cell_guide_crop_image1,
                       tl_cell_guide_crop_image2,
                       tr_cell_guide_crop_image2,
                       bl_cell_guide_crop_image2,
                       br_cell_guide_crop_image2]
        # images_process = zpmc_std_mean(images_resize_list)
        input_np = np.array(images_list)
        inputs[0].host = np.ascontiguousarray(input_np, dtype=np.float32)
        cfx.push()
        trt_outputs = common.do_inference_v2(context, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)
        cfx.pop()
        batch_size = len(images_list)
        trt_outputs_shape = [(batch_size, 2, 128, 128), (batch_size, 2, 128, 128)]
        heat_map = trt_outputs[1].reshape(trt_outputs_shape[0])
        p_w_h = trt_outputs[0].reshape(trt_outputs_shape[1])
        
        outputs_ = decode_bbox(heat_map, p_w_h, 0.3, False)
        results = postprocess(outputs_, image_shape, input_shape, False, 0.3)
        
        container_corner_tl_x0 = data.container_corner_tl_x0
        container_corner_tl_y0 = data.container_corner_tl_y0
        container_corner_tr_x0 = data.container_corner_tr_x0
        container_corner_tr_y0 = data.container_corner_tr_y0
        container_corner_bl_x0 = data.container_corner_bl_x0
        container_corner_bl_y0 = data.container_corner_bl_y0
        container_corner_br_x0 = data.container_corner_br_x0
        container_corner_br_y0 = data.container_corner_br_y0
        
        clip1_cell_guide_tl_x = data.clip1_cell_guide_tl_x
        clip1_cell_guide_tl_y = data.clip1_cell_guide_tl_y
        clip1_cell_guide_tr_x = data.clip1_cell_guide_tr_x
        clip1_cell_guide_tr_y = data.clip1_cell_guide_tr_y
        clip1_cell_guide_bl_x = data.clip1_cell_guide_bl_x
        clip1_cell_guide_bl_y = data.clip1_cell_guide_bl_y
        clip1_cell_guide_br_x = data.clip1_cell_guide_br_x
        clip1_cell_guide_br_y = data.clip1_cell_guide_br_y
        
        clip2_cell_guide_tl_x = data.clip2_cell_guide_tl_x
        clip2_cell_guide_tl_y = data.clip2_cell_guide_tl_y
        clip2_cell_guide_tr_x = data.clip2_cell_guide_tr_x
        clip2_cell_guide_tr_y = data.clip2_cell_guide_tr_y
        clip2_cell_guide_bl_x = data.clip2_cell_guide_bl_x
        clip2_cell_guide_bl_y = data.clip2_cell_guide_bl_y
        clip2_cell_guide_br_x = data.clip2_cell_guide_br_x
        clip2_cell_guide_br_y = data.clip2_cell_guide_br_y
        
        # tl集装箱箱脚点识别
        tl_container_corner_result = results[0]
        tl_identifed_container_corner = get_final_container_corner_keypoint(tl_container_corner_result, image_shape, 1,
                                                                            xian_tl_container_point_x, xian_tl_container_point_y,
                                                                            container_corner_tl_x0, container_corner_tl_y0, 
                                                                            identifed_container_corner_distance) # cls_idx:0-cell_guide_point; 1-container corner  
        self.xian_keypoints_msg.tl_container_corner_x = tl_identifed_container_corner[0]
        self.xian_keypoints_msg.tl_container_corner_y = tl_identifed_container_corner[1]
        # if tl_container_corner_result is None:
        #     tl_identifed_container_corner_x = xian_tl_container_point_x
        #     tl_identifed_container_corner_y = xian_tl_container_point_y
        #     print('No container corner keypoints!')
        # else:
        #     final_results0 = get_keypoints(tl_container_corner_result, image_shape)            
        #     tl_container_corner = get_container_corner_keypoint(final_results0, cls_idx=1, 
        #                                               x=xian_tl_container_point_x, 
        #                                               y=xian_tl_container_point_y, 
        #                                               x0=container_corner_tl_x0,
        #                                               y0=container_corner_tl_y0,
        #                                               distance_threshold=identifed_container_corner_distance) # cls_idx:0-cell_guide_point; 1-container corner            
        #     tl_identifed_container_corner_x = tl_container_corner[0]
        #     tl_identifed_container_corner_y = tl_container_corner[1]
        #     # tl_container_corner_src = images_list[0]
        #     # tl_container_corner_show = xian_display(final_results0, tl_container_corner_src, color=(0, 255, 255))
        #     # cv2.imwrite('/root/code/xian_aqc_ws/xian_project_file/trt/results/tl_'+ str(datetime.datetime.now()) + '.jpg', 
        #     #             tl_container_corner_show)
        # self.xian_keypoints_msg.tl_container_corner_x = tl_identifed_container_corner_x
        # self.xian_keypoints_msg.tl_container_corner_y = tl_identifed_container_corner_y
        # # print("tl_identifed_container_corner_x:{}, tl_identifed_container_corner_y:{}".format(tl_identifed_container_corner_x, tl_identifed_container_corner_y))
        # # print("xian_tl_container_point_x:{}, xian_tl_container_point_y:{}".format(xian_tl_container_point_x, xian_tl_container_point_y))
        # # print("container_corner_tl_x0:{}, container_corner_tl_y0:{}".format(container_corner_tl_x0, container_corner_tl_y0))
        
        # tr集装箱箱脚点识别
        tr_container_corner_result = results[1]
        tr_identifed_container_corner = get_final_container_corner_keypoint(tr_container_corner_result, image_shape, 1,
                                                                            xian_tr_container_point_x, xian_tr_container_point_y,
                                                                            container_corner_tr_x0, container_corner_tr_y0, 
                                                                            identifed_container_corner_distance) # cls_idx:0-cell_guide_point; 1-container corner  
        self.xian_keypoints_msg.tr_container_corner_x = tr_identifed_container_corner[0]
        self.xian_keypoints_msg.tr_container_corner_y = tr_identifed_container_corner[1]
        # if tr_container_corner_result is None:
        #     tr_identifed_container_corner_x = xian_tr_container_point_x
        #     tr_identifed_container_corner_y = xian_tr_container_point_y
        #     print('No container corner keypoints!')
        # else:
        #     final_results1 = get_keypoints(tr_container_corner_result, image_shape)            
        #     tr_container_corner = get_container_corner_keypoint(final_results1, cls_idx=1, 
        #                                               x=xian_tr_container_point_x, 
        #                                               y=xian_tr_container_point_y, 
        #                                               x0=container_corner_tr_x0,
        #                                               y0=container_corner_tr_y0,
        #                                               distance_threshold=identifed_container_corner_distance) # cls_idx:0-cell_guide_point; 1-container corner          
        #     tr_identifed_container_corner_x = tr_container_corner[0]
        #     tr_identifed_container_corner_y = tr_container_corner[1]
        # self.xian_keypoints_msg.tr_container_corner_x = tr_identifed_container_corner_x
        # self.xian_keypoints_msg.tr_container_corner_y = tr_identifed_container_corner_y
        
        
        
        
        # bl集装箱箱脚点识别
        bl_container_corner_result = results[2]
        bl_identifed_container_corner = get_final_container_corner_keypoint(bl_container_corner_result, image_shape, 1,
                                                                            xian_bl_container_point_x, xian_bl_container_point_y,
                                                                            container_corner_bl_x0, container_corner_bl_y0, 
                                                                            identifed_container_corner_distance) # cls_idx:0-cell_guide_point; 1-container corner  
        self.xian_keypoints_msg.bl_container_corner_x = bl_identifed_container_corner[0]
        self.xian_keypoints_msg.bl_container_corner_y = bl_identifed_container_corner[1]
        # if bl_container_corner_result is None:
        #     bl_identifed_container_corner_x = xian_bl_container_point_x
        #     bl_identifed_container_corner_y = xian_bl_container_point_y
        #     print('No container corner keypoints!')
        # else:
        #     final_results2 = get_keypoints(bl_container_corner_result, image_shape)            
        #     bl_container_corner = get_container_corner_keypoint(final_results2, cls_idx=1, 
        #                                               x=xian_bl_container_point_x, 
        #                                               y=xian_bl_container_point_y, 
        #                                               x0=container_corner_bl_x0,
        #                                               y0=container_corner_bl_y0,
        #                                               distance_threshold=identifed_container_corner_distance) # cls_idx:0-cell_guide_point; 1-container corner            
        #     bl_identifed_container_corner_x = bl_container_corner[0]
        #     bl_identifed_container_corner_y = bl_container_corner[1]
        # self.xian_keypoints_msg.bl_container_corner_x = bl_identifed_container_corner_x
        # self.xian_keypoints_msg.bl_container_corner_y = bl_identifed_container_corner_y

        # br集装箱箱脚点识别
        br_container_corner_result = results[3]
        br_identifed_container_corner = get_final_container_corner_keypoint(br_container_corner_result, image_shape, 1,
                                                                            xian_br_container_point_x, xian_br_container_point_y,
                                                                            container_corner_br_x0, container_corner_br_y0, 
                                                                            identifed_container_corner_distance) # cls_idx:0-cell_guide_point; 1-container corner  
        self.xian_keypoints_msg.br_container_corner_x = br_identifed_container_corner[0]
        self.xian_keypoints_msg.br_container_corner_y = br_identifed_container_corner[1]
        # if br_container_corner_result is None:
        #     br_identifed_container_corner_x = xian_br_container_point_x
        #     br_identifed_container_corner_y = xian_br_container_point_y
        #     print('No container corner keypoints!')
        # else:
        #     final_results3 = get_keypoints(br_container_corner_result, image_shape)            
        #     br_container_corner = get_container_corner_keypoint(final_results3, cls_idx=1, 
        #                                               x=xian_br_container_point_x, 
        #                                               y=xian_br_container_point_y, 
        #                                               x0=container_corner_br_x0,
        #                                               y0=container_corner_br_y0,
        #                                               distance_threshold=identifed_container_corner_distance) # cls_idx:0-cell_guide_point; 1-container corner            
        #     br_identifed_container_corner_x = br_container_corner[0]
        #     br_identifed_container_corner_y = br_container_corner[1]
        # self.xian_keypoints_msg.br_container_corner_x = br_identifed_container_corner_x
        # self.xian_keypoints_msg.br_container_corner_y = br_identifed_container_corner_y
        
        # tl导轨crop1
        tl_cell_guide_crop1_result = results[4]
        if tl_cell_guide_crop1_result is None:
            tl_identifed_cell_guide1_x = -1
            tl_identifed_cell_guide1_y = -1
            print('No cell guide1 keypoints!')
        else:
            final_results4 = get_keypoints(tl_cell_guide_crop1_result, image_shape)            
            tl_cell_guide_crop1 = get_cell_guide_keypoint(final_results4, cls_idx=0, 
                                                          x0=clip1_cell_guide_tl_x,
                                                          y0=clip1_cell_guide_tl_y) # cls_idx:0-cell_guide_point; 1-container corner             
            tl_identifed_cell_guide1_x = tl_cell_guide_crop1[0]
            tl_identifed_cell_guide1_y = tl_cell_guide_crop1[1]
        self.xian_keypoints_msg.tl_cell_guide_crop1_x = tl_identifed_cell_guide1_x
        self.xian_keypoints_msg.tl_cell_guide_crop1_y = tl_identifed_cell_guide1_y
        
        print('clip1_cell_guide_tl_x:', clip1_cell_guide_tl_x)
        print('clip1_cell_guide_tl_y:', clip1_cell_guide_tl_y)
        
        # tr导轨crop1
        tr_cell_guide_crop1_result = results[5]
        if tr_cell_guide_crop1_result is None:
            tr_identifed_cell_guide1_x = -1
            tr_identifed_cell_guide1_y = -1
            print('No cell guide1 keypoints!')
        else:
            final_results5 = get_keypoints(tr_cell_guide_crop1_result, image_shape)            
            tr_cell_guide_crop1 = get_cell_guide_keypoint(final_results5, cls_idx=0, 
                                                          x0=clip1_cell_guide_tr_x,
                                                          y0=clip1_cell_guide_tr_y) # cls_idx:0-cell_guide_point; 1-container corner             
            tr_identifed_cell_guide1_x = tr_cell_guide_crop1[0]
            tr_identifed_cell_guide1_y = tr_cell_guide_crop1[1]
        self.xian_keypoints_msg.tr_cell_guide_crop1_x = tr_identifed_cell_guide1_x
        self.xian_keypoints_msg.tr_cell_guide_crop1_y = tr_identifed_cell_guide1_y
        
        print('clip1_cell_guide_tr_x:', clip1_cell_guide_tr_x)
        print('clip1_cell_guide_tr_y:', clip1_cell_guide_tr_y)
        
        # bl导轨crop1
        bl_cell_guide_crop1_result = results[6]
        if bl_cell_guide_crop1_result is None:
            bl_identifed_cell_guide1_x = -1
            bl_identifed_cell_guide1_y = -1
            print('No cell guide1 keypoints!')
        else:
            final_results6 = get_keypoints(bl_cell_guide_crop1_result, image_shape)            
            bl_cell_guide_crop1 = get_cell_guide_keypoint(final_results6, cls_idx=0, 
                                                          x0=clip1_cell_guide_bl_x,
                                                          y0=clip1_cell_guide_bl_y) # cls_idx:0-cell_guide_point; 1-container corner             
            bl_identifed_cell_guide1_x = bl_cell_guide_crop1[0]
            bl_identifed_cell_guide1_y = bl_cell_guide_crop1[1]
        self.xian_keypoints_msg.bl_cell_guide_crop1_x = bl_identifed_cell_guide1_x
        self.xian_keypoints_msg.bl_cell_guide_crop1_y = bl_identifed_cell_guide1_y
        
        # br导轨crop1
        br_cell_guide_crop1_result = results[7]
        if br_cell_guide_crop1_result is None:
            br_identifed_cell_guide1_x = -1
            br_identifed_cell_guide1_y = -1
            print('No cell guide1 keypoints!')
        else:
            final_results7 = get_keypoints(br_cell_guide_crop1_result, image_shape)            
            br_cell_guide_crop1 = get_cell_guide_keypoint(final_results7, cls_idx=0, 
                                                          x0=clip1_cell_guide_br_x,
                                                          y0=clip1_cell_guide_br_y) # cls_idx:0-cell_guide_point; 1-container corner             
            br_identifed_cell_guide1_x = br_cell_guide_crop1[0]
            br_identifed_cell_guide1_y = br_cell_guide_crop1[1]
        self.xian_keypoints_msg.br_cell_guide_crop1_x = br_identifed_cell_guide1_x
        self.xian_keypoints_msg.br_cell_guide_crop1_y = br_identifed_cell_guide1_y
        
        # tl导轨crop2
        tl_cell_guide_crop2_result = results[8]
        if tl_cell_guide_crop2_result is None:
            tl_identifed_cell_guide2_x = -1
            tl_identifed_cell_guide2_y = -1
            print('No cell guide2 keypoints!')
        else:
            final_results8 = get_keypoints(tl_cell_guide_crop2_result, image_shape)            
            tl_cell_guide_crop2 = get_cell_guide_keypoint(final_results8, cls_idx=0, 
                                                          x0=clip2_cell_guide_tl_x,
                                                          y0=clip2_cell_guide_tl_y) # cls_idx:0-cell_guide_point; 1-container corner             
            tl_identifed_cell_guide2_x = tl_cell_guide_crop2[0]
            tl_identifed_cell_guide2_y = tl_cell_guide_crop2[1]
        self.xian_keypoints_msg.tl_cell_guide_crop2_x = tl_identifed_cell_guide2_x
        self.xian_keypoints_msg.tl_cell_guide_crop2_y = tl_identifed_cell_guide2_y

        # tr导轨crop2
        tr_cell_guide_crop2_result = results[9]
        if tr_cell_guide_crop2_result is None:
            tr_identifed_cell_guide2_x = -1
            tr_identifed_cell_guide2_y = -1
            print('No cell guide2 keypoints!')
        else:
            final_results9 = get_keypoints(tr_cell_guide_crop2_result, image_shape)            
            tr_cell_guide_crop2 = get_cell_guide_keypoint(final_results9, cls_idx=0, 
                                                          x0=clip2_cell_guide_tr_x,
                                                          y0=clip2_cell_guide_tr_y) # cls_idx:0-cell_guide_point; 1-container corner             
            tr_identifed_cell_guide2_x = tr_cell_guide_crop2[0]
            tr_identifed_cell_guide2_y = tr_cell_guide_crop2[1]
        self.xian_keypoints_msg.tr_cell_guide_crop2_x = tr_identifed_cell_guide2_x
        self.xian_keypoints_msg.tr_cell_guide_crop2_y = tr_identifed_cell_guide2_y
        
        # bl导轨crop2
        bl_cell_guide_crop2_result = results[10]
        if bl_cell_guide_crop2_result is None:
            bl_identifed_cell_guide2_x = -1
            bl_identifed_cell_guide2_y = -1
            print('No cell guide2 keypoints!')
        else:
            final_results10 = get_keypoints(bl_cell_guide_crop2_result, image_shape)            
            bl_cell_guide_crop2 = get_cell_guide_keypoint(final_results10, cls_idx=0, 
                                                          x0=clip2_cell_guide_bl_x,
                                                          y0=clip2_cell_guide_bl_y) # cls_idx:0-cell_guide_point; 1-container corner             
            bl_identifed_cell_guide2_x = bl_cell_guide_crop2[0]
            bl_identifed_cell_guide2_y = bl_cell_guide_crop2[1]
        self.xian_keypoints_msg.bl_cell_guide_crop2_x = bl_identifed_cell_guide2_x
        self.xian_keypoints_msg.bl_cell_guide_crop2_y = bl_identifed_cell_guide2_y
        
        # br导轨crop2
        br_cell_guide_crop2_result = results[11]
        if br_cell_guide_crop2_result is None:
            br_identifed_cell_guide2_x = -1
            br_identifed_cell_guide2_y = -1
            print('No cell guide1 keypoints!')
        else:
            final_results11 = get_keypoints(br_cell_guide_crop2_result, image_shape)            
            br_cell_guide_crop2 = get_cell_guide_keypoint(final_results11, cls_idx=0, 
                                                          x0=clip2_cell_guide_br_x,
                                                          y0=clip2_cell_guide_br_y) # cls_idx:0-cell_guide_point; 1-container corner             
            br_identifed_cell_guide2_x = br_cell_guide_crop2[0]
            br_identifed_cell_guide2_y = br_cell_guide_crop2[1]
        self.xian_keypoints_msg.br_cell_guide_crop2_x = br_identifed_cell_guide2_x
        self.xian_keypoints_msg.br_cell_guide_crop2_y = br_identifed_cell_guide2_y
          
        self.xian_keypoints_msg.tl_image = data.tl_image
        self.xian_keypoints_msg.tr_image = data.tr_image
        self.xian_keypoints_msg.bl_image = data.bl_image
        self.xian_keypoints_msg.br_image = data.br_image

        self.keypoints_publisher.publish(self.xian_keypoints_msg)
        
        self.timediff = (self.cur_time - self.pre_time).total_seconds()
        rospy.set_param("/xian_aqc_dynamic_parameters_server/xian_keypoints_recognition_fps", 1.0/self.timediff)
        xian_keypoints_recognition_fps = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_keypoints_recognition_fps")
        print('FPS {:2.3f}'.format(xian_keypoints_recognition_fps))
        


class HB:
    def __init__(self):
        self.counter = 0

    def xian_heat_beat_callback(self, event):
        rospy.set_param("/xian_aqc_dynamic_parameters_server/xian_keypoints_recognition_heart_beat", self.counter)
        xian_keypoints_recognition_heart_beat = rospy.get_param("/xian_aqc_dynamic_parameters_server/xian_keypoints_recognition_heart_beat")
        if self.counter>1000:
            self.counter = 0
        self.counter += 1
        print("xian_keypoints_recognition_heart_beat:", xian_keypoints_recognition_heart_beat)



if __name__ == '__main__':
    try:
        tt = HB()
        rospy.init_node('xian_aqc_keypoints_recognition', anonymous=True)  # 初始化ROS节点
        rospy.Timer(rospy.Duration(1), tt.xian_heat_beat_callback, oneshot=False)
        publisher = xian_aqc_keypoints_recognition()

    except rospy.ROSInterruptException:
        pass