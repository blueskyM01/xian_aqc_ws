#!/usr/bin/env python3
#!coding=utf-8
import rospy
from zpmc_unloading_interface_pkg.msg import ZpmcLockHolePreprocessImages, ZpmcLockHoleMasks

# from cuda import cudart
import sys
import numpy as np
import cv2, time, datetime, os, json, logging
import tensorrt as trt
sys.path.append('/usr/src/tensorrt/samples/python') 
import common
from cv_bridge import CvBridge, CvBridgeError

import pycuda.driver as cuda
# cfx = cuda.Device(0).make_context()


def zpmc_decode(loc, priors):
    variances = [0.1, 0.2]
        
    boxes = np.concatenate((
        priors[:, :2] + loc[:, :2] * variances[0] * priors[:, 2:],
        priors[:, 2:] * np.exp(loc[:, 2:] * variances[1])), 1)
    boxes[:, :2] -= boxes[:, 2:] / 2
    boxes[:, 2:] += boxes[:, :2]
    return boxes


def intersect(box_a, box_b):
    """ We resize both tensors to [A,B,2] without new malloc:
    [A,2] -> [A,1,2] -> [A,B,2]
    [B,2] -> [1,B,2] -> [A,B,2]
    Then we compute the area of intersect between box_a and box_b.
    Args:
      box_a: (tensor) bounding boxes, Shape: [n,A,4].
      box_b: (tensor) bounding boxes, Shape: [n,B,4].
    Return:
      (tensor) intersection area, Shape: [n,A,B].
    """
    n = box_a.shape[0]
    A = box_a.shape[1]
    B = box_b.shape[1]

    box_a_br_max = np.expand_dims(box_a[:, :, 2:], 2)#.expand(n, A, B, 2)
    box_b_br_max = np.expand_dims(box_b[:, :, 2:], 1)#.expand(n, A, B, 2)
    box_a_br_max = np.broadcast_to(box_a_br_max, (n, A, B, 2))
    box_b_br_max = np.broadcast_to(box_b_br_max, (n, A, B, 2))
    max_xy = np.minimum(box_a_br_max, box_b_br_max)

    box_a_br_min = np.expand_dims(box_a[:, :, :2], 2)#.expand(n, A, B, 2)
    box_b_br_min = np.expand_dims(box_b[:, :, :2], 1)#.expand(n, A, B, 2)
    box_a_br_min = np.broadcast_to(box_a_br_min, (n, A, B, 2))
    box_b_br_min = np.broadcast_to(box_b_br_min, (n, A, B, 2))
    min_xy = np.maximum(box_a_br_min, box_b_br_min)


    # inter = torch.clamp((max_xy - min_xy), min=0)
    inter = np.clip((max_xy - min_xy), a_min=0, a_max=10e10)
    return inter[:, :, :, 0] * inter[:, :, :, 1]

def jaccard(box_a, box_b, iscrowd:bool=False):
    """Compute the jaccard overlap of two sets of boxes.  The jaccard overlap
    is simply the intersection over union of two boxes.  Here we operate on
    ground truth boxes and default boxes. If iscrowd=True, put the crowd in box_b.
    E.g.:
        A ∩ B / A ∪ B = A ∩ B / (area(A) + area(B) - A ∩ B)
    Args:
        box_a: (tensor) Ground truth bounding boxes, Shape: [num_objects,4]
        box_b: (tensor) Prior boxes from priorbox layers, Shape: [num_priors,4]
    Return:
        jaccard overlap: (tensor) Shape: [box_a.size(0), box_b.size(0)]
    """
    use_batch = True
    # if box_a.dim() == 2:
    #     use_batch = False
    #     box_a = box_a[None, ...]
    #     box_b = box_b[None, ...]

    inter = intersect(box_a, box_b)
    inter_shape = inter.shape
    area_a_br = (box_a[:, :, 2]-box_a[:, :, 0]) * (box_a[:, :, 3]-box_a[:, :, 1])
    area_a_br = np.expand_dims(area_a_br, 2)
    area_a = np.broadcast_to(area_a_br, inter_shape)

    area_b_br = (box_b[:, :, 2]-box_b[:, :, 0]) * (box_b[:, :, 3]-box_b[:, :, 1])
    area_b_br = np.expand_dims(area_b_br, 1)
    area_b = np.broadcast_to(area_b_br, inter_shape)
    # area_a = ((box_a[:, :, 2]-box_a[:, :, 0]) * 
    #           (box_a[:, :, 3]-box_a[:, :, 1])).unsqueeze(2).expand_as(inter)  # [A,B]
    # area_b = ((box_b[:, :, 2]-box_b[:, :, 0]) *
    #           (box_b[:, :, 3]-box_b[:, :, 1])).unsqueeze(1).expand_as(inter)  # [A,B]
    union = area_a + area_b - inter

    out = inter / area_a if iscrowd else inter / union
    # return out if use_batch else out.squeeze(0)
    return out

def fast_nms(boxes, masks, scores, iou_threshold:float=0.5, top_k:int=200):
    # scores, idx = scores.sort(1, descending=True)

    idx = np.argsort(scores, axis=1)
    idx = idx[:, ::-1]
    scores = scores[:, idx[0]]
    idx = idx[:, :top_k]
    scores = scores[:, :top_k]

    num_classes, num_dets = idx.shape

    boxes = boxes[idx.reshape(-1), :].reshape(num_classes, num_dets, 4)
    masks = masks[idx.reshape(-1), :].reshape(num_classes, num_dets, -1)

    iou = jaccard(boxes, boxes)
    iou=np.triu(iou, k=1)

    # iou_max_idx = np.argmax(iou, axis=1)
    iou_max =np.max(iou, axis=1)    

    # Now just filter out the ones higher than the threshold
    keep = (iou_max <= iou_threshold)

    classes = np.arange(num_classes)[:, None]
    classes = np.broadcast_to(classes, keep.shape)
    classes = classes[keep]

    boxes = boxes[keep]
    masks = masks[keep]
    scores = scores[keep]

    idx = np.argsort(scores, axis=0)
    idx = idx[::-1]
    scores = scores[idx]

    idx = idx[:100] # 最终的检测目标数不超过100
    scores = scores[idx]

    classes = classes[idx]
    boxes = boxes[idx]
    masks = masks[idx]

    return boxes, masks, classes, scores

def zpmc_detect(batch_idx, conf_preds, decoded_boxes, mask_data, conf_thresh):
    cur_scores = conf_preds[batch_idx, 1:, :]
    conf_scores = np.max(cur_scores, axis=0)

    keep = (conf_scores > conf_thresh)
    scores = cur_scores[:, keep]
    boxes = decoded_boxes[keep, :]
    masks = mask_data[batch_idx, keep, :]

    if scores.shape[1] == 0:
        return None

    boxes, masks, classes, scores = fast_nms(boxes, masks, scores, iou_threshold=0.5, top_k=200)

    return {'box': boxes, 'mask': masks, 'class': classes, 'score': scores}


def zpmc_PostProcess(classes, proto_data, prior_data, mask_data, conf_data, loc_data, conf_thresh):
    num_classes = len(classes)
    batch_size = loc_data.shape[0]
    num_priors = prior_data.shape[0]
    
    conf_preds = conf_data.reshape(batch_size, num_priors, num_classes)
    conf_preds = np.transpose(conf_preds, (0, 2, 1))
    out = []
    for batch_idx in range(batch_size):
        decoded_boxes = zpmc_decode(loc_data[batch_idx], prior_data)
        result = zpmc_detect(batch_idx, conf_preds, decoded_boxes, mask_data, conf_thresh)
        if result is not None:
            result['proto'] = proto_data[batch_idx]
        out.append(result)
    return out
def zpmc_sigmoid(x):
    return 1 / (1 + np.exp(-x))

def zpmc_sanitize_coordinates(_x1, _x2, img_size:int, padding:int=0, cast:bool=True):
    """
    Sanitizes the input coordinates so that x1 < x2, x1 != x2, x1 >= 0, and x2 <= image_size.
    Also converts from relative to absolute coordinates and casts the results to long tensors.

    If cast is false, the result won't be cast to longs.
    Warning: this does things in-place behind the scenes so copy if necessary.
    """
    _x1 = _x1 * img_size
    _x2 = _x2 * img_size
    if cast:
        _x1 = _x1.long()
        _x2 = _x2.long()
    x1 = np.minimum(_x1, _x2)
    x2 = np.maximum(_x1, _x2)
    x1 = np.clip(x1-padding, a_min=0, a_max=10e10)
    x2 = np.clip(x2+padding, a_min=-10e10, a_max=img_size)

    return x1, x2

def zpmc_crop(masks, boxes, padding:int=1):
    """
    "Crop" predicted masks by zeroing out everything not in the predicted bbox.
    Vectorized by Chong (thanks Chong).

    Args:
        - masks should be a size [h, w, n] tensor of masks
        - boxes should be a size [n, 4] tensor of bbox coords in relative point form
    """
    h, w, n = masks.shape
    x1, x2 = zpmc_sanitize_coordinates(boxes[:, 0], boxes[:, 2], w, padding, cast=False)
    y1, y2 = zpmc_sanitize_coordinates(boxes[:, 1], boxes[:, 3], h, padding, cast=False)

    # rows = torch.arange(w, device=masks.device, dtype=x1.dtype).view(1, -1, 1).expand(h, w, n)
    # cols = torch.arange(h, device=masks.device, dtype=x1.dtype).view(-1, 1, 1).expand(h, w, n)
    rows = np.arange(w, dtype=x1.dtype).reshape(1, -1, 1)
    rows = np.broadcast_to(rows, (h, w, n))
    cols = np.arange(h, dtype=x1.dtype).reshape(-1, 1, 1)
    cols = np.broadcast_to(cols, (h, w, n))
    
    # masks_left  = rows >= x1.view(1, 1, -1)
    # masks_right = rows <  x2.view(1, 1, -1)
    # masks_up    = cols >= y1.view(1, 1, -1)
    # masks_down  = cols <  y2.view(1, 1, -1)
    masks_left  = rows >= x1.reshape(1, 1, -1)
    masks_right = rows <  x2.reshape(1, 1, -1)
    masks_up    = cols >= y1.reshape(1, 1, -1)
    masks_down  = cols <  y2.reshape(1, 1, -1)
    
    crop_mask = masks_left * masks_right * masks_up * masks_down
    
    return masks * crop_mask.astype(np.float32)

def zpmc_display(dets_out, imgs, h, w, mask_alpha=0.45, score_threshold=0.15):
    pre_classes, pre_scores, pre_boxes, pre_masks = [], [], [], []
    for img, dets in zip(imgs, dets_out):
        # img_gpu = img / 255.0
        # h, w, _ = img.shape
        if dets != None:
            # for dets in dets_out:
            if dets is None:
                return None

            if score_threshold > 0:
                keep = dets['score'] > score_threshold

                for k in dets:
                    if k != 'proto':
                        dets[k] = dets[k][keep]

                if dets['score'].shape[0] == 0:
                    # return None
                    pre_classes.append(None)
                    pre_scores.append(None)
                    pre_boxes.append(None)
                    pre_masks.append(None)
                    continue

            # Actually extract everything from dets now
            classes = dets['class']
            boxes   = dets['box']
            scores  = dets['score']
            masks   = dets['mask']
            proto_data = dets['proto']

            masks = proto_data @ masks.T
            masks = zpmc_sigmoid(masks)
            masks = zpmc_crop(masks, boxes) * 255

            masks = np.sum(masks, axis=2)
            masks = np.clip(masks, a_min=0.0, a_max=255.0)

            # masks = cv2.resize(masks, (w, h), interpolation=cv2.INTER_LINEAR)
            # masks = np.where(masks > 80, 255, 0)
            masks = cv2.cvtColor(masks.astype(np.uint8), cv2.COLOR_GRAY2BGR)

            boxes[:, 0], boxes[:, 2] = zpmc_sanitize_coordinates(boxes[:, 0], boxes[:, 2], w, cast=False)
            boxes[:, 1], boxes[:, 3] = zpmc_sanitize_coordinates(boxes[:, 1], boxes[:, 3], h, cast=False)



            pre_classes.append(classes)
            pre_scores.append(scores)
            pre_boxes.append(boxes)
            pre_masks.append(masks)
        else:
            pre_classes.append(None)
            pre_scores.append(None)
            pre_boxes.append(None)
            pre_masks.append(None)
    return pre_classes, pre_scores, pre_boxes, pre_masks

def zpmc_onnx2trt(onnxFile, trtFile_save_dir, trtFile_save_name, FPMode):
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
    outputTensor2 = network.get_output(2)
    outputTensor3 = network.get_output(3)
    outputTensor4 = network.get_output(4)
    print('inputTensor:', inputTensor.name, inputTensor.shape)
    print('outputTensor0:', outputTensor0.name, outputTensor0.shape)
    print('outputTensor1:', outputTensor1.name, outputTensor1.shape)
    print('outputTensor2:', outputTensor2.name, outputTensor2.shape)
    print('outputTensor3:', outputTensor3.name, outputTensor3.shape)
    print('outputTensor4:', outputTensor4.name, outputTensor4.shape)

    _, nHeight, nWidth, _ = inputTensor.shape
    profile.set_shape(inputTensor.name, (4, nHeight, nWidth, 3), (4, nHeight, nWidth, 3), (4, nHeight, nWidth, 3)) # 最小batch，常见batch，最大batch
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
project_file_dir = '/root/code/zpmc_unloading_loading_ws/zpmc_project_file/onnx_trt/corner_line/'
onnxFile = project_file_dir + 'yolact.onnx'
trtFile_save_dir = project_file_dir
trtFile_save_name = 'yolact16.trt'
FPMode = 'FP16'
context, inputs, outputs, bindings, stream = zpmc_onnx2trt(onnxFile, trtFile_save_dir, trtFile_save_name, FPMode)


class zpmc_lock_hole_recognition:
    def __init__(self):
        
        print("start zpmc_lock_hole_recognition!")
        self.lock_hole_mask_publisher = rospy.Publisher('zpmc_lock_hole_masks', ZpmcLockHoleMasks, queue_size=1)  # 创建消息发布者
        # self.rate = rospy.Rate(1)  # 设置消息发布频率为1Hz
        rospy.Subscriber('zpmc_lock_hole_preprocess_images', ZpmcLockHolePreprocessImages, self.callback)

        self.pre_time = datetime.datetime.now()
        self.cur_time = datetime.datetime.now()
        self.timediff = 1
        self.counter = 0

        self.lock_hole_mask_msg = ZpmcLockHoleMasks()

        rospy.spin()


    def callback(self, data):
        
        zpmc_firstlanding_enable = rospy.get_param("/zpmc_unloading_parameters_node/zpmc_firstlanding_enable")
        zpmc_loading_enable = rospy.get_param("/zpmc_unloading_parameters_node/zpmc_loading_enable")
        zpmc_unloading_enable = rospy.get_param("/zpmc_unloading_parameters_node/zpmc_unloading_enable")

        self.pre_time = self.cur_time
        self.cur_time = datetime.datetime.now()
        if zpmc_firstlanding_enable == 1 and zpmc_loading_enable == 0 and zpmc_unloading_enable == 0:

            image_src_0 = CvBridge().imgmsg_to_cv2(data.tl_preprocess_image, "bgr8") # 获取输入的原始图像，并且ros图像格式转opencv图像格式
            image_src_1 = CvBridge().imgmsg_to_cv2(data.tr_preprocess_image, "bgr8") # 获取输入的原始图像，并且ros图像格式转opencv图像格式
            image_src_2 = CvBridge().imgmsg_to_cv2(data.bl_preprocess_image, "bgr8") # 获取输入的原始图像，并且ros图像格式转opencv图像格式
            image_src_3 = CvBridge().imgmsg_to_cv2(data.br_preprocess_image, "bgr8") # 获取输入的原始图像，并且ros图像格式转opencv图像格式

            images_resize_list = [image_src_0, image_src_1, image_src_2, image_src_3]
            # images_process = zpmc_std_mean(images_resize_list)

            inputs[0].host = np.ascontiguousarray(images_resize_list, dtype=np.float32)
            # cfx.push()
            trt_outputs = common.do_inference_v2(context, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)
            # cfx.pop()
            batch_size = len(images_resize_list)
            trt_outputs_shape = [(batch_size, 138, 138, 32), (batch_size, 19248, 4), (batch_size, 19248, 32), (batch_size, 19248, 2), (19248, 4)]
            
            proto_data = trt_outputs[0].reshape(trt_outputs_shape[0])
            loc_data = trt_outputs[1].reshape(trt_outputs_shape[1])
            mask_data = trt_outputs[2].reshape(trt_outputs_shape[2])
            conf_data = trt_outputs[3].reshape(trt_outputs_shape[3])
            prior_data = trt_outputs[4].reshape(trt_outputs_shape[4])
            
            CLASSES = ['BG', 'LOCKHOLE']
            result = zpmc_PostProcess(CLASSES, proto_data, prior_data, mask_data, conf_data, loc_data, conf_thresh=0.05)
            classes, scores, boxes, masks = zpmc_display(result, images_resize_list, 720, 1280, score_threshold=0.15)

            # cv2.imwrite('/root/code/zpmc_unloading_loading_ws/zpmc_project_file/onnx_trt/'+str(self.cur_time)+'_0.jpg', masks[0])
            # cv2.imwrite('/root/code/zpmc_unloading_loading_ws/zpmc_project_file/onnx_trt/'+str(self.cur_time)+'_1.jpg', masks[1])
            # cv2.imwrite('/root/code/zpmc_unloading_loading_ws/zpmc_project_file/onnx_trt/'+str(self.cur_time)+'_2.jpg', masks[2])
            # cv2.imwrite('/root/code/zpmc_unloading_loading_ws/zpmc_project_file/onnx_trt/'+str(self.cur_time)+'_3.jpg', masks[3])

            self.lock_hole_mask_msg.tl_image = data.tl_image
            self.lock_hole_mask_msg.tr_image = data.tr_image
            self.lock_hole_mask_msg.bl_image = data.bl_image
            self.lock_hole_mask_msg.br_image = data.br_image

            if masks[0] is not None:
                self.lock_hole_mask_msg.tl_mask_image = CvBridge().cv2_to_imgmsg(masks[0],"bgr8")
            else:
                temp = np.zeros_like(image_src_0)
                self.lock_hole_mask_msg.tl_mask_image = CvBridge().cv2_to_imgmsg(temp,"bgr8")

            if masks[1] is not None:
                self.lock_hole_mask_msg.tr_mask_image = CvBridge().cv2_to_imgmsg(masks[1],"bgr8")
            else:
                temp = np.zeros_like(image_src_1)
                self.lock_hole_mask_msg.tr_mask_image = CvBridge().cv2_to_imgmsg(temp,"bgr8")

            if masks[2] is not None:
                self.lock_hole_mask_msg.bl_mask_image = CvBridge().cv2_to_imgmsg(masks[2],"bgr8")
            else:
                temp = np.zeros_like(image_src_2)
                self.lock_hole_mask_msg.bl_mask_image = CvBridge().cv2_to_imgmsg(temp,"bgr8")

            if masks[3] is not None:
                self.lock_hole_mask_msg.br_mask_image = CvBridge().cv2_to_imgmsg(masks[3],"bgr8")
            else:
                temp = np.zeros_like(image_src_3)
                self.lock_hole_mask_msg.br_mask_image = CvBridge().cv2_to_imgmsg(temp,"bgr8")

            self.lock_hole_mask_publisher.publish(self.lock_hole_mask_msg)
            print('This is firstlanding work!')
        else:
            print('This is not firstlanding work!')
        self.timediff = (self.cur_time - self.pre_time).total_seconds()
        rospy.set_param("/zpmc_unloading_parameters_node/zpmc_corner_line_ai_fps", 1.0/self.timediff)
        zpmc_corner_line_ai_fps = rospy.get_param("/zpmc_unloading_parameters_node/zpmc_corner_line_ai_fps")
        print('FPS {:2.3f}'.format(zpmc_corner_line_ai_fps))
        


class HB:
    def __init__(self):
        self.counter = 0

    def zpmc_heat_beat_callback(self, event):
        rospy.set_param("/zpmc_unloading_parameters_node/zpmc_corner_line_ai_heat_beat", self.counter)
        zpmc_corner_line_ai_heat_beat = rospy.get_param("/zpmc_unloading_parameters_node/zpmc_corner_line_ai_heat_beat")
        if self.counter>1000:
            self.counter = 0
        self.counter += 1
        print("zpmc_corner_line_ai_heat_beat:", zpmc_corner_line_ai_heat_beat)



if __name__ == '__main__':
    try:
        tt = HB()
        rospy.init_node('xian_keypoints_recognition', anonymous=True)  # 初始化ROS节点
        rospy.Timer(rospy.Duration(1), tt.zpmc_heat_beat_callback, oneshot=False)
        publisher = zpmc_lock_hole_recognition()

    except rospy.ROSInterruptException:
        pass