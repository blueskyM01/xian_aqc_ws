import rospy
import cv2
import numpy as np
import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from xian_msg_pkg.msg import xian_spreader_images_msg

class XianSpreaderImagesShow:
    def __init__(self):
        self.bridge = CvBridge()
        self.counter = 0

        # Initialize ROS node
        rospy.init_node('xian_spreader_images_show_node', anonymous=True)

        # Subscribe to the topic
        self.command_subscribe = rospy.Subscriber('xian_aqc_spreader_images', xian_spreader_images_msg, self.command_callback)

        # Create a wall timer for heartbeat
        self.m_timer_HeartBeat = rospy.Timer(rospy.Duration(1.0), self.m_timer_HeartBeat_f)

        # Initialize time variables
        self.cur_time = rospy.Time.now()
        self.pre_time = rospy.Time.now()
        self.timediff = 1.0

        # Initialize OpenCV windows
        # cv2.namedWindow("xian_spreader_images_show01", cv2.WINDOW_NORMAL)

    def m_timer_HeartBeat_f(self, event):
        self.counter = self.counter + 1 if self.counter < 1000 else 0
        rospy.loginfo(f"xian_spreader_images_show_heart_beat: {self.counter}")

    def command_callback(self, data):
        timeStr = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        self.pre_time = self.cur_time
        self.cur_time = rospy.Time.now()

        try:
            tl_image = self.bridge.imgmsg_to_cv2(data.tl_image, "bgr8")
            tr_image = self.bridge.imgmsg_to_cv2(data.tr_image, "bgr8")
            bl_image = self.bridge.imgmsg_to_cv2(data.bl_image, "bgr8")
            br_image = self.bridge.imgmsg_to_cv2(data.br_image, "bgr8")

            # Merging images
            mask_merge_col_0 = np.vstack((tl_image, bl_image))
            mask_merge_col_1 = np.vstack((tr_image, br_image))
            merge_row1 = np.hstack((mask_merge_col_0, mask_merge_col_1))

            # Resize using CUDA
            merge_row1_gpu = cv2.cuda_GpuMat()
            merge_row1_resize_gpu = cv2.cuda_GpuMat()
            merge_row1_gpu.upload(merge_row1)
            merge_row1_resize_gpu = cv2.cuda.resize(merge_row1_gpu, (merge_row1.shape[1] // 4, merge_row1.shape[0] // 4))
            merge_row1_resize = merge_row1_resize_gpu.download()
            
            # merge_row1_resize = cv2.resize(merge_row1, (merge_row1.shape[1] // 4, merge_row1.shape[0] // 4))

            if merge_row1_resize is not None:
                cv2.imshow("xian_spreader_images_show01", merge_row1_resize)
                cv2.waitKey(10)
                print(merge_row1_resize.shape)
                pass
            else:
                rospy.logerr("Failed to load image")

            # Calculate FPS
            elapsedTimeP = (self.cur_time - self.pre_time).to_sec() * 1000
            self.timediff = elapsedTimeP
            rospy.loginfo(f"FPS: {1000.0 / self.timediff}")

        except Exception as e:
            rospy.logerr(f"Error in processing images: {e}")

if __name__ == '__main__':
    try:
        xian_spreader_images_show = XianSpreaderImagesShow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
