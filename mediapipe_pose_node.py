#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import mediapipe as mp
import cv2

class MediaPipePoseNode:

    def __init__(self):
        rospy.init_node('mediapipe_pose_node', anonymous=True)
        self.bridge = CvBridge()
        self.pose = mp.solutions.pose.Pose(min_detection_confidence=0.5)
        self.pub_right_hand = rospy.Publisher('/mediapipe/right_hand', PointStamped, queue_size=10)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.loginfo("MediaPipe Pose Node started")
        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"CV bridge error: {e}")
            return

        results = self.pose.process(cv_image)

        if results.pose_landmarks:
            right_hand = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_WRIST]
            point_msg = PointStamped()
            point_msg.header = msg.header
            # Normalized image coordinates (0-1 range)
            point_msg.point.x = right_hand.x
            point_msg.point.y = right_hand.y
            point_msg.point.z = right_hand.z
            self.pub_right_hand.publish(point_msg)
            rospy.loginfo(f"Right hand detected at ({right_hand.x:.3f}, {right_hand.y:.3f}, {right_hand.z:.3f})")

if __name__ == '__main__':
    try:
        MediaPipePoseNode()
    except rospy.ROSInterruptException:
        pass
