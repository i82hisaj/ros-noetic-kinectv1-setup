#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import mediapipe as mp
import cv2
import paho.mqtt.client as mqtt
import time
import threading

class MediaPipePoseNode:

    def __init__(self):
        rospy.init_node('mediapipe_pose_node', anonymous=True)
        self.bridge = CvBridge()
        self.pose = mp.solutions.pose.Pose(min_detection_confidence=0.5)

        self.pub_right_hand = rospy.Publisher('/mediapipe/right_hand', PointStamped, queue_size=10)
        self.pub_left_hand = rospy.Publisher('/mediapipe/left_hand', PointStamped, queue_size=10)

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("localhost")

        # Inicia loop no bloqueante MQTT en segundo plano
        self.mqtt_client.loop_start()

        self.mqtt_topic = "domotica/brazo_presencia"

        self.threshold_y = 0.5  # Umbral para brazo levantado
        self.hand_join_threshold = 0.1  # Distancia XY para manos juntas
        self.last_publish_time = 0
        self.publish_interval = 1.0  # mínimo 1 segundo entre mensajes
        self.last_processed_time = 0
        self.process_interval = 0.1  # Procesar máximo 10 FPS

        self.person_detected = False

        rospy.loginfo("MediaPipe Pose Node with MQTT started")
        rospy.spin()

    def distance_xy(self, lm1, lm2):
        return ((lm1.x - lm2.x)**2 + (lm1.y - lm2.y)**2)**0.5

    def image_callback(self, msg):
        try:
            current_time = time.time()
            # Limita tasa de procesamiento
            if current_time - self.last_processed_time < self.process_interval:
                return
            self.last_processed_time = current_time

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.pose.process(cv_image)

            if results.pose_landmarks:
                if not self.person_detected:
                    rospy.loginfo("Persona detectada")
                    self.mqtt_client.publish(self.mqtt_topic, "PERSONA_DETECTADA")
                    self.person_detected = True
                    self.last_publish_time = current_time

                right_hand = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_WRIST]
                left_hand = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_WRIST]
                nose = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.NOSE]

                point_right = PointStamped(header=msg.header)
                point_right.point.x = right_hand.x
                point_right.point.y = right_hand.y
                point_right.point.z = right_hand.z
                self.pub_right_hand.publish(point_right)

                point_left = PointStamped(header=msg.header)
                point_left.point.x = left_hand.x
                point_left.point.y = left_hand.y
                point_left.point.z = left_hand.z
                self.pub_left_hand.publish(point_left)

                right_up = right_hand.y < self.threshold_y
                left_up = left_hand.y < self.threshold_y
                hands_together_above_head = (self.distance_xy(right_hand, left_hand) < self.hand_join_threshold) and \
                                            (right_hand.y < nose.y) and (left_hand.y < nose.y)

                if current_time - self.last_publish_time > self.publish_interval:
                    if right_up and left_up:
                        rospy.loginfo("Ambos brazos levantados")
                        self.mqtt_client.publish(self.mqtt_topic, "AMBOS_BRAZOS_LEVANTADOS")
                        self.last_publish_time = current_time
                    elif hands_together_above_head:
                        rospy.loginfo("Manos unidas encima cabeza")
                        self.mqtt_client.publish(self.mqtt_topic, "MANOS_UNIDAS_ENCIMA_CABEZA")
                        self.last_publish_time = current_time
                    elif right_up:
                        rospy.loginfo("Brazo derecho levantado")
                        self.mqtt_client.publish(self.mqtt_topic, "BRAZO_DERECHO_LEVANTADO")
                        self.last_publish_time = current_time
                    elif left_up:
                        rospy.loginfo("Brazo izquierdo levantado")
                        self.mqtt_client.publish(self.mqtt_topic, "BRAZO_IZQUIERDO_LEVANTADO")
                        self.last_publish_time = current_time

            else:
                if self.person_detected:
                    rospy.loginfo("Fin de detección persona")
                    self.mqtt_client.publish(self.mqtt_topic, "FIN_DETECCION_PERSONA")
                    self.person_detected = False
                    self.last_publish_time = current_time

        except Exception as e:
            rospy.logwarn(f"Error en procesamiento: {e}")

if __name__ == '__main__':
    try:
        MediaPipePoseNode()
    except rospy.ROSInterruptException:
        pass
