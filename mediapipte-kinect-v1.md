Pasos para usar MediaPipe con Kinect v1 y ROS en Ubuntu 20.04
Capturar video RGB del Kinect

Usa drivers libres como libfreenect o paquetes ROS que publiquen /camera/rgb/image_raw.

Instalar MediaPipe y dependencias

bash
sudo apt update
sudo apt install python3-pip
pip3 install mediapipe opencv-python
Crear nodo ROS Python para procesar frames con MediaPipe

Sucribe al topic de imagen RGB.

Convierte ROS Image a OpenCV.

Usa MediaPipe para detectar pose y extraer coordenadas esqueléticas.

Publica las coordenadas como nuevos topics ROS o envía eventos via MQTT.

Requisitos previos
Sensor Kinect v1 conectado.

Driver libre que publique RGB: freenect_launch disponible en ROS para Kinect.

Instalar MediaPipe y OpenCV para Python:

bash
pip3 install mediapipe opencv-python
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport


Cómo crear y usar el paquete mediapipe_pose en ROS
Crea tu workspace si no tienes uno:

bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
Dentro del src clona o crea el paquete mediapipe_pose:

bash
cd ~/catkin_ws/src
catkin_create_pkg mediapipe_pose rospy std_msgs sensor_msgs cv_bridge
Pon tu script mediapipe_pose_node.py dentro del folder mediapipe_pose/scripts/.

Da permisos de ejecución:

bash
chmod +x ~/catkin_ws/src/mediapipe_pose/scripts/mediapipe_pose_node.py
Edita el archivo package.xml y CMakeLists.txt si es necesario para asegurar dependencias correctas (normalmente catkin_create_pkg ya incluye lo básico para Python).

Compila el workspace para registrar el paquete:

bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
Corre tu nodo:

bash
rosrun mediapipe_pose mediapipe_pose_node.py
