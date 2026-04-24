import os
import rclpy
from rclpy.node import Node
import rclpy.duration
import rclpy.time
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from sensor_msgs.msg import Image


class ArucoTFPublisher(Node):
    
    def __init__(self):
        super().__init__('aruco_tf_publisher')

        self.package_path = os.path.dirname(os.path.abspath(__file__))
        self.object_point_file = os.path.join(self.package_path, 'object_point_from_robot.txt')

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        self.center_point_camera_ = PointStamped()
        self.center_point_aruco_ = PointStamped()
        self.center_point_robot_ = PointStamped()
        self.matriz_trans_aruco_robot_ = np.array([[1, 0, 0, -0.015], [0, 1, 0, 0.155], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.matriz_trans_inv_aruco_robot_ = np.linalg.inv(self.matriz_trans_aruco_robot_)

        # Crear publicador de transformaciones
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publishers
        self.publisher_center_point_ = self.create_publisher(PointStamped, '/coord_trans/center_point_from_robot', 10)
        
        # Suscriptores
        self.subscription_center_point_ = self.create_subscription(PointStamped, '/bbox/center_point', self.listener_callback_center_point, 10)
            
    def listener_callback_center_point(self, msg):

        self.center_point_camera_ = msg
        self.center_point_camera_.header.stamp = rclpy.time.Time().to_msg()
        
        try:
            self.center_point_robot_stamped_ = self.tf_buffer_.transform(
                self.center_point_camera_, 
                "wx250/base_link",
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            with open(self.object_point_file, 'w') as archivo:
                archivo.write('[%s, %s, %s]' % (
                    self.center_point_robot_stamped_.point.x,
                    self.center_point_robot_stamped_.point.y,
                    self.center_point_robot_stamped_.point.z
                ))
            
            self.center_point_robot_.header.frame_id = "wx250/base_link"
            self.center_point_robot_.point.x = self.center_point_robot_stamped_.point.x
            self.center_point_robot_.point.y = self.center_point_robot_stamped_.point.y
            self.center_point_robot_.point.z = self.center_point_robot_stamped_.point.z
            
            print(self.center_point_robot_)
            self.publisher_center_point_.publish(self.center_point_robot_)

        except Exception as e:
            self.get_logger().warn('TF no disponible aún: %s' % str(e))
       
        
def main(args=None):
    
    rclpy.init(args=args)

    aruco_publisher_node = ArucoTFPublisher()

    rclpy.spin(aruco_publisher_node)
    
    aruco_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
