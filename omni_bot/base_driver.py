import rclpy
import sys
import time
import tf2_ros
import math
from math import sin, cos, pi, radians, degrees
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

from SerialDataGateway3 import SerialDataGateway

class BaseDriver(Node):

    def __init__(self):
        super().__init__('omni_base_driver')
        self.get_logger().info('starting arduino control')
        
        
        self._SerialDataGateway = SerialDataGateway("/dev/ttyACM0", 115200,  self._HandleReceivedLine)
        #self.rosNow = Node.get_clock().now().to_msg()
        #quaternion = Quaternion()
        self.Start()
        self.subscription = self.create_subscription(String,'topic', self.listener_callback,10)
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
    def Start(self):
        #self.get_logger().info("Starting start function but wait")
        #print('in start')
        self._SerialDataGateway.Start()
        message = 's \r'
        self._WriteSerial(message)
        
    def Stop(self):
        self.get_logger().info("Stopping")
        message = 'r \r'
        self._WriteSerial(message)
        sleep(5)
        self._SerialDataGateway.Stop()
        
    def _WriteSerial(self, message):
        #self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
         self._SerialDataGateway.Write(message)

def main(args=None):
    rclpy.init(args=args)

    base_driver = BaseDriver()

    rclpy.spin(base_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
