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
        
        
        self._SerialDataGateway = SerialDataGateway("/dev/ttyAMC0", 115200,  self._HandleReceivedLine)
        #self.rosNow = Node.get_clock().now().to_msg()
        #quaternion = Quaternion()
        self.Start()
        self.subscription = self.create_subscription(Twist,'cmd_vel', self._HandleVelocityCommand,10)
        
    def _HandleReceivedLine(self,  line):
        msg = String()
        msg.data = line
        #self.publisher_.publish(msg)
        #self._Counter = self._Counter + 1
        #x = len(line)
        #self.get_logger().info(str(x))
        #if (self._Counter % 50 == 0):
        
        if (len(line) > 0):
                        lineParts = line.split('\t')                 
                        if (lineParts[0] == 'o'):
                                self._Broadcast_Odom(lineParts)
                                return
                             
        
        
    def _HandleVelocityCommand(self, twistCommand):
        """ Handle movement requests. """
        R = 42.43
        RevD = 1 # used to set forwards or backwards
        x = twistCommand.linear.y        # m/s
        y = twistCommand.linear.x        # m/s
        omega = twistCommand.angular.z      # rad/s
        
        if x < 0:
            RevD =-1
        A = x - omega*(30/R)
        B = x + omega*(30/R)
        C = y - omega*(30/R)
        D = y + omega*(30/R)
                
        j2 = math.atan2(A, D)*180 / math.pi #REAR left in degrees        
        j1 = math.atan2(A, C)*180 / math.pi #REAR right
        j4 = math.atan2(B, D)*180 / math.pi # FRONT left
        j3 = math.atan2(B, C)*180 / math.pi # FRONT right
        #convert angles to encoder ticks
        Front_left = j4 #int(self.translate(j1, -150, 150, 0, 1024))
        Front_right = j3 #+135#int(self.translate(j2, -150, 150, 0, 1024))
        Rear_left = j2 #+135#int(self.translate(j3, -150, 150, 0, 1024))
        Rear_right = j1 #+135#int(self.translate(j4, -150, 150, 0, 1024))



        #velocity commands to be turned into ticks
        V_F_L = math.sqrt((B*B)+(D*D))*1000
        V_F_R = math.sqrt((B*B)+(C*C))*1000
        V_R_R = math.sqrt((A*A)+(C*C))*1000
        V_R_L = math.sqrt((A*A)+(D*D))*1000

        #Front_rightprint(Front_left)
        #print(Front_right)
        #print(Rear_left)
        #print(Rear_right)
        a =[Front_left,Front_right,Rear_left,Rear_right]
        
        #self.get_logger().info("front angles: " + str(Front_right) +     "  " + str(Front_left))
        #self.get_logger().info("rear angles: " + str(Rear_left) +     "  " + str(Rear_right))
        self.get_logger().info("front speed: " + str(V_F_R) +     "  " + str(V_F_L))
        self.get_logger().info("rear speed: " + str(V_R_L) +     "  " + str(V_R_R))
        message = 's %d %d %d %d %d %d %d %d\r' % (Front_left, Front_right, Rear_left, Rear_right, V_F_L, V_F_R, V_R_L, V_R_R )
        self._WriteSerial(message)
        
    def Start(self):
        #self.get_logger().info("Starting start function but wait")
        #print('in start')
        self._SerialDataGateway.Start()
        message = 'x \r'
        self._WriteSerial(message)
        
    def Stop(self):
        self.get_logger().info("Stopping")
        #message = 'r \r'
        #self._WriteSerial(message)
        #sleep(5)
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
