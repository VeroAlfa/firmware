#!/usr/bin/python3
import rclpy
from rclpy.node import Node
# Yaml load
import os
import yaml 
from ament_index_python.packages import get_package_share_directory

# Publisher Message
from  nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from xicro_interfaces.msg import DiffDriveTwist
# Subscriber Message
from xicro_interfaces.msg import Odometry as odom
from xicro_interfaces.msg import Imu as imu
# Node internal value
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3


from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class NavMsgPublisher(Node):

    def __init__(self):
        super().__init__('nav_msg_publisher')
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.wheel_odometry_subscriber = self.create_subscription(odom, '/nav_stm32', self.callback_nav_stm32, 10)
        self.imu_subscriber = self.create_subscription(imu, '/imu_stm32', self.callback_imu_stm32, 10)

        self.wheel_odometry_publisher = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.robot_twist_publisher = self.create_publisher(DiffDriveTwist, '/cmd_vel_stm32', 10)

        # Node internal value
        self.robot_pose = Pose()
        self.robot_twist = Twist()
        self.robot_cmd_vel = DiffDriveTwist()
        self.imu_orientation = Quaternion()
        self.imu_angular_velocity = Vector3()
        self.imu_linear_acceleration = Vector3()

        # Load Imu calibration value from Yaml file
        calibration_gen_path = get_package_share_directory('calibration')
        path = os.path.join(calibration_gen_path,'config','sensor_properties.yaml')
        with open(path) as f:
            self.properties = yaml.load(f, Loader=yaml.loader.UnsafeLoader)
        
        print("Load IMU calibration file: 'sensor_properties.yaml' in share calibration package Success...")
        
        # Set imu calibration value from yaml file
        self.lin_acc_x_bias = self.properties['mean'][0]
        self.lin_acc_y_bias = self.properties['mean'][1]
        self.lin_acc_z_bias = self.properties['mean'][2]
        self.ang_vel_x_bias = self.properties['mean'][3]
        self.ang_vel_y_bias = self.properties['mean'][4]
        self.ang_vel_z_bias = self.properties['mean'][5]
        # Set imu variance from yaml file
        self.lin_acc_x_var = self.properties['covariance'][0]
        self.lin_acc_y_var = self.properties['covariance'][7]
        self.lin_acc_z_var = self.properties['covariance'][14]
        self.ang_vel_x_var = self.properties['covariance'][21]
        self.ang_vel_y_var = self.properties['covariance'][28]
        self.ang_vel_z_var = self.properties['covariance'][35]

        self.tf_broadcaster = TransformBroadcaster(self)

    def cmd_vel_callback(self,msg:Twist):
        # Assign value from twist msg to node internal value
        self.robot_cmd_vel.linear = msg.linear.x
        self.robot_cmd_vel.angular = msg.angular.z
        # Publish DiffDriveTwist to robot (with same rate of command velocity)
        twist_msg = DiffDriveTwist()
        twist_msg = self.robot_cmd_vel
        self.robot_twist_publisher.publish(twist_msg)

    def callback_nav_stm32(self,msg:odom):
        # Assign value from odom msg to node internal value
        self.robot_pose = msg.pose
        self.robot_twist = msg.twist
        # Publish Odometry to navigation (with same rate of controller)
        odom_msg = Odometry()
        odom_msg.header.frame_id="odom"
        now = self.get_clock().now()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.child_frame_id="base_footprint"

        # PoseWithCovariance
        odom_msg.pose.pose.position.x = self.robot_pose.position.x
        odom_msg.pose.pose.position.y = self.robot_pose.position.y
        odom_msg.pose.pose.position.z = self.robot_pose.position.z
        odom_msg.pose.pose.orientation.x = self.robot_pose.orientation.x
        odom_msg.pose.pose.orientation.y = self.robot_pose.orientation.y
        odom_msg.pose.pose.orientation.z = self.robot_pose.orientation.z
        odom_msg.pose.pose.orientation.w = self.robot_pose.orientation.w

        odom_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.000000001, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.000000001, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.000000001, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.1,]
        # TwistWithCovariance
        odom_msg.twist.twist.linear.x = self.robot_twist.linear.x
        odom_msg.twist.twist.linear.y = self.robot_twist.linear.y
        odom_msg.twist.twist.linear.z = self.robot_twist.linear.z
        odom_msg.twist.twist.angular.x = self.robot_twist.angular.x
        odom_msg.twist.twist.angular.y = self.robot_twist.angular.y
        odom_msg.twist.twist.angular.z = self.robot_twist.angular.z

        odom_msg.twist.covariance = [0.00477649365, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.000000001, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.000000001, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.000000001, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.000000001, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0530721471,]
        
        self.wheel_odometry_publisher.publish(odom_msg)
#################################################################################################
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header= odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.robot_pose.position.x
        t.transform.translation.y = self.robot_pose.position.y
        t.transform.translation.z = self.robot_pose.position.z
        t.transform.rotation.x = self.robot_pose.orientation.x
        t.transform.rotation.y = self.robot_pose.orientation.y
        t.transform.rotation.z = self.robot_pose.orientation.z
        t.transform.rotation.w = self.robot_pose.orientation.w

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
#################################################################################################

    def callback_imu_stm32(self,msg:imu):
        # Assign value from imu msg to node internal value
        self.imu_orientation = msg.orientation
        self.imu_angular_velocity = msg.angular_velocity
        self.imu_linear_acceleration = msg.linear_acceleration
        # Publish Imu to navigation (with same rate of controller)
        imu_msg = Imu()
        imu_msg.header.frame_id="base_footprint"
        now = self.get_clock().now()
        imu_msg.header.stamp = now.to_msg()

        # Linear acceleration
        imu_msg.linear_acceleration.x = self.imu_linear_acceleration.x - self.lin_acc_x_bias
        imu_msg.linear_acceleration.y = self.imu_linear_acceleration.y - self.lin_acc_y_bias
        imu_msg.linear_acceleration.z = self.imu_linear_acceleration.z - self.lin_acc_z_bias
        
        # Angular velocity
        imu_msg.angular_velocity.x = self.imu_angular_velocity.x - self.ang_vel_x_bias
        imu_msg.angular_velocity.y = self.imu_angular_velocity.y - self.ang_vel_y_bias
        imu_msg.angular_velocity.z = self.imu_angular_velocity.z - self.ang_vel_z_bias

        # Orientation
        imu_msg.orientation.x = self.imu_orientation.x
        imu_msg.orientation.y = self.imu_orientation.y
        imu_msg.orientation.z = self.imu_orientation.z
        imu_msg.orientation.w = self.imu_orientation.w

        imu_msg.linear_acceleration_covariance =[self.lin_acc_x_var,0.0,0.0,0.0,self.lin_acc_y_var,0.0,0.0,0.0,self.lin_acc_z_var]
        imu_msg.angular_velocity_covariance=[self.ang_vel_x_var,0.0,0.0,0.0,self.ang_vel_y_var,0.0,0.0,0.0,self.ang_vel_z_var]
        imu_msg.orientation_covariance=[0.000000001,0.0,0.0,0.0,0.000000001,0.0,0.0,0.0,0.000000001]
    
        self.imu_publisher.publish(imu_msg)

############################################################################
    def quaternion_from_euler(self,ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
##################################################################################



def main(args=None):
    rclpy.init(args=args)

    nav_msg_publisher = NavMsgPublisher()

    rclpy.spin(nav_msg_publisher)
    nav_msg_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()