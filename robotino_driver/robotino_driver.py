import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from robotino_interfaces.msg import MotorPositions, MotorVelocities, MotorCurrents, DistanceSensorVoltages
import numpy as np
from numpy import random 
from math import sin, cos, pi
import time

class RobotinoDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__motor_number = 3

        self.__mot_pos_msg = MotorPositions()
        self.__motor = []
        motor_order = [2, 0, 1]
        j = 0
        for i in motor_order:
            self.__motor.append(self.__robot.getDevice('wheel%d_joint' % i))
            self.__motor[j].setPosition(float('inf'))
            self.__motor[j].setVelocity(0)
            self.__mot_pos_msg.pos[i] = 0
            j = j + 1
    
        self.__sample_period = 64
        self.__distance_sensor_number = 9
        self.__distance_sensor = []
        j = 0
        for i in range(self.__distance_sensor_number):
            self.__distance_sensor.append(self.__robot.getDevice('ir%d' % (i+1)))
            self.__distance_sensor[j].enable(self.__sample_period)
            j = j + 1
        
        self.__bumper = self.__robot.getDevice('touch sensor')
        self.__bumper.enable(self.__sample_period)
        
        self.__encoder_resolution = 500
        self.__gear_ratio = 32
        self.__set_point_wheel_vel = [0, 0, 0]

        rclpy.init(args=None)
        self.__node_name = 'robotino_driver'
        self.__node = rclpy.create_node(self.__node_name)
        self.__node.create_subscription(Twist, self.__node_name + '/cmd_vel', self.__cmd_vel_callback, 10)
        self.__node.create_subscription(MotorVelocities, self.__node_name + '/cmd_mot_vel', self.__cmd_mot_vel_callback, 10)
        self.__bumper_publisher = self.__node.create_publisher(Bool, self.__node_name + '/bumper', 10)
        self.__mot_pos_publisher = self.__node.create_publisher(MotorPositions, self.__node_name + '/mot_pos', 10)
        self.__mot_vel_publisher = self.__node.create_publisher(MotorVelocities, self.__node_name + '/mot_vel', 10)
        self.__mot_cur_publisher = self.__node.create_publisher(MotorCurrents, self.__node_name + '/mot_cur', 10)
        self.__dist_sens_publisher = self.__node.create_publisher(DistanceSensorVoltages, self.__node_name + '/dist_sens', 10)

        self.__last_message_time = time.time() # s

    def __cmd_vel_callback(self, twist):
        self.__set_point_wheel_vel = self.inverse_kinematics(twist)
        # self.__node.get_logger().info('Receive robot speed: %f, %f, %f' % (twist.linear.x, twist.linear.y, twist.angular.z))
        self.__last_message_time = time.time()

    def __cmd_mot_vel_callback(self, motor_velocities):
        self.__set_point_wheel_vel = [vel / self.__gear_ratio for vel in motor_velocities.vel]
        # self.__node.get_logger().info('Receive motor velocity: %f, %f, %f' % (motor_velocities.vel[0], motor_velocities.vel[1], motor_velocities.vel[2]))
        self.__last_message_time = time.time()

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Check message timeout
        if((time.time() - self.__last_message_time) > 0.1):
            self.__set_point_wheel_vel = [0, 0, 0]
        # self.__node.get_logger().info('Wheel velocity: %f, %f, %f' % (self.__set_point_wheel_vel[0], self.__set_point_wheel_vel[1], self.__set_point_wheel_vel[2]))
        for i in range(self.__motor_number):
            self.__motor[i].setVelocity(self.__set_point_wheel_vel[i])

        bumper_msg = Bool()
        bumper_msg.data = bool(self.__bumper.getValue())
        self.__bumper_publisher.publish(bumper_msg)

        mot_vel_msg = MotorVelocities()
        mot_cur_msg = MotorCurrents()
        current_mean = 0.25
        current_sigma = 0.2
        for i in range(self.__motor_number):
            mot_vel_msg.vel[i] = float(self.__motor[i].getVelocity())
            self.__mot_pos_msg.pos[i] = self.__mot_pos_msg.pos[i] + \
                                        int(mot_vel_msg.vel[i] * self.__sample_period * 1e-3 / (2*pi) * self.__encoder_resolution)
            mot_cur_msg.cur[i] = float(np.random.normal(current_mean, current_sigma))
        
        self.__mot_pos_publisher.publish(self.__mot_pos_msg)
        self.__mot_vel_publisher.publish(mot_vel_msg)
        self.__mot_cur_publisher.publish(mot_cur_msg)

        dist_sens_msg = DistanceSensorVoltages()
        for i in range(self.__distance_sensor_number):
            dist_sens_msg.vol[i] = float(self.__distance_sensor[i].getValue())
        self.__dist_sens_publisher.publish(dist_sens_msg)

    def inverse_kinematics(self, twist):
        wheel_velocity = [0, 0, 0]

        wheel_angle = [pi/3, pi, 5*pi/3]
        wheel_radius_m = 63e-3
        robot_radius_m = 182.6e-3
        for i in range(self.__motor_number):
            wheel_velocity[i] = -(-twist.linear.x * sin(wheel_angle[i]) \
                             + twist.linear.y * cos(wheel_angle[i]) \
                             + twist.angular.z * robot_radius_m) / wheel_radius_m
        return wheel_velocity