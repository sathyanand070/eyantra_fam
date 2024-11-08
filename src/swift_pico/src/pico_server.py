#!/usr/bin/env python3

import time
import math
import numpy as np
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor



#import the action
from waypoint_navigation.action import NavToWaypoint
#pico control specific libraries
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose

class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.duration = 0


        self.drone_position =np.array([0.0, 0.0, 0.0,0.0])
        self.setpoint = np.array([0, 0, 27,0.0])
        self.dtime = 0

   

        #Kp, Ki and Kd values here
        
        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
 
        self.dt = 0 

        # [x_setpoint, y_setpoint, z_setpoint]
 # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        
        #self.vel.poses[0].position.x

        #initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        #after tuning and computing corresponding PID parameters, change the parameters

        self.Kp_1 = [100, 150, 7,0]
        self.Ki_1 = [0, 0, 0,0]
        self.Kd_1 = [0, 0, 0,0]
        #-----------------------Add other required variables for pid here ----------------------------------------------

        self.error = np.array([0.0, 0.0, 0.0, 0.0])
        self.error_prev = np.array([0.0, 0.0, 0.0, 0.0])
        self.errsum = np.array([0.0, 0.0, 0.0,0.0])
        self.current_rpt = [self.cmd.rc_roll, self.cmd.rc_pitch, self.cmd.rc_throttle]

        self.curr_v = np.array([0,0,0,0])
        self.prev_v = np.array([0,0,0,0])

        self.vel_error=np.array([0.0, 0.0, 0.0,0.0])
        self.prev_vel_error=np.array([0.0, 0.0, 0.0,0.0])
        self.vel_errsum=np.array([0.0, 0.0, 0.0,0.0])

        self.vel_setpoint=np.array([0.0, 0.0, 0.0,0.0])

        # self.Kp = [1, 1, 3,0]
        # self.Ki = [0, 0, 0,0]
        # self.Kd = [0, 0, 2,0]

        self.current_time = time.time()
        self.prev_time = time.time()


        

        #variables for storing different kinds of errors
        

        self.pid_error = PIDError()

        self.sample_time = 0.060

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        


        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        #Add other sunscribers here



        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)

        #create an action server for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'.
        #include the action_callback_group in the action server. Refer to executors in ROS 2 concepts
        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        
        self.arm()

        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_callback_group)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)


    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)


    def whycon_callback(self, msg):
        self.prev_time = self.current_time
        self.current_time = time.time()
        self.dt = self.current_time - self.prev_time
        

        if self.dt >= 0.07 :  
            self.prev_drone_position = np.copy(self.drone_position)

            self.drone_position[0] = msg.poses[0].position.x 
            self.drone_position[1] = msg.poses[0].position.y 
            self.drone_position[2] = msg.poses[0].position.z 
            self.prev_v= self.curr_v
            self.curr_v = (self.drone_position - self.prev_drone_position) *(1/self.dt)


            #self.vel_setpoint=(self.setpoint-self.drone_position)/5


            #print("time_ is ",self.dt)

            self.vel = Pose()
            self.vel.position.x = self.curr_v[0]
            self.vel.position.y = self.curr_v[1]
            self.vel.position.z = self.curr_v[2]



            #self.vel_pub.publish(self.vel)


            
            self.updateController()
            self.compute()

        else : 
            self.current_time = self.prev_time


        self.dtime = msg.header.stamp.sec

        

    def altitude_set_pid(self, alt):
        self.Kp[1] = alt.kp * 1.0 
        self.Ki[1] = alt.ki * 0.001
        self.Kd[1] = alt.kd * 1.0

    #Define callback function like altitide_set_pid to tune pitch, roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp * 1.0 
        self.Ki[0] = roll.ki * 0.001
        self.Kd[0] = roll.kd * 1.0
    def pitch_set_pid(self, pitch):

        self.Kp[2] = pitch.kp * 1.0 
        self.Ki[2] = pitch.ki * 0.001
        self.Kd[2] = pitch.kd * 1.0


    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.yaw_deg = math.degrees(yaw)
        self.drone_position[3] = self.yaw_deg	

    def updateController(self):
        #self.new_drone_position = self.drone_position
        self.error_prev=self.error
        self.error=(self.setpoint - self.drone_position) #* np.array([-1, 1, 1])
        self.errsum+=self.error * self.dt


        self.prev_vel_error = self.vel_error
        self.vel_error = (self.vel_setpoint - self.curr_v) * np.array([1, -1, -1,1])
        self.vel_errsum += self.vel_error*self.dt
        # print("update for this pid iteration")

    def compute(self, kp=-8, ki=0, kd=0):
        
        K_1 = np.array([self.Kp_1, self.Ki_1, self.Kd_1]).T

        error = self.error  # replace with actual value
        sum_error = (self.errsum)   # replace with actual value
        z = (self.error - self.error_prev) / self.dt 

        error_vector = (np.array([error, sum_error, z])).T
        print("setpoint",self.setpoint)

        print("error pos vector :", error_vector)


        rpt_pos = np.sum(K_1 * error_vector, axis=1) #from pos correction

        self.vel_setpoint=np.array([error[0]*2**(-0.1/error[0]**2),error[1]*2**(-0.1/error[1]**2),0.4*error[2]*2**(-1/(error[2]/1)**2)])



        vel_K = np.array([self.Kp, self.Ki, self.Kd]).T
        
        # print("K")
        # print(K)

        # Define the error, sum_error, and z
        vel_error = self.vel_error  # replace with actual value
        vel_sum_error = self.vel_errsum  # replace with actual value
        vel_z = (self.vel_error - self.prev_vel_error) / self.dt # replace with actual value

        #Vel error


        # Create the vector [error, sum_error, z]
        vel_error_vector = (np.array([vel_error, vel_sum_error, vel_z])).T
        #print("E")
        #print("error vector ===",vel_error_vector)

        # Perform the matrix multiplication
        # result = np.dot(K, error_vector)
        result = np.sum(vel_K * vel_error_vector, axis=1)

        # Add the constant vector [1500, 1500, 1500]
        constant_vector = np.array([1500, 1500, 1532])

        result_with_offset = result + constant_vector
        #print("resiltndjndjn",result_with_offset)

        # Set the roll, pitch, throttle values and apply limits

        rpt = np.clip(result_with_offset, 1000, 2000)
        # print("rpt",rpt)
        # print(rpt)

        msg = SwiftMsgs()

        msg.rc_roll =int(rpt[0]+rpt_pos[0])
        msg.rc_pitch =int(rpt[1]+rpt_pos[1])
        msg.rc_throttle =int(rpt[2]+rpt_pos[2]) 
        msg.rc_yaw = 1500

        # print("throttle :" ,msg.rc_throttle )
        #print(rpt)

        self.command_pub.publish(msg)


    def pid(self):

        #write your PID algorithm here. This time write equations for throttle, pitch, roll and yaw. 
        #Follow the steps from task 1b.


























 
        self.command_pub.publish(self.cmd)
        self.pid_error_pub.publish(self.pid_error)

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        self.setpoint=Pose()
        self.setpoint.position.x = goal_handle.request.waypoint.position.x
        self.setpoint.position.y = goal_handle.request.waypoint.position.y
        self.setpoint.position.z = goal_handle.request.waypoint.position.z
        self.get_logger().info(f'New Waypoint Set: {self.setpoint}')
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0
        self.duration = self.dtime

        #create a NavToWaypoint feedback object. Refer to Writing an action server and client (Python) in ROS 2 tutorials.
        
        #--------The script given below checks whether you are hovering at each of the waypoints(goals) for max of 3s---------#
        # This will help you to analyse the drone behaviour and help you to tune the PID better.
        feedback_msg = NavToWaypoint.Feedback()
        feedback_msg.current_waypoint.pose= self.setpoint
        while True:
            feedback_msg.current_waypoint.pose.position.x = self.drone_position[0]
            feedback_msg.current_waypoint.pose.position.y = self.drone_position[1]
            feedback_msg.current_waypoint.pose.position.z = self.drone_position[2]
            feedback_msg.current_waypoint.header.stamp.sec = self.max_time_inside_sphere

            goal_handle.publish_feedback(feedback_msg)

            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 0.4) #the value '0.4' is the error range in the whycon coordinates that will be used for grading. 
            #You can use greater values initially and then move towards the value '0.4'. This will help you to check whether your waypoint navigation is working properly. 

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        pass
            
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        self.point_in_sphere_start_time = self.dtime
                        self.get_logger().info('Drone in sphere for 1st time')                        #you can choose to comment this out to get a better look at other logs

            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                        self.get_logger().info('Drone in sphere')                                     #you can choose to comment this out to get a better look at other logs
                             
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.get_logger().info('Drone out of sphere')                                 #you can choose to comment this out to get a better look at other logs
                        self.point_in_sphere_start_time = None

            if self.time_inside_sphere > self.max_time_inside_sphere:
                 self.max_time_inside_sphere = self.time_inside_sphere

            if self.max_time_inside_sphere >= 3:
                 break
                        

        goal_handle.succeed()

        #create a NavToWaypoint result object. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        result = NavToWaypoint.Result()
        goal_handle.set_result(result)
        result.hov_time = self.dtime - self.duration #this is the total time taken by the drone in trying to stabilize at a point
        return result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
        return (
            (drone_pos[0] - sphere_center.request.waypoint.position.x) ** 2
            + (drone_pos[1] - sphere_center.request.waypoint.position.y) ** 2
            + (drone_pos[2] - sphere_center.request.waypoint.position.z) ** 2
        ) <= radius**2


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
         executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
         waypoint_server.destroy_node()
         rclpy.shutdown()


if __name__ == '__main__':
    main()

