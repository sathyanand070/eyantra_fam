#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

        PUBLICATIONS            SUBSCRIPTIONS
        /drone_command            /whycon/poses
        /pid_error            /throttle_pid
                        /pitch_pid
                        /roll_pid
                    
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.    
'''

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node
import numpy as np
import time 


class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # initializing ros node with name pico_controller

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.dt = 0 

        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = np.array([2, 2, 19])  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        
        #self.vel.poses[0].position.x

        #initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        #after tuning and computing corresponding PID parameters, change the parameters

        self.Kp_1 = [100, 150, 7]
        self.Ki_1 = [0, 0, 0]
        self.Kd_1 = [0, 0, 0]
        #-----------------------Add other required variables for pid here ----------------------------------------------

        self.error = np.array([0.0, 0.0, 0.0])
        self.error_prev = np.array([0.0, 0.0, 0.0])
        self.errsum = np.array([0.0, 0.0, 0.0])
        self.current_rpt = [self.cmd.rc_roll, self.cmd.rc_pitch, self.cmd.rc_throttle]

        self.curr_v = np.array([0,0,0])
        self.prev_v = np.array([0,0,0])

        self.vel_error=np.array([0.0, 0.0, 0.0])
        self.prev_vel_error=np.array([0.0, 0.0, 0.0])
        self.vel_errsum=np.array([0.0, 0.0, 0.0])

        self.vel_setpoint=np.array([0.0, 0.0, 0.0])

        self.setpoint = np.array(
            [2, 2, 19]
        ) 

        self.Kp = [1, 1, 3]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 2]

        
        



        # Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]        #         Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        #                                                    self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        #                                                                    You can change the upper limit and lower limit accordingly. 
        #-----------------------
        #-----------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit.
    
        self.sample_time = 0.060  # in seconds

        # Publishing /drone_command, /pid_error
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        #self.vel_pub = self.create_publisher(Float32MultiArray, '/velocities', 10)
        self.vel_pub = self.create_publisher(Pose, '/velocities', 10)

        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.current_time = time.time()
        self.prev_time = time.time()

        #------------------------Add other ROS 2 Publishers here-----------------------------------------------------
    

        # Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
        
        # self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        # self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        # self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        # self.command_pub_vel = self.create_publisher(PoseArray, '/whycon/poses', 10)


  
            #self.vel = (self.drone_position - self.prev_drone_position)
         
        #--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------





        #------------------------Add other ROS Subscribers here-----------------------------------------------------
    

        self.arm()  # ARMING THE DRONE

        # Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)

    def updateController(self):
        self.error_prev=self.error
        self.error=(self.setpoint - self.drone_position) #* np.array([-1, 1, 1])
        self.errsum+=self.error * self.dt


        self.prev_vel_error = self.vel_error
        self.vel_error = (self.vel_setpoint - self.curr_v) * np.array([1, -1, -1])
        self.vel_errsum += self.vel_error*self.dt
        # print("update for this pid iteration")
    def compute(self, kp=-8, ki=0, kd=0):
        
        K_1 = np.array([self.Kp_1, self.Ki_1, self.Kd_1]).T

        error = self.error  # replace with actual value
        sum_error = (self.errsum)   # replace with actual value
        z = (self.error - self.error_prev) / self.dt 

        error_vector = (np.array([error, sum_error, z])).T

        print("error pos vector :", error_vector)

        self.vel_setpoint = np.sum(K_1 * error_vector, axis=1)


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
        msg.rc_roll =np.int(rpt[0])
        msg.rc_pitch =np.int(rpt[1])
        msg.rc_throttle =np.int(rpt[2]) 
        # print("throttle :" ,msg.rc_throttle )
        #print(rpt)

        self.command_pub.publish(msg)

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
        self.command_pub.publish(self.cmd)  # Publishing /drone_command


    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses 
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



            self.vel_pub.publish(self.vel)


            
            self.updateController()
            self.compute()

        else : 
            self.current_time = self.prev_time

        

        #--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------



    
        #---------------------------------------------------------------------------------------------------------------


    # Callback function for /throttle_pid
    # This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6

    def roll_set_pid(self, rll):
        self.Kp[2] = rll.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = rll.ki * 0.008
        self.Kd[2] = rll.kd * 0.6
    
    def pitch_set_pid(self, pch):
        self.Kp[2] = pch.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = pch.ki * 0.008
        self.Kd[2] = pch.kd * 0.6

    #----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

    #----------------------------------------------------------------------------------------------------------------------


    def pid(self):
    #-----------------------------Write the PID algorithm here--------------------------------------------------------------

    # Steps:
    #     1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
    #    2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
    #    3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
    #    4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
    #    5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
    #    6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
    #                                                                                                                        self.cmd.rcPitch = self.max_values[1]
    #    7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
    #    8. Add error_sum












    #------------------------------------------------------------------------------------------------------------------------
        self.command_pub.publish(self.cmd)
        # calculate throttle error, pitch error and roll error, then publish it accordingly
        self.pid_error_pub.publish(self.pid_error)



def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
