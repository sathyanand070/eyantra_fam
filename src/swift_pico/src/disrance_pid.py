#!/usr/bin/env python3

"""
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

        PUBLICATIONS            SUBSCRIPTIONS
        /drone_command            /whycon/poses
        /pid_error            /throttle_pid
                        /pitch_pid
                        /roll_pid
                    
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.    
"""

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
import threading


class Swift_Pico(Node):
    def __init__(self):
        super().__init__(
            "pico_controller"
        )  # initializing ros node with name pico_controller

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = np.array([0.0, 0.0, 0.0])

        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = np.array(
            [2, 2, 19]
        )  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        self.dt = 0.01

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters

        # self.Kp =[0,0,0]
        # self.Ki =[0,0,0]
        # self.Kd =[0,0,0]
        self.Kp = [3.5, 3.5, 28]
        self.Ki = [0, 0, 0]
        self.Kd = [3.1, 3.1, 9.1]
        # self.Kp =[2.5, 2.5, 32] Best
        # self.Ki =[0, 0, 0]
        # self.Kd =[2.5, 2.5, 7]

        # -----------------------Add other required variables for pid here ----------------------------------------------

        self.error = np.array([0.0, 0.0, 0.0])
        self.error_prev = np.array([0.0, 0.0, 0.0])
        self.errsum = np.array([0.0, 0.0, 0.0])
        self.current_rpt = [self.cmd.rc_roll, self.cmd.rc_pitch, self.cmd.rc_throttle]

        # Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]        #         Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        #                                                     self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        #                                                                     You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit.

        """self.sample_time = 0.05  # in seconds"""

        # Publishing /drone_command, /pid_error

        self.pid_error_pub = self.create_publisher(PIDError, "/pid_error", 10)
        self.command_pub = self.create_publisher(SwiftMsgs, "/drone_command", 10)

        # ------------------------Add other ROS 2 Publishers here-----------------------------------------------------

        # Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PoseArray, "/whycon/poses", self.whycon_callback, 1)

        # ------------------------Add other ROS Subscribers here-----------------------------------------------------

        self.arm()  # ARMING THE DRONE
        # self.pid()
        # t1 = threading.Thread(target=self.subfun, args=(10,))
        # t2 = threading.Thread(target=self.worker, args=(10,))

        # t1.start()
        # t2.start()

    def worker(self):
        # print("working")

        
            self.compute()
            self.updateController()

    # def subfun(self) :
    #     self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)

    # self.disarm()

    # Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Pytho

    # def pid(self,drone_position,setpoint) :
    #     prevtime = currtime = time.time()

    #     # Define Model and Controller

    #     while not(abs(drone_position[0] - self.setpoint[0]) <= 0.4) or not(abs(drone_position[1] - self.setpoint[1]) <= 0.4) or not(abs(drone_position[2] - self.setpoint[2]) <= 0.4):
    #         prevtime = currtime
    #         currtime = time.time()
    #         while currtime - prevtime <= 0.01:
    #             currtime = time.time()
    #             pass
    #         self.model.dt=currtime-prevtime

    #         self.compute(kp=-5,ki=2,kd=1)
    #         self.updateController()

    def updateController(self):
        self.error_prev = self.error
        self.error = (self.setpoint - self.drone_position) * np.array([1, -1, -1])
        if abs(self.error[1]) <0.5:
            self.errsum[1] += self.error[1]
        if abs(self.error[0]) <0.5:
            self.errsum[0] += self.error[0]
        if abs(self.error[2]) < 1:  # and abs(self.error[1])<=2 :
            self.errsum[2] += (self.error[2])
        # print("update for this pid iteration")

    def compute(self, kp=-8, ki=0, kd=0):
    



        K = np.array([self.Kp, self.Ki, self.Kd]).T


        # Define the error, sum_error, and z
        error = self.error  # replace with actual value
        sum_error = (self.errsum) * self.dt  # replace with actual value
        z = (self.error - self.error_prev) / self.dt # replace with actual value

        # Create the vector [error, sum_error, z]
        error_vector = (np.array([error, sum_error, z])).T
        # print("E")
        # print(error_vector)

        # Perform the matrix multiplication
        # result = np.dot(K, error_vector)
        result = np.sum(K * error_vector, axis=1)

        # Add the constant vector [1500, 1500, 1500]
        constant_vector = np.array([1500, 1500, 1532])

        result_with_offset = result + constant_vector

        # Set the roll, pitch, throttle values and apply limits

        rpt = np.clip(result_with_offset, 1000, 5000)
        # print(rpt)

        msg = SwiftMsgs()
        msg.rc_roll = np.int(rpt[0])
        msg.rc_pitch = np.int(rpt[1])
        msg.rc_throttle = np.int(rpt[2])  # +1/500*(rpt[0]+rpt[1]-3000))

        # print("throttle :" ,msg.rc_throttle )

        self.command_pub.publish(msg)

    def disarm(self):
        # print("byr")
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)
        
        self.worker()

    def arm(self):
        # self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)
        # print("published")  # Publishing /drone_command

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        # print("drone pos :", self.drone_position)
        self.worker()

        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /throttle_pid
    # This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
    def altitude_set_pid(self, alt):
        # print("Setting K")
        self.Kp[2] = (
            alt.kp
        )  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = alt.ki
        self.Kd[2] = alt.kd
        self.Kp[0] = (
            alt.kp / 2
        )  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = alt.ki / 2
        self.Kd[0] = alt.kd / 2
        self.Kp[1] = (
            alt.kp / 2
        )  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = alt.ki / 2
        self.Kd[1] = alt.kd / 2

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # prevtime = currtime = time.time()

        # # Define Model and Controller

        # while (
        #     not (abs(self.drone_position[0] - 2) <= 0.2)
        #     or not (abs(self.drone_position[1] - 2) <= 0.2)
        #     or not (abs(self.drone_position[2] - 19) <= 0.01)
        # ):
        #     prevtime = currtime
        #     currtime = time.time()
        #     while currtime - prevtime <= 2:
        #         currtime = time.time()
        #         pass

        #     self.compute()
        #     self.updateController()

        # self.disarm()

        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        #     1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
        #     2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #     3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #     4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #     5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #     6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
        #                                                                                                                         self.cmd.rcPitch = self.max_values[1]
        #     7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #     8. Add error_sum

        # ------------------------------------------------------------------------------------------------------------------------
        self.command_pub.publish(self.cmd)
        # calculate throttle error, pitch error and roll error, then publish it accordingly
        # self.pid_error_pub.publish(self.pid_error)
        self.pid_error_pub.publish(self.error)


def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()