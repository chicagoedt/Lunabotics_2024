#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ActCommand, Commands
from robot_interfaces.msg import MotorsVel, LinActPos
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy
import numpy as np
import time

class AutCommandNode(Node):

    def __init__(self):
        super().__init__("Automatic_Controller")

        # Establish publishers and listeners
        self.command_sub_ = self.create_subscription(Joy,'joy',self.get_command,10)
        self.publisher_vibr = self.create_publisher(Bool,'vibr',10) 
        self.publisher_success = self.create_publisher(Bool,'digging_commands_success',10) 
        self.subscriber_success_ = self.create_subscription(Bool,'PID_success',self.PID_success, 10)
        self.motor_publisher_ = self.create_publisher(MotorsVel,'motor_vel',10)
        self.lift_publisher_ = self.create_publisher(MotorsVel,'lift_vel',10)
        self.tilt_publisher_ = self.create_publisher(MotorsVel,'tilt_vel',10)
        self.in_progress_pub_ = self.create_publisher(Bool,'in_progress',10)
        self.command_pub_ = self.create_publisher(String,'aut_command',10)
        self.STOP_pub = self.create_publisher(Bool,'STOP',10)
        

        self.get_logger().info("Digging automatic algorithms node activated")
        self.timer_ = self.create_timer(0.1,self.main_algorithm)
        self.command = ''
        self.partial_done = False
        self.in_progress = False    # One of the Autonomous movements are currently in progress
        self.i = 0
        self.counter = 0
        self.command_received = 0
        self.started = False
        self.button = []
        self.axes = []
        self.vib_ON = False
        self.distances = []
        

    #target has teh shape [14_inch,4_inch,motors,vibrator,time=None]
    #target has teh shape [14_inch,4_inch,motors,vibrator,time=None]
    ## get_command
    #   Abstract:
    #       If an Autonomous movement is in progress,
    #       do not update self.command.
    ##
    def get_command(self,msg):
        self.button = msg.buttons
        self.axes = msg.axes
        if not(self.in_progress):
            if self.button[0] > 0:
                self.command = 'restpos'
            elif self.button[1] > 0:
                self.command = 'discharge'
            elif self.button[5] > 0:
                self.command = 'scoop'
            elif self.button[2] > 0:
                self.command = 'scoop2'
            
            if not(self.started):
                self.started = True
        if self.button[3] > 0:
            self.command = 'stop'

    def take_distance(self,msg):
        self.distances = msg.rl_lin_act

    def main_algorithm(self):

        done = Bool()
        inprogress = Bool()
        command = String()


        ## Select command to act
        #   self.command == "restpos" ?
        #       true ==> 

        if self.command == "stop":
            
            target_angles = [-10.0,-10.0,0.0,False]
            self.service_callback(target_angles)
            self.in_progress = True
            if self.partial_done:
                self.get_logger().info("Rest position achieved")
                done.data = True
                self.publisher_success.publish(done)
                self.partial_done = False
                self.in_progress = False
                self.command = ''
        
        elif self.command == "restpos":
            if not(self.in_progress):
                self.counter = 0
                linact = self.angle2inch([0.350,0.898])
                target = [linact[0],linact[1],0.0,False]
                self.get_logger().info(str(target))
                self.service_callback(target)
                self.in_progress = True
            if self.partial_done:
                self.get_logger().info("Rest position achieved")
                done.data = True
                self.publisher_success.publish(done)
                self.partial_done = False
                self.in_progress = False
                self.command = ''

        elif self.command == "discharge":
            if not(self.in_progress):
                self.counter = 0
                linact = self.angle2inch([1.325,0.898])
                target = [linact[0],linact[1],0.0,False]
                self.service_callback(target)
                self.in_progress = True
            if self.partial_done:
                vibr = Bool()
                vibr.data = True
                self.publisher_vibr.publish(vibr)
                time.sleep(10)
                vibr.data = False
                self.publisher_vibr.publish(vibr)
                self.get_logger().info("Digging performed")
                done.data = True
                self.publisher_success.publish(done)
                self.in_progress = False
                self.partial_done = False
                self.command = 'restpos'
        
        elif self.command == "scoop":
            



            ## Array: target_angles
            #   [0] - lift.rl_motor target POS 
            #   [1] - tilt.rl_motor target POS
            #   [2] - motor.rl_motor target POS
            #   [3] - vibr.data
            #   [4] -
            ##
            target_angles = [ [ 0.629,  -0.490,     0.0,   False,   0.0],
                              [   0.0,  -0.350,     0.0,    True,   0.0],
                              [  -0.5,   -10.0,    -0.5,    True,   0.0],
                              [ 0.100,   0.200,     0.0,    True,   0.0],
                              [ 0.350,   0.898,     0.0,   False,   0.0] ]


            if self.i == 0 or self.partial_done == True:
                self.counter = 0
                #if self.i == 2:
                   #time.sleep(5)
                if self.i>= np.shape(target_angles)[0]:
                    self.i = 0
                    self.get_logger().info("Scooping performed")
                    done.data = True
                    self.publisher_success.publish(done)
                    self.in_progress = False
                    self.partial_done = False
                    self.command = ''
                else:
                    linact = self.angle2inch(target_angles[self.i][:2])
                    target = [linact[0],linact[1],target_angles[self.i][2],target_angles[self.i][3],target_angles[self.i][4]]
                    self.service_callback(target)
                    self.partial_done = False
                    self.i += 1
                    self.in_progress = True

        elif self.command == "scoop2":
            



            ## Array: target_angles
            #   [0] - lift.rl_motor target POS 
            #   [1] - tilt.rl_motor target POS
            #   [2] - motor.rl_motor target POS
            #   [3] - vibr.data
            #   [4] -
            ##
            target_angles = [ [ 0.300,  -0.150,     0.0,   False,   0.0],
                              [   0.0,   -10.0,     0.0,    True,   0.0],
                              [  -0.3,   -10.0,    -0.5,    True,   0.0],
                              [ 0.150,   0.898,     0.0,    True,   0.0],
                              [ -10.0,   -10.0,     0.0,   False,   0.0] ]


            if self.i == 0 or self.partial_done == True:
                self.counter = 0
                #if self.i == 2:
                   #time.sleep(5)
                if self.i>= np.shape(target_angles)[0]:
                    self.i = 0
                    self.get_logger().info("Scooping performed")
                    done.data = True
                    self.publisher_success.publish(done)
                    self.in_progress = False
                    self.partial_done = False
                    self.command = ''
                else:
                    linact = self.angle2inch(target_angles[self.i][:2])
                    target = [linact[0],linact[1],target_angles[self.i][2],target_angles[self.i][3],target_angles[self.i][4]]
                    self.service_callback(target)
                    self.partial_done = False
                    self.i += 1
                    self.in_progress = True

        elif not(self.in_progress) and self.started:
            #motor_client_ = self.create_client(ActCommand,'des_motor_vel')
            #request_motor = ActCommand.Request()
            request_motor = MotorsVel()
            request_lift = MotorsVel()
            request_tilt = MotorsVel()

            #while not motor_client_.wait_for_service(1.0):
            #    self.get_logger().warn("Waiting for motor PID node...")

            mot_target = [-self.axes[4]-0.5*self.axes[3],self.axes[4]-0.5*self.axes[3]]
            if abs(mot_target[0])>1:
                mot_target[0] = mot_target[0]/abs(mot_target[0])
            if abs(mot_target[1])>1:
                mot_target[1] = mot_target[1]/abs(mot_target[1])
            if abs(mot_target[0])<0.1:
                mot_target[0] = 0.0
            if abs(mot_target[1])<0.1:
                mot_target[1] = 0.0
            request_motor.rl_motor = mot_target
            #future_motor = motor_client_.call_async(request_motor) 
            #future_motor.add_done_callback(self.response_mot_callback)
            self.motor_publisher_.publish(request_motor)

            lift_target = [0.4*self.axes[0],0.4*self.axes[0]]
            request_lift.rl_motor = lift_target
            self.lift_publisher_.publish(request_lift)

            tilt_target = [self.axes[1]*2.0,self.axes[1]*2.0]
            request_tilt.rl_motor = tilt_target
            self.tilt_publisher_.publish(request_tilt)

            if self.button[7] == 1:
                vibr = Bool()
                vibr.data = not self.vib_ON
                self.publisher_vibr.publish(vibr)
                self.vib_ON = not self.vib_ON

            
        inprogress.data = self.in_progress
        command.data = self.command
        self.in_progress_pub_.publish(inprogress)
        self.command_pub_.publish(command)

    def angle2inch(self,angles):

        act_14 = np.sqrt(686.6443-453.2148*np.cos(angles[0]+1.3415))-19.5098
        act_4 = np.sqrt((3.1811*np.cos(angles[1])-1.9201)**2+(11.0201+3.1811*np.sin(angles[1]))**2)-9.5102
        if angles[1] == -10.0:
            act_4 = -10.0
        if angles[0] == -10.0:
            act_14 = -10.0
        return [act_14,act_4]

        
    def service_callback(self,target):
        
        liftact_client_ = self.create_client(ActCommand,'des_lift_pos')
        tiltact_client_ = self.create_client(ActCommand,'des_tilt_pos')
        motor_client_ = self.create_client(ActCommand,'des_motor_vel')
        vibr = Bool()

        while not liftact_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for lift act PID node...")
        
        while not tiltact_client_.wait_for_service(1.0):
           self.get_logger().warn("Waiting for tilt act PID node...")
        
        while not motor_client_.wait_for_service(1.0):
           self.get_logger().warn("Waiting for motor PID node...")

        if len(target) == 5:
            time = target[4]
        else:
            time = 0.0

        

        ##### Propose Sorting Items like this #####
        ## Prepare Request for Server 'des_lift_pos'
        # request_lift = ActCommand.Request()
        # request_lift.rl_motor = [target[0],time]
        
        ## Prepare Request for Server 'des_tilt_pos'
        # request_tilt = ActCommand.Request()
        # request_tilt.rl_motor = [target[1],time]

        ## Prepare Request for Server 'des_motor_vel'
        # request_motor = ActCommand.Request()
        # request_motor.rl_motor = [target[2],target[2]]

        # vibr.data = target[3]

        request_lift = ActCommand.Request()
        request_tilt = ActCommand.Request()
        request_motor = ActCommand.Request()
        request_lift.rl_motor = [target[0],time]
        request_tilt.rl_motor = [target[1],time]
        request_motor.rl_motor = [target[2],-target[2]]
        vibr.data = target[3]



        ## Publish Future State Received from Server
        #       Makes Requests and Assigns results to
        #       -- future_lift
        #       -- future_tilt
        #       -- future_motor
        #       Publishes Future State of Vibr
        future_lift = liftact_client_.call_async(request_lift)
        future_tilt = tiltact_client_.call_async(request_tilt)
        future_motor = motor_client_.call_async(request_motor) 
        self.publisher_vibr.publish(vibr)

        future_lift.add_done_callback(self.response_callback)
        future_tilt.add_done_callback(self.response_callback)
        future_motor.add_done_callback(self.response_callback)
    









    def response_callback(self, future):
        try:
            response = future.result()
            #self.get_logger().info(f'Response: {response.done}')
            if response.done:
                self.command_received += 1
            if self.command_received == 3:
                self.command_received = 0
                self.get_logger().info("Commands received")
            return response.done
        except Exception as e:
            self.get_logger().error(f'Exception in service call: {e}')
    
    def response_mot_callback(self, future):
        try:
            response = future.result()
            if not(response.done):
                self.get_logger().error("Command not performed")
            return response.done
        except Exception as e:
            self.get_logger().error(f'Exception in service call: {e}')

    def PID_success(self,msg):

        if msg.data:
            self.counter +=1

        if self.counter == 3:
            self.counter = 0
            self.partial_done = True


def main(args=None):
    rclpy.init(args=args)
    node = AutCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()
#!/usr/bin/env python3

import rclpy
