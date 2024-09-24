#!/usr/bin/env python3

#last update: in v7
#the auto controller node will take buttons values from the Joy Node and take signals from the Lidar node, 
#it will return the commands to the PID nodes and control the robot autonomous movements 

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ActCommand, Commands
from robot_interfaces.msg import MotorsVel, LinActPos , GuiGraphInput
from std_msgs.msg import Bool, String, Float32
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
        self.motor_publisher_ = self.create_publisher(MotorsVel,'motor_vel',10)
        self.lift_publisher_ = self.create_publisher(MotorsVel,'lift_vel',10)
        self.tilt_publisher_ = self.create_publisher(MotorsVel,'tilt_vel',10)
        self.subscriber_success_ = self.create_subscription(Bool,'PID_success',self.PID_success, 10)
        self.subscriber_depth = self.create_subscription(LinActPos,"compressed_pub/grid", self.max_min_distance, 10)
        self.GUI_pub_ = self.create_publisher(GuiGraphInput,'GUI_values',10)
        self.angles_sub = self.create_subscription(Float32,'imu',self.get_angle,10)
        self.camera_switching = self.create_publisher(Bool,'camera_active',10)

        self.get_logger().info("Digging automatic algorithms node activated")
        self.timer_ = self.create_timer(0.1,self.main_algorithm)
        self.command = ''
        self.partial_done = False   # One of the Autonomous movements phase is finished
        self.in_progress = False    # One of the Autonomous movements are currently in progress
        self.i = 0                  # Indication of the phase of the low-level autonomous movement
        self.counter = 0            # Counter for the number of reached positions
        self.command_received = 0   # Counter if the command is received
        self.started = False        # The node is started
        self.button = []            # Buttons from the Joystick
        self.axes = []              # Axes from the Joystick
        self.vib_ON = False         # variable for vibrator value 
        self.auto = False           # Automatic mode activated
        self.phase = 0              # Phase number of teh high-level autonomous movement
        self.wall_distance = 0      # Distance from the wall
        self.forward_complete = False
        self.turn_complete = False
        self.auto = False
        self.forward_in_progress = False
        self.turn_in_progress = False
        self.current_angle = 0.0
        self.min_distance = 0
        self.active = True
        self.pressed = False
        self.obstacle = False
        self.initial_angle = 0.0
    #target has the shape [14_inch,4_inch,motors,vibrator,time=None]
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
            elif self.button[4] > 0:
                self.command = 'autodump'
            elif self.button[6] > 0:
                self.command = 'autotravel'
            elif self.button[7] > 0:
                if not self.auto:
                    self.command = 'one-cycle'
                    self.auto = True
            if not(self.started):
                self.started = True
        if self.button[10] > 0:
            if not self.pressed :
                switch = Bool()
                self.active = not self.active
                self.pressed = True
                switch.data = self.active
                self.camera_switching.publish(switch)
        if self.button[10] == 0:
            self.pressed = False
        if self.button[9] > 0:
            if not self.pressed:
                switch = Bool()
                self.pressed = True
                self.initial_angle = self.current_angle
        if self.button[9] == 0:
            self.pressed = False
        if self.button[3] > 0:
            self.command = 'stop'

    def max_min_distance(self,msg):
        if msg.rl_lin_act[1] != 0:
            self.wall_distance = msg.rl_lin_act[1]
        if msg.rl_lin_act[0] != 0:
            self.min_distance = msg.rl_lin_act[0]
        #self.get_logger().info('max' + str(self.wall_distance) + 'min' + str(self.min_distance))
        self.obstacle = bool(msg.rl_lin_act[2])
    
    def get_angle(self,msg):
        self.current_angle = msg.data
        #self.get_logger().info('angles: ' + str(self.current_angle))
    def main_algorithm(self):

        done = Bool()
        GUI_values = GuiGraphInput()


        ## Select command to act
        #   self.command == "restpos" ?
        #       true ==> 

        if self.command == "stop":
           
            self.counter = 0
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
                self.i = 0
                self.auto = False

        elif self.command == "restpos":
            if not(self.in_progress):
                self.counter = 0
                self.i = 0
                linact = self.angle2inch([0.400,0.898])
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
                self.i = 0
                linact = self.angle2inch([1.325,0.898])
                target = [linact[0],linact[1],0.0,False]
                self.service_callback(target)
                self.in_progress = True
            if self.partial_done:
                vibr = Bool()
                vibr.data = True
                self.publisher_vibr.publish(vibr)
                time.sleep(7)
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
            target_angles = [ [   0.3,  -0.150,     0.0,   False,   0.0],
                              [   0.0,   -10.0,     0.0,    True,   0.0],
                              [  -0.3,   -10.0,   -0.75,    True,   0.0],
                              [ 0.250,   0.898,     0.0,    True,   0.0],
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
                    self.command = 'stop'
                    if self.auto:
                        self.command = 'one-cycle'
                else:
                    linact = self.angle2inch(target_angles[self.i][:2])
                    target = [linact[0],linact[1],target_angles[self.i][2],target_angles[self.i][3],target_angles[self.i][4]]
                    self.service_callback(target)
                    self.partial_done = False
                    self.i += 1
                    self.in_progress = True

        elif self.command == 'autodump':
            target_pos = 800
            self.forward(target_pos)
            if self.forward_complete :
                self.forward_complete = False
                self.command = 'discharge'
                self.partial_done = False
                self.i = 0
        
        elif self.command =='autotravel':
            self.get_logger().info("\033[31mIn AutoTravel\033[0m")
            #the main idea should be moving forward and right and try to avoid obstacles if possible, at the end
            #make a right turn, if we see an obstacle try to turn a bit right but always keeping the same distance
            target_pos = 500
            #initialize the autotravel
            self.get_logger().warn(f"autotravel -- i {self.i}")

            if self.i == 0:
                self.get_logger().info("\033[32mIn AutoTravel-->first If\033[0m")
                self.forward(target_pos)
                
                if self.forward_complete:
                    self.get_logger().warn("self.forward is complete, in conditional")
                    self.i = 1
            
            if self.i == 1:
                self.get_logger().info("\033[33mIn AutoTravel---->second If\033[0m")
                self.forward_complete = False
                self.turn(1.57+ self.initial_angle)
                if self.turn_complete:
                    self.i = 2

            if self.i == 2:
                self.get_logger().info("\033[34mIn AutoTravel-------->third If\033[0m")
                self.turn_complete = False 
                self.forward(2000)

                if self.forward_complete:
                    self.i = 3
                    self.forward_complete = False

                    if self.auto:
                        self.command = 'one-cycle'
                        self.phase = 1

        elif self.command == 'one-cycle':

            if self.phase == 0:
                self.command = 'autotravel'
                self.i = 0
            #check max_distance

            if self.phase == 1:
                self.i = 0
                if self.min_distance < 1000 and self.wall_distance < 1000: #maybe minimum
                    self.turn()

                else:
                    self.shut_turn()
                    self.command = 'scoop2'
                    self.phase = 2

            if self.phase == 2:
                self.i = 0
                self.turn(3.14 + self.initial_angle)

                if self.turn_complete:
                    self.phase = 3
            if self.phase == 3:
                self.turn_complete = False
                self.command = 'autodump'
            pass

        elif not(self.in_progress) and self.started:
            request_motor = MotorsVel()
            request_lift = MotorsVel()
            request_tilt = MotorsVel()

            mot_target = [-self.axes[4]-self.axes[3],self.axes[4]-self.axes[3]]
            if abs(mot_target[0])>1:
                mot_target[0] = mot_target[0]/abs(mot_target[0])
            if abs(mot_target[1])>1:
                mot_target[1] = mot_target[1]/abs(mot_target[1])
            if abs(mot_target[0])<0.1:
                mot_target[0] = 0.0
            if abs(mot_target[1])<0.1:
                mot_target[1] = 0.0
            request_motor.rl_motor = mot_target
            self.motor_publisher_.publish(request_motor)

            lift_target = [0.4*self.axes[0],0.4*self.axes[0]]
            request_lift.rl_motor = lift_target
            self.lift_publisher_.publish(request_lift)

            tilt_target = [self.axes[1]*2.0,self.axes[1]*2.0]
            request_tilt.rl_motor = tilt_target
            self.tilt_publisher_.publish(request_tilt)

            if self.button[8] == 1:
                vibr = Bool()
                vibr.data = not self.vib_ON
                self.publisher_vibr.publish(vibr)
                self.vib_ON = not self.vib_ON

        
        msg = GuiGraphInput()
        msg.in_progress = self.in_progress
        msg.current_command = self.command
        msg.part_number = self.i
        msg.phase_number = self.phase
        msg.angle_min_max = [float(self.current_angle), float(self.min_distance) , float(self.wall_distance)]
        self.GUI_pub_.publish(msg)

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

    def forward(self,distance):
        request_motor = MotorsVel()
        if not self.forward_in_progress :
            self.initial_distance = self.wall_distance
            self.initial_angle = self.current_angle
            self.forward_in_progress = True
            self.get_logger().info(f"a")

        max_turn_angle = 5/180*3.14
        current_angle = self.current_angle
        current_distance = self.wall_distance
        angles_difference = current_angle - self.initial_angle
        distance_difference = current_distance - self.initial_distance

        if abs(angles_difference) > max_turn_angle:
            self.get_logger().info('b') 
            self.turn(self.initial_angle)

        else:
            self.get_logger().info('c') 
            self.turn_complete = False
            self.turn_in_progress = False

            if current_distance > distance:
                request_motor.rl_motor = [-0.4,0.4]

            else:
                request_motor.rl_motor = [0.0,0.0]
                self.forward_in_progress = False
                self.forward_complete = True

            self.motor_publisher_.publish(request_motor)  


    #turn will turn right if the angle is positive while will turn left if the angle is negative
    def turn(self,angle = None):
        request_motor = MotorsVel()
        toll = 3.14/90
        if not self.turn_in_progress :
            self.turn_in_progress = True
        
        if angle != None:
           current_angle = self.current_angle
           angles_difference = - current_angle + angle
           if abs(angles_difference)> toll:
               request_motor.rl_motor =[ np.sign(angles_difference)*0.5,np.sign(angles_difference)*0.5]
               self.motor_publisher_.publish(request_motor)    
           else :
               self.turn_in_progress = False
               self.turn_complete = True
        else:
           request_motor.rl_motor = [0.4,0.4]

    def stop(self):
        if self.turn_in_progress == True:
            self.turn_in_progress = False
        request_motor = MotorsVel()
        request_motor.rl_motor = [0.0,0.0]
        self.motor_publisher_.publish(request_motor)

def main(args=None):
    rclpy.init(args=args)
    node = AutCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()

