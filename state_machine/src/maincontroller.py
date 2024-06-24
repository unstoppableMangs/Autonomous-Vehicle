#! /usr/bin/env python
import rospy, time
from std_msgs.msg import Float32, Bool
from state_machine.msg import Depth
from state_machine.msg import Friction 
import pololu_maestro_ros.srv

servo_left = rospy.get_param('/master_state_machine/servo_left', 5200)
servo_center = rospy.get_param('/master_state_machine/servo_center', 6400)
servo_right = rospy.get_param('/master_state_machine/servo_right', 7500)

motor_stop = rospy.get_param('/master_state_machine/motor_stop', 6200)
motor_slow = rospy.get_param('/master_state_machine/motor_slow', 6390) #6400
motor_fast = rospy.get_param('/master_state_machine/motor_fast', 6500) #6560
motor_reverse = rospy.get_param('/master_state_machine/motor_reverse', 5700)

turn_depth = rospy.get_param('/master_state_machine/turn_depth', 1280) 
stop_depth = rospy.get_param('/master_state_machine/stop_depth', 800)
slow_down_depth = rospy.get_param('/master_state_machine/slow_down_depth', 1450)
enable_friction=1

class MainController(object):
    def __init__(self):
        self.sub_depth = rospy.Subscriber('/camera/depth', Depth, callback=self.sub_depth_callback)
        self.sub_pid = rospy.Subscriber('/pid_output', Float32, callback=self.sub_pid_callback)
        self.sub_stop_sign = rospy.Subscriber('/stop_sign_found', Bool, callback=self.sub_stop_sign_callback)
        if enable_friction==1:
            self.pub_state =  rospy.Publisher('/friction',Friction,queue_size=10)
            self.car_state = 0 #0:Stop; 1:Turning; 2:Moving
            self.motor_state = 0 #0:Stop; 1:RunningSlow; 2:RunningFast
        self.pololu = rospy.ServiceProxy('set_servo', pololu_maestro_ros.srv.set_servo)
        self.center_depth_arr, self.left_depth_arr, self.right_depth_arr = [2000] * 5, [2000] * 5, [2000] * 5
        self.center_depth, self.left_depth, self.right_depth = 2000, 2000, 2000
        self.pid_value= 0
        self.turning = False
        self.stop_sign_found = False
        self.turn_count=0
        self.corner=0
        self.cooldown_value=20
        self.cooldown=0

    def sub_depth_callback(self, data):
        #Calculate rolling averages
        self.center_depth = self.rolling_avg(self.center_depth_arr, data.center_depth)
        self.left_depth  = self.rolling_avg(self.left_depth_arr, data.left_depth)
        self.right_depth = self.rolling_avg(self.right_depth_arr, data.right_depth)
    
    def rolling_avg(self, arr, measurement):
        arr.append(measurement)
        arr.pop(0)
        return int(sum(arr)/len(arr))

    def sub_pid_callback(self, data):
        self.pid_value = data.data
        if self.pid_value > 1:
            self.pid_value = 1
        elif self.pid_value < -1:
            self.pid_value = -1

    def set_motors(self, servo_pos, motor_pos):
        try:
            self.pololu(0, int(servo_pos))
            self.pololu(1, int(motor_pos))
        except:
            rospy.loginfo("POLULU NOT FOUND")

    def kill_motors(self):
        try:
            self.pololu(0, int(servo_center))
            self.pololu(1, int(motor_stop))
        except:
            rospy.loginfo("POLULU NOT FOUND")

    # To utilize the stop sign detection launch stop_sign.launch instead to run the detection node
    def sub_stop_sign_callback(self, data):
        self.stop_sign_found = data.data
        
    def servo_value(self):
        range = abs(servo_right - servo_left)
        return (self.pid_value * range/2) + servo_center # extremes at 5600 and 7200

    def determine_state(self):
        if self.center_depth < stop_depth or self.stop_sign_found:
            rospy.loginfo("STOPPING - %d - L: %d - C: %d - R: %d", motor_stop, self.left_depth, self.center_depth, self.right_depth)
            if enable_friction==1:
                self.car_state=0
                self.motor_state=0
            self.set_motors(self.servo_value(), motor_stop)
            if self.stop_sign_found:
                rospy.loginfo("STOP SIGN FOUND")
        elif self.center_depth > turn_depth and not self.turning:
            if self.center_depth < slow_down_depth:
                rospy.loginfo("MOVING STRAIGHT, SLOWING DOWN - %d - L: %d - C: %d - R: %d", motor_slow, self.left_depth, self.center_depth, self.right_depth)
                if enable_friction==1:
                    self.car_state=2
                    self.motor_state=1
                self.set_motors(self.servo_value(), motor_slow)
            else:
                rospy.loginfo("MOVING STRAIGHT, SPEEDING UP - %d - L: %d - C: %d - R: %d", motor_fast, self.left_depth, self.center_depth, self.right_depth)
                if enable_friction==1:
                    self.car_state=2
                    self.motor_state=2
                self.set_motors(self.servo_value(), motor_fast)
        elif self.turning and (self.center_depth > turn_depth or self.right_depth<920):
            self.set_motors(servo_left, motor_slow) #correction
            time.sleep(.28)
            rospy.loginfo('FINISHING TURNING: L: %d - C: %d - R: %d', self.left_depth, self.center_depth, self.right_depth)
            if enable_friction==1:
                self.car_state=2
                self.motor_state=1
            if self.turn_count>6:
                self.cooldown=self.cooldown_value
                self.corner+=1
                rospy.loginfo('Corner[%d] DONE:',self.corner)
                self.turn_count=0
            self.turning = False
        elif self.center_depth < turn_depth and self.right_depth>900 and self.cooldown<=0:
            rospy.loginfo("TURNING RIGHT -  L: %d - C: %d - R: %d cooldown: %d", self.left_depth, self.center_depth, self.right_depth, self.cooldown)
            if enable_friction==1:
                self.car_state=1
                self.motor_state=1
            self.turn_count+=1
            self.set_motors(servo_right, motor_slow)
            self.turning = True
        else:
            rospy.loginfo("UNSURE, MOVING SLOW - %d - L: %d - C: %d - R: %d", motor_slow, self.left_depth, self.center_depth, self.right_depth)
            self.set_motors(self.servo_value(), motor_slow)
        rospy.loginfo("Cooldown[%d] corner[%d]",self.cooldown,self.corner)
        if enable_friction==1:
            my_state=Friction()
            my_state.car_state=self.car_state
            my_state.motor_state=self.motor_state
            self.pub_state.publish(my_state)
        self.cooldown-=1

if __name__ =='__main__':
    rospy.wait_for_service('set_servo')
    rospy.init_node('main_controller')
    controller = MainController()
    rate = rospy.Rate(10) # 5hz # will increasing the frequency work??
    controller.set_motors(servo_center, motor_stop)
    rospy.on_shutdown(controller.kill_motors)
    time.sleep(5)
    try:
        while not rospy.is_shutdown():
            controller.determine_state()
            rate.sleep()
    except rospy.ROSInterruptException:
        controller.set_motors(servo_center, motor_stop)
    controller.set_motors(servo_center, motor_stop)

    
