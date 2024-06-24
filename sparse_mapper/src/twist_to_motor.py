#!/usr/bin/env python
import rospy
import pololu_maestro_ros.srv
from geometry_msgs.msg import Twist, TwistWithCovariance
from nav_msgs.msg import Odometry

set_servo = None
get_input = None
odom_pub = None

def twist_controller():
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

def callback(data):
    angular_vel = data.angular.z # Yaw angular velocity
    linear_vel = data.linear.x
    rospy.loginfo("x': %f | theta': %f\n", linear_vel, angular_vel)

    # Assume that 1 rad/s is the max for angular
    # Assume that 0.5 m/s is the max for linear
    motor_pwm = 300*(linear_vel/0.5) + 5700
    servo_pwm = 1000*(angular_vel/-1.0) + 6400
    set_servo(0, servo_pwm)
    set_servo(1, motor_pwm)

    send_odometry(motor_pwm)

    
def send_odometry(motor_pwm):
    odom = Odometry()
    odom.twist.twist.linear.x = (motor_pwm - 5700)/300
    odom_pub.publish(odom)


if __name__ == "__main__":
    rospy.wait_for_service('set_servo')
    set_servo = rospy.ServiceProxy('set_servo', pololu_maestro_ros.srv.set_servo)
    get_input = rospy.ServiceProxy('get_input', pololu_maestro_ros.srv.get_input)
    odom_pub = rospy.Publisher('wheel_vel/odom',Odometry, queue_size=10)
    rospy.init_node("twist_to_motorpwm")
    set_servo(0, 6400)
    set_servo(1, 5720)
    try:
        twist_controller()
    except rospy.ROSInterruptException: pass
