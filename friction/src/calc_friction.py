#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from friction.msg import Friction
import numpy as np
import copy
import math
'''
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance

'''
thres=0.68
std=max(0.01,rospy.get_param('/std'))
std_angle=max(0.01,rospy.get_param('/std_angle'))
lin_offset=eval(rospy.get_param('/lin_offset'))
ang_offset=eval(rospy.get_param('/ang_offset'))
known_coeff=0.6 # Rubber-Concrete 
acc_known=[-1,1.5,3.71] #-1 is invalid; 1st param: slow_motor; 2nd param: fast_motor
newton_g=9.8
valid_calc=[]
valid_calc_len=10001

class friction_node(object):
    def __init__(self):
        self.imu_pub = rospy.Publisher('/imu/data1',Imu, queue_size=10)
        self.fric_pub=rospy.Publisher('/coeff_fr',String,queue_size=10)
        rospy.Subscriber("/imu/data", Imu ,self.imufix_callback)
        rospy.Subscriber("/friction",Friction,self.friction_callback)
        self.plin_acc= Vector3()
        self.plin_acc.x=0
        self.plin_acc.y=0
        self.plin_acc.z=-9.8
        self.dlin=copy.deepcopy(self.plin_acc)
        self.pang_vel=Vector3()
        self.pang_vel.x=0
        self.pang_vel.y=0
        self.pang_vel.z=0
        self.dang=copy.deepcopy(self.pang_vel)

    def prob(self,x,u,cov):
        return 1/(cov*np.sqrt((2*np.pi)))*np.exp(-0.5*((x-u)/cov)**2)
    
    def correct_offset(self,data):
        data.linear_acceleration.x-=lin_offset[0]
        data.linear_acceleration.y-=lin_offset[1]
        data.linear_acceleration.z-=lin_offset[2]
        data.angular_velocity.x-=ang_offset[0]
        data.angular_velocity.y-=ang_offset[1]
        data.angular_velocity.z-=ang_offset[2]

    def correct_hyst(self,data,plin_acc,pang_vel):
        px=self.prob(data.linear_acceleration.x,plin_acc.x,std) 
        py=self.prob(data.linear_acceleration.y,plin_acc.y,std)
        pz=self.prob(data.linear_acceleration.z,plin_acc.z,std)
        if px>thres:
            data.linear_acceleration.x=plin_acc.x
        if py>thres:
            data.linear_acceleration.y=plin_acc.y
        if pz>thres:
            data.linear_acceleration.z=plin_acc.z
        pr=self.prob(data.angular_velocity.x,pang_vel.x,std_angle)
        pp=self.prob(data.angular_velocity.y,pang_vel.y,std_angle)
        pyaw=self.prob(data.angular_velocity.z,pang_vel.z,std_angle)
        #rospy.loginfo('pr[%f] pp[%f] pyaw[%f]',pr,pp,pyaw)
        self.plin_acc= Vector3()
        self.plin_acc.x=0
        self.plin_acc.y=0
        self.plin_acc.z=-9.8
        if pr>thres:
            data.angular_velocity.x=pang_vel.x
        if pp>thres:
            data.angular_velocity.y=pang_vel.y
        if pyaw>thres:
            data.angular_velocity.z=pang_vel.z    
        return data
    
    def imufix_callback(self, data):
        self.correct_offset(data)
        self.correct_hyst(data,self.dlin,self.dang)
        self.correct_hyst(data,self.plin_acc,self.pang_vel)
        self.plin_acc=data.linear_acceleration
        self.pang_vel=data.angular_velocity
        self.imu_pub.publish(data)
    
    def friction_callback(self,data):
        net_acc=math.sqrt(self.plin_acc.x**2+self.plin_acc.y**2)
        rospy.loginfo('MS[%d] CS[%d] Acc[%f]',data.motor_state,data.car_state,net_acc)
        if data.motor_state==0:
            pass 
        if data.car_state==2: #car is moving elif
            # calculate frictiion
            unknown_coeff=(acc_known[data.motor_state]-net_acc)/newton_g + known_coeff
            # basic check; generally coeff of friction is between 0 and 1
            if unknown_coeff>0 and unknown_coeff<=1:
                valid_calc.append(unknown_coeff)
            # TBD use PQ for more optimized calculation
            m=len(valid_calc)
            if m==0:
                return
            if m>=valid_calc_len:
                valid_calc.pop(0)
            median_coeff=valid_calc[m//2] if m%2!=0 else (valid_calc[m//2]+valid_calc[m//2-1])/2.0
            rospy.loginfo('Correct Coeff Of Friction[%f]',median_coeff)
            self.fric_pub.publish(str(median_coeff))
            

if __name__ == "__main__":
    rospy.init_node('friction_node')
    friction_node = friction_node()
    rospy.spin()

