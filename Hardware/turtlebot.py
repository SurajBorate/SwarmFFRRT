
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from controllernew import PID_Controller
import tf
import numpy as np



class turtlebot3():
    def __init__(self,name='burger'):
        self.bot_name = name

        self.x = 0
        self.y = 0

        self.bot_quat = [1.0,0,0,0]

        self.PIDController = PID_Controller()

        self.turtle_vel = rospy.Publisher('/'+self.bot_name+'/cmd_vel', geometry_msgs.msg.Twist,queue_size=10)
        rospy.Subscriber('/'+self.bot_name+'/odom',Odometry,self.odom_Callback)

    def odom_Callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.bot_quat = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ]
   

    def move(self,dest):
        x_dest = dest[0]
        y_dest = dest[1]
        # print([self.x,self.y],[x_dest,y_dest])
        # euler = [i for i in tf.transformations.euler_from_quaternion(self.bot_quat)]
        # d=0.04
        # if self.bot_name == "waffle":
        #     d=0.08
        # # d=0
        # # [new_x,new_y] = 

        self.error = [x_dest - self.x ,
                  y_dest - self.y]        
        
        
        linear, angular = self.PIDController.track(self.error,self.bot_quat)
        
        # if self.bot_name=="burger_1" or self.bot_name=="burger_2":
        #     print(self.bot_name, linear)
        cmd = geometry_msgs.msg.Twist()
        # print(linear)
        cmd.linear.x = linear
        # cmd.linear.x = 0
        cmd.angular.z = angular
        # cmd.angular.z = 0
        self.turtle_vel.publish(cmd)

        return 0

    def stop(self):
        
        linear, angular = 0
        # angular = 0
        # linear = 1*(2-self.x)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear

        cmd.angular.z = 0
        self.turtle_vel.publish(cmd)

