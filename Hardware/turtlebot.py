
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from controller1 import PID_Controller



class turtlebot3():
    def __init__(self,name=['burger','waffle_pi']):
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

        error = [x_dest - self.x,
                  y_dest - self.y]
        
        linear , angular= 0,0
        #if abs(error[0])>0.001 or abs(error[1])>0.001: 
        linear, angular = self.PIDController.track(error,self.bot_quat)
        

        cmd = geometry_msgs.msg.Twist()
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


            # rate.sleep()

# name = sys.argv[1]
# # print(type(name))
# a = turtlebot3(name)
# a.move()
