
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty


class ControlMobileRobot():
	def __init__(self):
		rospy.init_node('ControlMobileRobot',anonymous=False)
	  
	        rospy.on_shutdown(self.shutdown)
	        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)		
                # TurtleBot will receive the message 10 times per second.
                rate = rospy.Rate(10);
		#Twist is geometry_msgs for linear and angular velocity	        
		move_cmd = Twist()
		move_cmd.linear.x=0.3
		move_cmd.angular.z=0

		self.cmd_vel.publish(move_cmd)
		rate.sleep()
	def shutdown(self):
		rospy.loginfo("Stopping TurtleBot")

		self.cmd_vel.publish(Twist())
		#Give turtlebot time to stop
		rospy.sleep(1)
        
	speed = .2
	turn = 1

	def vels(speed,turn):
	    return "currently:\tspeed %s\tturn %s " % (speed,turn)
	
	if __name__=='__main--':
		try:	
			ControlMobileRobot()
		except:
			rospy.loginfo("End of the trip for MobileRobot")
