#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseStamped


def rangeCallback(range_data):
	global dis_up
	global dis_down
	
	dis_up = range_data.data[0]
	dis_down = range_data.data[1]
	
	output_to_file()
	
	
def poseCallback(pose_data):
	global pos
	
	#header = pose_data.header
	pos = pose_data.pose.position
	#orientation = pose_data.pose.orientation
	#covariance = pose_data.pose.covariance
	
def baroCallback(baro_data):
	global elevation
	
	pressure = baro_data.fluid_pressure #current air pressure in Pascals
	elevation = 44330 * (1 - (pressure / 101325)**(1/5.255)) #elevation above sea-level in metres
	
def output_to_file():
	string = str(pos.x) + "," + str(pos.y) + "," + str(elevation) + "," + str(dis_up) + "," + str(dis_down) + "\n"
	print("string: " + string)
	
	with open("data_output/text.txt" , "a") as file_output:
		file_output.write(string)

def subscribe_to_data():
	rospy.init_node("Subscriber_Node", anonymous = True)
	rospy.Subscriber('range_topic', Float64MultiArray, rangeCallback)
	rospy.Subscriber('slam_out_pose', PoseStamped, poseCallback)
	rospy.Subscriber('/mavros/imu/static_pressure', FluidPressure, baroCallback )
	
	rospy.spin()
	
if __name__ == '__main__':
	try:
		subscribe_to_data()
	except rospy.ROSInterruptException:
		sys.exit(0)