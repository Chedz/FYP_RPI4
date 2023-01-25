#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

# Libraries
# Two vl53l1x lidar sensors attached to TCA9548A channels 0 and 1.
#import time
import board
import adafruit_vl53l1x
import adafruit_tca9548a
import signal
import sys

def signal_handler(signal, frame): #ctrl + c -> exit program
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Create I2C bus as normal
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(i2c)

# For each sensor, create it using the TCA9548A channel instead of the I2C object
vl53_0 = adafruit_vl53l1x.VL53L1X(tca[0])
vl53_1 = adafruit_vl53l1x.VL53L1X(tca[1])

vl53_0.start_ranging()
vl53_1.start_ranging()

# After initial setup, can just use sensors as normal.
#while True:
 #   print(vl53_0.distance, " , ", vl53_1.distance)
 #   vl53_0.clear_interrupt()
 #   vl53_1.clear_interrupt()
 #   time.sleep(0.1)
 
def get_distance_0():
	if vl53_0.data_ready:
		return vl53_0.distance
			#sensor_0 = vl53_0.distance #sensor value
			
			#if sensor_0 is None: #if sensor value is null ie. == "None"
			#	return -1.0
			#else:
			#	return sensor_0
				
def get_distance_1():
	if vl53_1.data_ready:
		return vl53_1.distance #sensor value
			
			#if sensor_1 is None: #if sensor value is null ie. == "None"
			#	return -1.0
			#else:
			#	return sensor_1
	
	
def publish_data():
	pub = rospy.Publisher('range_topic', Float64MultiArray, queue_size=1)
	rospy.init_node('range_publisher', anonymous=True)
	r = rospy.Rate(20)
	rospy.loginfo('Range_publisher node started, now publishing messages to range_topic')
	
	while not rospy.is_shutdown():
		data_to_send = Float64MultiArray()

		val_0 = vl53_0.distance
		if val_0 is None:
			val_0 = -1
			
		val_1 = vl53_1.distance
		if val_1 is None:
			val_1 = -1
		
		sensor_data = [val_0, val_1]	
		data_to_send.data = sensor_data
		pub.publish(data_to_send)
		vl53_0.clear_interrupt()
		vl53_1.clear_interrupt()
		r.sleep()
		
	
if __name__ == '__main__':
	try:
		publish_data()
	except rospy.ROSInterruptException:
		sys.exit(0)
