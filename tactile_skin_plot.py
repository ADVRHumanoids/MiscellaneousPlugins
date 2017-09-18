import matplotlib.pyplot as plt
import numpy as np
import time
import signal
import sys
import rospy
from ADVR_ROS.msg import tactile_sensor

#rostopic pub /tactile_sensor ADVR_ROS/tactile_sensor "row: 8 
#col: 3 
#array: [5, 2, 4, 5, 2, 4, 5, 2, 4, 5, 2, 4, 5, 2, 4, 5, 2, 4, 5, 2, 4, 5, 2, 4,]" 

maxForceVal = 10
minForceVal = 1

global matrix

def signal_handler(signal, frame):
        sys.exit(0)

def callback(data):
	global matrix
	temp = np.array(data.array)
	#print (temp)
	matrix = temp.reshape(data.row,data.col)
	#matrix = np.asmatrix(temp)
	print (matrix)
	#matrix[0,0] = data.array[0]
	

def listener():
	signal.signal(signal.SIGINT, signal_handler)

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("tactile_sensor", tactile_sensor, callback)
	
	global matrix
	#array= [[1,2],[3,4]]
	array =  np.random.rand(8,3);
	global minForceVal
	global maxForceVal
	array[0] = minForceVal
 	array[1:] = maxForceVal
	matrix = np.matrix(array)	

	#print (matrix)
	# create the figure
	
	fig = plt.figure()
	ax = fig.add_subplot(111)
	im = plt.imshow(matrix,interpolation="nearest", cmap="hot")
	plt.colorbar(orientation ='vertical')
	plt.show(block=False)

	
	# draw some data in loop
	while(1):
	    # wait for a second
		time.sleep(0.1)
	    # replace the image contents
		im.set_array(matrix)
	    # redraw the figure
		fig.canvas.draw()

	#rospy.spin()	

if __name__ == '__main__':
	listener()

