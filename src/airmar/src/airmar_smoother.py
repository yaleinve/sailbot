#!/usr/bin/env python2.7

#Import statements
import rospy
import time
from math import sin,cos,atan2
from airmar.msg import AirmarData
from numpy import pi

pub_smoothed = rospy.Publisher("/airman_smoothed_1", AirmarData, queue_size = 10)
#Publish tactics output message, target_heading.  This function contains the actual algorithm.

n=50
n_smooth=20


def initGlobals():
	global airmar_msg
	global apVecs,truVecs,apRe,truRe
	apRe,truRe=[],[]

	apVecs=[]
	truVecs=[]

	airmar_msg=AirmarData()

	airmar_msg.heading = 0
	airmar_msg.amrRoll = 0
	airmar_msg.apWndDir = 0
	airmar_msg.apWndSpd = 0
	airmar_msg.cog = 0
	airmar_msg.sog = 0
	airmar_msg.truWndSpd = 0
	airmar_msg.truWndDir = 0
	airmar_msg.lat = 0
	airmar_msg.long = 0


#actual algoritham, in rads

from random import shuffle
def get_smoothed_data(apVecs,truVecs,apRe,truRe):
	def mean_of_circular_quantities(v):
		return atan2(1.0/len(v)*sum(sin(x) for x in v),1.0/len(v)*sum(cos(x) for x in v))%(2*pi)
	def distAngAbs(a,b):
		return min(abs((a-b)%(2*pi)),abs((b-a)%(2*pi)))
	# def bin_smoothed(v):
	# 	if(len(v)==0):
	# 		return (0,.0001)#so doesnt cause any errors
	# 	vSorted=sorted(v[:],key =lambda x:x[0])
	# 	pairs=[]
	# 	e=0
	# 	s=0
	# 	n=len(vSorted)
	# 	while(distAngAbs(vSorted[s][0],vSorted[0][0])<offsize):
	# 		s-=1
	# 		if(s==-n):
	# 			break
	# 	while(distAngAbs(vSorted[e][0],vSorted[0][0])<offsize and e<s+n):
	# 		e+=1
	# 	pairs.append((vSorted[0],e-s))
	# 	for i in range(1,n):
	# 		while(not distAngAbs(vSorted[s][0],vSorted[i][0])<offsize):
	# 			s+=1
	# 		while(distAngAbs(vSorted[e%n][0],vSorted[i][0])<offsize and e<s+n):
	# 			e+=1
	# 		pairs.append((vSorted[i],e-s))
	# 	shuffle(pairs)
	# 	pairs.sort(key=lambda x:x[1])
	# 	return pairs[0][0]
	def mean_smoothed(vs):
		a=mean_of_circular_quantities([v[0] for v in vs])
		s=1.0/len(vs)*sum(v[1] for v in vs)
	# print "APTRURE",apRe,ls
		return a,s
	if(len(apVecs)==0):
		return (0,.001),(0,.001)
	apRe.append(mean_smoothed(apVecs))
	if(len(apRe)>n_smooth):
		apRe.pop(0)
	truRe.append(mean_smoothed(truVecs))
	if(len(truRe)>n_smooth):
		truRe.pop(0)
	#apRe[-1],truRe[-1]#
	return mean_smoothed(apRe),mean_smoothed(truRe)



def publish_smoothed():
	global airmar_msg,apVecs,truVecs,apRe,truRe

	airmar_data_msg = AirmarData()
	airmar_data_msg.heading = airmar_msg.heading
	airmar_data_msg.amrRoll = airmar_msg.amrRoll
	airmar_data_msg.cog = airmar_msg.cog
	airmar_data_msg.sog = airmar_msg.sog
	airmar_data_msg.lat = airmar_msg.lat
	airmar_data_msg.long = airmar_msg.long

	(apWndDir, apWndSpd), (truWndDir, truWndSpd) = get_smoothed_data(apVecs,truVecs,apRe,truRe)
	airmar_data_msg.apWndDir  = apWndDir
	airmar_data_msg.apWndSpd  = apWndSpd
	airmar_data_msg.truWndSpd = truWndSpd
	airmar_data_msg.truWndDir = truWndDir

	pub_smoothed.publish(airmar_data_msg) #Publish the message


#Put the data from the airmar message into global variables
def airmar_callback(data):
	global airmar_msg
	airmar_msg=data

	apVecs.append((airmar_msg.apWndDir ,airmar_msg.apWndSpd))
	if(len(apVecs)>=n):
		apVecs=apVecs[1:]
	truVecs.append((airmar_msg.truWndDir ,airmar_msg.truWndSpd))
	if(len(truVecs)>=n):
		truVecs=truVecs[1:]

	publish_smoothed()

def listener():
	initGlobals()

	rospy.init_node("data_smoother")  #Must init node to subscribe
	rospy.Subscriber("/airmar_data", AirmarData, airmar_callback)
	rospy.loginfo("[data_smoother] All subscribed, data_smoother has started!")

	rospy.spin()


if __name__ == "__main__":
	listener() 	#Listen to our subscriptions
