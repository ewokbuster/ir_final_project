#!/usr/bin/python3

"""

"""

import rospy
from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,Pose,Point,
                               PoseArray, Quaternion )
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from threading import Lock
import sys

import numpy as np
import time
import uuid


from copy import deepcopy

class PizzaGenerationNode(object):

	def __init__(self):		
		self.occupancy_map = OccupancyGrid()        
		self.markerArray = MarkerArray()
		self._marker_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray,queue_size=100)
		


		self.res_w = 0.0
		self.res_h = 0.0
		self.resolution = 0.0

		rospy.loginfo("Waiting for a map...")
		try:
		    ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
		except:
		    rospy.logerr("Problem getting a map. Check that you have a map_server"
		             " running: rosrun map_server map_server <mapname> " )
		    sys.exit(1)
		rospy.loginfo("Map received. %d X %d, %f px/m." %
		              (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
		               ocuccupancy_map.info.resolution))
		self.set_map(ocuccupancy_map)  # 🚩
		self.generate_pizzas(1)

	


	def delete_pizza(self,id):
		for i in range(len(self.pizzaArray)):
			if self.pizzaArray[i]['id'] == id:
				self.delete_from_markerArray(self.pizzaArray[i][id])

				del self.pizzaArray[i]
				break





		



	def delete_from_markerArray(self,id):
		marker_array_msg = MarkerArray()
		marker = Marker()
		marker.id = id
		marker.ns = "my_namespace"
		marker.action = Marker.DELETE
		marker_array_msg.markers.append(marker)
		self.publish_pizza(marker_array_msg)
		rospy.sleep(0.2)


	def generate_pizzas(self,number=1):
		marker_array_msg = MarkerArray()
		for x in range(number):
			pos=self.generate_particle_normal()
			id= str(uuid.uuid4())
			pizza = {
				"pos": pos,
				"id": id
			}
			self.pizzaArray.append(pizza)
			marker=self.marker_generation(pos,id)
			#self.markerArray.markers.append(marker)
			marker_array_msg.markers.append(marker)

		self.publish_pizza(marker_array_msg)
		rospy.sleep(0.2)

	def publish_pizza(self,markerArray):
		rospy.loginfo("publish_pizza")
		
		self._marker_publisher.publish(markerArray)
    

	def marker_generation(self, pose,guid):
		rospy.loginfo("marker_generation")
		marker = Marker()
		
		marker.ns="my_namespace"
		marker.header.frame_id = "base_link"
		marker.type = marker.CYLINDER
		marker.action = marker.ADD
		marker.id=guid
		marker.scale.x = 100
		marker.scale.y = 100
		marker.scale.z = 100
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose=pose
		marker.pose.orientation.w = 1.0
		return marker
                
	def check_position_occupancy(self, x, y):
		"""
		# read map data from self.occupancy_map; compare intersecting data points
		# if dis is smaller than threshold, then do resample or give this point a random movement
		return occupancy of position
		"""

		x_px = round(x / self.occupancy_map.info.resolution)
		y_px = round(y / self.occupancy_map.info.resolution)
		index = int(y_px * self.occupancy_map.info.width + x_px)
		rospy.loginfo(self.occupancy_map.data)
		return (self.occupancy_map.data[index] == 0)


	def generate_particle_normal(self):
		rospy.loginfo("generate_particle_normal")
		while 1:
			x= np.random.uniform(0,self.res_w* self.resolution)
			y= np.random.uniform(0,self.res_h* self.resolution)
			rospy.loginfo("generate_particle_normal________")
			if x < 0 or y < 0 or x >= self.res_w * self.resolution or y >= self.res_h* self.resolution:
				rospy.loginfo("generate_particle_normal________continue")
				continue
			if self.check_position_occupancy(x, y): 
				rospy.loginfo("generate_particle_normal_"+str(x)+"___"+str(y))
				return  Pose(Point(round(x), round(y), 0),
			            Quaternion())

	def set_map(self, occupancy_map):
		""" Set the map for localisation """
		self.occupancy_map = occupancy_map

		# ----- Map has changed, so we should reinitialise the particle cloud
		self.get_map()

	def get_map(self):
		if not (self.res_w and self.res_h):
			map_occ = self.occupancy_map.info
			self.resolution = map_occ.resolution
			self.res_w = map_occ.width  # * self.resolution
			self.res_h = map_occ.height  # * self.resolution
		#if not len(self.map_free_grid):
		#	self.generate_free_xy()

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("pizza_generation_node")
    node = PizzaGenerationNode()
    rospy.spin()
