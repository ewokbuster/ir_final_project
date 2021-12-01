#!/usr/bin/python3

"""

"""

import rospy
from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                               PoseArray, Quaternion )
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from threading import Lock
import sys
from copy import deepcopy

class PizzaGenerationNode(object):
	def __init__(self):		
		self.occupancy_map = OccupancyGrid()        
		self.markerArray = MarkerArray()
		self._marker_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray)

		self._pose_publisher = rospy.Publisher("/estimatedpose", PoseStamped)
		self._amcl_pose_publisher = rospy.Publisher("/amcl_pose",
		                                            PoseWithCovarianceStamped)
		self._cloud_publisher = rospy.Publisher("/particlecloud", PoseArray)
		self._tf_publisher = rospy.Publisher("/tf", tfMessage)  # transform

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
		



    
	def generate_pizza(self,number=6):
		for x in range(6):
			pos=self.generate_particle_normal()	
			marker=self.marker_generation(pos)
			self.markerArray.markers.append(marker)
		self._marker_publisher.publish(markerArray)
    

	def marker_generation(self, pose):
		marker = Marker()
		marker.header.frame_id = "/neck"
		marker.type = marker.CYLINDER
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
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
		if not self.random_initialised:
		    return 1
		x_px = round(x / self.occupancy_map.info.resolution)
		y_px = round(y / self.occupancy_map.info.resolution)
		index = int(y_px * self.occupancy_map.info.width + x_px)
		return (self.occupancy_map.data[index] == 0)


	def generate_particle_normal(self):
		while 1:
		    x, y = np.random.normal((pos.position.x, pos.position.y), self.res_h* self.resolution / self.RATIO_PARTICLES_INIT, 2)
		    if x < 0 or y < 0 or x >= self.res_w * self.resolution or y >= self.res_h* self.resolution:
		        continue
		    if self.check_position_occupancy(x, y):
		        return Pose(
		            Point(x, y, 0),
		        )

	def set_map(self, occupancy_map):
		""" Set the map for localisation """
		self.occupancy_map = occupancy_map
		self.set_map(occupancy_map)
		# ----- Map has changed, so we should reinitialise the particle cloud

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("pizza_generation_node")
    node = PizzaGenerationNode()
    rospy.spin()
