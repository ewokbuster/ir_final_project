import json

from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from .pf_base import PFLocaliserBase
import math
import rospy
import numpy as np
import random
# import os.path
# import json
from datetime import datetime
from .util import rotateQuaternion, getHeading
from copy import deepcopy


def normalise_weights(arr):
    return np.array(arr) / sum(arr) if len(arr) else arr


class PFLocaliser(PFLocaliserBase):
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)
        self.VARIENCE_THETA = 0.2
        self.INIT_X = 0  # Initial x location of robot (metres)
        self.INIT_Y = 0  # Initial y location of robot (metres)
        self.random_initialised = False

        # ----- Set motion model parameters
        # TODO which ones belong to motion model params, which ones belong to sensor model params?

        # define how many particles we need in the initial cluster
        self.NUMBER_PARTICLES_CLOUD = 200  # FIXME Number of readings to predict
        self.NUMBER_PARTICLES_RND_PERCENT = 0.1  # define how many we need for random p
        self.NUMBER_PARTICLES_RND = int(
            self.NUMBER_PARTICLES_CLOUD * self.NUMBER_PARTICLES_RND_PERCENT)  # define how many we need for random p
        self.RATIO_PARTICLES_INIT = 20  # how large the particle cloud should be initially
        #
        self.weights = []  # update the weights arr each iteration
        self.WEIGHT_TO_THROW = 0.1  # give up this particle and spawn new

        # ----- Sensor model parameters
        # TODO @J do we need to change these numbers? if we do, where to obtain these data
        self.ODOM_ROTATION_NOISE = 3  # Odometry model rotation
        self.ODOM_TRANSLATION_NOISE = 3  # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 3  # Odometry model y axis (side-to-side) noise

        self.delta_u = 1 /( self.NUMBER_PARTICLES_CLOUD+self.NUMBER_PARTICLES_RND)
        self.res_w = 0.0
        self.res_h = 0.0
        self.resolution = 0.0

        self.map_free_grid = []

        # self.estimatedpose.pose.pose.position.x = self.sensor_model.map_origin_x
        # self.estimatedpose.pose.pose.position.y = self.sensor_model.map_origin_y

    def get_map(self):
        if not (self.res_w and self.res_h):
            map_occ = self.occupancy_map.info
            self.resolution = map_occ.resolution
            self.res_w = map_occ.width #* self.resolution
            self.res_h = map_occ.height# * self.resolution
        if not len(self.map_free_grid):
            self.generate_free_xy()

    def generate_particle_random(self):
        # self.get_map()
        # x = np.random.uniform(0, self.res_w)
        # y = np.random.uniform(0, self.res_h)
        qz = np.random.uniform(0, np.pi * 2)
        grid_ind = self.map_free_grid[int(np.random.uniform(0, len(self.map_free_grid)))]  # 1-362404
        # point = self.occupancy_map.data[grid_ind]
        x = grid_ind % (self.res_w  )  * self.resolution
        y = int(grid_ind / self.res_w)   * self.resolution

        # if self.check_position_occupancy(x, y):
        return Pose(
            Point(x, y, 0),
            Quaternion(*quaternion_from_euler(0, 0, qz))
        )

        # else:
        #     return self.generate_particle_random()

    def generate_particle_normal(self):
        # self.get_map()
        pos = self.estimatedpose.pose.pose
        # TODO consider the situation that estimatedpose is around the corner; same for generate_particle_random
        # will it take considerably too much time to generate a random position
        # that's inside the free area?
        # TODO add variants? for poses and orientations
        qz = np.random.normal(getHeading(pos.orientation), self.VARIENCE_THETA)
        while 1:
            # x = np.random.normal(pos.position.x, self.res_w / 15)
            # y = np.random.normal(pos.position.y, self.res_h / 15)  # / self.RATIO_PARTICLES_INIT
            x, y = np.random.normal((pos.position.x, pos.position.y), self.res_h* self.resolution / self.RATIO_PARTICLES_INIT, 2)
            # if(x < 0 or y < 0 or x >self.res_w or y > self.res_h)
            # filter those poses that intersect with walls
            if x < 0 or y < 0 or x >= self.res_w * self.resolution or y >= self.res_h* self.resolution:
                continue
            if self.check_position_occupancy(x, y):
                return Pose(
                    Point(x, y, 0),
                    Quaternion(*quaternion_from_euler(0, 0, qz))
                )
            # else:
            #     return self.generate_particle_normal()

    def generate_particles_rnd(self):
        # indexs_free = np.random.uniform(0, len(self.map_free_grid), self.NUMBER_PARTICLES_RND)
        # indexs_real = [self.occupancy_map.data[i] for i in indexs_free]
        return [self.generate_particle_random() for _ in range(self.NUMBER_PARTICLES_RND)]

    def generate_particle_cloud(self, initialpose):
        pos = Pose(initialpose.pose.pose.position, initialpose.pose.pose.orientation)  # initialpose
        return [pos] + [self.generate_particle_normal() for i in range(self.NUMBER_PARTICLES_CLOUD - 1)]

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
        # x_px = round(x)
        # y_px = round(y)
        index = int(y_px * self.occupancy_map.info.width + x_px)
        # res = not self.occupancy_map.data[index]  # < 1  # <zero
        # return self.occupancy_map.data[index] < 1 if only_wall else
        return (self.occupancy_map.data[index] == 0)

    def generate_free_xy(self):
        # self.map_free_grid[np.where(not self.occupancy_map.data)]
        for (ind, d) in enumerate(self.occupancy_map.data):
            if not d:
                self.map_free_grid.append(ind)
        # return [x for x in self.occupancy_map.data]

    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise
        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        :Args:
            | initialpose: the initial pose estimate
        :Return:np.random.uniform(0
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        # if not len(self.occupancy_map.data):
        #     return

        arr = PoseArray()
        # TODO CONFIRM read from input initialpose. spread around initialpose ~R
        # what's the radius of this circle?
        # let's say 5 for now; need to fixme self.RATIO_PARTICLES_INIT
        # and add particles across the whole map

        self.get_map()  # initialise map width & height
        if initialpose and self.random_initialised:
            # arr1=arr2=[]
            arr2 = self.generate_particles_rnd()
            arr1 = self.generate_particle_cloud(initialpose)
            arr.poses = arr1 + arr2

            self.particlecloud = arr
            # assume all particles have same weights at beginning
            # even the random ones
            self.weights = normalise_weights(np.ones(len(self.particlecloud.poses)))

        self.random_initialised = 1
        return arr

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        :Args:/map
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
         """
        # we need to introduce the minimal threshold so it won't update too frequent to save cal time
        if not self.odom_initialised:
            return
        if not self.last_odom_pose:
            return
        twist = self.last_odom_pose.twist.twist
        if twist.linear.x <= 0 and twist.linear.y <= 0 and twist.linear.z <= 0 \
                and twist.angular.x <= 0 and twist.angular.y <= 0 and twist.angular.z <= 0:
            # if don't have sufficient movement since last scan, then don't update the cloud
            return

        # resample new particles TODO adaptive
        weights = [self.sensor_model.get_weight(scan, p) for p in self.particlecloud.poses]
        self.weights = normalise_weights(weights)

        arr = []
        uj = 0.0
        ci = 0.0
        i = 0  # will +1 at 1st step so begins with 0 actually
        # converging particles
        for j in range(self.NUMBER_PARTICLES_CLOUD):  # len(self.particlecloud.poses)
            while uj > ci and i < (len(self.weights) - 1):
                ci += self.weights[i]
                i += 1  # fixme dk y suddenly out of boundry
            particle = self.particlecloud.poses[i]
            arr.append(deepcopy(particle))
            uj += self.delta_u

            # random particles
        arr += self.generate_particles_rnd()  # TODO read from params

        self.particlecloud = PoseArray(poses=arr)
        self.weights = normalise_weights(np.ones(len(self.particlecloud.poses)))

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after
        throwing away any which are outliers
        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """

        # rospy.loginfo("estimate_pose")

        # do the simplest one first,
        sum_x = 0.0
        sum_y = 0.0
        sum_h = 0.0

        local_particlecloud_poses = self.particlecloud.poses
        # local_weight=self.weights

        # local_particlecloud_poses_dictionary=[]
        # for i, pose in  enumerate(local_particlecloud_poses):
        #     local_particlecloud_poses_dictionary.append(
        #         {
        #             "pose": pose,
        #             "weight":local_weight[i],
        #             "id": i,
        #         }
        #     )

        # # TODO  @Olzhas I comment the below temporarily to speed up feel free to uncomment and work
        # # calculate weight of each cluster
        # clusters = self.clustering(local_particlecloud_poses_dictionary)
        # '''
        #longest
        # longest=clusters[0]
        # for cluster in clusters:
        #     if len(longest)<len(cluster):
        #         longest=cluster
        #
        # local_particlecloud_poses=longest
        #end of longest
        # '''
        #
        # total_clusters_weights = []
        #
        # for cluster in clusters:
        #
        #     rospy.loginfo("cluster:  " + str(len(cluster)))
        #     if len(cluster):
        #         cluster_weights = 0
        #         for pos in cluster:
        #             cluster_weights += pos["weight"]
        #         total_clusters_weights.append({"weight": cluster_weights / len(cluster), "cluster": cluster})
        #
        # # get heaviest cluster
        # heaviest_cluster = total_clusters_weights[0]
        # for cluster in total_clusters_weights:
        #     if heaviest_cluster["weight"] < cluster["weight"]:
        #         heaviest_cluster = cluster


        # Use heaviest cluster
        # local_particlecloud_poses = heaviest_cluster["cluster"]

        # rospy.loginfo("heaviest_cluster :  " + str(len(local_particlecloud_poses)))
        # END OF heaviest

        # heaviest

        # choose the most likely cloud here
        # replace the self.particlecloud.poses in enumerate with this most likely cloud
        # TODO end




        for ind, p in enumerate(local_particlecloud_poses):
            w = self.weights[ind]
            sum_x += p.position.x * w
            sum_y += p.position.y  * w
            sum_h += getHeading(p.orientation) * w
            # n_poses += 1

        avg_pose = Pose(
            Point(sum_x, sum_y, 0),  # position=
            Quaternion(*quaternion_from_euler(0, 0, sum_h % np.pi))  # orientation=
        )

        return avg_pose

    def clustering(self, particlecloud_poses_dictionary):
        '''
        return 2  arrays with particles, each array is cluster
        '''
        k = 2  # claster number
        # cluster_centroids = []  # centroids of claster
        # cluster_positions = []  # data of claster

        positions = particlecloud_poses_dictionary
        # n = len(self.particlecloud.poses)
        cluster_centroids = []  # centroids of claster
        # cluster_positions = []  # data of claster
        # n = len(positions)
        for x in range(k):
            cluster_centroids.append(random.choice(positions))

        cluster_positions = self.data_distribution(positions, cluster_centroids, k)

        previous_clusters = cluster_centroids
        while 1:
            cluster_centroids = self.cluster_update(cluster_centroids, cluster_positions)
            cluster_positions=[]
            cluster_positions = self.data_distribution(positions, cluster_centroids, k)
            if  cluster_centroids == previous_clusters: #self.cluster_diverged(cluster_centroids, previous_clusters, k):
                break


        return cluster_positions

    def cluster_diverged(self, centre: Pose, previous: Pose, k):
        flag = True
        for i in range(k):
            x1 = centre[i]["pose"].position.x
            y1 = centre[i]["pose"].position.y
            x2 = previous[i]["pose"].position.x
            y2 = previous[i]["pose"].position.y
            # x2, y2 = previous[i].position
            flag &= (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) < 0.1  # threshhold
        return flag

    def data_distribution(self, positions, cluster_centroids, k):
        cluster_positions = [[] for i in range(k)]

        for pos in positions:
            min_distance = float('inf')
            cluster_index = -1
            for j in range(len(cluster_centroids)):
                distance = 0

                centroid = cluster_centroids[j]["pose"]
                #rospy.loginfo("cluster_centroids[j]   " + str(cluster_centroids))


                distance += (pos["pose"].position.x - centroid.position.x) ** 2  # todo Add calculation of euclide distance
                distance += (pos["pose"].position.y - centroid.position.y) ** 2  # todo Add calculation of euclide distance


                distance = np.sqrt(distance)

                if distance < min_distance:
                    min_distance = distance
                    cluster_index = j
                    # rospy.loginfo("new_cluster_index " + str(new_cluster_index))
            cluster_positions[cluster_index].append(pos)
        return cluster_positions

    def cluster_update(self, cluster_centroids, cluster_positions):
        for i in range(len(cluster_centroids)):
            new_centroid_x = 0
            new_centroid_y = 0
            for j in range(len(cluster_positions[i])):
                new_centroid_x += cluster_positions[i][j]["pose"].position.x
                new_centroid_y += cluster_positions[i][j]["pose"].position.y
            if len(cluster_positions[i]) != 0:
                new_centroid_x = new_centroid_x / len(cluster_positions[i])
                new_centroid_y = new_centroid_y / len(cluster_positions[i])


            cluster_centroids[i] ={
                "pose": Pose(
                    Point(new_centroid_x, new_centroid_y, 0),
                    cluster_centroids[i]["pose"].orientation
                ),
                "weight": 0,
                "id": 0
            }

        return cluster_centroids