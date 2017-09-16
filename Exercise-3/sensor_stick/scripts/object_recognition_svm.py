#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
	get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
	return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

	#  Convert ROS msg to PCL data
	cloud = ros_to_pcl(pcl_msg)

	#  Voxel Grid Downsampling
	vox = cloud.make_voxel_grid_filter()
	LEAF_SIZE = 0.01
	vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
	cloud_filtered = vox.filter()
	down_sampled = cloud_filtered

	#  PassThrough Filter
	pass_thorugh = cloud_filtered.make_passthrough_filter()
	filter_axis = 'z'
	pass_thorugh.set_filter_field_name(filter_axis)
	axis_min = 0.774
	axis_max = 1.5
	pass_thorugh.set_filter_limits(axis_min,axis_max)
	cloud_filtered = pass_thorugh.filter()
	pass_through = cloud_filtered

	#  RANSAC Plane Segmentation
	seg = cloud_filtered.make_segmenter()
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	max_distance = 0.001
	seg.set_distance_threshold(max_distance)
	inliers,coefficients = seg.segment()

	#  Extract inliers and outliers
	extracted_inliners = cloud_filtered.extract(inliers,negative=False)
	table_msg = extracted_inliners
	extracted_outliers = cloud_filtered.extract(inliers,negative=True)
	obj_msg = extracted_outliers

	#  Euclidean Clustering
	white_cloud =  XYZRGB_to_XYZ(obj_msg)
	tree = white_cloud.make_kdtree()
	ec = white_cloud.make_EuclideanClusterExtraction()
	ec.set_ClusterTolerance(0.015)
	ec.set_MinClusterSize(200)   #large enough to ignore minute samples
	ec.set_MaxClusterSize(3000)  #not so large to combie two objects..should be a deciding parameter..
									#MInclusterSIze plays major role
	ec.set_SearchMethod(tree)
	cluster_indices = ec.Extract()

	#  Create Cluster-Mask Point Cloud to visualize each cluster separately
	cluster_color = get_color_list(len(cluster_indices))
	color_cluster_point_list = []

	for j,indices in enumerate(cluster_indices):
			for i,indice in enumerate(indices):
				color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2],
											rgb_to_float(cluster_color[j])])

	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)

	#  Convert PCL data to ROS messages
	obj_msg_ros = pcl_to_ros(obj_msg) 
	table_msg_ros = pcl_to_ros(table_msg)
	cluster_cloud_ros = pcl_to_ros(cluster_cloud)

	#  Publish ROS messages
	pcl_clusters_pub.publish(cluster_cloud_ros)  #publish cluster cloud
	pcl_objects_pub.publish(obj_msg_ros)        #publish segmented objects
	pcl_table_pub.publish(table_msg_ros)        #publish table plane point cloud

# Exercise-3 TODOs: 
	detected_objects_labels = []
	detected_objects = []
	# Classify the clusters! (loop through each detected cluster one at a time)
	for index, pts_list in enumerate(cluster_indices):
		 # Grab the points for the cluster       
		pcl_cluster = obj_msg.extract(pts_list)
		pcl_cluster_ros = pcl_to_ros(pcl_cluster)
		# Compute the associated feature vector
		chists = compute_color_histograms(pcl_cluster_ros)
		normals = get_normals(pcl_cluster_ros)
		nhists = compute_normal_histograms(normals)
		feature = np.concatenate((chists, nhists))
		# Make the prediction
		prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
		label = encoder.inverse_transform(prediction)[0]
		detected_objects_labels.append(label)
		# Publish a label into RViz
		label_pos = list(white_cloud[pts_list[0]])
		label_pos[2] += .4
		object_markers_pub.publish(make_label(label,label_pos, index))

		# Add the detected object to the list of detected objects.
		do = DetectedObject()
		do.label = label
		do.cloud = pcl_cluster_ros
		detected_objects.append(do)

	# Publish the list of detected objects
	rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
	detected_objects_pub.publish(detected_objects)

if __name__ == '__main__':

	
	model = pickle.load(open('model.sav', 'rb'))
	clf = model['classifier']
	encoder = LabelEncoder()
	encoder.classes_ = model['classes']
	scaler = model['scaler']

	#  ROS node initialization
	rospy.init_node("clustering", anonymous=True)
	#  Create Subscribers
	pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud",pc2.PointCloud2,pcl_callback,queue_size=1)
	#  Create Publishers
	pcl_clusters_pub = rospy.Publisher("/pcl_cluster",PointCloud2,queue_size=1)
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
	object_markers_pub  = rospy.Publisher("/object_markers",Marker,queue_size=1)
	detected_objects_pub = rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size=1)
	# Initialize color_list
	get_color_list.color_list = []

	#  Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()
