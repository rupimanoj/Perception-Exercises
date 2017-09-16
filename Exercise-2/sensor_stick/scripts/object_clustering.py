#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

	# TODO: Convert ROS msg to PCL data
	cloud = ros_to_pcl(pcl_msg)

	# TODO: Voxel Grid Downsampling
	vox = cloud.make_voxel_grid_filter()
	LEAF_SIZE = 0.01
	vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
	cloud_filtered = vox.filter()
	down_sampled = cloud_filtered

	# TODO: PassThrough Filter
	pass_thorugh = cloud_filtered.make_passthrough_filter()
	filter_axis = 'z'
	pass_thorugh.set_filter_field_name(filter_axis)
	axis_min = 0.774
	axis_max = 1.5
	pass_thorugh.set_filter_limits(axis_min,axis_max)
	cloud_filtered = pass_thorugh.filter()
	pass_through = cloud_filtered

	# TODO: RANSAC Plane Segmentation
	seg = cloud_filtered.make_segmenter()
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	max_distance = 0.001
	seg.set_distance_threshold(max_distance)
	inliers,coefficients = seg.segment()

	# TODO: Extract inliers and outliers
	extracted_inliners = cloud_filtered.extract(inliers,negative=False)
	table_msg = extracted_inliners
	extracted_outliers = cloud_filtered.extract(inliers,negative=True)
	obj_msg = extracted_outliers
	
	# TODO: Euclidean Clustering
	white_cloud =  XYZRGB_to_XYZ(obj_msg)
	tree = white_cloud.make_kdtree()
	ec = white_cloud.make_EuclideanClusterExtraction()
	ec.set_ClusterTolerance(0.015)
	ec.set_MinClusterSize(200)
	ec.set_MaxClusterSize(3000)
	ec.set_SearchMethod(tree)
	cluster_indices = ec.Extract()


	# TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
	cluster_color = get_color_list(len(cluster_indices))
	color_cluster_point_list = []

	for j,indices in enumerate(cluster_indices):
			for i,indice in enumerate(indices):
				color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2],
											rgb_to_float(cluster_color[j])])

	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)

	# TODO: Convert PCL data to ROS messages
	obj_msg_ros = pcl_to_ros(obj_msg)
	table_msg_ros = pcl_to_ros(table_msg)
	cluster_cloud_ros = pcl_to_ros(cluster_cloud)

	# TODO: Publish ROS messages
	pcl_clusters_pub.publish(cluster_cloud_ros)
	pcl_objects_pub.publish(obj_msg_ros)
	pcl_table_pub.publish(table_msg_ros)

if __name__ == '__main__':

	# TODO: ROS node initialization
	rospy.init_node("clustering", anonymous=True)
	# TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud",pc2.PointCloud2,pcl_callback,queue_size=1)
	# TODO: Create Publishers
	pcl_clusters_pub = rospy.Publisher("/pcl_cluster",PointCloud2,queue_size=1)
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
	# Initialize color_list
	get_color_list.color_list = []

	# TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()