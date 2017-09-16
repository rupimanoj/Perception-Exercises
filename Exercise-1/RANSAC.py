# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter to downsample
vox = cloud.make_voxel_grid_filter()
LEAF_SIZE = 0.01
vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd' #to store downsampled point cloud
pcl.save(cloud_filtered,filename)

# PassThrough filter
pass_thorugh = cloud_filtered.make_passthrough_filter()
filter_axis = 'z'
pass_thorugh.set_filter_field_name(filter_axis)
axis_min = 0.774	#achieved this value by trial and error methid..SLight change in this value will also give table edge 
					#as part of objects..which are treated as outliers..Hence fine tuning is required here.
axis_max = 1.5
pass_thorugh.set_filter_limits(axis_min,axis_max)
cloud_filtered = pass_thorugh.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered,filename)

# RANSAC plane segmentation
seg = cloud_filtered.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
max_distance = 0.001
seg.set_distance_threshold(max_distance)
inliers,coefficients = seg.segment()

# Extract inliers
#table corresponds to plane
#hence inliners correspond to table and outliers correspond to objects
extracted_inliners = cloud_filtered.extract(inliers,negative=False)
filename = "extracted_inliners.pcd"  #to store table top plane
pcl.save(extracted_inliners,filename)


extracted_outliers = cloud_filtered.extract(inliers,negative=True)
filename = 'extracted_outliers.pcd'	#to store objects other than table plane
pcl.save(extracted_outliers,filename)




