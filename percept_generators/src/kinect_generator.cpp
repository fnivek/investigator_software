#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/PointIndices.h>
//#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>

#include <stdio.h>


class object_extractor
{
private:	// Type defs
	typedef pcl::PointXYZ point;
	typedef pcl::PointCloud<point> pointCloud;
	typedef sensor_msgs::PointCloud2 point_msg;
	typedef Eigen::Matrix<float, 4, 1> vector4;

private:	// Vars
	double min_height_;
	double max_height_;
	//double cluster_tolerance_;			//Max distance in meters between points
	//int min_cluster_size_;				//Minimum number of points in a cluster
	//int max_cluster_size_;				//Maximum number of points in a cluster
	double voxel_size_;
	ros::Publisher scene_pub_;
	ros::Subscriber raw_pc_sub_;

public:		// Vars


private: 	// Functions
	pointCloud::Ptr threasholdAxis_(pointCloud::Ptr pc, const std::string axis, float min, float max);

	pointCloud::Ptr voxelDownSample_(pointCloud::Ptr pc, float voxel_size);

	//std::vector<pcl::PointIndices> euclideanSegment_(pointCloud::Ptr pc, float max_distance, int min_cluster_size, int max_cluster_size);

public:		// Funcitons
	object_extractor();

	void CloudCb_(const point_msg::ConstPtr& pc);

};

object_extractor::object_extractor() :
		min_height_(0),
		max_height_(5)
{

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	raw_pc_sub_ = nh.subscribe<point_msg>("/camera/depth/points", 10, &object_extractor::CloudCb_, this);

	scene_pub_ = nh.advertise<point_msg>(nh.resolveName("scene_pc"), 10);

	//Get some ros params
	std::string name = private_nh.resolveName("min_pc_height");

	private_nh.param<double>(name.c_str(), min_height_, -0.5);
	ROS_INFO("Param %s value %f", name.c_str(), min_height_);

	name = private_nh.resolveName("max_pc_height");
	private_nh.param<double>(name.c_str(), max_height_, 0.05);
	ROS_INFO("Param %s value %f", name.c_str(), max_height_);

	//name = private_nh.resolveName("euclidean_cluster_tolerance");					//euclidean_cluster_tolerance
	//private_nh.param<double>(name.c_str(), cluster_tolerance_, 0.05);
	//ROS_INFO("Param %s value %f", name.c_str(), cluster_tolerance_);

	//name = private_nh.resolveName("euclidean_min_cluster_size");					//euclidean_min_cluster_size
	//private_nh.param<int>(name.c_str(), min_cluster_size_, 5);
	//ROS_INFO("Param %s value %i", name.c_str(), min_cluster_size_);

	//name = private_nh.resolveName("euclidean_max_cluster_size");					//euclidean_max_cluster_size
	//private_nh.param<int>(name.c_str(), max_cluster_size_, 0xFFFFF);
	//ROS_INFO("Param %s value %i", name.c_str(), max_cluster_size_);

	name = private_nh.resolveName("voxel_size");									//Size of a voxel
	private_nh.param<double>(name.c_str(), voxel_size_, 0.01);
	ROS_INFO("Param %s value %f", name.c_str(), voxel_size_);

}

// Threashold axis
object_extractor::pointCloud::Ptr object_extractor::threasholdAxis_(pointCloud::Ptr pc, const std::string axis, float min, float max)
{
	//ROS_INFO("Pre-axis threasholded pc has %i data points", pc->width * pc->height);
	// Get rid of points to low and points that are to high
	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<point> pass;
	pass.setInputCloud (pc);
	pass.setFilterFieldName ("y");		// TODO: use axis instead
	pass.setFilterLimits (min, max);
	pointCloud::Ptr out_pc(new pointCloud);
	pass.filter (*out_pc);
	//ROS_INFO("Axis (%s) threasholded pc has %i data points", axis.c_str(), out_pc->width * out_pc->height);
	return out_pc;
}

// Down sample with voxels
object_extractor::pointCloud::Ptr object_extractor::voxelDownSample_(pointCloud::Ptr pc, float voxel_size)
{
	//ROS_INFO("Before %f cube voxel grid downsample point cloud has %i data points", voxel_size, pc->width * pc->height);
	pcl::VoxelGrid<point> vox;
	vox.setInputCloud(pc);
	vox.setLeafSize(voxel_size, voxel_size, voxel_size);
	pointCloud::Ptr vox_pc(new pointCloud);
	vox.filter(*vox_pc);
	//ROS_INFO("After %f cube voxel grid downsample point cloud has %i data points", voxel_size, vox_pc->width * vox_pc->height);
	return vox_pc;
}

/*
// Euclidean segmentation
// TODO: Should the vector be returned as a pointer???
std::vector<pcl::PointIndices> object_extractor::euclideanSegment_(pointCloud::Ptr pc, float max_distance, int min_cluster_size, int max_cluster_size)
{
	//Create KdTree for search method (see pcl documentation for more info on KdTrees)
	//And give it input
	pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
	tree->setInputCloud(pc);

	//Create a vector of vectors which represent the indices of each cluster
	std::vector<pcl::PointIndices> cluster_indices;

	//Set up the Euclidean Extractor
	pcl::EuclideanClusterExtraction<point> extractor;
	extractor.setClusterTolerance(max_distance);			//Points must be within cluster_tolerance meters of eachother
	extractor.setMinClusterSize(min_cluster_size);			//Min number of points in cluster
	extractor.setMaxClusterSize(max_cluster_size);			//Max number of points in cluster
	extractor.setSearchMethod(tree);						//search method
	extractor.setInputCloud(pc);							//Input cloud
	
	//Perform extraction
	extractor.extract(cluster_indices);
	//pcl::extractEuclideanClusters(*pc, cluster_indices, )
	//ROS_INFO("Number of clusters is %i", (int)cluster_indices.size());
	return cluster_indices;

}
*/

void object_extractor::CloudCb_(const point_msg::ConstPtr& pc)
{
	// Convert from ros msg
	pointCloud::Ptr raw_pc(new pointCloud);
	pcl::fromROSMsg(*pc, *raw_pc);

	// Down sampeling
	pointCloud::Ptr height_filtered_pc = threasholdAxis_(raw_pc, "y", min_height_, max_height_);

	// VoxelGrid downsample
	pointCloud::Ptr vox_pc = voxelDownSample_(height_filtered_pc, voxel_size_);

	// Output scene
	point_msg scene;
	pcl::toROSMsg(*vox_pc, scene);
	scene.header.frame_id = "/camera_depth_optical_frame";
	scene.header.stamp = ros::Time::now();
	scene_pub_.publish(scene);
	
	// Euclideian segmentation
	//std::vector<pcl::PointIndices> cluster_indices = euclideanSegment_(vox_pc, cluster_tolerance_, min_cluster_size_, max_cluster_size_);


	/*
	for(std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin();	//Iterate through clusters
			cluster_it != cluster_indices.end(); ++cluster_it)
	{
		pointCloud::Ptr cluster_pc(new pointCloud);												//Make a new point cloud
		//ROS_INFO("This euclidean segment has %i data points", cluster_pc->width * cluster_pc->height);

		pcl::copyPointCloud(*vox_pc, cluster_it->indices, *cluster_pc);							//Copy new point cloud
				
	}
	*/

}

/************************
 * 			Main		*
 ************************/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "object_extractor");

	object_extractor obj_extract;

	//ros::Rate update_rate(100);
	while(ros::ok())
	{
		//update_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
