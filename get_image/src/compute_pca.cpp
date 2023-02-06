#include <ros/ros.h>
#include "std_msgs/String.h"
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Range.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <thread>
#include <visp_bridge/image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudPCL;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace visualization_msgs;
using namespace geometry_msgs;

ros::Publisher g_marker_array_pub;

int g_marker_id;

MarkerArray g_marker_array;

void guidedFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, double epsilon);

void pushEigenMarker(pcl::PCA<pcl::PointXYZ>& pca,
                     int& marker_id,
                     MarkerArray& marker_array,
                     double scale,
                     const std::string& frame_id);

void callback( const PointCloudConstPtr& pointcloud
		) {
	
	
	std::cout << "Get the object PointCloud !!!" << std::endl;

	PointCloud2 pc2;

	convertPointCloudToPointCloud2(*pointcloud, pc2);

	// [ Ransac on the Door Pointcloud ]
	std::vector<int> inliers;
	PointCloudPCL::Ptr door_pointcloud_msg_ransac (new PointCloudPCL);
	//PointCloudPCL pointcloud_msg; // (new PointCloudPCL);


	pcl::PCLPointCloud2 pcl_pc;
	
	pcl_conversions::toPCL(pc2, pcl_pc);

	// pcl::fromPCLPointCloud2( pcl_pc, pointcloud_msg);

	PointCloudPCL::Ptr pointcloud_msg(new PointCloudPCL());

	pcl::fromPCLPointCloud2(pcl_pc, *pointcloud_msg);

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pointcloud_msg));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (.08);
	ransac.computeModel();
	ransac.getInliers(inliers);

	pcl::copyPointCloud(*pointcloud_msg, inliers, *door_pointcloud_msg_ransac);

	// [Add Filter on the point cloud]
	float radius = 0.01;
	float epsilon = 0.1;
	guidedFilter(door_pointcloud_msg_ransac,radius,epsilon);
	guidedFilter(door_pointcloud_msg_ransac,radius,epsilon);
	guidedFilter(door_pointcloud_msg_ransac,radius,epsilon);


	pcl::PCA<pcl::PointXYZ> pca;
	
	pca.setInputCloud(door_pointcloud_msg_ransac);

	g_marker_array.markers.clear();
	// orientation.markerarray.markers.clear();

	g_marker_id = 0;

	const string frame_id = "camera_depth_frame";

	// Get and publish the orientation of the door

	pushEigenMarker(pca, g_marker_id,g_marker_array, 0.1, frame_id);
	cv::waitKey(3);

	if(!ros::ok()){
		ros::shutdown();
	}
	
}


void pushEigenMarker(pcl::PCA<pcl::PointXYZ>& pca,
                     int& marker_id,
                     MarkerArray& marker_array,
                     double scale,
                     const std::string& frame_id){
	
	Marker marker;
	marker.lifetime = ros::Duration(5.0);
	marker.header.frame_id = frame_id;
	marker.ns = "eigenvector_object";
	marker.type = Marker::ARROW;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.pose.position.x = pca.getMean().coeff(0);
	marker.pose.position.y = pca.getMean().coeff(1);
	marker.pose.position.z = pca.getMean().coeff(2);

	Eigen::Quaternionf qx, qy, qz, q;
	Eigen::Matrix3f ev = pca.getEigenVectors();
	Eigen::Vector3f axis_x(ev.coeff(0, 0), ev.coeff(1, 0), ev.coeff(2, 0));
	Eigen::Vector3f axis_y(ev.coeff(0, 1), ev.coeff(1, 1), ev.coeff(2, 1));
	Eigen::Vector3f axis_z(ev.coeff(0, 2), ev.coeff(1, 2), ev.coeff(2, 2));
	qx.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_x);
	qy.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_y);
	qz.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_z);
	
	// [ Apply some rotation to obtain the z axes toward the camera]
	Eigen::Matrix3f mat(qz.toRotationMatrix());
	// std::cout << " Matrice rotation : " << mat << endl;
	Eigen::Vector3f rotz(0,0,1);
	Eigen::Vector3f vect = mat*rotz;
	//std::cout << " Axe Z : " << vect << endl;
	Eigen::Matrix3f m, m1, m2;
	m = Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	m1 = Eigen::AngleAxisf(0,Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	m2 = Eigen::AngleAxisf(0,Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ());

	if(axis_z.coeff(2) > 0){
		ev = ev*m;
		axis_x = Eigen::Vector3f(ev.coeff(0, 0), ev.coeff(1, 0), ev.coeff(2, 0));
		axis_y = Eigen::Vector3f(ev.coeff(0, 1), ev.coeff(1, 1), ev.coeff(2, 1));
		axis_z = Eigen::Vector3f(ev.coeff(0, 2), ev.coeff(1, 2), ev.coeff(2, 2));
		qx.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_x);
		qy.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_y);
		qz.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_z);
	}

	//std::cout << " Matrice de rotation: " << ev << endl;
	

	// Marker X axis
	marker.id = marker_id++;
	marker.scale.x = pca.getEigenValues().coeff(0) * scale+0.6;
	marker.pose.orientation.x = qx.x();
	marker.pose.orientation.y = qx.y();
	marker.pose.orientation.z = qx.z();
	marker.pose.orientation.w = qx.w();
	marker.color.b = 0.0;
	marker.color.g = 0.0;
	marker.color.r = 1.0;
	marker.color.a = 1.0;
	marker_array.markers.push_back(marker);

	// Marker Y axis
	marker.id = marker_id++;
	marker.scale.x = pca.getEigenValues().coeff(1) * scale+0.6;
	marker.pose.orientation.x = qy.x();
	marker.pose.orientation.y = qy.y();
	marker.pose.orientation.z = qy.z();
	marker.pose.orientation.w = qy.w();
	marker.color.b = 0.0;
	marker.color.g = 1.0;
	marker.color.r = 0.0;
	marker_array.markers.push_back(marker);

	// Marker Z axis
	marker.id = marker_id++;
	marker.scale.x = pca.getEigenValues().coeff(2) * scale+0.6;
	marker.pose.orientation.x = qz.x();
	marker.pose.orientation.y = qz.y();
	marker.pose.orientation.z = qz.z();
	marker.pose.orientation.w = qz.w();
	marker.color.b = 1.0;
	marker.color.g = 0.0;
	marker.color.r = 0.0;
	marker_array.markers.push_back(marker);
	g_marker_array_pub.publish(marker_array);
	
}


// GUIDED FILTER TO SMOOTH NOISY POINT CLOUD
void guidedFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, double epsilon){

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    kdtree.setEpsilon(epsilon);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
      pcl::PointXYZ searchPoint = cloud->points[i];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3)
      {

        pcl::PointCloud<pcl::PointXYZ>::Ptr neigbors(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::MatrixXd neighbors_as_matrix(3, pointIdxRadiusSearch.size());

        for (std::size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
        {
          neigbors->points.push_back(cloud->points[pointIdxRadiusSearch[j]]);
          neighbors_as_matrix(0, j) = cloud->points[pointIdxRadiusSearch[j]].x;
          neighbors_as_matrix(1, j) = cloud->points[pointIdxRadiusSearch[j]].y;
          neighbors_as_matrix(2, j) = cloud->points[pointIdxRadiusSearch[j]].z;
        }

        Eigen::Vector3d mean;
        mean = neighbors_as_matrix.rowwise().mean();
        neighbors_as_matrix.transposeInPlace();
        Eigen::MatrixXd centered = neighbors_as_matrix.rowwise() - neighbors_as_matrix.colwise().mean();
        Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(neighbors_as_matrix.rows() - 1);

        Eigen::MatrixXd e = (cov + epsilon * Eigen::MatrixXd::Identity(3, 3));
        e = e.inverse();

        Eigen::MatrixXd A = cov * e;
        Eigen::MatrixXd b = mean - A * mean;

        Eigen::Vector3d searchPointEigenType;
        searchPointEigenType[0] = searchPoint.x;
        searchPointEigenType[1] = searchPoint.y;
        searchPointEigenType[2] = searchPoint.z;

        searchPointEigenType = A * searchPointEigenType + b;

        searchPoint.x = searchPointEigenType[0];
        searchPoint.y = searchPointEigenType[1];
        searchPoint.z = searchPointEigenType[2];
        cloud->points[i] = searchPoint;
      }
    }

    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud);

}


int main(int argc, char **argv){

    ros::init(argc, argv, "get_orientation");

	ros::NodeHandle nh;

	g_marker_array_pub = nh.advertise<MarkerArray>("object_orientation_marker", 128);

	ros::Subscriber sub = nh.subscribe("/area_pointcloud", 1000, &callback);
	
	//ros::waitForShutdown();

	ros::spin();

	return 0;
}