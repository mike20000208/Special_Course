#pragma once
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <stdio.h>
#include <string>
#include <time.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
using namespace cv;
using namespace std;
using namespace std::chrono_literals;
using namespace rs2;
using namespace Eigen;
using namespace pcl;


void ProcessDone();


struct MyTime
{
	double time;
	bool is_ms;
};


MyTime get_duration(clock_t start, clock_t end, char func_name[]);


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points2PCL(const rs2::points& points);


pcl::visualization::PCLVisualizer::Ptr
Visualization(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers,
	Scalar color = (0, 0, 0),
	string window = "3D Viewer");


class Roughness
{
public:

	// attributes. 
	double a = 0, b = 0, c = 0, d = 0;
	vector<int> outliers;
	vector<double> rough;

	// methods
	Roughness();
	Roughness(Eigen::VectorXf& coefficients);
	Roughness(pcl::ModelCoefficients& coefficients);
	double get_distance(pcl::PointXYZRGB point);
	void get_Roughness(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
};


struct Slice
{
	double score = 0;
	cv::Vec3d centroid = cv::Vec3d(0.0, 0.0, 0.0);
	vector<int> indices;
};


class Score
{
public:

	// attributes. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	double maxX = 0, minX = 0;  // X board
	double x_len = 0;  // length of X
	double stride = 0.3, size = 0.6;  // stride along x-direction, window size
	double search_step = 1.0;  // step along z-direction
	double search_range = 5.0;  // search range within z-direction
	double num_slices = 0;  // number of slice along x-direction
	bool found_boundary = false, found_roi = false;
	double inlier_weight = 0.4, outlier_weight = 1.1, dis_weight = 1.7, angle_weight = 1.1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi;
	vector<Slice> slices;  // slices within specific z range
	cv::Vec3d dir = cv::Vec3d(-1.0, 0.0, 1.0);  // vector pointing the the destination
	vector<Slice> best_paths;  // best slice in each z range

	// methods
	Score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud);
	void setSearchRange(double z);
	void setSearchStep(double step);
	void setStride(double instride);
	void setSize(double insize);
	void setInlierWeight(double iw);
	void setOutlierWeight(double ow);
	void setDisWeight(double dw);
	void setDir(cv::Vec3d newDir);
	void setAngleWeight(double aw);
	void get_boundary(double z);
	void get_roi(double z);
	void get_slices_1(double z);
	void get_slices_2();
	void get_slices_3(double z);
	double get_angle(cv::Vec3d v);
	static double get_distance(cv::Vec3d v1, cv::Vec3d v2);
	bool get_score(double z, bool have_dst = true);
	bool find_best_path();
	void visualization(pcl::visualization::PCLVisualizer::Ptr viewer, 
		Slice slice);
};


bool PCL2CSV(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int num = 0, string type = "initial", bool save_complete = false, string path = "D:/Project_File/Matlab/SpecialCourse/CSVData/");


void PCL2PLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int num = 0, string type = "initial", string path = "./SavedPLY/");


void ImageSave(cv::Mat& img, int num = 0, string type = "stream", string path = "./SavedImages/");


bool TimeSave(vector<double> times, int num = 0, string type = "loop", string path = "./SavedData/");


class PointCloudProcess
{
public:
	// attributes
	string path;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr iniitalCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	Mat image;
	string mode;
	bool isDownSample;
	bool process_done;

	// methods
	PointCloudProcess();
	PointCloudProcess(string path);
	void setMode(string type);
	string getMode();
	void setIsDownSample(bool flag);
	bool getIsDownSample();
	//void getFrame();
	void PCProcessing();
	void distanceFilter(double dis);
	void planeFinding();
};
