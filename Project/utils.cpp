# include "utils.h"


/*
* @brief Show that the process is done.
*/
void ProcessDone()
{
	cout << "\n \n \nProcess Done. \n \n \n";
}


/*
* @brief Calculate the duration of a specific function.
* @param start the start time of the function. 
* @param end the end time of the function. 
* @paran func_name the name of the estimated function. 
*/
MyTime get_duration(clock_t start, clock_t end, char func_name[])
{
	MyTime time;
	double duration = double(end - start) / double(CLOCKS_PER_SEC);  // s

	if (duration < 1.0)
	{
		duration *= 1000;  // ms
		printf("\n\n%s takes %f ms. \n\n", func_name, duration);
		time.is_ms = true;
	}
	else
	{
		printf("\n\n%s takes %f s. \n\n", func_name, duration);
		time.is_ms = false;
	}
	time.time = duration;
	return time;
}


/*
* @brief Convert realsense pointcloud to pcl pointcloud.
* @param points pointcloud in realsense format.
* @return cloud pointcloud in pcl format.
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points2PCL(const rs2::points& points)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto vert = points.get_vertices();

	for (auto& p : cloud->points)
	{
		p.x = vert->x * (-1);
		p.y = vert->y * (-1);
		p.z = vert->z;
		p.r = 255;
		p.g = 255;
		p.b = 255;
		vert++;
	}

	return cloud;
}


/*
* @brief Create a simple pointcloud viewer.
* @param layers multiple pointcloud layers you want to display.
* @return viewer pointcloud viewer
*/
pcl::visualization::PCLVisualizer::Ptr
Visualization(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers,
	Scalar color,
	string window)
{
	pcl::visualization::PCLVisualizer::Ptr
		viewer(new pcl::visualization::PCLVisualizer(window));
	viewer->setBackgroundColor(color[0], color[1], color[2]);
	viewer->setPosition(50, 70);
	//viewer->addCoordinateSystem(max(cloud->width, cloud->height), "global");
	//viewer->addCoordinateSystem(10000, "global");
	viewer->addCoordinateSystem(10, "global");
	for (int i = 0; i < layers.size(); i++)
	{
		viewer->addPointCloud(layers[i], to_string(i));
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			4, to_string(i));
	}

	viewer->initCameraParameters();
	return viewer;
}


/*
* @brief Constructor of class Roughness.
*/
Roughness::Roughness()
{
	a = 0;
	b = 0;
	c = 0;
	d = 0;
}


/*
* @brief Constructor of class Roughness.
* @param coefficients plane model coefficients. 
*/
Roughness::Roughness(Eigen::VectorXf& coefficients)
{
	a = coefficients[0];
	b = coefficients[1];
	c = coefficients[2];
	d = coefficients[3];
}


/*
* @brief Constructor of class Roughness.
* @param coefficients plane model coefficients.
*/
Roughness::Roughness(pcl::ModelCoefficients& coefficients)
{
	a = coefficients.values[0];
	b = coefficients.values[1];
	c = coefficients.values[2];
	d = coefficients.values[3];
}


/*
* @brief Calculate the distance between point and plane.
* @param point one point from outliers. 
* @return distance distance between point and plane.
*/
double Roughness::get_distance(pcl::PointXYZRGB point)
{
	double distance = 0;
	distance = (abs(a * point.x + b * point.y + c * point.z + d)) / (sqrt(pow(a, 2.0) + pow(b, 2.0) + pow(c, 2.0)));
	return distance;
}


/*
* @brief Calculate the roughness of outliers of single frame. 
* @param cloud pointcloud with inliers in green. 
*/
void Roughness::get_Roughness(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
	vector<double> temp;
	outliers.clear();
	rough.clear();
	double d = 0;

	for (int i = 0; i < cloud.points.size(); i++)
	{
		if (cloud.points[i].r == 255 && cloud.points[i].b == 255)  // outliers
		{
			d = get_distance(cloud.points[i]);
			temp.push_back(d);
			outliers.push_back(i);
		}
	}
	normalize(temp, rough, 0, 255, cv::NORM_MINMAX, CV_64F);
}


/*
* @brief Constructor of class Score. 
*/
Score::Score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud)
{
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*incloud, *cloud);
	vector<int> X;

	// calculate the boudary for get_slice_1(). 
	for (auto& p : (*cloud).points)
	{
		X.push_back(p.x);
	}
}


/*
* @brief Set the searching range for the path planner. 
* @param z searching range. 
*/
void Score::setSearchRange(double range)
{
	Score::search_range = range;
}


/*
* @brief Set the searching step for the path planner.
* @param step searching step.
*/
void Score::setSearchStep(double step)
{
	Score::search_step = step;
}


/*
* @brief Set the moving stride for the path planner.
* @param instride stride for slicing.
*/
void Score::setStride(double instride)
{
	Score::stride = instride;
}


/*
* @brief Set the size of slice for the path planner.
* @param instride size of slice.
*/
void Score::setSize(double insize)
{
	Score::size = insize;
}


/*
* @brief Set the inlier-weight parameter of path planning. 
* @param iw inlier-weight parameter. 
*/
void Score::setInlierWeight(double iw)
{
	Score::inlier_weight = iw;
}


/*
* @brief Set the outlier-weight parameter of path planning.
* @param ow outlier-weight parameter.
*/
void Score::setOutlierWeight(double ow)
{
	Score::outlier_weight = ow;
}


/*
* @brief Set the distance-weight parameter of path planning.
* @param dw distance-weight parameter.
*/
void Score::setDisWeight(double dw)
{
	Score::dis_weight = dw;
}

/*
* @brief Set the direction of destinaiton. 
* @param dir direction of destination. 
*/
void Score::setDir(cv::Vec3d newDir)
{
	Score::dir = newDir;
}


/*
* @brief Set the angle-weight parameter of path planning.
* @param aw angle-weight parameter.
*/
void Score::setAngleWeight(double aw)
{
	Score::angle_weight = aw;
}


/*
* @brief Get the boundary of roi for both get_slice_1() and get_slice_2(). 
* @param z lower limit of z value.
*/
void Score::get_boundary(double z)
{
	// prepare some variables. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		slice_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> X;

	// build the condition.. 
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr
		condition(new pcl::ConditionAnd<pcl::PointXYZRGB>);
	(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z", 
												pcl::ComparisonOps::GE, 
												z)));
	(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z", 
												pcl::ComparisonOps::LT, 
												(z + Score::search_step))));

	// build the filter. 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> filter(true);  // default: extract_removed_indices = false
	filter.setCondition(condition);
	filter.setInputCloud(cloud);
	filter.setKeepOrganized(false);

	// apply the filter. 
	filter.filter(*slice_pc);

	// calculate the boudary for get_slice_2(). 
	for (auto& p : (*slice_pc).points)
	{
		X.push_back(p.x);
	}
	
	cv::minMaxLoc(X, &(Score::minX), &(Score::maxX));
	Score::x_len = Score::maxX - Score::minX;
	Score::num_slices = floor(Score::x_len / Score::stride) - 1;

	// set the flag
	Score::found_boundary = true;
}


/*
* @brief Get the roi as pointcloud for get_slice_2(). 
* @param z lower limit of z value.
*/
void Score::get_roi(double z)
{
	Score::roi = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	// build the condition. 
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr
		condition(new pcl::ConditionAnd<pcl::PointXYZRGB>);
	(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z",
												pcl::ComparisonOps::GE,
												z)));
	(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z",
												pcl::ComparisonOps::LT,
												(z + Score::search_step))));

	// build the filter. 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> filter;
	filter.setCondition(condition);
	filter.setInputCloud(cloud);
	filter.setKeepOrganized(true);

	// apply the filter. 
	filter.filter((*Score::roi));

	// set the flag
	Score::found_roi = true;
}


/*
* @brief Get the slices along x direction within specified z range. (by built-in function in PCL)
* @param z lower limit of z value. 
*/
void Score::get_slices_1(double z)
{
	// check the flag. 
	if (!Score::found_boundary)
	{
		cerr << "\n\nNeeds to find the boundary first! \n\n";
		exit(-1);
	}
	else
	{
		Score::found_boundary = false;
	}
	
	// reset attributes. 
	Score::slices.clear();

	// prepare some variables. 
	vector<int> X, Y, Z;

	for (int i = 0; i < num_slices; i++)
	{		
		// prepare some variables. 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			roi(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::IndicesConstPtr inliers;
		Slice slice;

		// build the condition. 
		pcl::ConditionOr<pcl::PointXYZRGB>::Ptr
			condition(new pcl::ConditionOr<pcl::PointXYZRGB>);
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("z", 
													pcl::ComparisonOps::LT, 
													z)));
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("z", 
													pcl::ComparisonOps::GE, 
													(z + Score::search_step))));
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("x",
													pcl::ComparisonOps::LT,
													(Score::minX + i * Score::stride))));
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("x",
													pcl::ComparisonOps::GE,
													(Score::minX + i * Score::stride + Score::size))));

		// build the filter. 
		pcl::ConditionalRemoval<pcl::PointXYZRGB> filter(true);  // default: extract_removed_indices = false
		filter.setCondition(condition);
		filter.setInputCloud(Score::cloud);
		filter.setKeepOrganized(true);

		// apply the filter. 
		filter.filter(*roi);
		inliers = filter.getRemovedIndices();

		if (inliers->size() == 0)  // there is no points in this slice. 
		{
			continue;
		}

		// save the slice. 
		slice.score = 0.0;
		slice.indices = (*inliers);
		
		for (int j = 0; j < (*inliers).size(); j++)
		{
			//slice.indices.push_back((*inliers)[j]);
			X.push_back((*cloud).points[(*inliers)[j]].x);
			Y.push_back((*cloud).points[(*inliers)[j]].y);
			Z.push_back((*cloud).points[(*inliers)[j]].z);
		}

		cv::Scalar cx = cv::sum(X);
		cv::Scalar cy = cv::sum(Y);
		cv::Scalar cz = cv::sum(Z);
		int len = X.size();
		slice.centroid = cv::Vec3d(double(cx[0] / len), double(cy[0] / len), double(cz[0] / len));
		Score::slices.push_back(slice);

		// reset 
		X.clear();
		Y.clear();
		Z.clear();
	}
}


/*
* @brief Get the slices along x direction within specified z range. This method relys on Score::get_boundary() and Score::get_roi(). 
*/
void Score::get_slices_2()
{
	// check the flag. 
	if (!Score::found_boundary || !Score::found_roi)
	{
		if (!Score::found_boundary)
		{
			cerr << "\n\nNeeds to find the boundary first! \n\n";
			exit(-1);
		}
		else
		{
			cerr << "\n\nNeeds to find the ROI first! \n\n";
			exit(-1);
		}
	}
	else
	{
		Score::found_boundary = false;
		Score::found_roi = false;
	}
	
	// reset attributes. 
	Score::slices.clear();

	// prepare some variables. 
	vector<int> X, Y, Z;

	for (int i = 0; i < num_slices; i++)
	{
		// prepare some variables. 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			slice_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::IndicesConstPtr inliers;
		Slice slice;

		// build the condition. 
		pcl::ConditionOr<pcl::PointXYZRGB>::Ptr
			condition(new pcl::ConditionOr<pcl::PointXYZRGB>);
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("x",
			pcl::ComparisonOps::LT,
			(Score::minX + i * Score::stride))));
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("x",
			pcl::ComparisonOps::GE,
			(Score::minX + i * Score::stride + Score::size))));

		// build the filter. 
		pcl::ConditionalRemoval<pcl::PointXYZRGB> filter(true);  // default: extract_removed_indices = false
		filter.setCondition(condition);
		filter.setInputCloud(Score::roi);
		filter.setKeepOrganized(true);

		// apply the filter. 
		filter.filter(*slice_pc);
		inliers = filter.getRemovedIndices();

		if (inliers->size() == 0)  // there is no points in this slice. 
		{
			continue;
		}

		// save the slice. 
		slice.score = 0.0;
		slice.indices = (*inliers);

		for (int j = 0; j < (*inliers).size(); j++)
		{
			X.push_back((*cloud).points[(*inliers)[j]].x);
			Y.push_back((*cloud).points[(*inliers)[j]].y);
			Z.push_back((*cloud).points[(*inliers)[j]].z);
		}

		cv::Scalar cx = cv::sum(X);
		cv::Scalar cy = cv::sum(Y);
		cv::Scalar cz = cv::sum(Z);
		int len = X.size();
		slice.centroid = cv::Vec3d(double(cx[0] / len), double(cy[0] / len), double(cz[0] / len));
		Score::slices.push_back(slice);

		// reset 
		X.clear();
		Y.clear();
		Z.clear();
	}
}


/*
* @brief Get the slice along x direction within specified z range. (by conventional iteration method)
*/
void Score::get_slices_3(double z)
{
	// check the flag. 
	if (!Score::found_boundary)
	{
		cerr << "\n\nNeeds to find the boundary first! \n\n";
		exit(-1);
	}
	else
	{
		Score::found_boundary = false;
	}

	// reset attributes. 
	Score::slices.clear();

	// prepare some variables. 
	vector<int> X, Y, Z;

	// slicing
	for (int i = 0; i < num_slices; i++)
	{
		// prepare some variables. 
		Slice slice;

		// set the condition to pick out the points. 
		for (int k = 0; k < Score::cloud->points.size(); k++)
		{
			if ((Score::cloud->points[k].z >= z) && 
				(Score::cloud->points[k].z < (z + Score::search_step)))
			{
				if ((Score::cloud->points[k].x >= (Score::minX + i * Score::stride)) &&
					(Score::cloud->points[k].x < (Score::minX + i * Score::stride + Score::size)))
				{
					slice.indices.push_back(k);
					X.push_back(Score::cloud->points[k].x);
					Y.push_back(Score::cloud->points[k].y);
					Z.push_back(Score::cloud->points[k].z);
				}
			}
		}

		// check slice. 
		if (slice.indices.size() == 0)
		{
			continue;
		}

		// save the slice. 
		slice.score = 0.0;
		cv::Scalar cx = cv::sum(X);
		cv::Scalar cy = cv::sum(Y);
		cv::Scalar cz = cv::sum(Z);
		int len = X.size();
		slice.centroid = cv::Vec3d(double(cx[0] / len), double(cy[0] / len), double(cz[0] / len));
		Score::slices.push_back(slice);

		// reset. 
		X.clear();
		Y.clear();
		Z.clear();
	}
}


/*
* @brief Calculate the angle between the given angle and desired direction. 
* @param v the vector pointing the centroid of a slice from the origin. 
* @return angle the angle between the given angle and desired direction in degree.
*/
double Score::get_angle(cv::Vec3d v)
{
	double angle = 0;
	double norm_v = sqrt(v.ddot(v));
	double norm_dir = sqrt(Score::dir.ddot(Score::dir));
	angle = acos((v.ddot(Score::dir)) / (norm_v * norm_dir));
	angle = angle * 180 / M_PI;
	return angle;
}


/*
* @brief Calculate the distance between two points. 
* @param v1 the first point. 
* @param v2 the second point. 
* @return distance distance between two points.
*/
double Score::get_distance(cv::Vec3d v1, cv::Vec3d v2)
{
	double distance = 0;
	cv::Vec3d v = v1 - v2;
	distance = sqrt(v.ddot(v));
	return distance;
}


/*
* @brief Calculate the score for each slice in second_slices. 
* @param z lower limit of z value. 
* @param have_dst whether to take the destination into account. 
*/
bool Score::get_score(double z, bool have_dst)
{
	bool is_Zero = false;

	if (Score::slices.size() != 0)
	{
		for (int i = 0; i < Score::slices.size(); i++)
		{
			for (int j = 0; j < Score::slices[i].indices.size(); j++)
			{
				if ((*Score::cloud).points[Score::slices[i].indices[j]].g == 127)  // inliers
				{
					Score::slices[i].score += Score::inlier_weight;
				}
				else if ((*Score::cloud).points[Score::slices[i].indices[j]].g == 0 &&
					(*Score::cloud).points[Score::slices[i].indices[j]].b == 0)  // outliers
				{
					//Score::slices[i].score -= 0.3;
					Score::slices[i].score +=
						((255 - (*Score::cloud).points[Score::slices[i].indices[j]].r) / 255) * Score::outlier_weight;
				}
				else
				{
					continue;
				}
			}

			// normalize the score by the number of points. 
			Score::slices[i].score = (Score::slices[i].score) / (Score::slices[i].indices.size());

			// check the distance between currrnt slice and the best slice from the last search. 
			if (Score::best_paths.size() != 0)
			{
				double distance = get_distance(Score::best_paths[Score::best_paths.size() - 1].centroid, Score::slices[i].centroid);
				double maxD = sqrt(pow(Score::search_step, 2) + pow(Score::x_len, 2));
				double minD = Score::search_step;
				distance = (distance - minD) / (maxD - minD);
				Score::slices[i].score -= distance * Score::dis_weight;
			}

			// if have destination to go, then take it into consideration when calculating the score. 
			if (have_dst)
			{
				double angle = get_angle(Score::slices[i].centroid);
				Score::slices[i].score -= (abs(angle) / 180) * Score::angle_weight;
			}
		}
		return is_Zero;
	}
	else
	{
		return !is_Zero;
	}
}


/*
* @brief Find the best path in the second slices. 
*/
bool Score::find_best_path()
{
	// prepare some vsriables. 
	bool found = true;
	double best_score = -9e9;
	int index = 0;
	Slice best_path;

	if (Score::slices.size() == 0)
	{
		return !found;
	}
	else
	{
		for (int i = 0; i < Score::slices.size(); i++)
		{
			if (Score::slices[i].score > best_score)
			{
				best_score = Score::slices[i].score;
				index = i;
			}
		}

		best_path.score = best_score;
		best_path.centroid = Score::slices[index].centroid;
		best_path.indices = Score::slices[index].indices;
		Score::best_paths.push_back(best_path);
		return found;
	}
}


/*
* @brief Visualize the slice to debug. 
* @param slice the slice or the roi that we want to see. 
*/
void Score::visualization(pcl::visualization::PCLVisualizer::Ptr viewer, 
	Slice slice)
{
	// reset
	(*viewer).removeAllPointClouds();
	
	// create the pointcloud in roi. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		roi(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud, slice.indices, *roi);

	// visualization. 
	(*viewer).addPointCloud(roi, "debug");
	(*viewer).setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "debug");

	while (!(*viewer).wasStopped())
	{
		(*viewer).spinOnce(1000);
		std::this_thread::sleep_for(100ms);

		if (GetAsyncKeyState(32) || GetAsyncKeyState(13))
		{
			printf("\n\nPress Space or Enter to see next slice. \n\n");
			break;
		}
	}
}


/*
* @brief Save point cloud information to *.csv file. 
* @param cloud the cloud you want to save.
* @param num number of *.csv file.
* @param path path to the designated folder.
* @param type type of point cloud.
*/
bool PCL2CSV(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int num, string type, bool save_complete, string path)
{
	bool flag = true;
	const string suffix = ".csv";
	string data_path = path + type + "_" + to_string(num) + suffix;
	fstream f(data_path, std::fstream::out);

	if (!f.is_open())
	{
		return !flag;
	}

	if (save_complete)
	{
		for (auto& p : cloud.points)
		{
			f << to_string(p.x) << ", " << to_string(p.y) << ", " << to_string(p.z) << ", " \
				<< to_string(p.r) << ", " << to_string(p.g) << ", " << to_string(p.b) << "\n";
		}
	}
	else
	{
		for (auto& p : cloud.points)
		{
			f << to_string(p.x) << ", " << to_string(p.y) << ", " << to_string(p.z) << "\n";
		}
	}

	f.close();
	printf("\n\nThe *.csv file of 3D point cloud data is saved! \n\n");
	return flag;
}


/*
* @brief Save point cloud to *.ply file.
* @param cloud the cloud you want to save.
* @param num number of *.ply file.
* @param path path to the designated folder.
* @param type type of point cloud.
*/
void PCL2PLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int num, string type, string path)
{
	const string suffix = ".ply";
	const string data_path = path + type + "_" + to_string(num) + suffix;
	pcl::PLYWriter writer;
	writer.write(data_path, *cloud);
	printf("\n\nThe *.ply file of 3D point cloud is saved! \n\n");
}


/*
* @brief Save provided image. 
* @param img image that want to save. 
* @param num number of *.png file.
* @param path path to the designated folder.
* @param type type of point cloud.
*/
void ImageSave(cv::Mat& img, int num, string type, string path)
{
	const string suffix = ".png";
	const string data_path = path + type + "_" + to_string(num) + suffix;
	cv::imwrite(data_path, img);
	printf("\n\nThe *.png file of image is saved! \n\n");
}


/*
* @brief Save the spending time of the path planning. 
* @param times a number of spending time that want to be saved.
* @param num number of *.csv file.
* @param path path to the designated folder.
* @param type type of time.
*/
bool TimeSave(vector<double> times, int num, string type, string path)
{
	bool flag = true;
	const string suffix = ".csv";
	string data_path = path + type + "_" + to_string(num) + suffix;
	fstream f(data_path, std::fstream::out);

	if (!f.is_open())
	{
		return !flag;
	}

	for (int i = 0; i < times.size(); i++)
	{
		f << to_string(times[i]) << "\n";
	}

	f.close();
	printf("\n\nThe *.csv file of spending time is saved! \n\n");
	return flag;
}


/*
* @brief Constructor of class PointCloudProcess. 
* @param path
*/
PointCloudProcess::PointCloudProcess(string path)
{
	PointCloudProcess::path = path;
	PointCloudProcess::mode = "single";
	PointCloudProcess::isDownSample = false;
	PointCloudProcess::process_done = true;
	PointCloudProcess::iniitalCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	PointCloudProcess::cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

}


/*
* @brief Set the mode of the processing.
* @param type mode of processing.
*/
void PointCloudProcess::setMode(string type)
{
	PointCloudProcess::mode = type;
}


/*
* @brief Get the mode of the processing.
*/
string PointCloudProcess::getMode()
{
	return PointCloudProcess::mode;
}



/*
* @brief Determime whether to downsample the point cloud.
* @param flag flag to determine whether to downsample.
*/
void PointCloudProcess::setIsDownSample(bool flag)
{
	PointCloudProcess::isDownSample = flag;
}


/*
* @brief Get the indicator of downsampling or not.
*/
bool PointCloudProcess::getIsDownSample()
{
	return PointCloudProcess::isDownSample;
}



/*
* @brief Build a point cloud in PCl format. (downsample or not)
*/
void PointCloudProcess::PCProcessing()
{
	// initialize cv objects.
	const string window = "Color Image";
	cv::namedWindow(window, WINDOW_NORMAL);
	cv::moveWindow(window, 900, 300);
	cv::Mat image;

	// initialze rs2 objects. 
	rs2::pipeline pipeline;
	rs2::config config;
	rs2::pointcloud pointcloud;
	rs2::points points;
	config.enable_device_from_file(path, false);
	rs2::frameset frames;
	rs2::frame depth_frame, color_frame;

	// initialize pcl objects. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	// start pipeline.
	pipeline.start(config);

	// take one frame. 
	frames = pipeline.wait_for_frames();
	depth_frame = frames.get_depth_frame();
	color_frame = frames.get_color_frame();

	// create color image.
	const int w = color_frame.as<rs2::video_frame>().get_width();
	const int h = color_frame.as<rs2::video_frame>().get_height();
	image = cv::Mat(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	PointCloudProcess::image = image;
	//cv::imwrite("./SavedImages/scene.png", image);

	// calculate realsense pointcloud and convert it into PCL format. 
	printf("\n\nGenerating pointcloud...... \n\n");
	points = pointcloud.calculate(depth_frame);
	cloud = Points2PCL(points);
	PointCloudProcess::iniitalCloud = cloud;
}


/*
* @brief
* @param
*/





/*
* @brief
* @param
*/
