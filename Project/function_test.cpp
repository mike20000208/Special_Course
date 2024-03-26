# include "function_test.h"


/*
* @brief Implement the plane segmentation tutorial. 
*/
void Plane_segmentation_tutorial()
{
	// create blank point cloud. 
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	pc->width = 15;
	pc->height = 1;
	pc->points.resize((pc->width) * (pc->height));

	// fill in the point cloud
	for (auto& point : *pc)
	{
		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1.0;
	}

	// create few outliers
	(*pc)[0].z = 2.0;
	(*pc)[3].z = -2.0;
	(*pc)[6].z = 4.0;

	// print point cloud data
	cout << "Point cloud data: " << pc->size() << " points" << std::endl;
	for (const auto& point : *pc)
		std::cerr << "    " << point.x << " "
		<< point.y << " "
		<< point.z << std::endl;

	// create segmentation object and set the model and method type. 
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setMaxIterations(1000);
	seg.setInputCloud(pc);
	seg.segment(*inliers, *coefficients);
}


/*
* @brief Implement random sample consensus model tutorial. 
*/
void RANSAC_model_turorial()
{
	// initialize the pointcloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

	// assign points in cloud. 
	cloud->width = 500;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	for (pcl::index_t i = 0; i < static_cast<pcl::index_t>(cloud->size()); i++)
	{
		(*cloud)[i].x = 1024 * rand() / (RAND_MAX + 1.0);
		(*cloud)[i].y = 1024 * rand() / (RAND_MAX + 1.0);

		if (i % 2 == 0)
		{
			(*cloud)[i].z = 1024 * rand() / (RAND_MAX + 1.0);
		}
		else
		{
			(*cloud)[i].z = -1 * ((*cloud)[i].x + (*cloud)[i].y);
		}
	}

	// create vector to save inliers. 
	vector<int> inliers;

	//// create RANSAC object and compute the appropriated model.
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
	ransac.setDistanceThreshold(.01);
	//ransac.setMaxIterations(1000);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copy all inliers of the model computed to another pointcloud. 
	pcl::copyPointCloud(*cloud, inliers, *final);
	
	// visualization
	pcl::visualization::PCLVisualizer::Ptr 
		viewer (new pcl::visualization::PCLVisualizer ("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(final, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(10000, "global");
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
}


/*
* @brief Convert realsense pointcloud to PCL pointcloud and visualize it (single frame) (by SampleConsensusModelPlane). 
* @param path path to *.bag file. 
*/
void Navigation_single_frame_1(string path)
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

	// initialize other variables.
	clock_t start, end;
	clock_t s, e;
	char path_planning[] = "Path planning";
	char finding[] = "Finding the best path";
	vector<int> inliers;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers;

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
	cv::imwrite("./SavedImages/scene.png", image);

	// calculate realsense pointcloud and convert it into PCL format. 
	printf("\n\nGenerating pointcloud...... \n\n");
	start = clock();
	points = pointcloud.calculate(depth_frame);
	cloud = Points2PCL(points);

	// filter the depth map with z-value. 
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	filter.setFilterFieldName("z");
	filter.setFilterLimits(0, 10);
	filter.filter(*cloud_filtered);

	// create RANSAC object and compute
	Eigen::VectorXf* coef = new Eigen::VectorXf;
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
		model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_filtered));
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
	ransac.setDistanceThreshold(.10);
	ransac.setMaxIterations(3000);
	ransac.setProbability(.80);  // default value is 0.99. 
	ransac.computeModel();
	ransac.getInliers(inliers);
	ransac.getModelCoefficients(*coef);

	// show the plane in dark green. 
	for (int n = 0; n < inliers.size(); n++)
	{
		cloud_filtered->points[inliers[n]].r = 0;
		cloud_filtered->points[inliers[n]].g = 127;
		cloud_filtered->points[inliers[n]].b = 0;
	}

	// calculate the roughness. 
	Roughness R(*coef);
	R.get_Roughness(*cloud_filtered);

	// show the roughness on the pointcloud in red gradient. 
	for (int i = 0; i < R.outliers.size(); i++)
	{
		(*cloud_filtered).points[R.outliers[i]].r = R.rough[i];
		(*cloud_filtered).points[R.outliers[i]].g = 0;
		(*cloud_filtered).points[R.outliers[i]].b = 0;
	}

	// calculate the best path. 
	s = clock();
	Score S(cloud_filtered); 
	S.setSearchRange(4.0);
	S.setSearchStep(0.70);
	S.setSize(0.80);
	S.setStride(0.5 * S.size);
	S.setInlierWeight(0.40);
	S.setOutlierWeight(1.65);
	S.setDisWeight(1.70);
	S.setAngleWeight(1.1);

	for (double z = 0.0; z < S.search_range; z += S.search_step)
	{
		S.get_boundary(z);
		S.get_slices_3(z);
		S.get_score(z);
		S.find_best_path();
	}

	// show the best path in the point cloud. 
	for (int k = 0; k < S.best_paths.size(); k++)
	{
		for (int n = 0; n < S.best_paths[k].indices.size(); n++)
		{
			(*cloud_filtered).points[S.best_paths[k].indices[n]].r = 0;
			(*cloud_filtered).points[S.best_paths[k].indices[n]].g = 255;
		}
	}

	// calculate the spent time
	end = clock();
	get_duration(start, end, path_planning);
	get_duration(s, end, finding);

	// visualization
	Scalar color(30, 30, 30);
	//Scalar color(0, 0, 0);
	layers.push_back(cloud_filtered);
	pcl::visualization::PCLVisualizer::Ptr viewer = Visualization(layers, color);

	while (!viewer->wasStopped())
	{
		imshow(window, image);
		waitKey(500);
		viewer->spinOnce(400);
		std::this_thread::sleep_for(100ms);

		if (GetAsyncKeyState(32) || GetAsyncKeyState(13))
		{
			printf("\n\nThe programme is terminated by keyboard. \n\n");
			break;
		}
	}

	destroyAllWindows();
	delete coef;
	pipeline.stop();
}


/*
* @brief Convert realsense pointcloud to PCL pointcloud and visualize it (single frame) (by SACSegmentation). 
* @param path path to *.bag file.
*/
void Navigation_single_frame_2(string path)
{
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		final(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	pcl::visualization::PCLVisualizer::Ptr viewer;

	// initialize other variables.
	clock_t start, end;
	double duration;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers;

	// start pipeline.
	pipeline.start(config);

	// take one frame. 
	frames = pipeline.wait_for_frames();
	depth_frame = frames.get_depth_frame();
	color_frame = frames.get_color_frame();

	// calculate realsense pointcloud and convert it into PCL format. 
	printf("\n\nGenerating pointcloud...... \n\n");
	start = clock();
	pointcloud.map_to(color_frame);
	points = pointcloud.calculate(depth_frame);
	cloud = Points2PCL(points);

	// filter the depth map with z-value. 
	filter.setInputCloud(cloud);
	filter.setFilterFieldName("z");
	filter.setFilterLimits(0, 10);
	filter.filter(*cloud_filtered);

	// create segmentation object and set the model and method type. 
	pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.10);
	seg.setMaxIterations(3000);
	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coef);

	// copy all inliers of the model computed to another pointcloud. 
	pcl::copyPointCloud(*cloud_filtered, *inliers, *final);

	// show the plane in green. 
	for (int i = 0; i < (*inliers).indices.size(); i++)
	{
		(*cloud_filtered).points[(*inliers).indices[i]].r = 0;
		(*cloud_filtered).points[(*inliers).indices[i]].b = 0;
	}

	// calculate the roughness. 
	Roughness R(*coef);
	R.get_Roughness(*cloud_filtered);

	// show the roughness on the pointcloud in red gradient. 
	for (int i = 0; i < R.outliers.size(); i++)
	{
		(*cloud_filtered).points[R.outliers[i]].r = R.rough[i];
		(*cloud_filtered).points[R.outliers[i]].g = 0;
		(*cloud_filtered).points[R.outliers[i]].b = 0;
	}

	// calculate the spent time. 
	end = clock();
	duration = double(end - start) / double(CLOCKS_PER_SEC);
	duration *= 1000;
	printf("\n\nGenerting recommended path takes %f ms. \n\n", duration);

	// visualization
	layers.push_back(cloud_filtered);
	//layers.push_back(cloud);
	//layers.push_back(final);
	viewer = Visualization(layers);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}

	pipeline.stop();
}


/*
* @brief Convert realsense pointcloud to PCL pointcloud and visualize it (streaming).
* @param path path to *.bag file.
* @param show_image whether to show the color image. 
*/
void Navigation_stream_test(string path, bool show_image)
{
	// initialize cv objects.
	Mat image;
	const string window = "Color Image";
	cv::namedWindow(window, WINDOW_NORMAL);
	cv::moveWindow(window, 900, 300);
	
	// initialize rs2 objects. 
	rs2::pipeline pipeline;
	rs2::config config;
	rs2::pointcloud pointcloud;
	rs2::points points;
	config.enable_device_from_file(path, false);
	rs2::frameset* frames = new rs2::frameset;
	//rs2::frameset frames;
	rs2::frame depth_frame, color_frame;

	// initialize pcl objects. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::visualization::PCLVisualizer::Ptr
		viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(30, 30, 30);
	viewer->setPosition(50, 70);
	viewer->addCoordinateSystem(10, "global");
	viewer->initCameraParameters();

	// initialize other variables.
	vector<int> inliers;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers;
	clock_t start, end;
	char process[] = "Path planning";
	int cnt = 0;
	vector<double> times;

	// start pipeline.
	pipeline.start(config);

	// start streaming. 
	bool flag;

	while (1)
	{
		flag = pipeline.try_wait_for_frames(frames);

		if (!flag)
		{
			break;
		}

		depth_frame = frames->get_depth_frame();
		color_frame = frames->get_color_frame();

		// create color image. 
		const int w = color_frame.as<rs2::video_frame>().get_width();
		const int h = color_frame.as<rs2::video_frame>().get_height();
		image = Mat(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

		// calculate realsense pointcloud and convert it into PCL format. 
		points = pointcloud.calculate(depth_frame);
		cloud = Points2PCL(points);

		// filter the depth map with z-value. 
		pcl::PassThrough<pcl::PointXYZRGB> filter;
		filter.setInputCloud(cloud);
		filter.setFilterFieldName("z");
		filter.setFilterLimits(0, 10);
		filter.filter(*cloud_filtered);

		// create RANSAC object and compute
		Eigen::VectorXf* coef = new Eigen::VectorXf;
		pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
			model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_filtered));
		pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
		ransac.setDistanceThreshold(0.10);
		ransac.setMaxIterations(3000);
		ransac.computeModel();
		ransac.getInliers(inliers);
		ransac.getModelCoefficients(*coef);

		// show the plane in dark green. 
		for (int n = 0; n < inliers.size(); n++)
		{
			cloud_filtered->points[inliers[n]].r = 0;
			cloud_filtered->points[inliers[n]].g = 127;
			cloud_filtered->points[inliers[n]].b = 0;
		}

		// calculate the roughness. 
		Roughness R(*coef);
		R.get_Roughness(*cloud_filtered);

		// show the roughness on the pointcloud in red-scale. 
		for (int i = 0; i < R.outliers.size(); i++)
		{
			(*cloud_filtered).points[R.outliers[i]].r = R.rough[i];
			(*cloud_filtered).points[R.outliers[i]].g = 0;
			(*cloud_filtered).points[R.outliers[i]].b = 0;
		}

		// calculate the best path. 
		start = clock();
		Score S(cloud_filtered);
		S.setSearchRange(4.0);
		S.setSearchStep(0.5);
		S.setSize(0.8);
		S.setStride(0.5 * S.size);
		S.setInlierWeight(0.4);
		S.setOutlierWeight(1.65);
		S.setDisWeight(1.75);  // the best value in experiment is 1.70. 
		S.setAngleWeight(1.10);

		for (double z = 0.0; z < S.search_range; z += S.search_step)
		{
			// the most successful method. 
			S.get_boundary(z);
			S.get_slices_3(z);
			S.get_score(z);
			S.find_best_path();
		}

		// show the best path in bright green in the point cloud. 
		for (int k = 0; k < S.best_paths.size(); k++)
		{
			for (int n = 0; n < S.best_paths[k].indices.size(); n++)
			{
				(*cloud_filtered).points[S.best_paths[k].indices[n]].r = 0;
				(*cloud_filtered).points[S.best_paths[k].indices[n]].g = 255;
			}
		}

		end = clock();
		get_duration(start, end, process);

		// save the generated point cloud as *.ply file and corresponding image. 
		if (cnt % 5 == 0)
		{
			PCL2PLY(cloud_filtered, cnt, "0.5", "./SavedPLY/inlier/");
			ImageSave(image, cnt, "0.5", "./SavedImages/inlier/");
		}


		// visualization
		layers.push_back(cloud_filtered);
		for (int i = 0; i < layers.size(); i++)
		{
			viewer->addPointCloud(layers[i], to_string(i));
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, to_string(i));
		}

		if (show_image)
		{
			imshow(window, image);
			waitKey(600);
		}

		viewer->spinOnce(600);

		// reset
		inliers.clear();
		layers.clear();
		viewer->removeAllPointClouds();

		if (GetAsyncKeyState(32) || GetAsyncKeyState(13))
		{
			printf("\n\nThe programme is terminated by keyboard. \n\n");
			break;
		}

		cnt += 1;
	}

	destroyAllWindows();
	pipeline.stop();
	delete frames;
}


/*
* @brief Implement ConditionalRemoval filter tutorial. 
*/
void ConditionalRemoval_filter_tutorial()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	// fill the pointcloud.
	(*cloud).width = 5;
	(*cloud).height = 1;
	(*cloud).points.resize((*cloud).width * (*cloud).height);

	for (std::size_t i = 0; i < cloud->size(); ++i)
    {
		(*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		(*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		(*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
		(*cloud)[i].r = 255;
		(*cloud)[i].g = 255;
		(*cloud)[i].b = 255;
    }

	// build the condition. 
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr
		cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());

	(*cond).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.0)));

	(*cond).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 0.8)));

	// build the filter. 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> filter;
	filter.setCondition(cond);
	filter.setInputCloud(cloud);
	filter.setKeepOrganized(true);

	// apply the filter. 
	filter.filter(*cloud_filtered);

	// visualization. 
	Scalar color(0, 0, 0);
	const string window = "3D Viewer";
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers;
	layers.push_back(cloud);
	pcl::visualization::PCLVisualizer::Ptr viewer = Visualization(layers, color, window);

	while (!viewer->wasStopped())
	{
		(*viewer).spinOnce(1000);
		std::this_thread::sleep_for(100ms);
	}
}


/*
* @brief Show the potential path based on 3D pointcloud. 
* @param path path to *.bag file. 
*/
void Navigation(string path, bool show_image)
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
	rs2::frameset* frames = new rs2::frameset;
	rs2::frame depth_frame, color_frame;

	// initialize pcl objects. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	pcl::visualization::PCLVisualizer::Ptr
		viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(192, 192, 192);
	viewer->setPosition(50, 70);
	viewer->addCoordinateSystem(10, "global");
	viewer->initCameraParameters();

	// initialize other variables.
	vector<int> inliers;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers;

	// start pipeline.
	pipeline.start(config);

	// start streaming. 
	bool flag;

	while (true)
	{
		flag = pipeline.try_wait_for_frames(frames);

		if (!flag)
		{
			break;
		}

		depth_frame = frames->get_depth_frame();
		color_frame = frames->get_color_frame();

		// create color image.
		const int w = color_frame.as<rs2::video_frame>().get_width();
		const int h = color_frame.as<rs2::video_frame>().get_height();
		image = cv::Mat(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

		// calculate realsense pointcloud and convert it into PCL format. 
		pointcloud.map_to(color_frame);
		points = pointcloud.calculate(depth_frame);
		cloud = Points2PCL(points);

		// filter the depth map with z-value. 
		filter.setInputCloud(cloud);
		filter.setFilterFieldName("z");
		filter.setFilterLimits(0, 10);
		filter.filter(*cloud_filtered);

		// create RANSAC object and find the plane. 
		Eigen::VectorXf* coef = new Eigen::VectorXf;
		pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
			model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_filtered));
		pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
		ransac.setDistanceThreshold(0.10);
		ransac.setMaxIterations(3000);
		ransac.computeModel();
		ransac.getInliers(inliers);
		ransac.getModelCoefficients(*coef);

		// show the plane in green. 
		for (int n = 0; n < inliers.size(); n++)
		{
			cloud_filtered->points[inliers[n]].r = 0;
			//cloud_filtered->points[inliers[n]].g = 127;
			cloud_filtered->points[inliers[n]].b = 0;
		}

		// calculate the roughness. 
		Roughness R(*coef);
		R.get_Roughness(*cloud_filtered);

		// show the roughness on the pointcloud in red gradient. 
		for (int i = 0; i < R.outliers.size(); i++)
		{
			(*cloud_filtered).points[R.outliers[i]].r = R.rough[i];
			(*cloud_filtered).points[R.outliers[i]].g = 0;
			(*cloud_filtered).points[R.outliers[i]].b = 0;
		}

		// visualization
		layers.push_back(cloud_filtered);
		for (int i = 0; i < layers.size(); i++)
		{
			viewer->addPointCloud(layers[i], to_string(i));
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, to_string(i));
		}

		if (show_image)
		{
			imshow(window, image);
			waitKey(500);
		}

		viewer->spinOnce(500);

		// reset
		inliers.clear();
		layers.clear();
		viewer->removeAllPointClouds();

		if (GetAsyncKeyState(32) || GetAsyncKeyState(13))
		{
			printf("\n\nThe programme is terminated by keyboard. \n\n");
			break;
		}

		delete coef;
	}

	destroyAllWindows();
	pipeline.stop();
	delete frames;
}


/*
* @brief Test the voxel downsampling. 
* @param path path to steam file. 
*/
void Voxel_test(string path)
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

	// initialize other variables.
	clock_t start, end;
	clock_t s, e;
	char path_planning[] = "Path planning";
	char finding[] = "Finding the best path";
	vector<int> inliers;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers;

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
	cv::imwrite("./SavedImages/scene.png", image);

	// calculate realsense pointcloud and convert it into PCL format. 
	printf("\n\nGenerating pointcloud...... \n\n");
	start = clock();
	points = pointcloud.calculate(depth_frame);
	cloud = Points2PCL(points);

	// create the filter object. 
	pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
	//printf("Successful! ");
	voxel.setInputCloud(cloud);
	//voxel.setLeafSize(1.0f, 1.0f, 1.0f);
	voxel.setLeafSize(0.1f, 0.1f, 0.1f);
	//voxel.setLeafSize(0.01f, 0.01f, 0.01f);
	voxel.filter(*cloud_filtered);

	// visualization
	//Scalar color(30, 30, 30);
	Scalar color(0, 0, 0);
	layers.push_back(cloud_filtered);
	pcl::visualization::PCLVisualizer::Ptr viewer = Visualization(layers, color);

	while (!viewer->wasStopped())
	{
		imshow(window, image);
		waitKey(500);
		viewer->spinOnce(400);
		std::this_thread::sleep_for(100ms);

		if (GetAsyncKeyState(32) || GetAsyncKeyState(13))
		{
			printf("\n\nThe programme is terminated by keyboard. \n\n");
			break;
		}
	}

	destroyAllWindows();
	pipeline.stop();
}
