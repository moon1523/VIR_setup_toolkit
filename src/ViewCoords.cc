#include "ViewCoords.hh"

int View_Coordinates(int argc, char** argv)
{
	string cam_path = ".";
	if (argc > 2) {
		cam_path = string(argv[2]);
	}

	vector<cv::Affine3d> board_pose, cam_pose;
	vector<cv::viz::WCameraPosition> board_coords, cam_coords, cam_frustum;
	vector<vector<cv::Vec3d>> point_clouds;
	vector<vector<cv::Vec3b>> point_clouds_color;
	Read_Board_Pose(board_pose, board_coords, "MatrixTF.dat");
	Read_Camera_Pose(cam_pose, cam_coords, cam_frustum, cam_path);
	Read_Point_Clouds(point_clouds, point_clouds_color, cam_path);
	
	cv::viz::Viz3d window_est("Estimation Coordinate Frame");
	window_est.setBackgroundColor();
	window_est.registerKeyboardCallback(&keyboard_callback);

	cv::Affine3d view = cv::Affine3d((cv::Mat_<double>(3,3) << 1,0,0, 0,0,1, 0,-1,0), cv::Vec3d(0,0,0));
	window_est.setViewerPose(view);
	while(!window_est.wasStopped())
	{
		for (size_t i=0;i<board_pose.size();i++) {
			window_est.showWidget("Board"+to_string(i), board_coords[i], board_pose[i] );		
		}
		for (size_t i=0;i<cam_pose.size();i++) {
			window_est.showWidget("Cam"+to_string(i), cam_coords[i], cam_pose[i] );
			window_est.showWidget("Frustum"+to_string(i), cam_frustum[i], cam_pose[i]);
		}
		for (size_t i=0;i<point_clouds.size();i++) {
			cv::viz::WCloud cloud_widget( point_clouds[i], point_clouds_color[i] );
			window_est.showWidget( "point_cloud" + to_string(i), cloud_widget );
		}
		
		// isocenter will be set
		// cv::viz::WSphere center_widget2(Point3d(-5.66686,-30.0832,0), 200., 10, cv::viz::Color::cyan());
		cv::viz::WSphere center_widget(Point3d(0,0,0), 10., 10, cv::viz::Color::cyan());
		center_widget.setRenderingProperty(cv::viz::OPACITY, 0.3);
		// window_est.showWidget("center2", center_widget2);

		// cv::viz::WSphere center_widget(Point3d(-5.66686,-30.0832,0), 10., 10, cv::viz::Color::cyan());
		// center_widget.setRenderingProperty(cv::viz::OPACITY, 1.0);
		window_est.showWidget("center", center_widget);
		window_est.spinOnce(10000, true);
	}
	
	return EXIT_SUCCESS;
}

void Read_Board_Pose(vector<cv::Affine3d> &board_pose,
	                 vector<cv::viz::WCameraPosition> &board_coords,
                     string fileName)
{
	board_pose.push_back(cv::Affine3d(cv::Mat::eye(3,3,CV_64F), cv::Vec3d(0,0,0)));
	board_coords.push_back(cv::viz::WCameraPosition(100.0));
	ifstream ifs(fileName);
	if (!ifs.is_open()) {
		cerr << "MatrixTF is not opened" << endl;
	}
	string dump;
	while (getline(ifs, dump))
	{
		stringstream ss(dump);
		ss >> dump;
		if (dump == "m2d") {
			int mother, daughter;
			ss >> mother >> dump >> daughter;
			if (mother == 0) {
				double x,y,z,w;
				// Rotation
				cv::Mat R;
				getline(ifs, dump);
				stringstream sq(dump);
				sq >> dump >> x >> y >> z >> w;
				Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::Vector4d(x,y,z,w));			
				cv::eigen2cv(q.toRotationMatrix(), R);
				// Translation
				getline(ifs, dump);
				stringstream st(dump);
				st >> dump >> x >> y >> z;
				cv::Mat T = (cv::Mat_<double>(1,3) << x, y, z);
				board_pose.push_back(cv::Affine3d(R,T));
				board_coords.push_back(cv::viz::WCameraPosition(100.0));
			}
		}
	}
	ifs.close();
}

void Read_Camera_Pose(vector<cv::Affine3d> &cam_pose,
	                  vector<cv::viz::WCameraPosition> &cam_coords,
					  vector<cv::viz::WCameraPosition> &cam_frustum,
                      string pathName)
{
	char buffer[128];
	string cmd = "find "+pathName+" -name \"*isocenter_to_camera*.dat\"";
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) cerr << "popen faeild" << endl;

	vector<string> cam_files;
	while( !feof(pipe) ) {
		if (fgets(buffer, 128, pipe) != NULL) {
			string s(buffer);
			s.pop_back();
			cam_files.push_back(s);
		}
	}
	pclose(pipe);

	cout << "Camera pose lists"<< endl;
	for (auto itr: cam_files) {
		cout << itr << endl;
	}
	
	for (size_t i=0;i<cam_files.size();i++) 
	{
		ifstream ifs(cam_files[i]);
		if (!ifs.is_open()) {
			cerr << cam_files[i] << " is not opened" << endl;
			exit(1);
		}
		string dump;
		cv::Mat R,T;
		while ( getline(ifs,dump) ) {
			stringstream ss(dump);
			ss >> dump;
			if (dump == "q") {
				double x,y,z,w;
				ss >> x >> y >> z >> w;
				Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::Vector4d(x,y,z,w));			
				cv::eigen2cv(q.toRotationMatrix(), R);
				
			}
			else if (dump == "t") {
				double x,y,z;
				ss >> x >> y >> z;
				T = (cv::Mat_<double>(1,3) << x, y, z);
			}
		}
		ifs.close();
		cam_pose.push_back(cv::Affine3d(R,T));
		cam_coords.push_back(cv::viz::WCameraPosition(20.0));

		if ( cam_files[i].find("000") != string::npos ||
		     cam_files[i].find("k") != string::npos ) { // kinect
			cv::Matx33d K = cv::Matx33d( 612, 0, 637,  0, 612, 364,  0, 0, 1);
			cam_frustum.push_back(cv::viz::WCameraPosition(K, 20.0, cv::viz::Color::magenta()));
		}
		else { // zed
			cv::Matx33d K = cv::Matx33d( 534, 0, 646,  0, 534, 364,  0, 0, 1);
			cam_frustum.push_back(cv::viz::WCameraPosition(K, 20.0, cv::viz::Color::yellow()));
		}
		
	}
}

void Read_Point_Clouds(	vector<vector<cv::Vec3d>> &point_clouds,
	                    vector<vector<cv::Vec3b>> &point_clouds_color,
                        string pathName)
{
	char buffer[128];
	string cmd = "find "+pathName+" -name \"*.ply\"";
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) cerr << "popen faeild" << endl;

	vector<string> ply_files;
	while( !feof(pipe) ) {
		if (fgets(buffer, 128, pipe) != NULL) {
			string s(buffer);
			s.pop_back();
			ply_files.push_back(s);
		}
	}
	pclose(pipe);

	cout << "ply lists"<< endl;
	for (auto itr: ply_files) {
		cout << itr << endl;
	}

	string dump;
	int vertNum(0);
	vector<cv::Vec3d> pointVec;
	vector<cv::Vec3b> colorVec;

	for (size_t i=0; i<ply_files.size(); i++)
	{
		ifstream ifs(ply_files[i]);
		while (getline(ifs,dump)) 
		{
			stringstream ss(dump);
			ss >> dump;
			if (dump == "element") {
				ss >> dump >> vertNum;
			}
			else if (dump == "end_header") {
				for (int i=0; i<vertNum; i++) {
					getline(ifs,dump);
					stringstream sv(dump);
					double x,y,z; int r,g,b;
					sv >> x >> y >> z >> r >> g >> b;
					pointVec.push_back(cv::Vec3d(x*0.1,y*0.1,z*0.1));
					colorVec.push_back(cv::Vec3b(b,g,r));
				}
			}
		}
		ifs.close();
		point_clouds.push_back( pointVec );
		point_clouds_color.push_back( colorVec );
	}

}