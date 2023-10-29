#include "SyncCamera.hh"

int Sync_ZED2(int argc, char** argv) 
{
	cout << "[Sync_ZED2]" << endl;
	cout << "[Hot key]" << endl;
	cout << "  f: switch camera mode (fixed / connector)" << endl;
	cout << "  t: tick accumulation" << endl;
	cout << "  r: export point cloud (transformed to reference board)" << endl;
	cout << "  q: quit" << endl;
	cout << "[Extra hot key]" << endl;
	cout << "  c: clear accumulating data" << endl;
	cout << "  w: write transformation matrix" << endl;
	cout << "  s: set detecting and cumulating number" << endl;
	cout << endl;

	vector<sl::DeviceProperties> devList = sl::Camera::getDeviceList();
	if (devList.size() == 0) {
		cout << "No ZED2 camera detected" << endl;
		return EXIT_FAILURE;
	}
	int nb_detected_zed = devList.size();
	cout << "Found " << nb_detected_zed << " connected devices:" << endl;
	for (int z=0; z<nb_detected_zed; z++) {
		cout << "ID : " << devList[z].id << ", model : " << devList[z].camera_model << " , S/N : " << devList[z].serial_number << " , state : "<<devList[z].camera_state<< endl;
	
		// Create ZED2 camera
		sl::Camera zed;

		// Init parameters
		sl::InitParameters init_parameters;
		init_parameters.input.setFromCameraID(devList[z].id);
		init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
		init_parameters.sdk_verbose = true;
		init_parameters.camera_resolution = sl::RESOLUTION::HD2K;
		init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
		init_parameters.coordinate_units = sl::UNIT::MILLIMETER;

		auto returned_state  = zed.open(init_parameters);
		if (returned_state != sl::ERROR_CODE::SUCCESS) {
			cout << returned_state << endl;
			return EXIT_FAILURE;
		}
		
		// Camera information
		sl::CameraInformation camera_informations = zed.getCameraInformation();
		string serial_number = to_string(camera_informations.serial_number);
		
		// ChArUco board
		int boardNo = 3;
		int boardType = (int)BoardType::HYUMC;
		double scalingFactor = 0.6;
		string camParm = "./data/" + serial_number + "_HD2K_left_intrinsic.yml";
		string detParm("./data/detector_params.yml");
		CoordMatch sync(boardType, boardNo);
		sync.SetParameters(camParm, detParm);
		sync.SetScalingFactor(scalingFactor);
		sync.SetFixedCameraSerialNumber(serial_number);

		sl::Mat slMat;	
		int frameNo(0);
		while (true) {
			returned_state = zed.grab();
			if (returned_state == sl::ERROR_CODE::SUCCESS) {
				zed.retrieveImage(slMat, sl::VIEW::LEFT);
				cv::Mat cvMatRGBA(slMat.getHeight(), slMat.getWidth(), CV_8UC4, slMat.getPtr<sl::uchar1>(sl::MEM::CPU));
				cv::Mat cvMat;
				cv::cvtColor(cvMatRGBA, cvMat, cv::COLOR_RGBA2RGB);
	
				sync.EstimatePose(cvMat);
				sync.Render();

				char key = (char)cv::waitKey(1);
				if (key == 'f') {
					sync.ClearData();
					sync.TickFixedCameraMode();
				}
				else if (key == 'c')
					sync.ClearData();
				else if (key == 't')
					sync.TickSwitch();
				else if (key == 'q') {
					cv::imwrite("./sync/" + serial_number + "_left.png", cvMat);
					break;
				}
				else if (key == 'w') {
					Write_Fixed_Camera_Transformation_Matrix(serial_number);
				}
				else if (key == 'r') {
					WriteTrasnformedPointCloudToRefBoard(zed, slMat, serial_number);
				}
				else if (key == 's') {
					int detectingNo, cumulatingNo;
					cout << "Set detecting #: "; cin >> detectingNo;
					cout << "Set cumulating #: "; cin >> cumulatingNo;
					sync.SetDetectingNo(detectingNo);
					sync.SetCumulatingNo(cumulatingNo);
				}
				frameNo++;

				slMat.free();
				cvMat.release();
				cvMatRGBA.release();
			}		
		}
		// Close the camera
		zed.close();
	
	}
	
	return EXIT_SUCCESS;
}


int Sync_Webcam(int argc, char** argv) {

	int boardNo = 3;
	int boardType = (int)BoardType::HYUMC;
	int image_width(0);
	int image_height(0);
	double scalingFactor = 1.0;
	string camParm;
	string webcam;
	for (int i=0;i < argc;i++) {
		if (string(argv[i]) == "-num") {
			boardNo = stoi(string(argv[i+1]));
			i++;
		}
		if (string(argv[i]) == "-uhd2160") {
			image_width = 3840;
			image_height = 2160;
			scalingFactor = 0.4;
			webcam = "UHD2160";
			camParm = "./data/newcam_UHD_intrinsic.yml";
		}
		if (string(argv[i]) == "-apc930") {
			image_width = 2592;
			image_height = 1944;
			scalingFactor = 0.6;
			webcam = "APC930";
			camParm = "./data/webcam_QHD_intrinsic.yml";
		}
	}

	string device_list;
    FILE* pipe = popen("v4l2-ctl --list-devices", "r");
    if (pipe == nullptr) {
        cerr << "Error: Could not execute v4l2-ctl command." << endl;
        exit(1);
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        device_list += buffer;
    }
    pclose(pipe);

    stringstream ss(device_list);
    string line, dump;
	int cam_id(0);
    while (ss >> dump) {
        if (dump == webcam) {
            getline(ss, line);
            getline(ss, line);
            stringstream ss2(line);
            ss2 >> dump;
            cam_id = stoi(dump.substr(string("/dev/video").size(), dump.size()));
        }
    }
	
	cv::VideoCapture cap;
	cap.open(cam_id, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
	cap.set(cv::CAP_PROP_FRAME_WIDTH,  image_width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
	cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
	cout << "Width : " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
	cout << "Height: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
	cout << "FPS   : " << cap.get(cv::CAP_PROP_FPS) << endl;
	
	if (!cap.isOpened()) {
		cerr << "Error! Unable to open camera" << endl;
		return 1;
	}

    // ChArUco board
    string detParm("./data/detector_params.yml");
    CoordMatch sync(boardType, boardNo);
    sync.SetParameters(camParm, detParm);
    sync.SetScalingFactor(scalingFactor);
	sync.SetFixedCameraSerialNumber(to_string(cam_id));

    int frameNo(0);
    while(true)
    {
        cv::Mat color_mat;
		cap >> color_mat;
		
        sync.EstimatePose(color_mat);
        sync.Render();

        char key = (char)cv::waitKey(1);
		if (key == 'f') {
			sync.ClearData();
			sync.TickFixedCameraMode();
		}
        else if (key == 'c')
			sync.ClearData();
		else if (key == 't')
			sync.TickSwitch();
        else if (key == 'q') {
			cv::imwrite("./sync/" + to_string(cam_id) + ".png", color_mat);
            break;
		}
		else if (key == 'w') {
			Write_Fixed_Camera_Transformation_Matrix(to_string(cam_id));
		}
		else if (key == 's') {
			int detectingNo, cumulatingNo;
			cout << "Set detecting #: "; cin >> detectingNo;
			cout << "Set cumulating #: "; cin >> cumulatingNo;
			sync.SetDetectingNo(detectingNo);
			sync.SetCumulatingNo(cumulatingNo);
		}

        frameNo++;
	}
	cv::destroyAllWindows();
	cout << "camera pose is written" << endl;
	cap.release();
	
	return EXIT_SUCCESS;
}

#ifdef K4A_FOUND
int Sync_Kinect(int argc, char** argv) 
{	
	cout << "[Sync_Kinect]" << endl;
	cout << "Select color resolution" << endl;
	cout << "  1. 4096 x 3072 (ChArUco board)" << endl;
	cout << "  2. 1280 x 720  (point cloud)" << endl;
	int opt(0);
	cin >> opt;
	k4a_color_resolution_t color_resolution;
	double scalingFactor;
	switch (opt) {
		case 1:
			color_resolution = K4A_COLOR_RESOLUTION_3072P;
			scalingFactor = 0.3;
			break;
		case 2:
			color_resolution = K4A_COLOR_RESOLUTION_720P;
			scalingFactor = 1.0;
			break;
		default:
			cout << "Invalid option" << endl;
			return EXIT_FAILURE;
	}
	int boardType = (int)BoardType::HYUMC;
	int boardNo = 3;
	for (int i=0;i < argc;i++) {
		if (string(argv[i]) == "-num") {
			boardNo = stoi(string(argv[i+1]));
			i++;
		}
	}
	
    // Azure Kinect
    //
    // Start camera
	int device_count = k4a_device_get_installed_count();
	cout << "Found " << device_count << " connected devices:" << endl;
	cout << "[Main hot key]" << endl;
	cout << "  f: switch camera mode (fixed / connector)" << endl;
	cout << "  t: tick accumulation" << endl;
	cout << "  e: export point cloud (raw)" << endl;
	cout << "  r: export point cloud (transformed to reference board)" << endl;
	cout << "  q: quit" << endl;
	cout << "[Extra hot key]" << endl;
	cout << "  c: clear accumulating data" << endl;
	cout << "  a: show average value" << endl;
	cout << "  w: write transformation matrix" << endl;
	cout << "  s: set detecting and cumulating number" << endl;
	cout << endl;

	for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
	{
		k4a_device_t device = nullptr;
		VERIFY(k4a_device_open(deviceIndex, &device), "Open K4A Device failed!");
		char *serial_number = NULL;
		size_t serial_number_length = 0;
		k4a_device_get_serialnum(device, NULL, &serial_number_length);
		serial_number = (char*)malloc(serial_number_length);
		k4a_device_get_serialnum(device, serial_number,&serial_number_length);

		// Start camera. Make sure depth camera is enabled.
		k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		deviceConfig.color_resolution = color_resolution;
		deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;
		deviceConfig.subordinate_delay_off_master_usec = 0;
		deviceConfig.synchronized_images_only = true;
		VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

		// Get calibration information
		k4a_calibration_t sensorCalibration;
		VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration), "Get depth camera calibration failed!");

		k4a_capture_t sensorCapture = NULL;
		k4a_image_t color_image = NULL;
		k4a_image_t depth_image = NULL;
		k4a_image_t point_image = NULL;
		k4a_image_t colorlike_depth_image = NULL;
		k4a_transformation_t transformation = NULL;
		transformation = k4a_transformation_create(&sensorCalibration);

		int image_width  = sensorCalibration.color_camera_calibration.resolution_width;
		int image_height = sensorCalibration.color_camera_calibration.resolution_height;

		// ChArUco board
		string detParm("./data/detector_params.yml");
		string camParm;
		if (opt == 1) camParm = "./data/"+string(serial_number)+"_HXGA_intrinsic.yml";
		if (opt == 2) camParm = "./data/"+string(serial_number)+"_HD_intrinsic.yml";
		CoordMatch sync(boardType, boardNo);
		sync.SetParameters(camParm, detParm);
		sync.SetScalingFactor(scalingFactor);
		sync.SetFixedCameraSerialNumber(string(serial_number));

		int frameNo(0);
		while(true)
		{
			k4a_device_get_capture(device, &sensorCapture, 1000);
			color_image = k4a_capture_get_color_image(sensorCapture);
			depth_image = k4a_capture_get_depth_image(sensorCapture);
			k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, image_width, image_height, image_width * 3 * (int)sizeof(uint16_t), &point_image);
			k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, image_width, image_height, image_width * (int)sizeof(uint16_t), &colorlike_depth_image);
			k4a_transformation_depth_image_to_color_camera(transformation, depth_image, colorlike_depth_image);
			k4a_transformation_depth_image_to_point_cloud(transformation, colorlike_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_image);

			cv::Mat color_mat = color_to_opencv(color_image);
			sync.EstimatePose(color_mat);
			sync.Render();

			char key = (char)cv::waitKey(1);
			if (key == 'f') {
				sync.ClearData();
				sync.TickFixedCameraMode();
			}
			else if (key == 'e') {
				WritePointCloud(point_image, color_image, (string)serial_number);
			}
			else if (key == 'r') {
				WriteTrasnformedPointCloudToRefBoard(point_image, color_image, "./sync/" + (string)serial_number);
			}
			else if (key == 'c')
				sync.ClearData();
			else if (key == 't')
				sync.TickSwitch();
			else if (key == 'q') {
				cv::imwrite("./sync/" + (string)serial_number + ".png", color_mat);
				break;
			}
			else if (key == 'a')
			{
				sync.ShowAvgValue(color_mat);
				char key2 = waitKey(0);
				if(key2=='q') break;
			}
			else if (key == 'w') {
				Write_Fixed_Camera_Transformation_Matrix((string)serial_number);
			}
			else if (key == 's') {
				int detectingNo, cumulatingNo;
				cout << "Set detecting #: "; cin >> detectingNo;
				cout << "Set cumulating #: "; cin >> cumulatingNo;
				sync.SetDetectingNo(detectingNo);
				sync.SetCumulatingNo(cumulatingNo);
			}

			k4a_capture_release(sensorCapture);
			k4a_image_release(color_image);
			k4a_image_release(depth_image);
			k4a_image_release(colorlike_depth_image);
			k4a_image_release(point_image);
			frameNo++;
		}
		k4a_transformation_destroy(transformation);
		k4a_device_close(device);
		cout << "k4a device was succesfully closed" << endl;
	}


	return EXIT_SUCCESS;
}
#endif

int Batch_Write_Camera_Pose(int argc, char** argv)
{
	vector<string> sn_vec;
	vector<Quaterniond> q_vec;
	vector<Vector3d> t_vec;

	string isocenter_file;
	if (argc > 2) isocenter_file = string(argv[2]);
	string pathName = "./sync";
	char buffer[128];
	string cmd = "find "+pathName+" -name \"*fixedMode*.dat\"";
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) cerr << "popen faeild" << endl;

	vector<string> cam_files;
	while( !feof(pipe) ) {
		if (fgets(buffer, 128, pipe) != NULL) {
			string s(buffer);
			s.pop_back();
			cam_files.push_back(s);
			if (s.find("_") != string::npos) {
				sn_vec.push_back( s.substr(7, s.find("_")-7) );
			}
		}
	}
	pclose(pipe);

	cout << "fixedMode.dat lists"<< endl;
	for (auto itr: cam_files) {
		cout << itr << endl;
	}

	string dump;
	int mainBoardNo(-1);

	Quaterniond q_fixed, q_conn;
	Vector3d t_fixed, t_conn;
	Eigen::Affine3d a_fixed, a_conn, a;
	
	for (auto file: cam_files) { 
		ifstream ifs(file);
		if (!ifs.is_open()) { 
			cerr << file << " was not opened. Run the 'fixed camera mode' again." << endl;
			exit(1);
		}
		string dump;
		int mainBoardNo(-1);
		
		while ( getline(ifs,dump) ) {
			stringstream ss(dump);
			ss >> dump;
			if (dump == "mainBoardNo:") {
				ss >> mainBoardNo;
			}
			else if (dump == "q(xyzw)") {
				double x,y,z,w;
				ss >> x >> y >> z >> w;
				q_fixed = Quaterniond(w,x,y,z);
			}
			else if (dump == "t(xyz,cm)") {
				double x,y,z;
				ss >> x >> y >> z;
				t_fixed = Vector3d(x,y,z);
			}
		}
		a_fixed.linear() = q_fixed.normalized().matrix();
		a_fixed.translation() = t_fixed;
		ifs.close();

		if (mainBoardNo == 0) {
			a = a_fixed.inverse();
		}
		else {
			ifs.open("MatrixTF.dat");
			if (!ifs.is_open()) { cerr << "'MatrixTF.dat' was not opened. Run the 'connector mode'" << endl; exit(1); }
			while(getline(ifs,dump)) {
				stringstream ss(dump);
				ss >> dump;
				if (dump == "m2d") {
					int mother, daughter;
					ss >> mother >> dump >> daughter;
					if (mother == 0 && daughter == mainBoardNo) {
						double x,y,z,w;
						getline(ifs, dump);
						stringstream sq(dump);
						sq >> dump >> x >> y >> z >> w;
						q_conn = Quaterniond(w,x,y,z);
						getline(ifs, dump);
						stringstream st(dump);
						st >> dump >> x >> y >> z;
						t_conn = Vector3d(x,y,z);
						a_conn.linear() = q_conn.normalized().matrix();
						a_conn.translation() = t_conn;
						break;
					}
				}
				
			}
			ifs.close();
			a = a_conn * a_fixed.inverse();
		}
		cout << "-----------------------" << endl;
		
		Vector3d isocenter_calibration(0,0,0);
		bool isIso(false);
		ifs.open(isocenter_file);
		if (!ifs.is_open()) { cerr << "There is no isocenter file" << endl; }
		while (getline(ifs, dump)) {
			stringstream ss(dump);
			ss >> dump;
			if (dump == "Center_calib(cm):") {
				double x,y,z;
				ss >> x >> y >> z;
				isocenter_calibration = Vector3d(x,y,z);
				isIso = true;
				cout << "Find sphere fitting file !!" << endl; 
				cout << "Isocetner position from reference board (xyz,cm): " << isocenter_calibration.transpose() << endl;
				break;
			}
			else if (dump == "Translation_calib(xyz,cm):") {
				double x,y,z;
				ss >> x >> y >> z;
				isocenter_calibration = Vector3d(x,y,z);
				isIso = true;
				cout << "Find position file !!" << endl; 
				cout << "Isocetner position from reference board (xyz,cm): " << isocenter_calibration.transpose() << endl;
				break;
			}
		}
		ifs.close();

		ofstream ofs;
		string file_index, outFile;
		file_index = file.substr(0, int(file.find("_")));

		outFile = file_index + ".dat";
		cout << "'" << outFile << "' is generated" << endl;
		ofs.open(outFile);
		Quaterniond q = Quaterniond(a.rotation());
		Vector3d t = a.translation();
		ofs << "mainBoardNo: " << mainBoardNo << endl;
		ofs << "q(xyzw) "   << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
		ofs << "t(xyz,cm) " << t.x() << " " << t.y() << " " << t.z() << endl;
		ofs << "t_iso(xyz,cm) " << (t-isocenter_calibration).transpose() << endl;
		time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
		ofs << ctime(&now) << endl;
		ofs.close();

		outFile = file_index + "_VIR(isocenter_to_camera).dat";
		cout << "'" << outFile << "' is generated" << endl;
		ofs.open(outFile);
		q = Quaterniond(a.rotation());
		t = a.translation() - isocenter_calibration;
		ofs << "q " << q.coeffs().transpose() << endl;
		ofs << "t " << t.transpose() << endl;
		now = chrono::system_clock::to_time_t(chrono::system_clock::now());
		ofs << ctime(&now) << endl;
		ofs.close();
		q_vec.push_back(q);
		t_vec.push_back(t);

		outFile = file_index + "_VIR(camera_to_isocenter).dat";
		cout << "'" << outFile << "' is generated" << endl;
		ofs.open(outFile);
		a.translation() = t;
		a = a.inverse();
		Quaterniond qinv = Quaterniond(a.rotation());
		Vector3d tinv = a.translation();
		ofs << "q " << qinv.coeffs().transpose() << endl;
		ofs << "t " << tinv.transpose() << endl;
		now = chrono::system_clock::to_time_t(chrono::system_clock::now());
		ofs << ctime(&now) << endl;
		ofs.close();
	}

	ofstream ofs("VIR_cams.json");
	ofs << "{" << endl;
	for (size_t i=0;i<sn_vec.size(); i++) {
		Eigen::AngleAxisd angleAxis(q_vec[i]);
		Eigen::Vector3d rvec = angleAxis.angle() * angleAxis.axis();
	ofs << "  \""+sn_vec[i]+"\": {" << endl;
	ofs	<< "    \"input\": {" << endl;
	ofs	<< "      \"fusion\": {" << endl;
	ofs << "		\"type\": \"INTRA_PROCESS\"" << endl;
	ofs << "      }," << endl;
	ofs	<< "      \"zed\": {" << endl;
	ofs << "		\"configuration\": \""+sn_vec[i]+"\"," << endl;
	ofs << "		\"type\": \"USB_SERIAL\"" << endl;
	ofs << "      }" << endl;
	ofs << "    }," << endl;
	ofs << "	\"world\": {" << endl;
	ofs << "		\"rotation\": [" << endl;
	ofs << "			" << rvec(0) << "," << endl;
	ofs << "			" << rvec(1) << "," << endl;
	ofs << "			" << rvec(2) << endl;
	ofs << "		]," << endl;
	ofs << "		\"translation\": [" << endl;
	ofs << "			" << t_vec[i].x() * 0.01 << "," << endl;
	ofs << "			" << t_vec[i].y() * 0.01 << "," << endl;
	ofs << "			" << t_vec[i].z() * 0.01 << endl;
	ofs << "		]" << endl;
	ofs << "	}" << endl;
	if (i == sn_vec.size()-1) {
	ofs << "  }" << endl;
	}
	else {
	ofs << "  }," << endl;
	}
	}
	ofs << "}" << endl;
	ofs.close();

	q_vec[0].x();
	
	
	return EXIT_SUCCESS;
}

int Measure_Profile(int argc, char** argv)
{

	
	return EXIT_SUCCESS;
}