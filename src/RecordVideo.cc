#include "RecordVideo.hh"

int Recording(int argc, char** argv)
{
	struct stat st;
	if (argc > 2)
		output_path = "./" + string(argv[2]);
	if (stat(output_path.c_str(), &st) != 0)
		mkdir(output_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	else {
		cerr << output_path + " directory already exists" << endl;
		exit(1);
	}
	if (output_path.back() != '/')
		output_path += "/";
	
	vector<int> zed_ids = get<0>(findCameraIndices());
	vector<int> cam_ids = get<1>(findCameraIndices());
	vector<string> cam_names = get<2>(findCameraIndices());
	int NB_ZED = zed_ids.size();
	int NB_CAM = cam_ids.size();
	total_cam_num = NB_ZED + NB_CAM;
	SetCtrlHandler();
	cout << "Start recording, use Ctrl-C to stop on terminal." <<endl;
	cout << "  ZED #: " << NB_ZED << " | /dev/video id: "; 
	for (auto z: zed_ids) cout << " " << z; cout << endl;
	cout << "  CAM #: " << NB_CAM << " | /dev/video id: ";
	for (auto c: cam_ids) cout << " " << c; cout << endl;

	vector<std::thread> zed_pool(NB_ZED), cam_pool(NB_CAM);
	for (int z=0; z<NB_ZED; z++)
		zed_pool[z] = std::thread(record_ZED, z, std::ref(exit_app));
	for (int c=0; c<NB_CAM; c++)
		cam_pool[c] = std::thread(record_CAM, std::ref(cam_ids[c]), std::ref(cam_names[c]), std::ref(exit_app));

	while (!exit_app)
		sl::sleep_us(50);

	for (int z=0; z<NB_ZED; z++)
		zed_pool[z].join();
	for (int c=0; c<NB_CAM; c++)
		cam_pool[c].join();
	
	cv::destroyAllWindows();
	
	return EXIT_SUCCESS;
}

int Synchronize_Recording(int argc, char** argv)
{
	string input_path = string(argv[2]);
	if (input_path.back() != '/')
		input_path += "/";
	if (input_path.front() == '.')
		input_path = input_path.substr(2, input_path.size());
	string output_path = input_path + "cut/";

	vector<string> svo_lists = findFileList(input_path, "svo");
	vector<string> avi_lists = findFileList(input_path, "avi");

	

	// Sync Time
	struct stat st;
	if (stat(output_path.c_str(), &st) != 0) {
		mkdir(output_path.c_str(), 0777);
	} else if (argc <= 3) {
		system(("rm -rf " + output_path + "*").c_str());
		mkdir(output_path.c_str(), 0777);
	}	
	vector<double> times2frames;
	vector<string> sync_names;

	for (int i=0; i<svo_lists.size()+avi_lists.size(); i++) {
		string s;
		if (i < svo_lists.size()) {
			s = svo_lists[i];
			sync_names.push_back(s.substr(s.find("ZED"), s.size()));
		}
		else {
			s = avi_lists[i-svo_lists.size()];
			sync_names.push_back(s.substr(s.find("CAM"), s.size()));
		}
		cout << "Recording name: " << sync_names[i] << endl;
		string YYMMDD = s.substr(s.find_first_of('/')+1, 8); cout << "YYMMDD: " << YYMMDD << endl;
		string HHMMSS = s.substr(s.find_first_of('_')+1, 6); cout << "HHMMSS: " << HHMMSS << endl;
		string MS = s.substr(s.find_first_of('_')+8, s.find_last_of('m')-(s.find_first_of('_')+8)); cout << "MS: " << MS << endl;
		times2frames.push_back(convertToSeconds(YYMMDD, HHMMSS, MS));
		cout << endl;
	}
	cout << "Sync time done." << endl;
	// Start Frame
	double max_time = *max_element(times2frames.begin(), times2frames.end());
	int count(0);
	set<int> skip_cameras;
	for (auto& t: times2frames) {
		t = floor( fabs((t - max_time) * 15) + 0.5 );
		if ( t == 0 )
			skip_cameras.insert(count);
		cout << setw(20) << sync_names[count++] << ": " << setw(5) <<  t << " start frames will be cut." << endl;
	}
	// End Frame
	for (int i=0; i<svo_lists.size(); i++) {
		sl::Camera zed; sl::InitParameters init_parameters;
		init_parameters.input.setFromSVOFile(svo_lists[i].c_str());
		auto state = zed.open(init_parameters);
		int end_frame = zed.getSVONumberOfFrames();
		cout << end_frame << endl;
		zed.close();
	}
	for (int i=0; i<avi_lists.size(); i++) {
		cout << avi_lists[i] << endl;
		cv::VideoCapture cap;
		cap.open(avi_lists[i].c_str());
		int end_frame = cap.get(cv::CAP_PROP_FRAME_COUNT);
		cout << end_frame << endl;
		cap.release();
	}

	for (int i=0; i<times2frames.size(); i++) {
		cout << "[Sync " << sync_names[i] << " at " << times2frames[i] << " frames]" << endl;
		if (i < svo_lists.size()) {
			if (skip_cameras.find(i) != skip_cameras.end()) {
				string cmd = "cp " + svo_lists[i] + " " + output_path + sync_names[i];
				system(cmd.c_str());
			} else {
				string cmd = "ZED_SVO_Editor -cut " + svo_lists[i] + " -s " + to_string((int)times2frames[i]) 
				// + " -e " + to_string((int)times2frames[i]+500) // frames
				// + " -e " + to_string((int)times2frames[i]+min_end_frame) // frames
				+ " " + output_path + sync_names[i];
				system(cmd.c_str());
			}
		}
		else 
		{
			if (skip_cameras.find(i) != skip_cameras.end()) {
				string cmd = "cp " + avi_lists[i-svo_lists.size()] + " " + output_path + sync_names[i];
				system(cmd.c_str());
			} else {
				cv::VideoCapture cap(avi_lists[i-svo_lists.size()]);
				cout << avi_lists[i-svo_lists.size()] << endl;
				if (!cap.isOpened()) {
					cerr << "input error" << endl;
					return 1;
				}
				int fc = cap.get(cv::CAP_PROP_FRAME_COUNT);
				cout << "Total Frame: " << fc << endl;
				// int fps = record_fps[i];
				int fps = 15;
				cv::Size frame_size = cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT));
				int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
				cv::VideoWriter out(output_path + sync_names[i], fourcc, fps, frame_size);
				if (!out.isOpened()) {
					cerr << "output error" << endl;
					return 1;
				}

				int frame_count(0);
				int cut_count(0);
				while (true) {
					cv::Mat frame;
					cap >> frame;
					if (frame.empty())
						break;
					if (frame_count >= times2frames[i]) {
						out.write(frame);
						// print cut and frame counts
						if (frame_count % 100 == 0)
							cout << "\rFrame change: " << frame_count << " -> " << cut_count << flush;
						cut_count++;
					}
					frame_count++;
				}
				out.release();
				cout << endl;
				cout << "Frame change: " << frame_count << " -> " << cut_count << endl;
			}
		}
	}
	cout << "Synchonization done." << endl;

	return EXIT_SUCCESS;
}

int View_Realtime(int argc, char** argv)
{
	vector<int> zed_ids = get<0>(findCameraIndices());
	vector<int> cam_ids = get<1>(findCameraIndices());
	vector<string> cam_names = get<2>(findCameraIndices());
	int NB_ZED = zed_ids.size();
	int NB_CAM = cam_ids.size();
	total_cam_num = NB_ZED + NB_CAM;
	cout << "  ZED #: " << NB_ZED << " | /dev/video id: "; 
	for (auto z: zed_ids) cout << " " << z; cout << endl;
	cout << "  CAM #: " << NB_CAM << " | /dev/video id: ";
	for (auto c: cam_ids) cout << " " << c; cout << endl;

	vector<int> ids;
	ids.insert(ids.end(), zed_ids.begin(), zed_ids.end());
	ids.insert(ids.end(), cam_ids.begin(), cam_ids.end());

	vector<cv::VideoCapture> caps(total_cam_num);
	int count(0);
	for (auto& cap: caps) {
		cap.open(ids[count], cv::CAP_V4L2);
		cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
		// if zed id
		if (count < NB_ZED) {
			cap.set(cv::CAP_PROP_FRAME_WIDTH,  2560);
			cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		} else {
			cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
			cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		}
		if (!cap.isOpened()) {
			cerr << "Error! Unable to open camera: " << endl;
			return EXIT_FAILURE;
		}
		cout << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "@" << cap.get(cv::CAP_PROP_FPS) << "fps" << endl;
		count++;
	}


	vector<cv::Mat> disp_images(total_cam_num);
	int num_rows = (disp_images.size() + 2) / 3;
	int num_cols = disp_images.size() < 3 ? disp_images.size() : 3;
	int frame_count(0);

	char key = ' ';
	while(key != 'q') {
		// show cap image
		for (int i=0; i<total_cam_num; i++) {
			cv::Mat img;
			caps[i] >> img;

			if (i < NB_ZED) {
				cv::Rect roi = cv::Rect(0, 0, img.cols/2, img.rows);
				img = img(roi);
			}
			disp_images[i] = img;
			cv::resize(disp_images[i], disp_images[i], cv::Size(int(1280*0.5), int(720*0.5)));
			cv::putText(disp_images[i], std::to_string(ids[i]), 
			cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255.f,255.f,0.f), 2.0);
		}

		cv::Mat concatened_image;
		int image_idx(0);
		for (int row=0; row<num_rows; row++) {
			cv::Mat row_image;
			for (int col=0; col<num_cols; col++) {
				if (image_idx < disp_images.size()) {
					if (col == 0) {
						row_image = disp_images[image_idx++];
					}
					else {
						cv::hconcat(row_image, disp_images[image_idx++], row_image);
					}
				} 
				else 
				{
					// if row image is not full, fill it with black image
					cv::Mat black_image = cv::Mat::zeros(disp_images[0].size(), disp_images[0].type());
					if (col == 0) {
						row_image = black_image;
					}
					else {
						cv::hconcat(row_image, black_image, row_image);
					}
				}
			}
			if (row == 0) {
				concatened_image = row_image;
			}
			else {
				cv::vconcat(concatened_image, row_image, concatened_image);
			}
		}
		cv::imshow("Concatenated", concatened_image);
		key = (char)cv::waitKey(1);
		cout << "\rFrame: " << frame_count++ << flush;
	}
	cout << endl;

	return EXIT_SUCCESS;
}

int View_Recording(int argc, char** argv)
{
	// Sync View
	string input_path = string(argv[2]);
	if (input_path.back() != '/')
		input_path += "/";
	if (input_path.front() == '.')
		input_path = input_path.substr(2, input_path.size());
	string cmd = "find " + input_path + " -name \"*.*\"";
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) cerr << "popen faeild" << endl;

	char buffer[128];
	vector<string> zed_files, cam_files;
	while( !feof(pipe) ) {
		if (fgets(buffer, 128, pipe) != NULL) {
			string s(buffer);
			s.pop_back();
			if (s.find("ZED") != string::npos)
				zed_files.push_back(s);
			else if (s.find("CAM") != string::npos)
				cam_files.push_back(s);
		}
	}
	pclose(pipe);

	cout << "ZED files:" << endl;
	for (auto& z: zed_files)
		cout << z << endl;
	cout << "CAM files:" << endl;
	for (auto& c: cam_files)
		cout << c << endl;

	vector<sl::Camera> zeds(zed_files.size());
	vector<sl::InitParameters> init_parameters(zed_files.size());
	for (int z=0;z<zeds.size();z++) {
		init_parameters[z].input.setFromSVOFile(zed_files[z].c_str());
		init_parameters[z].depth_mode = sl::DEPTH_MODE::NONE;
		auto state = zeds[z].open(init_parameters[z]);
		if (state != sl::ERROR_CODE::SUCCESS) {
			cerr << zed_files[z] + " can not be opened, " << state << endl;
			return EXIT_FAILURE;
		}
	}
	
	vector<cv::VideoCapture> cams(cam_files.size());
	for (int c=0;c<cams.size();c++) {
		cams[c].open(cam_files[c]);
		if (!cams[c].isOpened()) {
			cerr << "Error! Unable to open camera: " << cam_files[c] << endl;
			return EXIT_FAILURE;
		}
	}

	cout << "View start" << endl;
	vector<cv::Mat> disp_images(zed_files.size() + cam_files.size());
	sl::Mat zed_image;
	char key = ' ';
	bool exitLoop(false);

	int num_rows = (disp_images.size() + 2) / 3;
	int num_cols = disp_images.size() < 3 ? disp_images.size() : 3;
	int frame_count(0);
	while (key != 'q') {
		int idx(0);
		for (auto &z: zeds) {
			if (z.grab() != sl::ERROR_CODE::SUCCESS) {
				exitLoop = true;
				continue;
			}
			z.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
			cv::Mat c4toc3;
			cv::cvtColor(slMat2cvMat(zed_image), c4toc3, cv::COLOR_RGBA2RGB);
			disp_images[idx] = c4toc3;
			idx++;
		}

		for (int c=0;c<cams.size();c++) {
			cams[c] >> disp_images[idx];
			if(disp_images[idx].empty()) {
				exitLoop = true;
				continue;
			}
			idx++;
		}

		if (exitLoop)
			break;

		for (int i=0;i<disp_images.size();i++) {
			double sf = 0.5;
			if (i<zed_files.size()) {
				cv::putText(disp_images[i], zed_files[i], 
				cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255.f,255.f,0.f), 2.0);
			}
			else {
				cv::putText(disp_images[i], cam_files[i-zed_files.size()], 
				cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255.f,255.f,0.f), 2.0);
			}
			
			cv::resize(disp_images[i], disp_images[i], cv::Size(int(1280*sf), int(720*sf)));
		}

		cv::Mat concatened_image;
		int image_idx(0);
		for (int row=0; row<num_rows; row++) {
			cv::Mat row_image;
			for (int col=0; col<num_cols; col++) {
				if (image_idx < disp_images.size()) {
					if (col == 0) {
						row_image = disp_images[image_idx++];
					}
					else {
						cv::hconcat(row_image, disp_images[image_idx++], row_image);
					}
				} 
				else 
				{
					// if row image is not full, fill it with black image
					cv::Mat black_image = cv::Mat::zeros(disp_images[0].size(), disp_images[0].type());
					if (col == 0) {
						row_image = black_image;
					}
					else {
						cv::hconcat(row_image, black_image, row_image);
					}
				}
			}
			if (row == 0) {
				concatened_image = row_image;
			}
			else {
				cv::vconcat(concatened_image, row_image, concatened_image);
			}
		}
		cv::imshow("Concatenated", concatened_image);
		// key = waitKey(0);
		cout << "\rFrame: " << frame_count++ << flush;
		
		key = waitKey(1/15.f*1000);
		// key = waitKey(1.);

	}
	cout << endl;

	zed_image.free();
	for (auto& z: zeds)
		z.close();
	for (auto& c: cams)
		c.release();

	return EXIT_SUCCESS;
}





void record_ZED(int id, bool& exitLoop)
{
	sl::Camera zed;
	sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.camera_fps = 15;
	init_parameters.input.setFromCameraID(id);
	sl::RecordingParameters rec_parameters;
    rec_parameters.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
	sl::RecordingStatus state;
	sl::Mat zed_image;
	cv::Mat cv_image;
	sl::ERROR_CODE err = zed.open(init_parameters);
	auto SN = zed.getCameraInformation().serial_number;
	if (err != sl::ERROR_CODE::SUCCESS) {
		cout << "ZED [" << id << "] can not be opened, " << err << endl;
		exit(1);
	}
	
	cout << "ZED_" << SN << "'s setting: " << zed.getCameraInformation().camera_configuration.resolution.width << "x" 
		<< zed.getCameraInformation().camera_configuration.resolution.height << "@" << zed.getCameraInformation().camera_configuration.fps << "fps" << endl;
	
	char key = ' ';
	int sn = SN;
	string svo_name(output_path + getComputerTime(sn,false) + "_ZED_" + to_string(SN) + ".svo");
	rec_parameters.video_filename = svo_name.c_str();
	zed.enableRecording(rec_parameters);

	bool isFirst(true);
	string new_time;
	
	init_cam_num++;
	while(init_cam_num < total_cam_num)
		sl::sleep_us(50);

	while (!exitLoop)
	{
		if (zed.grab() != sl::ERROR_CODE::SUCCESS)
			continue;
		state = zed.getRecordingStatus();
		if (isFirst) {
			new_time = getComputerTime(sn);
			isFirst = false;
		}

		// zed.retrieveImage(zed_image, sl::VIEW::LEFT);
		// cv_image = slMat2cvMat(zed_image);
		// cv::imshow("ZED_" + to_string(SN), cv_image);
		// key = (char)cv::waitKey(1);
		// if (key == 'q') {
		// 	break;
		// }
	}
	string cmd = "mv " + svo_name + " " + output_path + new_time + "_ZED_" + to_string(SN) + ".svo";
	system(cmd.c_str());
	cout << "ZED_" + to_string(SN) << " stopped recording" << endl;

	zed.disableRecording();
	zed.close();
}

void record_CAM(int& id, string& cam_name, bool& exitLoop)
{
	int fourcc = cv::VideoWriter::fourcc('X','V','I','D');
	cv::VideoCapture cap;
	cap.open(id, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

	if (cam_name == "APC930") { // face ID
		cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		cap.set(cv::CAP_PROP_FPS, 15);
	}
	else if (cam_name == "UHD2160") { // face ID
		cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		cap.set(cv::CAP_PROP_FPS, 30);
	}
	else if (cam_name == "K4A") { // lead glass and monitor (ArUco) (UHD)
		cap.set(cv::CAP_PROP_FRAME_WIDTH,  3840);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 2160);
		cap.set(cv::CAP_PROP_FPS, 15);
	}
	else { // capture board (FHD)
		cap.set(cv::CAP_PROP_FRAME_WIDTH,  1920);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
		cap.set(cv::CAP_PROP_FPS, 15);
	}
	cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
	
	
	cout << "CAM_" << id << "'s setting: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" 
		<< cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "@" << cap.get(cv::CAP_PROP_FPS) << "fps" << endl;
	
	if (!cap.isOpened()) {
		cerr << "Error! Unable to open camera: " << endl;
		return;
	}

	cv::Mat img;
	cv::VideoWriter out;
	cv::Size img_size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT));
	
	char key = ' ';
	string avi_name = output_path + getComputerTime(id,false) + "_" + "CAM_" + to_string(id) + ".avi";
	int fps = cap.get(cv::CAP_PROP_FPS);
	out.open(avi_name, fourcc, fps, img_size, true);
	bool isFirst(true);
	string new_time;

	init_cam_num++;
	while(init_cam_num < total_cam_num)
		sl::sleep_us(50);

	while(!exitLoop)
	{
		cap >> img;
		out.write(img);
		if (isFirst) {
			new_time = getComputerTime(id);
			isFirst = false;
		}

		// resize image QHD,FHD -> HD
		// if (cam_name != "APC930")
		// 	cv::resize(img, img, cv::Size(1280, 720));
		// cv::imshow("CAM_" + to_string(id), img);
		// key = (char)cv::waitKey(1);
		// if (key == 'q') {
		// 	break;
		// }
	}

	
	string cmd = "mv " + avi_name + " " + output_path + new_time + "_" + "CAM_" + to_string(id) 
	// + "_FPS_" + to_string(int(cap.get(cv::CAP_PROP_FPS))) 
	+ ".avi";
	system(cmd.c_str());
	cout << "CAM_" + to_string(id) << " stopped recording" << endl;

	cap.release();
}