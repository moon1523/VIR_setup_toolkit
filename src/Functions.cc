#include "Functions.hh"

#ifdef K4A_FOUND
k4a_device_configuration_t get_default_config()
{
	k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	camera_config.subordinate_delay_off_master_usec = 0;
	camera_config.synchronized_images_only = true;

	return camera_config;
}

cv::Mat color_to_opencv(const k4a_image_t im)
{
	cv::Mat cv_image_with_alpha(k4a_image_get_height_pixels(im), k4a_image_get_width_pixels(im), CV_8UC4, (void*)k4a_image_get_buffer(im));
    cv::Mat cv_image_no_alpha;
    cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
    return cv_image_no_alpha;
}

cv::Mat depth_to_opencv(const k4a_image_t im)
{
    return cv::Mat(k4a_image_get_height_pixels(im),
    			   k4a_image_get_width_pixels(im),
				   CV_16U,
        (void*)k4a_image_get_buffer(im),
        static_cast<size_t>(k4a_image_get_stride_bytes(im)));
}

void WritePointCloud(const k4a_image_t point_image, 
                     const k4a_image_t color_image,
                     string fileName)
{
	cout << "WritePointCloud - camera view" << endl;
    vector<RowVector3d> xyz;
    vector<RowVector3i> rgb;

    int width  = k4a_image_get_width_pixels(point_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t *point_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_image);
    uint8_t *color_image_data = k4a_image_get_buffer(color_image);

    for (int i=0; i<width*height; i++) {
        
        double Z = point_image_data[3 * i + 2];
        if (Z == 0) continue;
        double X = point_image_data[3 * i + 0];
        double Y = point_image_data[3 * i + 1];

        int b = color_image_data[4 * i + 0];
        int g = color_image_data[4 * i + 1];
        int r = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (b == 0 && g == 0 && r == 0 && alpha == 0)
            continue;

        xyz.push_back(RowVector3d(X,Y,Z));
        rgb.push_back(RowVector3i(r,g,b));
    }

    ofstream ofs("./sync/" + fileName + ".ply");
    ofs << "ply" << endl;
    ofs << "format ascii 1.0" << endl;
    ofs << "element vertex "  << xyz.size() << endl;
    ofs << "property float x" << endl;
    ofs << "property float y" << endl;
    ofs << "property float z" << endl;
    ofs << "property uchar red" << endl;
    ofs << "property uchar green" << endl;
    ofs << "property uchar blue" << endl;
    ofs << "end_header" << endl;

    for (size_t i=0;i<xyz.size();i++) {
        ofs << xyz[i] << " " << rgb[i] << endl;
    }
    ofs.close();
}


void WriteTrasnformedPointCloudToRefBoard(const k4a_image_t point_image,
                     					  const k4a_image_t color_image,
										  string fileName)
{
	ifstream ifs(fileName + ".dat");
	if (!ifs.is_open()) { cerr << fileName << " was not opened" << endl; return; }
	string dump;
	
	Quaterniond q; Vector3d t;
	Eigen::Affine3d a;
	while(getline(ifs, dump)) {
		stringstream ss(dump);
		ss >> dump;
		if (dump == "q(xyzw)") {
			double x,y,z,w;
			ss >> x >> y >> z >> w;
			q = Quaterniond(w,x,y,z);
		}
		else if (dump == "t(xyz,cm)") {
			double x,y,z;
			ss >> x >> y >> z;
			t = Vector3d(x*10,y*10,z*10);
		}
		else if (dump == "t_iso(xyz,cm)") {
			double x,y,z;
			ss >> x >> y >> z;
			t = Vector3d(x*10,y*10,z*10);
		}
	}
	ifs.close();
	a.linear() = q.normalized().matrix();
	a.translation() = t;
	

	vector<RowVector3d> xyz;
	vector<RowVector3i> rgb;

	int width  = k4a_image_get_width_pixels(point_image);
	int height = k4a_image_get_height_pixels(color_image);

	int16_t *point_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);

	for (int i=0; i<width*height; i++) {
		if (point_image_data[3*i+2] == 0) continue;
		Vector3d pointVec(point_image_data[3*i+0], point_image_data[3*i+1], point_image_data[3*i+2]);
		pointVec = a * pointVec;

		double X = pointVec(0);
		double Y = pointVec(1);
		double Z = pointVec(2);

		int b = color_image_data[4 * i + 0];
		int g = color_image_data[4 * i + 1];
		int r = color_image_data[4 * i + 2];
		uint8_t alpha = color_image_data[4 * i + 3];

		if (b == 0 && g == 0 && r == 0 && alpha == 0)
			continue;

		xyz.push_back(RowVector3d(X,Y,Z));
		rgb.push_back(RowVector3i(r,g,b));
	}

	cout << "'" << fileName+"_tf.ply' is generated "<< endl;
	ofstream ofs(fileName + "_tf.ply");
	ofs << "ply" << endl;
	ofs << "format ascii 1.0" << endl;
	ofs << "element vertex "  << xyz.size() << endl;
	ofs << "property float x" << endl;
	ofs << "property float y" << endl;
	ofs << "property float z" << endl;
	ofs << "property uchar red" << endl;
	ofs << "property uchar green" << endl;
	ofs << "property uchar blue" << endl;
	ofs << "end_header" << endl;

	for (size_t i=0;i<xyz.size();i++) {
		ofs << xyz[i] << " " << rgb[i] << endl;
	}
	ofs.close();
}
#endif

void WriteTrasnformedPointCloudToRefBoard(sl::Camera& zed,
										  const sl::Mat slMat,
										  string serial_number)
{
	ifstream ifs("./sync/" + serial_number + ".dat");
	if (!ifs.is_open()) { cerr << "file was not opened" << endl; exit(1); }
	string dump;
	
	Quaternionf q; Vector3f t;
	Eigen::Affine3f a;
	while(getline(ifs, dump)) {
		stringstream ss(dump);
		ss >> dump;
		if (dump == "q(xyzw)") {
			double x,y,z,w;
			ss >> x >> y >> z >> w;
			q = Quaternionf(w,x,y,z);
		}
		else if (dump == "t(xyz,cm)") {
			double x,y,z;
			ss >> x >> y >> z;
			t = Vector3f(x*10,y*10,z*10);
		}
		else if (dump == "t_iso(xyz,cm)") {
			double x,y,z;
			ss >> x >> y >> z;
			t = Vector3f(x*10,y*10,z*10);
		}
	}
	ifs.close();
	a.linear() = q.normalized().matrix();
	a.translation() = t;

	// sl::Resolution res(slMat.getWidth(), slMat.getHeight()); 
	// sl::Mat point_cloud(res, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
	sl::Mat point_cloud;
	zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU);

	sl::float4 *point_cloud_ptr = point_cloud.getPtr<sl::float4>(sl::MEM::CPU);
	int width = point_cloud.getWidth();
	int height = point_cloud.getHeight();

	for (size_t y=0;y<height;y++) {
		for (size_t x=0;x<width; x++) {
			sl::float4 &point = point_cloud_ptr[y * width + x];
			if (!isValidMeasure(point.z)) // if the point is not valid, skip it
				continue;

			Vector3f tf_point = (a.matrix() * Vector4f(point.x, point.y, point.z, 1.)).hnormalized();
			point.x = tf_point.x();
			point.y = tf_point.y();
			point.z = tf_point.z();
		}
	}
	auto write_suceed = point_cloud.write(("./sync/" + serial_number + "_tf.ply").c_str());
	if(write_suceed == sl::ERROR_CODE::SUCCESS)
		std::cout << "Current .ply file saving succeed" << std::endl;                
	else
		std::cout << "Current .ply file saving failed" << std::endl;
	
	point_cloud.free();
}

void Write_Fixed_Camera_Transformation_Matrix(string serial_number)
{
	cout << "==========================================" << endl;
	cout << " Write fixed camera transformation matrix " << endl;
	cout << "==========================================" << endl;
	Quaterniond q_fixed, q_conn;
	Vector3d t_fixed, t_conn;
	Eigen::Affine3d a_fixed, a_conn, a;
	
	string fileName = "./sync/" + serial_number + "_fixedMode.dat";
	ifstream ifs(fileName);
	if (!ifs.is_open()) { 
		cerr << fileName << " was not opened. Run the 'fixed camera mode' again." << endl;
		return;
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
		// return;
	}
	else {
		ifs.open("MatrixTF.dat");
		if (!ifs.is_open()) { cerr << "'MatrixTF.dat' was not opened. Run the 'connector mode'" << endl; return; }
		while(getline(ifs,dump)) {
			stringstream ss(dump);
			ss >> dump;
			if (dump == "m2d") {
				int mother, daughter;
				ss >> mother >> dump >> daughter;
				if (mother == 0 && daughter == mainBoardNo) {
					double x,y,z,w;
					getline(ifs, dump);
					cout << dump << endl;
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

	
	Vector3d isocenter_calibration;
	bool isIso(false);
	cout << "Enter the file name of HTC VIVE Tracker sphere fitting or position: "; cin >> fileName;
	ifs.open(fileName);
	if (!ifs.is_open()) { cerr << "There is no isocenter file such" << endl; }
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
	fileName = "./sync/" + serial_number + ".dat";
	cout << "'" << fileName << "' is generated" << endl;
	ofs.open(fileName);
	Quaterniond q = Quaterniond(a.rotation());
	Vector3d t = a.translation();
	ofs << "mainBoardNo: " << mainBoardNo << endl;
	ofs << "q(xyzw) "   << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
	ofs << "t(xyz,cm) " << t.x() << " " << t.y() << " " << t.z() << endl;
	if (isIso)
		ofs << "t_iso(xyz,cm) " << (t-isocenter_calibration).transpose() << endl;
	cout << endl;
	time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
	ofs << ctime(&now) << endl;
	ofs.close();

	if(isIso) {
		fileName = "./sync/" + serial_number + "_VIR(isocenter_to_camera).dat";
		cout << "'" << fileName << "' is generated" << endl;
		ofstream ofs(fileName);
		Quaterniond q = Quaterniond(a.rotation());
		Vector3d t = a.translation() - isocenter_calibration;
		ofs << "q " << q.coeffs().transpose() << endl;
		ofs << "t " << t.transpose() << endl;
		time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
		ofs << ctime(&now) << endl;
		ofs.close();

		fileName = "./sync/" + serial_number + "_VIR(camera_to_isocenter).dat";
		cout << "'" << fileName << "' is generated" << endl;
		ofs.open(fileName);
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
}

double averaging_weights(const std::vector<double> &weights)
{
	double sum_weight(0);
	for (const auto &w:weights) {
		sum_weight += w;
	}
	return sum_weight/(double)weights.size();
}

Eigen::Vector3d averaging_translations(const std::vector<Eigen::Vector3d>& translations)
{
	Eigen::Vector3d t;
	for (auto itr: translations) {
		t += itr;
	}		
	return t / (double)translations.size();
}

Eigen::Vector4d averaging_quaternions(const std::vector<Eigen::Vector4d>& quaternions)
{
    if (quaternions.empty())
        return Eigen::Vector4d(0,0,0,1);

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    for (int q = 0; q < quaternions.size(); ++q)
        A += quaternions[q] * quaternions[q].transpose();

    // normalise with the number of quaternions
    A /= quaternions.size();

    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();

    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex;
    float largestEigenValue;
    bool first = true;

    for (int i = 0; i < singularValues.rows(); ++i)
    {
        if (first)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
            first = false;
        }
        else if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
        }
    }

    Eigen::Vector4d average;
    average(0) = -U(0, largestEigenValueIndex);
    average(1) = -U(1, largestEigenValueIndex);
    average(2) = -U(2, largestEigenValueIndex);
    average(3) = -U(3, largestEigenValueIndex);

    return average;
}

Eigen::Vector4d weighted_averaging_quaternions(
  const std::vector<Eigen::Vector4d>& quaternions,
  const std::vector<double>& weights)
{
	Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, quaternions.size());
	Eigen::Vector3d vec;
	for (size_t i = 0; i < quaternions.size(); ++i)
	{
		// Weigh the quaternions according to their associated weight
		Eigen::Vector4d quat = quaternions[i] * weights[i];
		// Append the weighted Quaternion to a matrix Q.
		Q(0,i) = quat.x();
		Q(1,i) = quat.y();
		Q(2,i) = quat.z();
		Q(3,i) = quat.w();
	}

	// Creat a solver for finding the eigenvectors and eigenvalues
	Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());

	// Find index of maximum (real) Eigenvalue.
	auto eigenvalues = es.eigenvalues();
	size_t max_idx = 0;
	double max_value = eigenvalues[max_idx].real();
	for (size_t i = 1; i < 4; ++i)
	{
		double real = eigenvalues[i].real();
		if (real > max_value)
		{
		  max_value = real;
		  max_idx = i;
		}
	}

	// Get corresponding Eigenvector, normalize it and return it as the average quat
	auto eigenvector = es.eigenvectors().col(max_idx).normalized();

	Eigen::Vector4d mean_orientation(
		eigenvector[0].real(),
		eigenvector[1].real(),
		eigenvector[2].real(),
		eigenvector[3].real());

  return mean_orientation;
}

void cv2eigen_Affine3d(const cv::Affine3d& c, Eigen::Affine3d& e)
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;
	cv::cv2eigen(c.rotation(), rot);
	cv::cv2eigen(c.translation(), trans);
	e.linear() = rot;
	e.translation() = trans;
}

Eigen::Affine3d averaging_Affine3d(const vector<Eigen::Affine3d>& affs)
{
	vector<Eigen::Vector4d> Qs;
	vector<Eigen::Vector3d> Ts;
	for (auto itr: affs) {
		Qs.push_back(Eigen::Vector4d( Quaterniond(itr.linear()).coeffs() ));
		Ts.push_back(Eigen::Vector3d( itr.translation() ));
	}
	Vector4d q = averaging_quaternions(Qs);
	Vector3d t = averaging_translations(Ts);
	Eigen::Affine3d a;
	a.linear() = Quaterniond(q).normalized().toRotationMatrix();
	a.translation() = t;
	return a;
}