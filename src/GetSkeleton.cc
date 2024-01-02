#include <GetSkeleton.hh>
#include <fstream>


GetSkeleton::GetSkeleton(int argc, char** argv)
{
#ifdef _SL_JETSON_
    const bool isJetson = true;
#else
    const bool isJetson = false;
#endif
    SetVars();
    // Create ZED Bodies
    Camera zed;
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::AUTO;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_parameters.coordinate_units = UNIT::CENTIMETER;
    init_parameters.camera_fps = 15;

    //parseArgs(argc, argv, init_parameters);
    std::string outFile = argv[2];
    std::cout << outFile  << " :: Get skeleton length "<< std::endl;
    outFile += ".skeleton";

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Open Camera", returned_state, "\nExit program.");
        zed.close();
        return;
    }

    // Enable Positional tracking (mandatory for object detection)
    PositionalTrackingParameters positional_tracking_parameters;
    //If the camera is static, uncomment the following line to have better performances
    //positional_tracking_parameters.set_as_static = true;

    returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enable Positional Tracking", returned_state, "\nExit program.");
        zed.close();
        return;
    }

    // Enable the Body tracking module
    BodyTrackingParameters body_tracker_params;
    body_tracker_params.enable_tracking = true; // track people across images flow
    body_tracker_params.enable_body_fitting = true; // smooth skeletons moves
    body_tracker_params.body_format = sl::BODY_FORMAT::BODY_34;
    body_tracker_params.detection_model = isJetson ? BODY_TRACKING_MODEL::HUMAN_BODY_FAST : BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
    //body_tracker_params.allow_reduced_precision_inference = true;

    returned_state = zed.enableBodyTracking(body_tracker_params);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enable Object Detection", returned_state, "\nExit program.");
        zed.close();
        return;
    }

    auto camera_config = zed.getCameraInformation().camera_configuration;

    // For 2D GUI
    float image_aspect_ratio = camera_config.resolution.width / (1.f * camera_config.resolution.height);
    int requested_low_res_w = min(1280, (int)camera_config.resolution.width);
    sl::Resolution display_resolution(requested_low_res_w, requested_low_res_w / image_aspect_ratio);

    cv::Mat image_left_ocv(display_resolution.height, display_resolution.width, CV_8UC4, 1);
    Mat image_left(display_resolution, MAT_TYPE::U8_C4, image_left_ocv.data, image_left_ocv.step);
    sl::float2 img_scale(display_resolution.width / (float) camera_config.resolution.width, display_resolution.height / (float) camera_config.resolution.height);


    // Create OpenGL Viewer
    GLViewer viewer;
    viewer.init(argc, argv);

    Pose cam_pose;
    cam_pose.pose_data.setIdentity();

    // Configure object detection runtime parameters
    BodyTrackingRuntimeParameters body_tracker_parameters_rt;
    body_tracker_parameters_rt.detection_confidence_threshold = 60;
    body_tracker_parameters_rt.skeleton_smoothing = 0.7;
    
    // Create ZED Bodies filled in the main loop
    Bodies bodies;

    // Main Loop
    bool quit = false;
    string window_name = "ZED| 2D View";
    int key_wait = 10;
    char key = ' ';

    std::vector<float> BEnorm(BE_SIZE, 0.);
    std::vector<sl::float3> FaceCoor(4, sl::float3(0., 0., 0.));
    std::map<int, std::vector<float>> BEPerson;
    int i = 0;
    while (!quit) {
        // Grab images
        auto err = zed.grab();
        if (err == ERROR_CODE::SUCCESS) {
            // Retrieve Detected Human Bodies
            zed.retrieveBodies(bodies, body_tracker_parameters_rt);

            //OCV View
            zed.retrieveImage(image_left, VIEW::LEFT, MEM::CPU, display_resolution);
            zed.getPosition(cam_pose, REFERENCE_FRAME::WORLD);

            //Update GL View
            viewer.updateData(bodies, cam_pose.pose_data);

            //printf("bodies is tracked %d \n", bodies.is_tracked);
            render_2D(image_left_ocv, img_scale, bodies.body_list, bodies.is_tracked);

            for (auto &itr : bodies.body_list)
            {
                for(int j = 0; j < BE_SIZE; j++)
                {
                    sl::float3 BEvec = GetBEVector(itr.keypoint, BE_joint,j);
                    BEnorm[j] += BEvec.norm();
                }
                BEPerson[itr.id] = BEnorm;
                if(BEnorm[1] > 0) i++;
            }


            cv::imshow(window_name, image_left_ocv);

            key = cv::waitKey(key_wait);

            if (key == 'q') quit = true;
            if (key == 'm') {
                if (key_wait > 0) key_wait = 0;
                else key_wait = 10;
            }
            if (!viewer.isAvailable()) quit = true;
            std::cout << i << std::endl;
            if(i == 100)
            {
                std::ofstream ofs;
                ofs.open("./data/" + outFile);

                for(auto &itr : BEPerson)
                {
                    //fs << itr.first << std::endl;
                    for(auto &iitr: itr.second)
                    {
                        ofs << iitr * 0.01 << std::endl;
                    }
                }
                ofs.close();
                quit = true;
            }
        } 
        else if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        {
            zed.setSVOPosition(0);
        }
        else
            quit = true;
    }

    // Release Bodies
    viewer.exit();
    image_left.free();
    bodies.body_list.clear();

    // Disable modules
    zed.disableBodyTracking();
    zed.disablePositionalTracking();
    zed.close();

    return;
}

void GetSkeleton::SetVars()
{
    BE_vec.resize(BE_SIZE);
    BE_leng.resize(BE_SIZE);
    BE_joint[0] = make_pair(3, 26);
    BE_joint[1] = make_pair(0, 1);
    BE_joint[2] = make_pair(1, 2);
    BE_joint[3] = make_pair(2, 4);
    BE_joint[4] = make_pair(4, 5);
    BE_joint[5] = make_pair(5, 6);
    BE_joint[6] = make_pair(6, 7);
    BE_joint[7] = make_pair(7, 8);
    BE_joint[8] = make_pair(2, 3);
    BE_joint[9] = make_pair(2, 11);
    BE_joint[10] = make_pair(11, 12);
    BE_joint[11] = make_pair(12, 13);
    BE_joint[12] = make_pair(13, 14);
    BE_joint[13] = make_pair(14, 15);
    BE_joint[14] = make_pair(0, 18);
    BE_joint[15] = make_pair(18, 19);
    BE_joint[16] = make_pair(19, 20);
    BE_joint[17] = make_pair(20, 21);
    BE_joint[18] = make_pair(0, 22);
    BE_joint[19] = make_pair(22, 23);
    BE_joint[20] = make_pair(23, 24);
    BE_joint[21] = make_pair(24, 25);
}

sl::float3 GetSkeleton::GetBEVector(std::vector<sl::float3> &keypoint, std::map<int ,pair<int,int>> &BE_joint, int BE_order)
{
    int BE_0 = BE_joint[BE_order].first;
    int BE_1 = BE_joint[BE_order].second;
    //std::cout << "BE_0 : " << BE_0 << std::endl << "BE_1 : " << BE_1 << std::endl;
    float x = keypoint[BE_1].x - keypoint[BE_0].x;
    float y = keypoint[BE_1].y - keypoint[BE_0].y;
    float z = keypoint[BE_1].z - keypoint[BE_0].z;
    //Vector3<float> a = Vector3<float>(0.1);
    sl::float3 BEVec = sl::float3(x,y,z);
    return BEVec;
}


void GetSkeleton::parseArgs(int argc, char **argv, InitParameters& param) {
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    } else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        } else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        } else if (arg.find("HD2K") != string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        }else if (arg.find("HD1200") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1200;
            cout << "[Sample] Using Camera in resolution HD1200" << endl;
        } else if (arg.find("HD1080") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        } else if (arg.find("HD720") != string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        }else if (arg.find("SVGA") != string::npos) {
            param.camera_resolution = RESOLUTION::SVGA;
            cout << "[Sample] Using Camera in resolution SVGA" << endl;
        }else if (arg.find("VGA") != string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    }
}

void GetSkeleton::print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout << "[Sample]";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error]";
    cout << " " << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}
