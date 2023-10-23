#include "BoardCalib.hh"

bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

int Calibrate_with_ChArUco(int argc, char** argv)
{
    int squaresX = 5;
    int squaresY = 7;
    float squareLength = 0.0760f;
    float markerLength = 0.0595f;
    string outputFile = "calib.yml";
    if (argc > 2)
        outputFile = string(argv[2]);

    bool showChessboardCorners = true;

    int calibrationFlags = 0;
    float aspectRatio = 1;

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    bool readOk = readDetectorParameters("./data/detector_params.yml", detectorParams);
    if(!readOk) {
        cerr << "Invalid detector parameters file" << endl;
        return 0;
    }

    bool refindStrategy = true;
    String video;

    int opt(0);
    double scaling_factor = 1.0;
    std::pair<int, int> calib_resolution = {0,0};
    cout << "Select calibration type" << endl;
    cout << " 1. UHD (3840 x 2160)" << endl;
    cout << " 2. FHD (1920 x 1080)" << endl;
    cout << " 2.  HD (1280 x  720)" << endl;
    cout << "Enter the number: "; cin >> opt;
    switch (opt)
    {
    case 1: calib_resolution = {3840, 2160}; scaling_factor = 0.4; break;
    case 2: calib_resolution = {1920, 1080}; scaling_factor = 0.6; break;
    case 3: calib_resolution = {1280, 720};  break;
    default: cout << "Enter the number: "; cin >> opt; break;
    }
    
    VideoCapture cap;
    cap.open(0,cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  calib_resolution.first);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, calib_resolution.second);
    cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
    if (!cap.isOpened()) {
        cerr << "Error! Unabale to open camera" << endl;
        return 1;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_4X4_1000));

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard =
            aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    // collect data from each frame
    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector< Mat > allImgs;
    Size imgSize;

    int frameNo(0);
    while(cap.grab()) {
        Mat image, imageCopy;
        cap.retrieve(image);

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // interpolate charuco corners
        Mat currentCharucoCorners, currentCharucoIds;
        if(ids.size() > 0)
            aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners, currentCharucoIds);

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners);

        if(currentCharucoCorners.total() > 0)
            aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

        putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 20/scaling_factor), FONT_HERSHEY_SIMPLEX, 0.5/scaling_factor, Scalar(255, 0, 0), 1.5/scaling_factor);
        putText(imageCopy, "captured frame #: " + to_string(frameNo),
                Point(10, 40/scaling_factor), FONT_HERSHEY_SIMPLEX, 0.5/scaling_factor, Scalar(255, 0, 0), 1.5/scaling_factor);
        putText(imageCopy, "detected ArUco #: " + to_string(ids.size()),
                Point(10, 60/scaling_factor), FONT_HERSHEY_SIMPLEX, 0.5/scaling_factor, Scalar(255, 0, 0), 1.5/scaling_factor);                
        
        Mat resized_img;
        cv::resize(imageCopy, resized_img, cv::Size(imageCopy.cols * scaling_factor, imageCopy.rows * scaling_factor));

        imshow("calibration", resized_img);
        char key = (char)waitKey(1);
        if(key == 27) break;
        if(key == 'c' && ids.size() == 17) {
            frameNo++;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(image);
            imgSize = image.size();
        }
    }

    if(allIds.size() < 1) {
        cerr << "Not enough captures for calibration" << endl;
        return 0;
    }

    Mat cameraMatrix, distCoeffs;
    vector< Mat > rvecs, tvecs;
    double repError;

    if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = aspectRatio;
    }

    // prepare data for calibration
    vector< vector< Point2f > > allCornersConcatenated;
    vector< int > allIdsConcatenated;
    vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                              markerCounterPerFrame, board, imgSize, cameraMatrix,
                                              distCoeffs, noArray(), noArray(), calibrationFlags);

    // prepare data for charuco calibration
    int nFrames = (int)allCorners.size();
    vector< Mat > allCharucoCorners;
    vector< Mat > allCharucoIds;
    vector< Mat > filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);

    for(int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        Mat currentCharucoCorners, currentCharucoIds;
        aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                         distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs[i]);
    }

    if(allCharucoCorners.size() < 4) {
        cerr << "Not enough corners for calibration" << endl;
        return 0;
    }

    // calibrate camera using charuco
    repError =
        aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
                                      cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

    bool saveOk =  saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
                                    cameraMatrix, distCoeffs, repError);
    if(!saveOk) {
        cerr << "Cannot save output file" << endl;
        return 0;
    }

    cout << "Rep Error: " << repError << endl;
    cout << "Rep Error Aruco: " << arucoRepErr << endl;
    cout << "Calibration saved to " << outputFile << endl;

    // show interpolated charuco corners for debugging
    if(showChessboardCorners) {
        for(unsigned int frame = 0; frame < filteredImages.size(); frame++) {
            Mat imageCopy = filteredImages[frame].clone();
            if(allIds[frame].size() > 0) {

                if(allCharucoCorners[frame].total() > 0) {
                    aruco::drawDetectedCornersCharuco( imageCopy, allCharucoCorners[frame],
                                                       allCharucoIds[frame]);
                }
            }
            Mat resized_img;
            cv::resize(imageCopy, resized_img, cv::Size(imageCopy.cols * scaling_factor, imageCopy.rows * scaling_factor));
            imshow("check", resized_img);
            char key = (char)waitKey(0);
            if(key == 27) break;
        }
    }

    return 0;
}
