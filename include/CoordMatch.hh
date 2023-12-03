#ifndef COORDMATCH_HH_
#define COORDMATCH_HH_

#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp> // this header should be invoked after <Eigen/Sparse>
#include <opencv2/calib3d.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <set>
#include <sys/stat.h>
#include "Functions.hh"

using namespace std;
using namespace cv;
using namespace Eigen;

enum class BoardType
{
	TEST,
	HYUMC,
};

typedef pair<int,int> MD;

class CoordMatch
{
public:
	CoordMatch(int type, int boardNo);
    virtual ~CoordMatch();
    bool SetParameters(string camParam, string detParam);
	void TickFixedCameraMode() { isFix = !isFix; };
    void EstimatePose(const Mat &color);
    void Render();
    void ShowAvgValue(const Mat &color);
    void WriteTransformationData(string file);
    void ClearData();
    void TickSwitch() { getPose = !getPose; }
    void SetScalingFactor(float s);
	void SetFixedCameraSerialNumber(string serial_number) { fixed_camera_serial_number = serial_number; }
	void SetDetectingNo(int _detectingNo) { detectingNo = _detectingNo; }
	void SetCumulatingNo(int _cumulatingNo) { cumulatingNo = _cumulatingNo; }

    void calculate_transformation_matrices_for_detected_boardID(int viewNo);
    void calculate_empty_transformation_matrices_from_averaged_transfrmation_matrices();
    // void averaging_stacked_transformation_matrices();
    void weighted_averaging_stacked_transformation_matrices();

	void real_time_camera_calibration();

private:
	string fixed_camera_serial_number;

    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Ptr<cv::aruco::DetectorParameters> params;
    Mat camMatrix;
    Mat distCoeffs;
	bool isFix;
    bool getPose;
    Mat display, display_resize;
    float sf;

    //avg values
    bool isStack;
    int boardType;

    // vector
    int boardNo, markerNo, mainBoardNo;
	vector<cv::Ptr<cv::aruco::CharucoBoard>> boardVec;
	vector<cv::Ptr<cv::aruco::Dictionary>> dictionaryVec;

	vector<vector<int>> markerIds;
	vector<vector<vector<cv::Point2f>>> markerCorners;
	vector<vector<int>> charucoIds;
	vector<vector<cv::Point2f>> charucoCorners;
	vector<cv::Vec3d> rvec;
	vector<cv::Vec3d> tvec;

	vector<vector<Vector4d>> init10_qVec;
	vector<Vector4d> 		 avg10_qVec;
	vector<vector<Vector4d>> cum_qVec;
	vector<Vec3d> 			 sum_tVec;
	vector<vector<double>> cum_wVec;
	vector<double>         sum_wVec;

	// To reference coordinate
	vector<int> board_frameNo;
	vector<int> detected_boardID_oneView;
	vector<vector<int>> detected_boardID_viewVec; // view, id

	map<MD, vector<Eigen::Affine3d>> cum_m2dMap;
	map<MD, Eigen::Affine3d> avg_m2dMap;
	map<MD, vector<double>> cum_m2dwMap;
	map<MD, double> avg_m2dwMap;

	vector<MD> mdVec, emptyVec;

	int detectingNo, cumulatingNo;

	vector<Point2f> pts1, pts2;
	int reprojCount;



//	map<int, int> dm;
//	map<MD, Eigen::Affine3d> m2dMap;
//	vector<Eigen::Affine3d>  r2d;
//	vector<Eigen::Affine3d> R2D;







};

#endif
