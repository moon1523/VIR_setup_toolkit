#ifndef VIEWCOORDS_HH_
#define VIEWCOORDS_HH_

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <vector>

#include "CoordMatch.hh"
#include "Functions.hh"

using namespace std;

void Read_Board_Pose(vector<cv::Affine3d> &board_pose,
	                 vector<cv::viz::WCameraPosition> &board_coords,
                     string fileName);
void Read_Camera_Pose(vector<cv::Affine3d> &cam_pose,
	                  vector<cv::viz::WCameraPosition> &cam_coords,
                      vector<cv::viz::WCameraPosition> &cam_frustum,
                      string pathName=".");
void Read_Point_Clouds(	vector<vector<cv::Vec3d>> &point_clouds,
	                    vector<vector<cv::Vec3b>> &point_clouds_color,
                        string pathName="./sync/");

int View_Coordinates(int argc, char** argv);
static void keyboard_callback(const cv::viz::KeyboardEvent &event, void* cookie) {}

#endif