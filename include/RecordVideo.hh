#ifndef RECORDVIDEO_HH_
#define RECORDVIDEO_HH_

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <malloc.h>
#include <thread>
#include <sys/stat.h>

#include "CoordMatch.hh"
#include "Functions.hh"

using namespace std;

void record_ZED(int id, bool& exitLoop);
void record_CAM(int& id, string& cam_name, bool& exitLoop);
int Recording(int argc, char** argv);
int Synchronize_Recording(int argc, char** argv);
int View_Recording(int argc, char** argv);
int View_Realtime(int argc, char** argv);

static bool exit_app = false;
static string output_path = "./";
static int init_cam_num = 0;
static int total_cam_num = 0;

// Handle the CTRL-C keyboard signal
#include <signal.h>
static void nix_exit_handler(int s) {
    exit_app = true;
}
// Set the function to handle the CTRL-C
static void SetCtrlHandler() {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = nix_exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
}

// Conversion function between sl::Mat and cv::Mat
static cv::Mat slMat2cvMat(sl::Mat &input) {
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

static std::string getComputerTime(int& id, bool print=true)
{
	auto currentTime = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(currentTime);
    std::tm localTimeInfo = *std::localtime(&time);
    char timeStr[100], timeStr2[100];
    std::strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", &localTimeInfo);
	std::strftime(timeStr2, sizeof(timeStr2), "%Y-%m-%d %H:%M:%S", &localTimeInfo);
    auto duration = currentTime.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
	if( print )
		std::cout << "  " << id << "'s starting time: " << timeStr2 << " ." << millis.count() << " ms" << std::endl;
    return std::string(timeStr) + "_" + std::to_string(millis.count()) + "ms";
}

static tuple<vector<int>, vector<int>, vector<string>> findCameraIndices()
{
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
    vector<int> zed_ids, cam_ids;
	vector<string> cam_names;
    while (ss >> dump) {
        if (dump == "ZED") {
            getline(ss, line);
            getline(ss, line);
            stringstream ss2(line);
            ss2 >> dump;
            zed_ids.push_back( stoi(dump.substr(string("/dev/video").size(), dump.size())) );
        }
		else if (dump == "Azure") {
            getline(ss, line);
            getline(ss, line);
            stringstream ss2(line);
            ss2 >> dump;
            cam_ids.push_back( stoi(dump.substr(string("/dev/video").size(), dump.size())) );
			cam_names.push_back("K4A");
        }
        else if (dump == "APC930") {
            getline(ss, line);
            getline(ss, line);
            stringstream ss2(line);
            ss2 >> dump;
            cam_ids.push_back( stoi(dump.substr(string("/dev/video").size(), dump.size())) );
			cam_names.push_back("APC930");
        }
        else if (dump == "UHD2160") {
            getline(ss, line);
            getline(ss, line);
            stringstream ss2(line);
            ss2 >> dump;
            cam_ids.push_back( stoi(dump.substr(string("/dev/video").size(), dump.size())) );
			cam_names.push_back("UHD2160");
        }
    }
	return make_tuple(zed_ids, cam_ids, cam_names);
}

static vector<string> findFileList(string path, string ext)
{
	char buffer[128];
	string cmd = "find " + path + " -name \"*."+ext+"\"";
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) cerr << "popen faeild" << endl;

	vector<string> fileLists;
	while( !feof(pipe) ) {
		if (fgets(buffer, 128, pipe) != NULL) {
			string s(buffer);
			s.pop_back();
			fileLists.push_back(s);
		}
	}
	pclose(pipe);

	return fileLists;
}


static double convertToSeconds(const std::string& yymmdd, const std::string& hhmmss, const std::string& ms) {
    double years = stoi(yymmdd.substr(0, 4));
	double months = stoi(yymmdd.substr(4, 2));
	double days = stoi(yymmdd.substr(6, 2));
	double hours = stoi(hhmmss.substr(0, 2));
    double minutes = stoi(hhmmss.substr(2, 2));
    double seconds = stoi(hhmmss.substr(4, 2));
	double miliseconds = stod(ms) * 0.001;
	// years * 31536000 + months * 2592000 + days * 86400 + hours * 3600 + minutes * 60 + seconds + miliseconds;
    return hours * 3600 + minutes * 60 + seconds + miliseconds;
}

#endif