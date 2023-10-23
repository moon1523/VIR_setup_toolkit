#include "SyncCamera.hh"
#include "RecordVideo.hh"
#include "ViewCoords.hh"
#include "BoardCalib.hh"

void PrintLogo()
{
    std::cout << "===================================" << std::endl;
    std::cout << " __  __     ______      ____       " << std::endl;
    std::cout << "/\\ \\/\\ \\   /\\__  _\\    /\\  _`\\     " << std::endl;
    std::cout << "\\ \\ \\ \\ \\  \\/_/\\ \\/    \\ \\ \\_\\ \\   " << std::endl;
    std::cout << " \\ \\ \\ \\ \\    \\ \\ \\     \\ \\ ,  /   " << std::endl;
    std::cout << "  \\ \\ \\_/ \\    \\_\\ \\__   \\ \\ \\\\ \\  " << std::endl;
    std::cout << "   \\ `\\___/    /\\_____\\   \\ \\_\\ \\_\\" << std::endl;
    std::cout << "    `\\/__/     \\/_____/    \\/_/\\/ /" << std::endl << std::endl;
    std::cout << "                      Setup Toolkit" << std::endl;
    std::cout << "Author: Sungho Moon" << std::endl;
    std::cout << "===================================" << std::endl;
}
void PrintUsage()
{
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "[Coordinate system matching option]" << std::endl;
    std::cout << "  --k4a (-board #, default: 3)" << std::endl;
    std::cout << "  --zed (-board #, default: 3)" << std::endl;
    std::cout << "  --uhd2160 (-num #, default: 3)" << std::endl;
    std::cout << "  --apc930 (-num #, default: 3)" << std::endl;
    std::cout << "[Recording option]" << std::endl;
    std::cout << "  --record [output_folder_name]" << std::endl;
    std::cout << "  --view-realtime" << std::endl;
    std::cout << "  --view-record [rsync_input_folder_name]" << std::endl;
    std::cout << "  --sync-record [input_folder_name]" << std::endl;
    std::cout << "[Extra option]" << std::endl;
    std::cout << "  --view-coords (ply_path, default: ./sync)" << std::endl;
    std::cout << "  --write (isocenter_file, default: no calibration)" << std::endl;
    std::cout << "  --calib (output_name)" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char** argv) 
{	
    PrintLogo();
    PrintUsage();
	
    if (argc < 2) {
        return EXIT_SUCCESS;
    }

    std::string option(argv[1]);
    std::cout << option << " is selected." << std::endl << std::endl;
    if (option == "--k4a") {
#ifdef K4A_FOUND
        Sync_Kinect(argc, argv);
#endif
    } else if (option == "--zed") {
        Sync_ZED2(argc, argv);
    } else if (option == "--uhd2160" || option == "--apc930") {
        Sync_Webcam(argc, argv);
    } else if (option == "--record") {
        Recording(argc, argv);
    } else if (option == "--sync-record") {
        Synchronize_Recording(argc, argv);
    } else if (option == "--view-realtime") {
        View_Realtime(argc, argv);
    } else if (option == "--view-record") {
        View_Recording(argc, argv);
    } else if (option == "--view-coords") {
        View_Coordinates(argc, argv);
    } else if (option == "--write") {
        Batch_Write_Camera_Pose(argc, argv);
    } else if (option == "--calib") {
        Calibrate_with_ChArUco(argc, argv);
    } else {
        std::cout << "Invalid option: " << option << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "EXIT_SUCCESS" << std::endl;
	return EXIT_SUCCESS;
}