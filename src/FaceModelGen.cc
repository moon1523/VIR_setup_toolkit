#include "FaceModelGen.hh"

int Generate_FaceModel(int argc, char** argv)
{
    string command;
    command = "cd ./face; sh 0_CropFaces.sh";
    system(command.c_str());
    command = "cd ./face; sh 1_GenListFile.sh";
    system(command.c_str());
    command = "cd ./face; sh 2_Train.sh";
    system(command.c_str());

    cout << "Generate_FaceModel... done." << endl << endl;
    return EXIT_SUCCESS;
}

int Test_FaceModel(int argc, char** argv)
{
    string command;
    command = "cd ./face; sh 3_TestModel.sh";
    system(command.c_str());

    cout << "Test_FaceModel... done." << endl << endl;
    return EXIT_SUCCESS;
}