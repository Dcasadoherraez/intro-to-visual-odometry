#include <iostream>
#include <string>
#include <iostream>
#include <fstream>

#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "Eigen/Core"

#include "frame.h"

using namespace std;



string getKITTIName(int img_num) {
    string name = to_string(img_num);
    name = string(6 - name.length(), '0') + name;

    return name + ".png";
}

int main() {
    string DATASET_PATH = "/ssd1_lin/Thesis/ORB-SLAM/datasets/KITTI/dataset/sequences/00/";
    string DATASET_SUFFIX = "image_0/";
    string CAM0 = "image_0/";
    string CAM1 = "image_1/";
    
    // read images from dataset
    string img_path = DATASET_PATH + DATASET_SUFFIX + "000000.png";
    cv::Mat curr_img0 = cv::imread(img_path);
    cv::Mat curr_img1;


    if (curr_img0.empty()) cout << "Image with name " << DATASET_PATH << " not read..." << endl;

    // read timestamps simultaneously
    string TIMESTAMPS_PATH = DATASET_PATH + "times.txt";
    ifstream timestamps_file (TIMESTAMPS_PATH);
    cout << TIMESTAMPS_PATH << endl;
    if (!timestamps_file.is_open()) cout << "Error opening timestamps file: " + TIMESTAMPS_PATH;

    int img_ct = 0;
    while (!curr_img0.empty()) {
        // read image file
        string img_name = getKITTIName(img_ct);
        string cam0_img_path = DATASET_PATH + CAM0 + img_name;
        string cam1_img_path = DATASET_PATH + CAM1 + img_name;

        curr_img0 = cv::imread(cam0_img_path);
        curr_img1 = cv::imread(cam1_img_path);

        // read timestamp
        string timestamp;
        getline(timestamps_file, timestamp);

        Frame new_frame = Frame(img_ct, curr_img0, curr_img1, stod(timestamp));
        new_frame.GetFeatures();
        new_frame.MatchFeatures();
        new_frame.DisplayMatches();

        img_ct++;
        break;
        if (img_ct >=5 ) break;
    }
    
    return 1;
}