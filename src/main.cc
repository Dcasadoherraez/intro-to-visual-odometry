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
    
    // read images from dataset
    string img_path = DATASET_PATH + DATASET_SUFFIX + "000000.png";
    cv::Mat curr_img = cv::imread(img_path);


    if (curr_img.empty()) cout << "Image with name " << DATASET_PATH << " not read..." << endl;

    // read timestamps simultaneously
    string TIMESTAMPS_PATH = DATASET_PATH + "times.txt";
    ifstream timestamps_file (TIMESTAMPS_PATH);
    cout << TIMESTAMPS_PATH << endl;
    if (!timestamps_file.is_open()) cout << "Error opening timestamps file: " + TIMESTAMPS_PATH;

    int img_ct = 0;
    while (!curr_img.empty()) {
        // read image file
        string img_name = getKITTIName(img_ct);
        img_path = DATASET_PATH + DATASET_SUFFIX + img_name;
        curr_img = cv::imread(img_path);

        // read timestamp
        string timestamp;
        getline(timestamps_file, timestamp);

        Frame new_frame = Frame(img_ct, curr_img, stod(timestamp));
        new_frame.GetFeatures(curr_img);

        img_ct++;
        break;
        if (img_ct >=5 ) break;
    }
    
    return 1;
}