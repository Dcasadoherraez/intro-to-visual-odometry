#include "common_include.h"
#include "frame.h"
#include "map.h"
#include "frontend.h"

using namespace std;

string getKITTIName(int img_num) {
    string name = to_string(img_num);
    name = string(6 - name.length(), '0') + name;

    return name + ".png";
}

vector<Camera> loadKITTI(string filename) {
    vector<Camera> cameras;
    ifstream fin(filename );
    if (!fin) {
        cout << "[ERROR] cannot find " << filename << "/calib.txt!" << endl;
        return {};
    }

    for (int i = 0; i < 4; ++i) {
        char camera_name[3];
        for (int k = 0; k < 3; ++k) {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }
        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        K = K * 0.5;
        Sophus::SE3d pose_init = Sophus::SE3d(Sophus::SO3d(), t);
        Camera new_camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                          t.norm(), pose_init);
        cameras.push_back(new_camera);
        cout << "[INFO] Camera " << i << " extrinsics: " << t.transpose() << endl;
    }
    fin.close();

    return cameras;
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
    ifstream timestamps_file(TIMESTAMPS_PATH);
    cout << TIMESTAMPS_PATH << endl;
    if (!timestamps_file.is_open()) cout << "Error opening timestamps file: " + TIMESTAMPS_PATH;

    vector<Camera> cameras = loadKITTI(DATASET_PATH + "calib.txt");

    int img_ct = 0;
    bool initialized = false;

    Map::Ptr map(new Map());
    Frontend::Ptr frontend(new Frontend(cameras[0], cameras[1]));

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

        // create frame class instance
        Frame::Ptr new_frame(new Frame(img_ct, curr_img0, curr_img1, stod(timestamp)));
        new_frame->GetFeatures();
        vector<DMatch> matches = new_frame->MatchFeatures();
        new_frame->DisplayMatches();

        // initialize map 
        if (!initialized) {
            initialized = !initialized;
            map->InitMap();
            frontend->_map = map;
            frontend->_current_frame = new_frame;
            frontend->ProjectFeatures(new_frame->_features_left, new_frame->_features_right, matches);
        }

        img_ct++;
        //break;
        if (img_ct >=5 ) break;
    }
    
    return 1;
}