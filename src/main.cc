#include "frame.h"
#include "frontend.h"
#include "map.h"
#include "mappublisher.h"
#include "mappoint.h"

#include "common_include.h"

using namespace std;

// SIGIN handling
volatile sig_atomic_t stop = 0;
void inthand(int signum) {
    stop = 1;
}

string getKITTIName(int img_num)
{
    string name = to_string(img_num);
    name = string(6 - name.length(), '0') + name;

    return name + ".png";
}

vector<Camera::Ptr> loadKITTI(string filename)
{
    vector<Camera::Ptr> cameras;
    ifstream fin(filename);
    if (!fin)
    {
        cout << "[ERROR] cannot find " << filename << "/calib.txt!" << endl;
        return {};
    }

    for (int i = 0; i < 2; ++i)
    {
        char camera_name[3];
        for (int k = 0; k < 3; ++k)
        {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k)
        {
            fin >> projection_data[k];
        }
        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        Sophus::SE3d pose_init = Sophus::SE3d(Sophus::SO3d(), t);
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), pose_init));
        // if rectified
        cameras.push_back(new_camera);
        cout << "[INFO] Camera " << i << " extrinsics: " << t.transpose() << endl;
        cout << "[INFO] Camera " << i << " intrinsics: \n"
             << K.matrix() << endl;
    }
    fin.close();

    return cameras;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_pcl");

    string DATASET_PATH = "/ssd1_lin/Thesis/ORB-SLAM/datasets/KITTI/dataset/sequences/00/";
    string DATASET_SUFFIX = "image_0/";
    string CAM0 = "image_0/";
    string CAM1 = "image_1/";

    // read images from dataset
    string img_path = DATASET_PATH + DATASET_SUFFIX + "000000.png";
    cv::Mat curr_img0 = cv::imread(img_path);
    cv::Mat curr_img1;

    if (curr_img0.empty())
        cout << "Image with name " << DATASET_PATH << " not read..." << endl;

    // read timestamps simultaneously
    string TIMESTAMPS_PATH = DATASET_PATH + "times.txt";
    ifstream timestamps_file(TIMESTAMPS_PATH);
    cout << TIMESTAMPS_PATH << endl;
    if (!timestamps_file.is_open())
        cout << "Error opening timestamps file: " + TIMESTAMPS_PATH;

    vector<Camera::Ptr> cameras = loadKITTI(DATASET_PATH + "calib.txt");

    int img_ct = 0;
    bool initialized = false;

    Map::Ptr map(new Map(0));
    Frontend::Ptr frontend(new Frontend(cameras[0], cameras[1]));
    MapPublisher::Ptr mapPub(new MapPublisher(map));

    while (!curr_img0.empty() && !stop)
    {
        signal(SIGINT, inthand);
        
        // read image file
        string img_name = getKITTIName(img_ct);
        string cam0_img_path = DATASET_PATH + CAM0 + img_name;
        string cam1_img_path = DATASET_PATH + CAM1 + img_name;

        curr_img0 = cv::imread(cam0_img_path, IMREAD_GRAYSCALE);
        curr_img1 = cv::imread(cam1_img_path, IMREAD_GRAYSCALE);

        // read timestamp
        string timestamp;
        getline(timestamps_file, timestamp);

        // create frame class instance
        Frame::Ptr new_frame(new Frame(img_ct, curr_img0, curr_img1, stod(timestamp)));
        new_frame->GetFeatures();
        vector<DMatch> matches = new_frame->MatchFeatures();
        // new_frame->DisplayMatches();

        // initialize map
        if (!initialized)
        {
            initialized = !initialized;
            map->InitMap();
            frontend->_map = map;
            frontend->_current_frame = new_frame;
            frontend->ProjectFeatures(new_frame->_features_left, new_frame->_features_right, matches);
        }

        geometry_msgs::TransformStamped tfS;
        tfS.header.stamp = ros::Time::now();
        tfS.transform.translation.x = 0;
        tfS.transform.translation.y = 0;
        tfS.transform.translation.z = 0;
        tfS.transform.rotation.x = 0;
        tfS.transform.rotation.y = 0;
        tfS.transform.rotation.z = 0;
        tfS.transform.rotation.w = 1;
        tfS.header.frame_id = "ORB_SLAM/World";
        tfS.child_frame_id = "ORB_SLAM/Camera";

        frontend->tfBr.sendTransform(tfS);

        // publish map
        Vec3 t(0, 0, 0);
        Mat33 R;
        R << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        Sophus::SE3d currentCamPose(R, t);
        mapPub->SetCurrentCameraPose(currentCamPose);
        mapPub->Refresh();

        img_ct++;
        // break;
        //  if (img_ct >=5 ) break;
    }

    return 1;
}