/*
 * @Author: your name
 * @Date: 2021-09-24 13:50:48
 * @LastEditTime: 2021-09-26 11:05:46
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /livox_lidar_camera_calib/base.hpp
 */

#ifndef BASE_H
#define BASE_H

#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace std;

struct CameraInfo
{
    cv::Mat intrinsic;
    cv::Mat distortion;
};

struct Option
{
    // for camera calibration
    bool calib_camera;
    bool undistort_image;
    string image_calib_file_path = "/home/tdt/Data_tdt/livox_calib_test/camera_calib/";  // 单独标定相机时图像的路径
    string image_calib_result_path = "/home/tdt/Data_tdt/livox_calib_test/camera_calib/intrinsic.txt";    // 单独标定相机时结果保存的路径
    int col_num;    // corners each row
    int row_num;    // corners each col
    int width;      // width mm
    int height;     // height mm
    // used if camera is pre-calibrated
    double fx, fy, cx, cy;
    double k1, k2, k3, p1, p2;
    // for converting lvx file to pcd file
    bool lvx2pcd;
    double pcd_duration;
    string lidar_path = "/home/tdt/Data_tdt/livox_calib_test/camera_lidar_calib/";   // 联合标定时lvx文件的路径

    // for joint calibration
    bool joint_calib;
    string joint_image_path = "/home/tdt/Data_tdt/livox_calib_test/camera_lidar_calib/";   // 联合标定时图像的路径
    string lidar_corner_path = "/home/tdt/Data_tdt/livox_calib_test/camera_lidar_calib/corner_lidar_raw.txt";
    string joint_calib_result_path = "/home/tdt/Data_tdt/livox_calib_test/camera_lidar_calib/extrinsic.txt";
    double scale;   // 有时候图像太大了，没法完全在屏幕上显示，因此先缩小，再显示
    
    // for virtualization
    double max_depth;       // max depth in depth map in meter

};

string float2str(float num);
float str2float(string str);

double str2double(string str);
string double2str(double num);

int str2int(string str);
string int2str(int num);

string long2str(long num);

void IterateFiles(string pathName, vector<string> &fileNames, string fileType);

// convert a int to a string
string int2str(int num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

int str2int(string str) {
    int d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to int" << endl;
    exit(0);
}

string float2str(float num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

float str2float(string str) {
    float d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to float" << endl;
    exit(0);
}

string double2str(double num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[double2str] - Code error" << endl;
        exit(0);
    }
}

double str2double(string str) {
    double d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to double" << endl;
    exit(0);
}

string long2str(long num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[long2str] - Code error" << endl;
        exit(0);
    }
}

void IterateFiles(string pathName, vector<string> &fileNames, string fileType)
{
    fileNames.clear();
    boost::filesystem::directory_iterator endIter;
    for (boost::filesystem::directory_iterator iter(pathName); iter != endIter; ++iter)
    {
        if (boost::filesystem::is_regular_file(iter->status()))
        {
            std::string type = iter->path().extension().string();
            transform(type.begin(), type.end(), type.begin(), (int (*)(int))tolower);
            if (type == fileType)
                fileNames.push_back(iter->path().string());
        }
        else if (boost::filesystem::is_directory(iter->path()))
        {
            cout << iter->path().string() << endl;
            vector<string> names;
            IterateFiles(iter->path().string(), names, fileType);
            fileNames.insert(fileNames.end(), names.begin(), names.end());
        }
    }
    sort(fileNames.begin(), fileNames.end(), std::less<std::string>());
}

int ReadExtrinsic(string path, Eigen::Matrix4d& T_cl)
{
    ifstream f(path);
    if(!f.is_open())
    {
        cout << "can not open file " << path << endl;
        return 0;
    }
    T_cl = Eigen::Matrix4d::Identity();
    for(int i = 0; i < 3; i++)
    {
        string line;
        getline(f, line);
        stringstream line_lidar(line);
        string str;
        for(int j = 0; j < 4; j++)
        {
            line_lidar >> str;
            T_cl(i,j) = str2double(str);
        }
    }
    return 0;
    
}

#endif