/*
 * @Author: your name
 * @Date: 2021-09-24 13:16:22
 * @LastEditTime: 2021-09-26 10:18:54
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /livox_lidar_camera_calib/main.cpp
 */
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <string.h>
#include <boost/program_options.hpp>
#include "Livox2PCD.hpp"
#include "CameraCalib.hpp"
#include "CameraLidarCalib.hpp"
#include "Virtualization.hpp"


using namespace std;



int main(int argc, char** argv)
{
    Option option;
    boost::program_options::options_description config("Options");
    config.add_options()
        ("calib_camera", boost::program_options::value<bool>(&option.calib_camera), "camera calibration")
        ("undistort_image", boost::program_options::value<bool>(&option.undistort_image), "output the image after calibration")
        ("image_calib_path", boost::program_options::value<string>(&option.image_calib_file_path), "path to image used for camera calibration")
        ("image_calib_result_path", boost::program_options::value<string>(&option.image_calib_result_path), "path to image calibration result")
        ("col_num", boost::program_options::value<int>(&option.col_num)->default_value(17))
        ("row_num", boost::program_options::value<int>(&option.row_num)->default_value(12))
        ("width", boost::program_options::value<int>(&option.width)->default_value(30))
        ("height", boost::program_options::value<int>(&option.height)->default_value(30))
        ("fx", boost::program_options::value<double>(&option.fx)->default_value(-1), "only used when calib_camera=false")
        ("fy", boost::program_options::value<double>(&option.fy)->default_value(-1), "only used when calib_camera=false")
        ("cx", boost::program_options::value<double>(&option.cx)->default_value(-1), "only used when calib_camera=false")
        ("cy", boost::program_options::value<double>(&option.cy)->default_value(-1), "only used when calib_camera=false")
        ("k1", boost::program_options::value<double>(&option.k1)->default_value(0), "only used when calib_camera=false")
        ("k2", boost::program_options::value<double>(&option.k2)->default_value(0), "only used when calib_camera=false")
        ("k3", boost::program_options::value<double>(&option.k3)->default_value(0), "only used when calib_camera=false")
        ("p1", boost::program_options::value<double>(&option.p1)->default_value(0), "only used when calib_camera=false")
        ("p2", boost::program_options::value<double>(&option.p2)->default_value(0), "only used when calib_camera=false")
        ("lvx2pcd", boost::program_options::value<bool>(&option.lvx2pcd), "convert lvx file to pcd")
        ("pcd_duration", boost::program_options::value<double>(&option.pcd_duration)->default_value(-1))
        ("joint_calib", boost::program_options::value<bool>(&option.joint_calib), "camera-lidar joint calibration")
        ("lidar_path",boost::program_options::value<string>(&option.lidar_path), "path to lvx folder")
        ("joint_image_path", boost::program_options::value<string>(&option.joint_image_path), "image folder for joint calibration")
        ("lidar_corner_path", boost::program_options::value<string>(&option.lidar_corner_path), "file contains lidar corner")
        ("joint_calib_result_path", boost::program_options::value<string>(&option.joint_calib_result_path))
        ("scale",boost::program_options::value<double>(&option.scale)->default_value(1))
        ("max_depth",boost::program_options::value<double>(&option.max_depth)->default_value(6))
    ;


    boost::program_options::variables_map vm;

	if (argc == 2)
	{
		std::string configFile = argv[1];
		std::ifstream ifs(configFile);
		if (!ifs)
		{
			cout << "Cannot open configfile: " << configFile << endl;
			return 1;
		}
		try
		{
			boost::program_options::store(boost::program_options::parse_config_file(ifs, config), vm);
			boost::program_options::notify(vm);
		}
		catch (const std::exception& e)
		{
			cout << e.what() << endl;
			return 1;
		}
	}
    else 
    {
        cout << "usage ./livox_reader config.txt" << endl;
        return 0;
    }
    
    // 1. 对相机进行标定
    CameraInfo camera_info;
    if(option.calib_camera)
        CameraCalib(option, camera_info);
    else 
    {
        camera_info.intrinsic = cv::Mat::eye(3, 3, CV_64FC1);
        camera_info.intrinsic.at<double>(0,0) = option.fx;
        camera_info.intrinsic.at<double>(1,1) = option.fy;
        camera_info.intrinsic.at<double>(0,2) = option.cx;
        camera_info.intrinsic.at<double>(1,2) = option.cy;
        camera_info.distortion = cv::Mat::zeros(1,5, CV_64FC1);
        camera_info.distortion.at<double>(0,0) = option.k1;
        camera_info.distortion.at<double>(0,1) = option.k2;
        camera_info.distortion.at<double>(0,2) = option.p1;
        camera_info.distortion.at<double>(0,3) = option.p2;
        camera_info.distortion.at<double>(0,4) = option.k3;

    }
    cout << camera_info.intrinsic << endl;
    cout << camera_info.distortion << endl;
    // 2. 把联合标定时需要用的雷达从lvx转换为pcd格式
    if(option.lvx2pcd)
        Livox2PCD(option.lidar_path, option.pcd_duration);
    // 3. 联合标定
    Eigen::Matrix4d T_cl = Eigen::Matrix4d::Identity();
    if(option.joint_calib)
        CameraLiDARCalib(option,camera_info, T_cl);
    // 4. 点云上色
    T_cl << 0.0326621, -0.999465, 0.00162067, 0.0197441,
        0.00771205, -0.00136946, -0.999969, 0.0831867,
        0.999437, 0.0326736, 0.0076632, -0.0576599,
        0,  0,  0,  1;
    ColorCloud(option, camera_info, T_cl);
    ProjectCloud2Image(option, camera_info, T_cl);
    return 0;
}