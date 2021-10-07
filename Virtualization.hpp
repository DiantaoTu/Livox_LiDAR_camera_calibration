/*
 * @Author: Diantao Tu
 * @Date: 2021-09-24 18:11:14
 * @LastEditTime: 2021-10-07 16:41:29
 */

#include "base.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> 

using namespace std;

int ColorCloud(Option option,
                CameraInfo camera_info, Eigen::Matrix4d T_cl)
{
    cout << "---------- texture point cloud begin --------" << endl;
    vector<string> pcdNames;
    IterateFiles(option.lidar_path, pcdNames, ".pcd");
    vector<string> imageNames;
    IterateFiles(option.joint_image_path, imageNames, ".jpg");
    assert(pcdNames.size() == imageNames.size());
    for(int i = 0; i < pcdNames.size(); i++)
    {
        cv::Mat src_img = cv::imread(imageNames[i]);
        cv::Mat undis_img;
        cv::undistort(src_img, undis_img, camera_info.intrinsic, camera_info.distortion);
        cv::Mat depth_map(src_img.size(), CV_32FC1);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::io::loadPCDFile(pcdNames[i], cloud);
        pcl::transformPointCloud(cloud, cloud, T_cl);
        pcl::PointCloud<pcl::PointXYZRGB> cloud_color;
        for(auto p : cloud.points)
        {
            // cv::Vec3f point(p.x, p.y, p.z);
            cv::Mat point(3,1,CV_64FC1,Scalar(0));
            point.at<double>(0,0) = p.x;
            point.at<double>(1,0) = p.y;
            point.at<double>(2,0) = p.z;
            point = camera_info.intrinsic * point;
            // point /= point.z;
            int u = round(point.at<double>(0,0) / point.at<double>(0,2));
            int v = round(point.at<double>(1,0) / point.at<double>(0,2));
            if(u >= undis_img.cols || v >= undis_img.rows || v < 0 || u < 0)
                continue;
            // u,v 是坐标，那么相应的行列就是第v行第u列
            cv::Vec3b bgr = undis_img.at<cv::Vec3b>(v, u);
            // opencv read image in BGR order
            pcl::PointXYZRGB point_color(bgr[2], bgr[1], bgr[0]);
            point_color.x = p.x;
            point_color.y = p.y;
            point_color.z = p.z;
            cloud_color.push_back(point_color);
            
        }
        // 把原本的路径名从 aaa/bbb/ccc/xxxx.pcd 变成 xxxx_color.pcd
        string::size_type pos = pcdNames[i].rfind('/');
        string cloud_name = pcdNames[i].substr(pos + 1);
        pos = cloud_name.rfind('.');
        cloud_name = cloud_name.substr(0, pos);
        cloud_name = option.joint_calib_result_path + '/' + cloud_name + "_color.pcd";
        pcl::io::savePCDFileASCII(cloud_name, cloud_color);
    }
    cout << "---------- texture point cloud end ----------" << endl;
    return 1;
}

//灰度图转为彩虹图:灰度值255~0分别对应：红、橙、黄、绿、青、蓝。
cv::Vec3b Gray2Color(uchar gray)
{
    cv::Vec3b pixel;
    if (gray == 0)
    {
        pixel[0] = 0;
        pixel[1] = 0;
        pixel[2] = 0;
    }
    else if (gray <= 51)
    {
        pixel[0] = 255;
        pixel[1] = gray * 5;
        pixel[2] = 0;
    }
    else if (gray <= 102)
    {
        gray -= 51;
        pixel[0] = 255 - gray * 5;
        pixel[1] = 255;
        pixel[2] = 0;
    }
    else if (gray <= 153)
    {
        gray -= 102;
        pixel[0] = 0;
        pixel[1] = 255;
        pixel[2] = gray * 5;
    }
    else if (gray <= 204)
    {
        gray -= 153;
        pixel[0] = 0;
        pixel[1] = 255 - static_cast<unsigned char>(128.0 * gray / 51.0 + 0.5);
        pixel[2] = 255;
    }
    else
    {
        gray -= 204;
        pixel[0] = 0;
        pixel[1] = 127 - static_cast<unsigned char>(127.0 * gray / 51.0 + 0.5);
        pixel[2] = 255;
    }
    return pixel;
}

int ProjectCloud2Image(Option option, CameraInfo camera_info, Eigen::Matrix4d T_cl)
{
    vector<string> pcdNames;
    IterateFiles(option.lidar_path, pcdNames, ".pcd");
    vector<string> imageNames;
    IterateFiles(option.joint_image_path, imageNames, ".jpg");
    assert(pcdNames.size() == imageNames.size());
    Eigen::Matrix3d intrinsic;
    cv::cv2eigen(camera_info.intrinsic, intrinsic);
    for(int i = 0; i < pcdNames.size(); i++)
    {
        cv::Mat src_img = cv::imread(imageNames[i]);
        cv::Mat undis_img;
        cv::undistort(src_img, undis_img, camera_info.intrinsic, camera_info.distortion);
        bool high_res = (undis_img.rows * undis_img.cols > 3000 * 2000);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::io::loadPCDFile(pcdNames[i], cloud);
        pcl::transformPointCloud(cloud, cloud, T_cl);
        for(auto p : cloud.points)
        {
            Eigen::Vector3d point(p.x, p.y, p.z);
            point = intrinsic * point;
            if(point[2] <= 0)
                continue;
            float real_depth = point[2];
            int u = ceil(point[0] / real_depth);
            int v = ceil(point[1] / real_depth);
            if(point[2] > option.max_depth)
                point[2] = option.max_depth;
            uchar relative_depth = static_cast<unsigned char>(point[2] / option.max_depth * 255);
            cv::Vec3b color = Gray2Color(relative_depth);
            if(u < undis_img.cols && v < undis_img.rows && v > 0 && u > 0)       
                undis_img.at<cv::Vec3b>(v,u) = color;   // u,v 是坐标，那么相应的行列就是第v行第u列
            // if the image is high resolution, set the project point to 4 pixel for better virtualization
            if(!high_res)
                continue;
            u = floor(point[0] / real_depth);
            v = floor(point[1] / real_depth);
            if(u < undis_img.cols && v < undis_img.rows && v > 0 && u > 0)       
                undis_img.at<cv::Vec3b>(v,u) = color;   

            u = ceil(point[0] / real_depth);
            v = floor(point[1] / real_depth);
            if(u < undis_img.cols && v < undis_img.rows && v > 0 && u > 0)       
                undis_img.at<cv::Vec3b>(v,u) = color;   

            u = floor(point[0] / real_depth);
            v = ceil(point[1] / real_depth);
            if(u < undis_img.cols && v < undis_img.rows && v > 0 && u > 0)       
                undis_img.at<cv::Vec3b>(v,u) = color;   
            
        }
        // 把原本的路径名从 aaa/bbb/ccc/xxxx.pcd 变成 xxxx_color.pcd
        string::size_type pos = imageNames[i].rfind('/');
        string depth_name = imageNames[i].substr(pos + 1);
        pos = depth_name.rfind('.');
        depth_name = depth_name.substr(0, pos);
        depth_name = option.joint_calib_result_path + '/' + depth_name + "_depth.jpg";
        cv::imwrite(depth_name, undis_img);
    }
    
}