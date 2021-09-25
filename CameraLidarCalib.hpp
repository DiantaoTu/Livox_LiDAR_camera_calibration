/*
 * @Author: your name
 * @Date: 2021-09-24 14:31:26
 * @LastEditTime: 2021-09-25 13:22:04
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /livox_lidar_camera_calib/CameraLiDARCalib.hpp
 */

#include "base.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ceres/ceres.h>
#include <set>


using namespace std;

vector<cv::Point2f> corners;    // save corners in one image

struct PnP
{
    double x, y, z, u, v;
    PnP(Eigen::Vector3d lidar, cv::Point2f image)
    {
        x = lidar.x();
        y = lidar.y();
        z = lidar.z();
        u = image.x;
        v = image.y;
    }
    PnP()
    {
        x = -1;
        y = -1;
        z = -1;
        u = -1;
        v = -1;
    }
};


class external_cali {
public:
    external_cali(PnP p, Eigen::Matrix3d _intrinsic) {
        pd = p;
        intrinsic = _intrinsic;
    }

    template <typename T>
    bool operator()(const T *_q, const T *_t, T *residuals) const {
        Eigen::Matrix<T, 3, 3> innerT = intrinsic.cast<T>();
        Eigen::Quaternion<T> q_incre{_q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[ 0 ], _t[ 1 ], _t[ 2 ]};

        Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
        Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
        Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;

        residuals[0] = p_2[0]/p_2[2] - T(pd.u);
        residuals[1] = p_2[1]/p_2[2] - T(pd.v);

        return true;
    }

    static ceres::CostFunction *Create(PnP p, Eigen::Matrix3d K) {
        return (new ceres::AutoDiffCostFunction<external_cali, 2, 4, 3>(new external_cali(p, K)));
    }


private:
    PnP pd;
    Eigen::Matrix3d intrinsic;

};

// save data on each click
void On_mouse(int event, int x, int y, int flage, void* data)
{
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        cv::Point2f p;
        p.x = x;
        p.y = y;
        cout << "x: " << x << "  y:  " << y << endl;
        corners.push_back(p);
    }
        
}

/**
 * @description: 读取雷达的各个角点坐标
 * @param {lidar_corner_path} 雷达角点保存的文件
 * @param {lidar_corner_all} 读取的结果
 * @return {*}
 */
int ReadLidarCorner(string lidar_corner_path, vector<Eigen::Vector3d>& lidar_corner_all)
{
    // 读取lidar的corner点的坐标
    ifstream fin(lidar_corner_path);
    if(!fin.is_open())
    {
        cout << "can not open lidar corner file: " << lidar_corner_path << endl;
        return 0;
    }
    string line;
    while(getline(fin, line))
    {
        if(line.size() <= 10 )
            continue;
        Eigen::Vector3d point;
        stringstream line_lidar(line);
        string str;
        line_lidar >> str;
        point[0] = str2double(str);
        line_lidar >> str;
        point[1] = str2double(str);
        line_lidar >> str;
        point[2] = str2double(str);
        lidar_corner_all.push_back(point);
    }
    fin.close();
    return 1;
}

int ReadImageCorner(string image, vector<cv::Point2f>& image_corner_all)
{
    // 读取lidar的corner点的坐标
    ifstream fin(image);
    if(!fin.is_open())
    {
        cout << "can not open image corner file: " << image << endl;
        return 0;
    }
    string line;
    while(getline(fin, line))
    {
        cv::Point2f point;
        stringstream line_lidar(line);
        string str;
        line_lidar >> str;
        point.x = str2double(str);
        line_lidar >> str;
        point.y = str2double(str);
        image_corner_all.push_back(point);
    }
    fin.close();
    return 1;
}

/**
 * @description: 每次显示一张图，用鼠标在上面点出四个角，之后精细化四个角的坐标并保存
 * @param {string} image_path, 保存了用于标定的图像的文件夹
 * @param {CameraInfo} camera_info, 相机内参
 * @return {*}
 */
int GetCameraCorner(string image_path, const CameraInfo camera_info, vector<cv::Point2f>& image_corner_all)
{
    vector<string> imageNames;
    IterateFiles(image_path, imageNames, ".jpg");
    if(imageNames.size() == 0)
    {
        cout << "no image in " << image_path << " that ends with .jpg" << endl;
        return 0;
    }
    for(string name :imageNames)
    {
        corners.clear();
        cv::Mat src_img = cv::imread(name);
        cv::Mat undistort_img;
        undistort(src_img, undistort_img, camera_info.intrinsic, camera_info.distortion);
        cv::namedWindow("source");
        cv::imshow("source", undistort_img);
        cv::setMouseCallback("source",On_mouse);
        cv::waitKey(0);
        // 提取亚像素精度的角点
        cv::Mat gray_img;
        cv::cvtColor(undistort_img, gray_img, cv::COLOR_BGR2GRAY);
        cv::Size winSize = cv::Size(5, 5);
        cv::Size zerozone = cv::Size(-1, -1);
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
        cv::cornerSubPix(gray_img, corners, winSize, zerozone, criteria);
        image_corner_all.insert(image_corner_all.end(), corners.begin(), corners.end());
    }
}

int JointCalibration(string calib_result_path, vector<PnP> pnp_all, Eigen::Matrix3d intrinsic,
                    Eigen::Matrix4d& T_cl)
{
    // init calibration 
    Eigen::Matrix3d rot_init = Eigen::Matrix3d::Zero();
    rot_init(0,1) = -1;
    rot_init(1,2) = -1;
    rot_init(2,0) = 1;
    Eigen::Quaterniond q(rot_init);
    Eigen::Vector3d t(0,0,0);   
    double ext[7] = {q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z()};

    Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
    Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);

    ceres::LocalParameterization * q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem problem;
    problem.AddParameterBlock(ext, 4, q_parameterization);
    problem.AddParameterBlock(ext + 4, 3);

    for(auto p: pnp_all)
    {
        ceres::CostFunction *cost_function = external_cali::Create(p, intrinsic);
        problem.AddResidualBlock(cost_function, NULL, ext, ext + 4);
    }
    if(problem.NumResidualBlocks() == 0)
    {
        cout << "no residual!" << endl;
        return 0;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;

    Eigen::Matrix3d R = m_q.toRotationMatrix();
    ofstream f(calib_result_path + "/extrinsic.txt");
    f << R(0, 0) << "  " << R(0, 1) << "  " << R(0, 2) << "  " << m_t[0] << endl;
    f << R(1, 0) << "  " << R(1, 1) << "  " << R(1, 2) << "  " << m_t[1] << endl;
    f << R(2, 0) << "  " << R(2, 1) << "  " << R(2, 2) << "  " << m_t[2] << endl;
    f.close();
    T_cl = Eigen::Matrix4d::Identity();
    T_cl.block<3,3>(0,0) = R;
    T_cl.block<3,1>(0,3) = m_t;
    return 1;
}

// project lidar corners to image and compute the reprojection error
/**
 * @description: project lidar corners to image and compute the reprojection error
 * @param {T_cl} the calibration, transform point from lidar to camera
 * @param {intrinsic} camera intrinsic parameters
 * @param {bad_ID} index of pnp points with error larger than threshold
 * @param
 */
int EvaluateCalibration(vector<Eigen::Vector3d> lidar_corner_all, vector<cv::Point2f> image_corner_all,
                        Eigen::Matrix4d T_cl, Eigen::Matrix3d intrinsic, 
                        double error_threshold, set<int>& bad_ID)
{
    bad_ID.clear();
    Eigen::Matrix3d rot = T_cl.block<3,3>(0,0);
    Eigen::Vector3d trans = T_cl.block<3,1>(0,3);
    double totalX = 0, totalY = 0;
    
    for(int i = 0; i < lidar_corner_all.size(); i++)
    {
        Eigen::Vector3d l_proj = intrinsic * (rot * lidar_corner_all[i] + trans);
        l_proj /= l_proj[2];
        double diffX = fabs(l_proj[0] - image_corner_all[i].x);
        double diffY = fabs(l_proj[1] - image_corner_all[i].y);
        if(diffX + diffY > error_threshold)
        {
            cout << "pnp data " << i << " has error larger than threshold" << endl;
            bad_ID.insert(i);
        }
        totalX += diffX;
        totalY += diffY;
    }
    cout << "average error in X: " << totalX / lidar_corner_all.size() << endl;
    cout << "average error in Y: " << totalY / lidar_corner_all.size() << endl;
    return 1;
}

/**
 * @description: calibration between a camera and a lidar
 * @param {T_cl} result of the calibration
 * @return {*}
 */
int CameraLiDARCalib(Option option, CameraInfo camera_info, Eigen::Matrix4d& T_cl)
{
    if(!boost::filesystem::exists(option.joint_calib_result_path))
        boost::filesystem::create_directories(option.joint_calib_result_path);

    vector<cv::Point2f> image_corner_all;
    string image_corner_path = option.joint_calib_result_path + "/image_corner.txt";
    if(boost::filesystem::exists(image_corner_path))
    {
        cout << "find exist image corner path at " << image_corner_path << endl;
        cout << "load existing image corner ..." << endl;
        ReadImageCorner(image_corner_path, image_corner_all);
    }
    else 
    {
        GetCameraCorner(option.joint_image_path, camera_info, image_corner_all);
        ofstream f(option.joint_calib_result_path + "/image_corner.txt");
        for(cv::Point2f p : image_corner_all)
        {
            f << p.x << " " << p.y << endl;
        }
        f.close();
    }

    vector<Eigen::Vector3d> lidar_corner_all;
    ReadLidarCorner(option.lidar_corner_path, lidar_corner_all);
    assert(lidar_corner_all.size() == image_corner_all.size());
    
    vector<PnP> pnp_all;
    for(int i = 0 ; i < lidar_corner_all.size(); i++)
    {
        PnP pnp(lidar_corner_all[i], image_corner_all[i]);
        pnp_all.push_back(pnp);
    }

    Eigen::Matrix3d intrinsic;
    cv::cv2eigen(camera_info.intrinsic, intrinsic);
    JointCalibration(option.joint_calib_result_path, pnp_all, intrinsic, T_cl);
    set<int> del_ID;
    EvaluateCalibration(lidar_corner_all, image_corner_all, T_cl, intrinsic, 12, del_ID);
    if(del_ID.empty())
    {
        cout << "all corners are good, camera LiDAR calibration finish" << endl;
        return 1;
    }
    else if (del_ID.size() == lidar_corner_all.size())
    {
        cout << "all corners are bad, maybe the error threshold is too small" << endl;
        return 0;
    }
    else 
    {
        // delete pnp data with large reprojection error
        vector<PnP> tmp;
        for(int i = 0; i < pnp_all.size(); i++)
            if(del_ID.count(i) > 0)
                continue;
            else 
                tmp.push_back(pnp_all[i]);
        pnp_all.clear();
        pnp_all = tmp;
        cout << "delete corners with large error and re-calibrate" << endl;
        JointCalibration(option.joint_calib_result_path, pnp_all, intrinsic, T_cl);
        return 1;
    }

}