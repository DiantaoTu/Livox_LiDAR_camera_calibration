/*
 * @Author: your name
 * @Date: 2021-09-24 13:49:28
 * @LastEditTime: 2021-09-25 13:19:06
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /livox_lidar_camera_calib/CameraCalib.hpp
 */

#include "base.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;



int CameraCalib(Option option, CameraInfo& camera_info)
{
    vector<string> imageNames;
    IterateFiles(option.image_calib_file_path, imageNames, ".jpg");
    int image_count = 0;  /* 图像数量 */
	Size image_size;      /* 图像的尺寸 */
	Size board_size = Size(option.row_num, option.col_num);             /* 标定板上每行、列的角点数 */
	vector<Point2f> image_points_buf;         /* 缓存每幅图像上检测到的角点 */
	vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
    for(string name : imageNames)
    {
        ++image_count;
        Mat imageInput = imread(name);
        // 读入第一张图片时获取图片大小
		if (image_count == 1) {
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
		}
        /* 提取角点 */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FILTER_QUADS)) {
			//cout << "can not find chessboard corners!\n";  // 找不到角点
			cout << "**" << name << "** can not find chessboard corners!\n";
			exit(1);
		}
		else {
			Mat view_gray;
			cvtColor(imageInput, view_gray, cv::COLOR_RGB2GRAY);  // 转灰度图

			/* 亚像素精确化 */
			// image_points_buf 初始的角点坐标向量，同时作为亚像素坐标位置的输出
			// Size(5,5) 搜索窗口大小
			// （-1，-1）表示没有死区
			// TermCriteria 角点的迭代过程的终止条件, 可以为迭代次数和角点精度两者的组合
			cornerSubPix(view_gray, image_points_buf, Size(5, 5), Size(-1, -1), TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

			image_points_seq.push_back(image_points_buf);  // 保存亚像素角点

			/* 在图像上显示角点位置 */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); // 用于在图片中标记角点

			imshow("Camera Calibration", view_gray);       // 显示图片

			waitKey(1000); //暂停1S      
		}
    }

    //-------------以下是摄像机标定------------------

	/*棋盘三维信息*/
    int width = 30;
    int height = 30;
	Size square_size = Size(width, height);         /* 实际测量得到的标定板上每个棋盘格的大小 */
	vector<vector<Point3f>> object_points;   /* 保存标定板上角点的三维坐标 */

	/*内外参数*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  /* 摄像机内参数矩阵 */
	vector<int> point_counts;   // 每幅图像中角点的数量
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));       /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	vector<Mat> tvecsMat;      /* 每幅图像的旋转向量 */
	vector<Mat> rvecsMat;      /* 每幅图像的平移向量 */

	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t<image_count; t++) {
		vector<Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++) {
			for (j = 0; j<board_size.width; j++) {
				Point3f realPoint;

				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}

	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	for (i = 0; i<image_count; i++) {
		point_counts.push_back(board_size.width * board_size.height);
	}

	/* 开始标定 */
	// object_points 世界坐标系中的角点的三维坐标
	// image_points_seq 每一个内角点对应的图像坐标点
	// image_size 图像的像素尺寸大小
	// cameraMatrix 输出，内参矩阵
	// distCoeffs 输出，畸变系数
	// rvecsMat 输出，旋转向量
	// tvecsMat 输出，位移向量
	// 0 标定时所采用的算法
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

	//------------------------标定完成------------------------------------

	// -------------------对标定结果进行评价------------------------------

	if(!boost::filesystem::exists(option.image_calib_result_path))
		boost::filesystem::create_directory(option.image_calib_result_path);
		
	double total_err = 0.0;         /* 所有图像的平均误差的总和 */
	double err = 0.0;               /* 每幅图像的平均误差 */
	vector<Point2f> image_points2;  /* 保存重新计算得到的投影点 */
    ofstream fout(option.image_calib_result_path + "/intrinsic.txt");
	fout << "Average error: \n";

	for (i = 0; i<image_count; i++) {
		vector<Point3f> tempPointSet = object_points[i];

		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);

		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);

		for (unsigned int j = 0; j < tempImagePoint.size(); j++) {
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		fout << "The error of picture " << i + 1 << " is " << err << " pixel" << endl;
	}
	fout << "Overall average error is: " << total_err / image_count << " pixel" << endl << endl;
    fout << "Intrinsic: " << endl;
	fout << cameraMatrix << endl << endl;
	fout << "Distortion parameters: " << endl;
	fout << distCoeffs << endl << endl << endl;
	cout << "Get result!" << endl;
    fout.close();

    camera_info.intrinsic = cameraMatrix;
    camera_info.distortion = distCoeffs;

    // 保存经过畸变矫正的图像
	if(option.undistort_image)
		for(string name : imageNames)
		{
			string filename;
			filename = name;
			// cout << filename << endl;
			cv::Mat img_color = imread(filename);
			if(img_color.empty())
				cout << "error in read image" << endl;
			cv::Mat img_undistort(img_color.rows, img_color.cols, CV_8UC3);
			undistort(img_color, img_undistort, cameraMatrix, distCoeffs);
			// 把名字从 aaa/bbb/cccc/xxx.jpg 变成 result_path/xxx_undistort.jpg
			string::size_type pos = name.rfind('/');
			string undistort_name = name.substr(pos + 1);
			pos = undistort_name.rfind('.');
			undistort_name = undistort_name.substr(0, pos);
			undistort_name = option.image_calib_result_path + '/' + undistort_name + "_undistort.jpg";

			cv::imwrite(undistort_name, img_undistort);
		}
    return 1;
}