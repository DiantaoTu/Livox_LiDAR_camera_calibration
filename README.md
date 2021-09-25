# Livox LiDAR-Camera Calibration

This method is from the official method of Livox(https://github.com/Livox-SDK/livox_camera_lidar_calibration)

It's just that the original method is not automated enough, and many operations need to be performed manually, so it is slightly modified to reduce the manual operation part.

However, it is still necessary to view the LiDAR point cloud in the software(cloudcompare meshlab) and manually record the position of the corner points of the calibration board.

## Pipeline

1. calibrate the camera
2. convert `lvx` file to `pcd` format
3. use software to view the `pcd` file and record the position of the corner points 
4. LiDAR-Camera calibration
5. project LiDAR points to image or color the LiDAR points

## Parameters

`config.txt` contains all parameters needed by the entire program

- `calib_camera`   calibrate the camera or not, if set to `0`, intrinsic should be provided in `config.txt`
- `undistort_image`   Whether to use the camera internal parameters to correct the distortion of the calibration picture and output it, which is used to check whether the calibration is accurate. It is invalid if `calib_camera=0` 
- `fx fy cx cy k1 k2 k3 p1 p2`   camera intrinsic parameters and distortion. It is invalid if `calib_camera=1` 
- `image_calib_path`   path of the images used for camera calibration
- `image_calib_result_path`   path of the result of camera calibration
- `lvx2pcd`   convert `lvx` file to `pcd` file
- `lidar_path`   path of the `lvx` files
- `joint_image_path`   path of the images used for LiDAR-camera calibration
- `lidar_corner_path`   path to save vertex coordinates in LiDAR data
- `joint_calib_result_path`   path to save the results of LiDAR-camera calibration
- `max_depth` the projection of the LiDAR point cloud onto the image is limited to make the projected depth map more beautiful

## Steps

1. set `lvx2pcd` to `1` and convert LiDAR data to `pcd`
2. view `pcd` files and record the coordinate of vertex in `lidar_corner_path` 
3. run the program again
4. During the joint calibration process, a picture will be displayed on the screen. Use the mouse to click the apex of the calibration board. **The order must be consistent with the order of the LiDAR vertex.** After an image is completed, press any key to start the next image
5. wait for the program to finish



# Livox 激光雷达和单目相机标定

这个方法就是来自于 Livox 官方的方法(https://github.com/Livox-SDK/livox_camera_lidar_calibration)

只是原本的方法不够自动化，很多操作需要手工进行，因此对其稍加修改，减少手动操作的部分

但是其中的选择雷达角点依然需要在 CloudCompare Meshlab 等软件中查看点云，然后手动记录下角点位置

## 程序流程

1. 单独对相机进行标定
2. 把 `lvx`文件转换为 `pcd`文件
3. 使用点云查看软件查看点云并记录下角点坐标
4. 雷达-相机联合标定
5. 把雷达点投影到图像上得到深度图, 把图像反投影回雷达得到上色的点云

## 配置参数

在 `config.txt` 中配置了整个程序需要的各种参数

- `calib_camera` 是否进行相机标定，如果不进行相机标定，那么就要在配置中写出内参以及畸变参数
- `undistort_image` 是否使用相机内参对标定图片进行畸变矫正并输出，这一项用来查看标定是否准确，`calib_camera=0` 时此项无效
- `fx fy cx cy k1 k2 k3 p1 p2` 相机内参和畸变参数，`calib_camera=1` 时此项无效
- `image_calib_path` 单独标定相机时图片的路径
- `image_calib_result_path` 相机标定结果保存的路径
- `lvx2pcd` 是否把`lvx`文件转为`pcd`文件
- `lidar_path` `lvx`文件的路径，也是转换后的`pcd`文件路径
- `joint_image_path` 联合标定时图片的路径
- `lidar_corner_path` 雷达数据中顶点坐标保存的路径
- `joint_calib_result_path` 联合标定的结果保存的路径
- `max_depth` 从雷达投影到图像上的深度限制，为了使投影后的深度图更美观

## 运行步骤

1. 在配置文件中修改为 `lvx2pcd=1`，其余为0，这样先转换雷达数据为pcd文件
2. 查看pcd文件并找到其中标定板的顶点记录下来，保存在`lidar_corner_path` 
3. 设置好配置文件，再运行一次程序
4. 在联合标定过程中，屏幕上会显示图片，用鼠标点击标定板的顶点，**顺序一定要和雷达顶点顺序一致**。一张图像完成后按下任意键开始下一张图片
5. 等待程序运行完毕

