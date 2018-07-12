# ros_object_analytics
      物体分析　Object Analytics (OA) 是一个ros包，
      支持实时物体检测定位和跟踪(realtime object detection, localization and tracking.)
      使用　RGB-D 相机输入,提供物体分析服务，为开发者开发更高级的机器人高级特性应用， 
      例如智能壁障(intelligent collision avoidance),和语义SLAM(semantic SLAM). 

      订阅:
            RGB-D　相机发布的点云消息[sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) 
      发布话题到:
            物体检测　[object detection](https://github.com/intel/object_msgs), 
            物体跟踪　[object tracking](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs), 
            物体3d定位　[object localization](https://github.com/intel/ros_object_analytics/object_analytics_msgs) 

优秀的算法：

* 基于 图形处理器(GPU) 运行的目标检测　, [ros_opencl_caffe](https://github.com/intel/ros_opencl_caffe), 
  Yolo v2 model and [OpenCL Caffe](https://github.com/01org/caffe/wiki/clCaffe#yolo2-model-support) framework

* 基于 视觉处理器（VPU） 运行的目标检测 , [ros_intel_movidius_ncs (devel branch)](https://github.com/intel/ros_intel_movidius_ncs/tree/devel/), with MobileNet SSD model and Caffe framework. 

      (Movidius神经计算棒,首个基于USB模式的深度学习推理工具和独立的人工智能（AI）加速器)
      英特尔的子公司Movidius宣布推出Movidius Myriad X视觉处理器（VPU），
      该处理器是一款低功耗SoC，主要用于基于视觉的设备的深度学习和AI算法加速，比如无人机、智能相机、VR/AR头盔。
      就在不久前的上个月，Movidius还推出了基于Myriad 2芯片的神经计算棒Movidius Neural Compute Stick。


## 编译依赖　compiling dependencies
  ROS packages from [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  * roscpp
  * nodelet
  * std_msgs
  * sensor_msgs
  * geometry_msgs
  * dynamic_reconfigure
  * pcl_conversions
  * cv_bridge
  * libpcl-all
  * libpcl-all-dev
  * ros-kinetic-opencv3

  其他包　来自intel
  * [object_msgs](https://github.com/intel/object_msgs)
  
  * [ros_intel_movidius_ncs](https://github.com/intel/ros_intel_movidius_ncs)  VPU运行mobileNet
    [opencl_caffe](https://github.com/intel/ros_opencl_caffe)　　　　　　　　　  GPU运行yolo_v2
  

        NOTE: 跟踪特征依赖 OpenCV (3.3 preferred, >= 3.2 minimum). 
            ROS Kinetic package "ros-kinetic-opencv3" (where OpenCV **3.3.1** is integrated). 
              However, if you're using an old version of ROS Kinetic (where OpenCV **3.2** is integrated), 
              其他来源 [opencv_contrib](https://github.com/opencv/opencv_contrib). 
              需要编译 opencv_contrib (self-built) 和　opencv (ROS Kinetic provided) 查看 "/opt/ros/kinetic/share/opencv3/package.xml"

## 编译和测试　build and test
  * to build　　编译
  ```bash
  cd ${ros_ws} # "ros_ws" is the catkin workspace root directory where this project is placed in
  catkin_make
  ```
  * to test　　测试　
  ```bash
  catkin_make run_tests
  ```

  * to install　安装
  ```bash
  catkin_make install
  ```

## 附加运行依赖　extra running dependencies
   传感器驱动　RGB-D camera
###  Intel RealSense D400
  * [librealsense2 tag v2.9.1](https://github.com/IntelRealSense/librealsense/tree/v2.9.1) and [realsense_ros_camera tag 2.0.2](https://github.com/intel-ros/realsense/tree/2.0.2) if run with Intel RealSense D400
  ```
  roslaunch realsense_ros_camera rs_rgbd.launch
  ```
### Microsoft XBOX 360 / Kinect
  * [openni_launch](http://wiki.ros.org/openni_launch) or [freenect_launch](http://wiki.ros.org/freenect_launch) and their dependencies if run with Microsoft XBOX 360 Kinect
  ```bash
  roslaunch openni_launch openni.launch
  ```
### Astra Camera
  * [ros_astra_camera](https://github.com/orbbec/ros_astra_camera) if run with Astra Camera
  ```bash
  roslaunch astra_launch astra.launch
  ```

## 运行目标分析 OA  command to launch object_analytics

* launch with OpenCL caffe as detection backend yolo_v2　目标检测后端
   ```bash
   roslaunch object_analytics_launch analytics_opencl_caffe.launch
   ```
   
* launch with Movidius NCS as detection backend mobileNetSSD 目标检测后端
   ```bash
   roslaunch object_analytics_launch analytics_movidius_ncs.launch
   ```

  Frequently used options
  * **input_points** Specify arg "input_points" for the name of the topic publishing the [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) messages by RGB-D camera. 
  Default is "/camera/depth_registered/points" (topic compliant with [ROS OpenNI launch](http://wiki.ros.org/openni_launch))
  
  * **aging_th** 跟踪队列长度　Specifiy tracking aging threshold, number of frames since last detection to deactivate the tracking. Default is 16.
  
  * **probability_th** 跟踪置信度　Specify the probability threshold for tracking object. Default is "0.5".
  ```bash
  roslaunch object_analytics_launch analytics_movidius_ncs.launch aging_th:=30 probability_th:="0.3"
  ```
## 节点订阅的　传感器发布的话题
  RGB图像　object_analytics/rgb ([sensor_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  点云　object_analytics/pointcloud ([sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

## 节点发布的处理后得到的信息

  定位信息(3d边框)　object_analytics/localization ([object_analytics_msgs::ObjectsInBoxes3D](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs/msg))

  跟踪信息 object_analytics/tracking ([object_analytics_msgs::TrackedObjects](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_msgs/msg))

  检测信息(2d边框)object_analytics/detection ([object_msgs::ObjectsInBoxes](https://github.com/intel/object_msgs/tree/master/msg))


## object_analytics 节点分析
      1. RGBD传感器预处理分割器 splitter  
         输入: /camera/depth_registered/points
        输出: pointcloud   3d点 
         输出: rgb 2d图像
         object_analytics_nodelet/splitter/SplitterNodelet 
      2. 点云分割处理器 segmenter
         输入: pointcloud   3d点
         输出: segmentation 分割
         object_analytics_nodelet/segmenter/SegmenterNodelet 
      3. 3d定位器　merger
         输入: 2d检测分割结果 detection
         输入: 3d检测分割结果 segmentation
         输出: 3d定位结果　　 localization
        object_analytics_nodelet/merger/MergerNodelet
      4. 目标跟踪器 tracker
         输入: 2d图像        rgb        input_rgb 
         输入: 2d检测分割结果 detection  input_2d 
         输出: 跟踪结果　　　 tracking　　output  
         参数: 跟踪帧队列长度: aging_th：default="30"；
         参数: 跟踪置信度: probability_th" default="0.5"
         object_analytics_nodelet/tracker/TrackingNodelet
## object_analytics_visualization 可视化
      5. 3d定位可视化　visualization3d　localization
         输入: 2d检测结果 "detection_topic" default="/object_analytics/detection" 
         输入: 3d检测结果 "localization_topic" default="/object_analytics/localization" 
         输入: 2d跟踪结果 "tracking_topic" default="/object_analytics/tracking" 
         输出: rviz可视化话题　"output_topic" default="localization_rviz" 

      6. 2d跟踪可视化　visualization2d 
         输入: 2d检测结果 "detection_topic" default="/object_analytics/detection" 
         输入: 2d跟踪结果 "tracking_topic" default="/object_analytics/tracking" 
         输入: 2d图像     "image_topic" default="/object_analytics/rgb" 
         输出: rviz可视化话题　""output_topic" default="tracking_rviz"    



## KPI of differnt detection backends
<table>
    <tr>
        <td></td>
        <td>topic</td>
        <td>fps</td>
        <td>latency <sup>sec</sup></td>
    </tr>
    <tr>
        <td rowspan='4'>OpenCL Caffe</td>
    </tr>
    <tr>
        <td>localization</td>
        <td>6.63</td>
        <td>0.23</td>
    </tr>
    <tr>
        <td>detection</td>
        <td>8.88</td>
        <td>0.17</td>
    </tr>
    <tr>
        <td>tracking</td>
        <td>12.15</td>
        <td>0.33</td>
    </tr>
    <tr>
        <td rowspan='4'>Movidius NCS</sup></td>
    </tr>
    <tr>
        <td>localization</td>
        <td>7.44</td>
        <td>0.21</td>
    </tr>
    <tr>
        <td>detection</td>
        <td>10.5</td>
        <td>0.15</td>
    </tr>
    <tr>
        <td>tracking</td>
        <td>13.85</td>
        <td>0.24</td>
    </tr>
</table>

* CNN model of Movidius NCS is MobileNet
* Hardware: Intel(R) Xeon(R) CPU E3-1275 v5 @3.60GHz, 32GB RAM, Intel(R) RealSense R45

## rviz中可视化　visualize tracking and localization results on RViz
  Steps to enable visualization on RViz are as following
  ```bash
  roslaunch object_analytics_visualization rviz.launch
  ```
###### *ROS 2 Object Analytics: https://github.com/intel/ros2_object_analytics*
###### *Any security issue should be reported using process at https://01.org/security*
