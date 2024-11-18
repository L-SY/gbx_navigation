
gbx_navigation

    Complete the navigation function with radar positioning and ros_navigation

navigation_config： Contains all the parameter files needed for ros_navigation.

fast_lio_sam_qn：https://github.com/engcang/FAST-LIO-SAM-QN

fast_lio_localization_qn : Base on https://github.com/engcang/FAST-LIO-Localization-QN

ranger_mini_v2 : Base on https://github.com/agilexrobotics/ugv_gazebo_sim, Cut out the original huge package and keep only the parts that are relevant to ranger_mini_v2.
Dependencies

    C++ >= 17, OpenMP >= 4.5, CMake >= 3.10.0, Eigen >= 3.2, Boost >= 1.54
    ROS

How to build

    GTSAM

wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
unzip gtsam.zip
cd gtsam-4.1.1/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j16

    Teaser++

git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
sudo make install -j16
sudo ldconfig

    tbb (is used for faster Quatro)

sudo apt install libtbb-dev

cd ~/your_workspace/src
git clone git@github.com:L-SY/gbx_navigation.git

cd ~/your_workspace
# nano_gicp, quatro first
catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
# Note the option!
catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON -DQUATRO_DEBUG=OFF
catkin build -DCMAKE_BUILD_TYPE=Release

How to use in real car

    open roscore , and ouster os1 in supervise.html
    open a terminal and run:

roslaunch real_v1_config ranger_mini_v2.launch

    open a terminal and run:

roslaunch real_v1_config load_move_base.launch

and then operate in Rviz, like pub a destination. ​
2024-11-15 TEST
CODE:

    介绍部分包的构成逻辑和作用

navigation_config

所有的运行luanch文件全部位于navigation_config文件夹下，为了方便最后的运行，每一台车单独作为一个软件包。内容如下表
名字 	内容 	作用
config 	move_base和雷达相关的.yaml参数文件 	move_base和雷达相关的参数加载
launch 	move_base和雷达相关的.launch参数文件 	加载.yaml文件中的参数和启动节点
map 	不同场地的.png和.yaml文件 	给map_serve提供发布2d地图的内容
pcd 	由fast_lio/fast_lio_sam生成的pcd地图 	处理后得到2D.png（由于文件过大仅存放在本地）
bag 	fast_lio_sam生成的记录包含关键帧信息的.bag文件 	给fast_lio_localization_qn提供关键帧定位
trajectory 	由topic_transit/TopicGenTrajectory生成的轨迹csv文件 	在gbx_manual中加载并发布给move_base
urdf/meshes 	机器人的urdf文件/仿真用的/stl文件 	描述机器人组成信息/仿真中加载模型
rviz 	rviz的配置文件 	保存Rivz所需文件，固定可视化配置
gbx_manual

    为了让使用者统一的和各个节点交互和构建运行流程

构建拥有STOP,MOVE,WAIT,PULL_OVER,ARRIVE五个状态的FSM来管理整个导航周期。

对外提供/gbx_manual/pub_trajectory的服务接口。初始化时加载navigation_config/real_v1_config/trajectory中轨迹，当收到包含正确轨迹信息的请求时会发布自身所存放的对应轨迹点到/click_point上。

可以在命令行中用如下调用：

rosservice call /gbx_manual/pub_trajectory "sender: 'user' path_name: 'E3_121'"

这样就会发送E3_121的请求给gbx_manual
How to run

    使用roslaunch而非mon launch来加载fast_xxx相关的launch文件

以real_v1_config为例:

roslaunch real_v1_config load_manual.launch

运行load_manual.launch文件，实现以下功能

    加载move_base所需的global_costmap,local_costmap等配置文件并启动节点
    加载map的配置文件并启动map_server节点，在/map话题上发布2d地图信息
    加载fast_lio和fast_lio_localization_qn所需配置文件并启动节点
    加载静态的tf_publisher和机器人的urdf以及robot_state_publisher
    加载cloud_transit配置文件并启动节点
    加载ranger_mini_v2配置文件并启动节点（松灵底盘）
    加载gbx_manual配置文件并启动节点

此外这些节点也可以单独运行，示例如下：

    单独启动fast_lio

roslaunch real_v1_config ouster_os1_mapping.launch

    单独启动fast_sam_localization_qn

roslaunch real_v1_config ouster_os1_loc_qn.launch

    单独运行松灵底盘控制器

roslaunch real_v1_config ranger_mini_v2.launch

完整流程
建图

运行fast_lio及fast_lio_sam生成关键帧点云的.bag及.pcd文件

roslaunch real_v1_config ouster_os1_sam.launch

定位

运行fast_lio及fast_sam_localization_qn，加载.bag文件

roslaunch real_v1_config ouster_os1_loc_qn.launch

利用pcd文件生成2d地图

roslaunch pcd2pgm run.launch
rosrun map_server map_saver -f map_name

加载2d地图及move_base

roslaunch real_v1_config load_move_base.launch

加载manual

roslaunch real_v1_config load_manual.launch

