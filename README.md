## gbx_navigation

> Complete the navigation function with radar positioning and ros_navigation

`navigation_config`： Contains all the parameter files needed for ros_navigation.

`fast_lio_sam_qn`：https://github.com/engcang/FAST-LIO-SAM-QN

`fast_lio_localization_qn` : Base on https://github.com/engcang/FAST-LIO-Localization-QN

`ranger_mini_v2` :  Base on https://github.com/agilexrobotics/ugv_gazebo_sim, Cut out the original huge package and keep only the parts that are relevant to ranger_mini_v2.

### How to build

- [`GTSAM`](https://github.com/borglab/gtsam)

```
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
unzip gtsam.zip
cd gtsam-4.1.1/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j16
```

- [`Teaser++`](https://github.com/MIT-SPARK/TEASER-plusplus)

```
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
sudo make install -j16
sudo ldconfig
```

- `tbb` (is used for faster `Quatro`)

```
sudo apt install libtbb-dev
```



```
cd ~/your_workspace/src
git clone git@github.com:L-SY/gbx_navigation.git

cd ~/your_workspace
# nano_gicp, quatro first
catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
# Note the option!
catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON -DQUATRO_DEBUG=OFF
catkin build -DCMAKE_BUILD_TYPE=Release
```

### How to use in real car

- open `roscore` , and `ouster os1` in `supervise.html`
- open a terminal and run:

```
roslaunch real_v1_config ranger_mini_v2.launch
```

- open a terminal and run:

```
roslaunch real_v1_config load_move_base.launch
```

and then operate in Rviz, like pub a destination.
​    