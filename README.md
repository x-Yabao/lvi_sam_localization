# LVI-SAM-LOCALIZATION
A simple version of system that can localize in a built map is developed in this repository. This sysytem is based on LVI-SAM, and also there are some new features that we developed.  

- Through **Visual-LiDAR Fusion Relocalization**, the system can launch anywhere in the built map, and the Visual-LiDAR Fusion Relocalization is based on Bag-of-Words and [Scan Context](https://github.com/irapkaist/scancontext).
- Use the **C++ boost serialization**, the system can change the folder map to binary format, so that reduce map loading time by more than 30%.
- Real-time parameter tuning: The system can adjust algorithm parameters in real time according to the current robot working environment, CPU and memory usage, etc., to improve system performance.
  
A video of this project can be found on [YouTube](https://youtu.be/ZFumQSqMsE0).

<p align='center'>
    <img src="./doc/fig/localization.png" alt="drawing" width="800"/>
</p>

---

## Dependency
The dependency of this repo is same as the official [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM). So if you occur a compile problem, we recommend you to compile the official LVI-SAM firstly. Right now we have only tested on Ubuntu 18.04 + ROS melodic environment.

In particular, this ropo relies on **jsk-rviz-plugins** to display text on rviz, so remember to install it.
```
sudo apt-get install ros-melodic-jsk-rviz-plugins
```

## Compile
You can use the following commands to download and compile the package.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/x-Yabao/lvi_sam_localization
cd ..
catkin_make
```

## Prepare a built map
Take the [M2DGR dataset](https://github.com/SJTU-ViSYS/M2DGR) for example.
1. Fisrtly, you need to **prepare a built map**. Please run our another work [lvi-sam-savemap](https://github.com/x-Yabao/lvi_sam_savemap) to build and save the map in a folder.
2. Change the above map to binary format, so that we can reduce the map loading time.
   
   - Make sure the [config file](./config/m2dgr/params_function.yaml) is right.
  
   - Run the launch file:
    ```
    roslaunch lvi_sam_localization build_map_m2dgr.launch
    ```


## Run the package
Take the [M2DGR dataset](https://github.com/SJTU-ViSYS/M2DGR) for example.
1. Make sure the [config file](./config/m2dgr/params_function.yaml) is right.

2. Run the launch file:
```
roslaunch lvi_sam_localization run_m2dgr.launch
```

3. Play existing bag files:
```
rosbag play your-bag.bag
```





## Notes
- **Initialization:** During the initialization stage, had better keep the robot still. Or if you play bags, fistly play the bag for about 0.5s, and then pause the bag until the initialization succeed. If initialization failure, move the robot to another place, or play the bag for a few seconds, than activate relocation again by give it a random guess on the the Rviz.

## Acknowledgement
- [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)
- [LVI-SAM-Easyused](https://github.com/Cc19245/LVI-SAM-Easyused)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization)
- [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM)


## TODO
+ [ ] Preprocess the built map and cut it into squares. Global localization extracts squares instead of keyframes when extracting local maps.


