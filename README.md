# LVI-SAM-LOCALIZATION
A simple version of system that can relocalize in a built map is developed in this repository. The sysytem is bsed on LVI-SAM.

---

## Dependency
The dependency of this repo is same as the official [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM). So if you occur a compile problem, we recommend you to compile the official LVI-SAM firstly. Right now we have only tested on Ubuntu 18.04 + ROS melodic environment.

---

## Compile
You can use the following commands to download and compile the package.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/x-Yabao/lvi_sam_localization
cd ..
catkin_make
```

---

## Run the package on different datasets
1. [M2DGR dataset](https://github.com/SJTU-ViSYS/M2DGR)
- Firstly, you need to run [lvi-sam-savemap](https://github.com/x-Yabao/lvi_sam_savemap), and then save the map in a folder.

- Run the launch file:
```
roslaunch lvi_sam_location run_m2dgr.launch
```

- Play existing bag files:
```
rosbag play your-bag.bag
```

A video of the demonstration of the method can be found on [YouTube](https://youtu.be/PRsH8SpuSIc).

--- 

## Notes
- **Initialization:** During the initialization stage, had better keep the robot still. Or if you play bags, fistly play the bag for about 0.5s, and then pause the bag until the initialization succeed. The initialization method requres you to give it initial guesses by hand on the the Rviz.

---

## Acknowledgement
- [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)
- [LVI-SAM-Easyused](https://github.com/Cc19245/LVI-SAM-Easyused)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization)