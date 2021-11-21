# ORB-LINE-SLAM

**Authors:** Ioannis Alamanos, Georgios Thanellas, Costas Tzafestas.

ORB-LINE-SLAM is a **Visual SLAM** library for **Stereo Cameras** which is able to function either by combining point and line features or by using line features exclusively. We provide examples to run ORB-LINE-SLAM in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and in the [UMA dataset](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/291-uma-visual-inertial-dataset.html). The system is also able to run in real-time using **ROS**.

This software is based on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) developed by Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

### Related Publications:

[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, IEEE Transactions on Robotics, 2021. **[PDF](https://arxiv.org/pdf/2007.11898.pdf)**.

[IMU-Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós, **Inertial-Only Optimization for Visual-Inertial Initialization**, *ICRA 2020*. **[PDF](https://arxiv.org/pdf/2003.05766.pdf)**

[ORBSLAM-Atlas] Richard Elvira, J. M. M. Montiel and Juan D. Tardós, **ORBSLAM-Atlas: a robust and accurate multi-map system**, *IROS 2019*. **[PDF](https://arxiv.org/pdf/1908.11585.pdf)**.

[ORBSLAM-VI] Raúl Mur-Artal, and Juan D. Tardós, **Visual-inertial monocular SLAM with map reuse**, IEEE Robotics and Automation Letters, vol. 2 no. 2, pp. 796-803, 2017. **[PDF](https://arxiv.org/pdf/1610.05949.pdf)**. 

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://arxiv.org/pdf/1610.06475.pdf)**.

[Monocular] Raúl Mur-Artal, José M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](https://arxiv.org/pdf/1502.00956.pdf)**.

[DBoW2 Place Recognition] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-LINE-SLAM is released under [GPLv3 license](https://github.com/Giannis-Alamanos/ORB-LINE-SLAM/blob/main/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/Giannis-Alamanos/ORB-LINE-SLAM/blob/main/Dependencies.md).

# 2. Prerequisites
We have tested the library in **Ubuntu 18.04**, but it should be easy to compile in other platforms. A powerful computer will ensure real-time performance and provide more stable and accurate results. In order to reduce the computational time, the user can decrease the value of the lsd_scale parameter in the yaml file. Moreover, through the same yaml file which can be found in PATH/Examples/Stereo-Line, the user can choose between different SLAM modes (only-line SLAM and point-line SLAM) and Line Detectors (LSD and EDlines).

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at least 3.0. Tested with OpenCV 3.2.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0. Tested with Eigen 3.3.8**.

## Python
Required to calculate the alignment of the trajectory with the ground truth. **Required Numpy module**.

* (win) http://www.python.org/downloads/windows
* (deb) `sudo apt install libpython2.7-dev`
* (mac) preinstalled with osx

## ROS (optional)

We provide some examples to process live input using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04.

# 3. Building ORB-LINE-SLAM library and examples

Clone the repository:
```
git clone https://github.com/Giannis-Alamanos/ORB-LINE-SLAM.git ORB-LINE-SLAM
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-LINE-SLAM*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB-LINE-SLAM
chmod +x build.sh
./build.sh
```


# 4. Examples

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
2. Execute the following command in the terminal:

```
./Examples/Stereo-Line/stereo_line_euroc ./Vocabulary/ORBvoc.txt ./Vocabulary/LSDvoc.txt ./Examples/Stereo-Line/EuRoC.yaml SEQUENCE ./Examples/Stereo-Line/EuRoC_TimeStamps/SEQUENCE.txt
```

## UMA Dataset

1. Download a sequence from http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/291-uma-visual-inertial-dataset.html
2. Execute the following command in the terminal:

```
./Examples/Stereo-Line/stereo_line_UMA ./Vocabulary/ORBvoc.txt ./Vocabulary/LSDvoc.txt ./Examples/Stereo-Line/UMA_ueye.yaml SEQUENCE ./Examples/Stereo-Line/UMA_TimeStamps/SEQUENCE.txt
```

To run sequences from UMA Dataset other than hall1-rev-eng and corridor-eng, the user has to create a file containing the Timestamps.

# 5. ROS Examples

Tested with ROS Melodic and ubuntu 18.04.


1. Open .bashrc file:
  ```
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB-LINE-SLAM:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB-LINE-SLAM/Examples/ROS
  ```

2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM3/Stereo. You will need to provide the vocabulary files and a settings file. If you **provide rectification matrices** (see Examples/Stereo-Line/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**.

  ```
  rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY1 PATH_TO_VOCABULARY2 PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```
  
  **Example**: Download a rosbag (e.g. V1_01_easy.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM3 Stereo ./Vocabulary/ORBvoc.txt ./Vocabulary/LSDvoc.txt ./Examples/Stereo-Line/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```
  
Once ORB-LINE-SLAM has loaded the vocabulary, press space in the rosbag tab. Enjoy!. Note: a powerful computer is required to run exigent sequences in real-time.
