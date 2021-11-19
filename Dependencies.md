##List of Known Dependencies
###ORB-LINE-SLAM

In this document we list all the pieces of code included by ORB-LINE-SLAM and linked libraries which are not property of the authors of ORB-LINE-SLAM and ORB-SLAM3.


#####Code in **src** and **include** folders

* *ORBextractor.cc*.
This is a modified version of orb.cpp of OpenCV library. The original code is BSD licensed.

* *PnPsolver.h, PnPsolver.cc*.
This is a modified version of the epnp.h and epnp.cc of Vincent Lepetit. 
This code can be found in popular BSD licensed computer vision libraries as [OpenCV](https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/epnp.cpp) and [OpenGV](https://github.com/laurentkneip/opengv/blob/master/src/absolute_pose/modules/Epnp.cpp). The original code is FreeBSD.

* *MLPnPsolver.h, MLPnPsolver.cc*.
This is a modified version of the MLPnP of Steffen Urban from [here](https://github.com/urbste/opengv). 
The original code is BSD licensed.

* Function *ORBmatcher::DescriptorDistance* in *ORBmatcher.cc*.
The code is from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel.
The code is in the public domain.

* *gridStructure.h, gridStructure.cpp, LineExtractor.h, LineExtractor.cpp, LineIterator.h, LineIterator.cpp, LineMatcher.h, LineMatcher.cpp*.
The code is from [PL-SLAM](https://github.com/rubengooj/pl-slam) and some parts are modified.
The original code is GNU General Public License v3.0 licensed.

* *Frame.h, Frame.cc, MapDrawer.h, MapDrawer.cc, Tracking.h, Tracking.cc, MapLine.h, MapLine.cc*.
Some parts of the code are from [here](https://github.com/robotseu/ORB_Line_SLAM) and are modified.
The code is in the public domain.

#####Code in Thirdparty folder

* All code in **DBoW2** folder.
This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are BSD licensed.

* All code in **g2o** folder.
This is a modified version of [g2o](https://github.com/RainerKuemmerle/g2o). All files included are BSD licensed.

* In line_descriptor folder all code in src/ED_Lib and include/ED.h, include/EDCircles.h, include/EDColor.h, include/EDLib.h, include/EDLines.h, include/EDPF.h, include/NFA.h.
The code is from [ED_Lib](https://github.com/CihanTopal/ED_Lib). All files included are [MIT licensed](https://en.wikipedia.org/wiki/MIT_License).

* All the rest of the code in line_descriptor folder.
The code is from [OpenCV](https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/epnp.cpp) and it is slightly modified. All files included are BSD licensed.

#####Library dependencies 

* **Pangolin (visualization and user interface)**.
[MIT license](https://en.wikipedia.org/wiki/MIT_License).

* **OpenCV**.
BSD license.

* **Eigen3**.
For versions greater than 3.1.1 is MPL2, earlier versions are LGPLv3.

* **ROS (Optional, only if you build Examples/ROS)**.
BSD license. In the manifest.xml the only declared package dependencies are roscpp, tf, sensor_msgs, image_transport, cv_bridge, which are all BSD licensed.




