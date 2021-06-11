[![License CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-blue.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
![Python 3.8](https://img.shields.io/badge/python-3.8-blue.svg)
# Deep Object Pose Estimation - ROS Inference

This is the official DOPE ROS package for detection and 6-DoF pose estimation of **known objects** from an RGB camera.  The network has been trained on the following YCB objects:  cracker box, sugar box, tomato soup can, mustard bottle, potted meat can, and gelatin box.  For more details, see our [CoRL 2018 paper](https://arxiv.org/abs/1809.10790) and [video](https://youtu.be/yVGViBqWtBI).

*Note:*  The instructions below refer to inference only.  Training code is also provided but not supported.

![DOPE Objects](dope_objects.png)

## Updates 
2021/06/11 - Migrated to ROS2

2020/03/09 - Added HOPE [weights to google drive](https://drive.google.com/open?id=1DfoA3m_Bm0fW8tOWXGVxi4ETlLEAgmcg), [the 3d models](https://drive.google.com/drive/folders/1jiJS9KgcYAkfb8KJPp5MRlB0P11BStft), and the objects dimensions to config. [Tremblay et al., IROS 2020](https://arxiv.org/abs/2008.11822).

2020/02/09 - Upgraded DOPE to use Python 3. Updated Dockerfile to use Python3-compatible ROS Noetic. The Python 2.7/ROS Kinetic is still available on the ['ros-kinetic' branch](https://github.com/NVlabs/Deep_Object_Pose/tree/ros-kinetic).

2020/16/03 - Added a wiki (thanks to [@saratrajput](https://github.com/saratrajput)) 

2019/03/07 - ROS interface update (thanks to Martin Günther)

2019/11/06 - Added bleach YCB weights 

## Installing

We have tested on Ubuntu 20.04 with ROS2 Foxy with an NVIDIA RTX 2070 with python 3.8. The code may work on other systems.

The following steps describe the native installation.

1. **Install ROS2**

    Follow these [instructions](https://docs.ros.org/en/foxy/Installation.html).

2. **Create a workspace** (if you do not already have one). To create a workspace, follow these [instructions](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html):
    ```
    $ mkdir -p ~/dev_ws/src   # Replace `dev_ws` with the name of your workspace
    $ cd ~/dev_ws/
    $ colcon build --symlink-install
    ```

3. **Download the DOPE code**
    ```
    $ cd ~/dev_ws/src
    $ git clone https://github.com/gentlebots/Deep_Object_Pose.git
    ```

4. **Install python dependencies**
    ```
    $ cd ~/dev_ws/src/Deep_Object_Pose
    $ python3 -m pip install -r requirements.txt
    ```

5. **Install ROS dependencies**
    ```
    $ cd ~/dev_ws
    $ rosdep install -i --from-path src --rosdistro foxy -y
    ```

6. **Build**
    ```
    $ cd ~/dev_ws
    $ colcon build --symlink-install
    ```

7. **Download [the weights](https://drive.google.com/open?id=1DfoA3m_Bm0fW8tOWXGVxi4ETlLEAgmcg)** and save them to the `weights` folder, *i.e.*, `~/dev_ws/src/Deep_Object_Pose/dope/weights/`.

## Running
1. **Bring up your favorite simulator or real robot with a camera**

2. **Edit config info** (if desired) in `~/dev_ws/src/Deep_Object_Pose/dope_launch/config/config_pose.yaml`
    * `topic_camera`: RGB topic to listen to
    * `topic_camera_info`: camera info topic to listen to
    * `topic_publishing`: topic namespace for publishing
    * `input_is_rectified`: Whether the input images are rectified. It is strongly suggested to use a rectified input topic.
    * `downscale_height`: If the input image is larger than this, scale it down to this pixel height. Very large input images eat up all the GPU memory and slow down inference. Also, DOPE works best when the object size (in pixels) has appeared in the training data (which is downscaled to 400 px). For these reasons, downscaling large input images to something reasonable (e.g., 400-500 px) improves memory consumption, inference speed *and* recognition results.
    * `objects`: List of objects
    * `weight_url`: objetc weight path
    * `dimensions`:dimensions for an objects
    * `class_id`: class id to be used in the messages published on the `/dope/detected_objects` topic 
    * `draw_color`: object colors
    * `model_transforms`: transforms that are applied to the pose before publishing
    * `meshe`: mesh filename for visualization
    * `mesh_scale`: scaling factors for the visualization mesh
    * `thresh_angle`: undocumented
    * `thresh_map`: undocumented
    * `sigma`: undocumented
    * `thresh_points`: Thresholding the confidence for object detection; increase this value if you see too many false positives, reduce it if  objects are not detected.

3. **Start DOPE node**
    ```
    $ ros2 launch dope_launch dope_launch.py
    ```   

## Debugging

* The following ROS topics are published (assuming `topic_publishing == 'dope'`):
    ```
    /dope/webcam_rgb_raw       # RGB images from camera
    /dope/dimension_[obj_name] # dimensions of object
    /dope/pose_[obj_name]      # timestamped pose of object
    /dope/rgb_points           # RGB images with detected cuboids overlaid
    /dope/detected_objects     # vision_msgs/Detection3DArray of all detected objects
    /dope/markers              # RViz visualization markers for all objects
    ```
    *Note:* `[obj_name]` is in {cracker, gelatin, meat, mustard, soup, sugar}

* To debug in RViz, run `rviz`, then add one or more of the following displays:
    * `Add > Image` to view the raw RGB image or the image with cuboids overlaid
    * `Add > Pose` to view the object coordinate frame in 3D.
    * `Add > MarkerArray` to view the cuboids, meshes etc. in 3D.
    * `Add > Camera` to view the RGB Image with the poses and markers from above.

    If you do not have a coordinate frame set up, you can run this static transformation: `rosrun tf2_ros static_transform_publisher 0 0 0 0.7071 0 0 -0.7071 world <camera_frame_id>`, where `<camera_frame_id>` is the `frame_id` of your input camera messages.  Make sure that in RViz's `Global Options`, the `Fixed Frame` is set to `world`. Alternatively, you can skip the `static_transform_publisher` step and directly set the `Fixed Frame` to your `<camera_frame_id>`.

* If `rosrun` does not find the package (`[rospack] Error: package 'dope' not found`), be sure that you called `source devel/setup.bash` as mentioned above.  To find the package, run `rospack find dope`.


## YCB 3D Models

DOPE returns the poses of the objects in the camera coordinate frame.  DOPE uses the aligned YCB models, which can be obtained using [NVDU](https://github.com/NVIDIA/Dataset_Utilities) (see the `nvdu_ycb` command).

## HOPE 3D Models

![HOPE 3D models rendered in UE4](https://i.imgur.com/V6wX64p.png)

We introduce new toy 3d models that you download [here](https://drive.google.com/drive/folders/1jiJS9KgcYAkfb8KJPp5MRlB0P11BStft). 
The folders are arranged like the YCB 3d models organization. 
You can buy the real objects using the following links 
[set 1](https://www.amazon.com/gp/product/B071ZMT9S2), 
[set 2](https://www.amazon.com/gp/product/B007EA6PKS), 
[set 3](https://www.amazon.com/gp/product/B00H4SKSPS), 
and 
[set 4](https://www.amazon.com/gp/product/B072M2PGX9). 

## How to cite DOPE 

If you use this tool in a research project, please cite as follows:
```
@inproceedings{tremblay2018corl:dope,
 author = {Jonathan Tremblay and Thang To and Balakumar Sundaralingam and Yu Xiang and Dieter Fox and Stan Birchfield},
 title = {Deep Object Pose Estimation for Semantic Robotic Grasping of Household Objects},
 booktitle = {Conference on Robot Learning (CoRL)},
 url = "https://arxiv.org/abs/1809.10790",
 year = 2018
}
```

## License

Copyright (C) 2018 NVIDIA Corporation. All rights reserved. Licensed under the [CC BY-NC-SA 4.0 license](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode).


## Acknowledgment

Thanks to Jeffrey Smith (jeffreys@nvidia.com) for creating the Docker image.


## Contact

Jonathan Tremblay (jtremblay@nvidia.com), Stan Birchfield (sbirchfield@nvidia.com)
