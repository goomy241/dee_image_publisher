# ROS2 Image Publisher
This is a ROS2 package that publishes images from various sources, such as rosbag, kitti dataset, and camera data. It supports both image message and pointcloud2 format message for kitti dataset. Additionally, it provides a CPU utilization monitoring script and can log publishing latency to a file.
## Get testing data
- Rosbag: Due to GitHub's file size restriction, the rosbag file has been saved in Google Drive and can be accessed via the [link](https://drive.google.com/drive/folders/1_Jn4lQfxXrxvIvLHaN3ElknRP7Vp-u8j?usp=sharing). To commence testing, please download the bag folder and place it in the dataset folder.
- KITTI dataset: Execute the `$ raw_data_downloader.sh` script in the `/dataset/kitti` folder to get full access to the KITTI dataset.

## How to publish image
- Build the package using `$ concol build --symlink-install`
- Source the workspace `$ source ~/ros2_dee_ws/install/setup.bash`
### Camera data
- Run `cheese` to check if the camera is working. If `cheese` is not installed, install it via `$ sudo apt-get install cheese`.
- Start the node via `$ ros2 run dee_image_publisher camera_publisher`. You should be able to view the callback message in your terminal.
- One optional step is to use `foxglove` to visualize the published message. (Not applicable for M2 chip)
  - Install `foxglove_bridge` via `$ sudo apt install ros-$ROS_DISTRO-foxglove-bridge` 
  - Once the camera node has started publishing camera images, execute the following command to establish the web socket. `$ ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`
  - Open `foxglove` desktop or website, choose `open connection` ans use default ip address(ws://localhost:8765) to connect to the web socket. You should be able to see the published image under the `/webcam` topic:
  <img width="1014" alt="Screen Shot 2023-04-01 at 1 57 53 PM" src="https://user-images.githubusercontent.com/90799662/229339613-618cdd8d-b995-4a6a-ab88-44ff559f000b.png">

### Kitti Dataset
- Start the node via `$ ros2 run dee_image_publisher kitti_publisher`. You should be able to view the callback message in your terminal. This node will publish data to two topics `/kitti_image` and `kitti_pointcloud`.
- You can also utilize `foxglove` to visualize the published image and point cloud. Once connected to the web socket, enable the visibility of the `/kitti_pointcloud` topic, and it should appear as shown below::
  <img width="945" alt="Screen Shot 2023-04-01 at 2 07 32 PM" src="https://user-images.githubusercontent.com/90799662/229339877-46bbabc9-eb18-435d-b194-87c838ccb51a.png">
  
### Rosbag
- Download the [bag file](#get-testing-data)
- Start the node via `$ ros2 run dee_image_publisher rosbag_publisher`. You should be able to view the callback message in your terminal.
