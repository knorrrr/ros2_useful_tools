# ros2_useful_tools
Simple useful tools for ROS 2 Humble

### Getting Started
```bash
mkdir -p ros2_ws/src && cd ros2_ws/src
git clone git@github.com:knorrrr/ros2_useful_tools.git
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## mono16torgb8
This node converts from mono16 to RGB8.(But Image keeps grayscale.) 
```bash
ros2 launch mono16torgb8 mono16torgb8.launch.py
```
### IO
| Name                   | Type    | Description                              | Default                   |
|------------------------|---------|------------------------------------------|---------------------------|
|input_image_topic       | String  | Input image topic(momno16)               | /davis/image_raw          |
|output_image_topic      | String  | Output image topic (rgb8)                | /davis/image_raw_rgb8     |

## cameraInfo_publisher
This node publishes cameraInfo topic from a yaml file.
```bash
ros2 launch camerainfo_publisher camerainfo_publisher.launch.py
```

### IO
| Name                   | Type    | Description                              | Default                   |
|------------------------|---------|------------------------------------------|---------------------------|
|camerainfo_yaml_path    | String  | Path to the YAML camera calibration file | /path/to/camera_info.yaml |
|camerainfo_topic        | String  | Topic of publishing camera infomation    | camera_info               |
|frame_id                | String  | FrameID of camerainfo topic              | davis                     |
|use_opencv              | Bool    | Whether it is a calibration file created by OpenCV.| True            |
 
## frameid_changer
This node changes topic's frame_id.
```bash
ros2 launch frameid_changer frameid_changer.launch.py 
```
| Name                   | Type    | Description                              | Default                   |
|------------------------|---------|------------------------------------------|---------------------------|
| frame_id               | String  | Frame ID to be converted                 | new_frame_id              |
| msg_type               | String  | Message type of original topic           | IMU                       |
| new_topic              | String  | Topic name to be published               | /altered_topic            |
| orig_topic             | String  | Topic name to be subscribed              | /orig_topic               |

## transformPCD
This node changes coordinate of PointCloud2 topic according to TF information.
We need to publish TF relation between original and output topics' frame id.
```bash
ros2 launch transformpcd transformpcd.launch.py
```
| Name                   | Type    | Description                              | Default                   |
|------------------------|---------|------------------------------------------|---------------------------|
| input_topic            | String  | Input topic of PointCloud2               | /vlp16/velodyne_points    |
| output_topic           | String  | Output topic of PointCloud2              | /hdl64e/vlp16_points      |
| output_frame           | String  | The coordinate axis of output topic      | hdl64e                    |

## twist_converter
This node converts from TwistStamped to TwistWithCovarianceStamped(Covariance:0).
```bash
ros2 launch twist_converter twist_converter.launch.py
```
| Name                   | Type    | Description                                | Default                   |
|------------------------|---------|--------------------------------------------|---------------------------|
| frame_id               | String  | Frame id to be converted.                  | base_link                 |
| orig_topic             | String  | Input topic of TwistStamped                | /xsens/velocity           |
| converted_topic        | String  | Output topic of TwistWithCovarianceStamped | /xsens/twist_with_covari  |
## ros2_useful_launcher
### crop_box.launch
This node launches crop box filter in pointcloud preprocessor(Autoware Universe). 
```bash
ros2 launch ros2_useful_launcher  crop_box.launch.py 
```

### tf_publisher
This node publishes tf relations loading yaml file, and launches tunable_tf_publisher in tier4/CalibrationTools (Optional).
```bash
ros2 launch ros2_useful_launcher  tf_publisher.launch.py 
```