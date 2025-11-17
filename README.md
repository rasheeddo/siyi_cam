# SIYI A8 Mini ROS2 package

This is an unofficial SIYI A8 Mini gimbal camera control package, the software is made up to control gimbal by UDP packet from official manual V1.6, and the video streaming is available via ROS image transport and also V4L2 devices (/dev/videoX).

## Deps

- `sudo apt install libuvc-dev libusb-1.0-0-dev`

- `cv_bridge` package.

## ROS parameters

- `cam_ip` : a string type, camera IP address by default as `192.168.144.25` from factory.

- `video_out` : a string type, V4L2 device name, e.g. `/dev/video30`, this device supposes to be made from `v4l2loopback` as virtual webcam, then this `siyi_cam` package will be streaming the camera frame to this specified V4L2 device. Default is `/dev/video30`, you must run `sudo modprobe v4l2loopback video_nr=30 exclusive_caps=1 card_label="Fake Device"` at first in order to see it.

- `use_gst` : a boolean type, to select either to use gstreamer pipeline or just RTSP url on the OpenCV video capture. On some computer, using gstreamer is faster that RTSP. Default is `true`.

## ROS Topics

- `/siyi/gimbal`  : std_msgs/msg/Int8MultiArray, to control gimbal movement, [pan_left, tilt_up, center, tilt_down, pan_right], we just need to publish a value of 1 in each index to move. 

```
    For example:
        To pan camera to left then publish as, [1,0,0,0,0].
        To tilt camera up then publish as [0,1,0,0,0].
        To tilt camera down then publish as [0,0,0,1,0].
        To pan camera to right then publish as [0,0,0,0,1].

        The gimbal will keep moving if the this topic keep publishing,
        to stop the movement then publish as [0,0,0,0,0] once.

        To bring gimbal back to center then publish as [0,0,1,0,0].

```

- `/siyi/zoom` : std_msgs/msg/Int8MultiArray, to zoom the camera in or out, [zoom_in, zoom_out], we just need to publish a value of 1 in each index to zoom.

```
    For example :
        To zoom in then publish as [1,0]
        To zoom out then publish as [0,1]

        The camera will keep zooming if the this topic keep publishing.

        To stop zoom then publish as [0,0] once.
```

- `/siyi/camera` : sensor_msgs/msg/Image, to see the video streaming from ROS topic. 

```
    The QoS reliability must be best_effort, e.g. 
    ros2 run image_tools showimage --ros-args -r image:=/siyi/image -p reliability:=best_effort
``` 

- `/siyi/mode` : std_msgs/msg/Int8, to select camera operating mode.

```
    For example :
        To select Follow Mode then publish as 0, the pan movement will be following with the attached frame, but tilt movement is lock to horizontal, all gimbal axes are able to be controlled.

        To select FPV Mode then publish as 1, the pan and tilt movements will be following with the attached frame, gimbal cannot be controlled.
        
        To select Lock Mode then publish as 2, the pan and tilt are locked in place even the attached frame is moving.
```

