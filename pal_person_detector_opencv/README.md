# pal_person_detector_opencv
ROS node integrating the person detector for RGB images based on OpenCV's HoG Adaboost cascade

## How to launch

`roslaunch pal_person_detector_opencv detector.launch image:=my_image max_rate:=10 scale:=0.5`

where

`mmy_image` is the image topic name that the node will subscribe to

`max_rate` is the maximum frequency at which the node will publish detections

`scale` is the scaling factor belonging to (0, 1] that will be applied to the images before running the detector. It is useful to downsample the images in order to speed up the detector.

If no arguments are provided the node will subscribe to `/xtion/rgb/image_raw' and will downscale the images by a factor of 0.5 and the maximum publication rate will be 5 Hz.

## Detections topic

The detections are published in the topic `/person_detector/detections` and the message type is [pal_detection_msgs::Detections2d](https://github.com/pal-robotics/pal_msgs)

## Debug image

The image `/person_detector/debug` shows the processed images with the ROIs corresponding to detected persons. In order to visualize the debug image you may do as follows:

`rosrun image_view image_view image:=/person_detector/debug`




