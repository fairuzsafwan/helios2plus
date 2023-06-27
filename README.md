# MN-ROBOT HELIOS2 CAMERA

![Sample data from Helios](/sample.png)

## To-do List
- [x] Create docker environment
- [x] include ROS-helios wrapper
- [x] include python-helios wrapper
- [x] include Nvidia-driver
- [x] include OpenCL
- [x] Test with actual helios Camera


## How to run
1. Build docker image
```
docker-compose build
```

2. Run image
```
docker-compose run
```

3. Run the node
```
rosrun arena_camera arena_camera_node
rosrun image_view image_view image:=/arena_camera_node/image_raw
rosrun camera_calibration cameracalibrator.py --no-service-check --size 8x6 --square 0.108 image:=/arena_camera_node/image_raw camera:=/camera
```



