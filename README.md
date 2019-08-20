# Theta Star

Theta Star is an any angle search algorithm library used for path planning.

This package includes a ROS interface, and support for standard messages.

## Release Notes
* Complete:
    * ROS Service Interface
    * Theta Star Algorithm
    * Unit Tests
* TODO:
    * Use ROS frame_ids between parent and grid for transforming grid origin 
    * Adapt Theta Star to work with grid cells that have an occupancy probability
    * Improve line of sight algorithm to avoid corner clipping
    * Add additional rostests
    * Stand-alone visualization for library

## ROS Interface
### Subscribers
* grid_sub - nav_msgs::OccupancyGrid - subscribes to occupancy grid

### Services
* plan_srv_server - nav_msgs::GetPlan - service client for path planner

### Parameters
* params.yaml - contains all ROS parameters

## Getting Started
These instructions will help you generate the necessary documentation for using this package, and list the required dependencies.

### Documentation

The documentation for this project is Doxygen based. To generate, execute the following commands:

```
cd <path>/theta_star
doxygen Doxyfile
```

### Dependencies

The follwing dependencies are required, and can be installed accordingly.

```
sudo apt install doxygen
sudo apt install libgtest-dev
sudo apt install build-essential
sudo apt install python-catkin-tools
sudo apt install ros-melodic-desktop-full

```
## Running the tests

To compile unit and pipeline tests, use the following command:
```
catkin build theta_star --no-deps --catkin-make-args run_tests
```
To run rostests for the theta_star package, use the following command:
```
rostest theta_star theta_star.test
```

### Break down into end to end tests

The unit test for this package extracts lane lines from test images validating functionality.

```
theta_star_test.cpp
```

## Built With

* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html) - Build tool used for compiling this project
* [Google Test](https://github.com/google/googletest) - Unit testing framework
* [ros_melodic](http://wiki.ros.org/melodic) - Open source meta-operating system


## Authors

* **Sean Crutchlow**

## License

This project is licensed under the MIT License - see the LICENSE file for details
