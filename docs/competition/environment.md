# Environment

The simulation environment is a representation of an order fulfillment workcell. The components that constitute the workcell are described on this page.

## Robots

The workcell consists of two robots, which can perform pick-and-place operations. The other robots located in the workcell are automated guided vehicles (AGVs), which can be used to transport parts between stations.

### Ceiling Robot

The ceiling robot is a robot constituted of one torso attached to rails mounted on the ceiling. A UR10e arm is attached to the torso. The ceiling robot is capable of performing both kitting and assembly.

![ceiling robot](../images/CeilingRobot.jpeg)

### Floor Robot

The floor robot is a UR10e arm which can move along a linear rail. The ceiling robot can only perform kitting.

![floor robot](../images/FloorRobot.png)



## Sensors

Competitors can place sensors around the environment in static locations. Sensors can be placed in any free space in the workcell. Sensors do not need to be attached to models in the environment. Sensors must be used in a realistic manner and must not exploit any simulation technicalities such as the logical camera seeing through obstructions. Each sensor has a cost that factors into the final score. Competitors can choose amoung six sensor/camera types.

### Break Beam

`break beam` reports when a beam is broken by an object. It does not provide distance information.

![break beam](../images/BreakBeam.png)

### Proximity

`proximity` outputs how far an object is from the sensor.

![proximity](../images/Proximity.png)
### Laser Profiler

`laser profiler` provides an array of distances to a sensed object.

![laser profiler](../images/LaserProfiler.png)
### Lidar

`lidar` provides a point cloud of detected objects.

![lidar](../images/Lidar.png)


### RGB Camera

`rgb camera` provides an RGB image of objects.

![rgb camera](../images/RGBCamera.png)

### RGBD Camera

`rgbd camera` provides an RGB and depth information of scanned objects.

![rgbd camera](../images/RGBDCamera.png)


### Basic Logical Camera

`basic logical camera` provides only poses of detected objects. The type and the color of an object are not reported by this sensor.

![proximity](../images/BasicLogicalCamera.png)

### Advanced Logical Camera

`advanced logical camera` reports the pose, the type, and the color of a detected object.

![proximity](../images/AdvancedLogicalCamera.png)

## Bins

## Conveyor Belt

## Assembly Stations

## Parts

## Kit Trays

## Quality Control Sensors

## Tool Changers

## Warehouse

## Disposal Bins
