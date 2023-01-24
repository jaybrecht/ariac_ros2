# Updates

## Release 2023, Jan 23 -- Beta Version

- This is the beta release of the software. This release contains the basic structure of the software and is missing some features that were highlighted during the launch telecon. These features will be added in the final release.
- A list of the features that are not currently supported is provided as follows:
  - Assembly tasks.
  - Combined tasks.
  - Faulty gripper challenge.
  - Human Operator challenge.
- Documentation for features that come with this version has been provided.
- Documentation for features that are not currently supported will be provided in the final release.
- During the beta release, competitors are expected to:
  - Build a ROS2 package.
  - Understand the ARIAC interfaces. At a minimum, competitors must be capable of doing the following:
    - Move AGVs using the service or the velocity controllers.
    - Start and end the competition.
    - Retrieve information on part locations.
    - Receive and submit orders.
    - Move the robots to perform pick-and-place.
    - Understand trial configuration files and write custom ones.
    - Place sensors:
      - Sensor placement requires a good understanding of part locations and the tasks in ARIAC.

## Improvements

One of the goals of the beta release is to identify improvements that can be made to the interface so they can be addressed in the final release. Some bugs may still present in this release, so make sure to report the issues you find on the [ARIAC GitHub](https://github.com/usnistgov/ARIAC) page.
