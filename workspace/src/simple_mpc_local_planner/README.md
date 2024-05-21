# Simple MPC Local Planner package

This package implements a custom local planner, studied for the navigation of two robots through a narrow corridor.

This local planner is a modified version of [DWA](http://wiki.ros.org/dwa_local_planner) local planner.

## Description

The SimpleMPCLocalPlanner uses a priority system to guarantee that only one robot at a time will pass through the corridor.

It is possible to assign to each robot a priority level (integer greater or equal to zero): a robot with a lower value has a higher level of priority.

A priority value of 0 (highest priority) will make the local planner to behave exactly like the DWA local planner.
This robot will be the first to traverse the corridor.

A robot with a lower level of priority will perform the following actions (in order).

- Check if the current global plan passes through a corridor.
- Check if the global plan of the other robot passes through the same corridor.
- Check if the corridor is in range (the furthest point of the local plan is in inside the corridor).
- If all these condition match:
    - STOP and wait for the other robot to traverse the corridor.
    - When the other robot has exited the corridor, continue as DWA.
- Otherwise continue normally as DWA.

To check if the other robot has traversed the corridor:
- Check if it has reached the center of the corridor (avoids the problem of the robot continously entering and exiting the corridor when rotating) and it is not inside the corridor.

To decide when to resume navigation:
- Check if the other robot traversed the corridor
- Check if the other robot's path still passes through the corridor (if other robot has replanned outside the corridor, the robot can proceed).
- Check if the goal of the other robot is close to its current position (other robot may never exit the corridor cause its goal is inside it).
- If any match, the navigation can resume.

## Implementation notes

1. To check if a point is inside a corridor, a simple raytrace [algorithm](https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/) is used.

2. When the robot checks if the corridor is in range, an inflated version of the corridor is used. This avoids a situation where the robot is waiting outside of the corridor but in the middle of it, making it difficult for the other robot to exit the corridor.

3. When the robot checks if the other robot has exited the corridor, another inflated version of the corridor is used (a bit more inflated than the previous one). This avoids the robot moving when the other robot is very close to it.

4. When checking if a path is in a corridor, not all of its points are checked but one every x meters. This is done for efficiency reasons.

## Parameters specification

All the parameters are set with the [SimpleMPCLocalPlanner.yaml](/workspace/src/husky_dynamic_navigation/config/SimpleMPCLocalPlanner_husky1.yaml) file. 

It has all the parameters used by the DWA local planner with these some other ones:

- **priority_level**: integer greater or equal to zero which represents the level of priority (0 is highest).

- **corridor«x»**: a list of vertices of the corridor.
It was tested with a rectangular corridor, but it should work with every type of polygon. The vertices are a list of two elements (coordinates x and y) and they **MUST** be ordered counterclockwise (starting point doesn't matter).
*«x»* indicates an integer: it starts from 1 and it is incremented by one for each corridor. (e.g. if there are 3 corridors: *corridor1*, *corridor2* and *corridor3* will be the 3 lists of vertices).

- **inflation_scaling_factor**: the scaling factor to inflate the corridor of. The inflated corridor is the one described in point 2 of the [implementation](#implementation-notes) section. A value of 1.25 works well; the inflated corridor will be 25% bigger than the original one.

- **inflation_scaling_factor2**: the scaling factor to inflate the corridor of. The inflated corridor is the one described in point 3 of the [implementation](#implementation-notes) section. A value of 1.6 works well; the inflated corridor will be 60% bigger than the original one.

- **tf_prefix**: the tf prefix of the robot.

- **consecutive_points_dist**: the distance (in meters) of the two consecutive points, when checking if a path is in a corridor. See point 4 of [this](#implementation-notes) section.

These previous parameters should be specified for both the robots, but it should not cause issues if they are specified only for the robot with lower priority. 
The following parameters must be specified **ONLY** for the robot with lower priority.

- **robot«x»**
    - **name**: the name of the robot number x (other robot 1, 2, 3 ...).
    - **global_plan_topic**: the full name of topic where the global plan of *robot«x»* is published on.
    - **amcl_topic**: the full name of topic where the amcl pose of *robot«x»* is published on.

*«x»* represents an integer, starting from 1 and incremented by one for each robot with a higher priority than the current one. 

Currently only one is supported but future work could be adding many.