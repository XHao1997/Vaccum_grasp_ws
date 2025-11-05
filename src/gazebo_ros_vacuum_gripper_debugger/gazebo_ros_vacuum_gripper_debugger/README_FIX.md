# Vacuum Gripper Bug Fix

## Problem
The vacuum gripper in Gazebo was unable to pick up objects. The issue had several causes:

## Issues Found and Fixed

### 1. Missing Inertial Properties in Test World (box.world)
**Problem**: The box model in the test world was missing inertial properties (mass, inertia matrix).
**Impact**: Gazebo couldn't properly simulate physics without these properties.
**Fix**: Added proper inertial properties to the box:
- Mass: 1.0 kg
- Inertia matrix for a 1×1×1 box at its center
- Added friction coefficients

**File**: `worlds/box.world`

### 2. Insufficient Vacuum Force
**Problem**: The suction force was too weak (max 20 N) to reliably grip objects.
**Fix**: Increased the suction force from 20 N to 100 N.
**File**: `src/gazebo_ros_vacuum_gripper.cpp` (lines ~215-223)

### 3. Excessive Debug Output
**Problem**: The plugin was logging info every frame for all objects in the scene, causing performance issues.
**Fix**: Changed to debug-level logging that only logs when objects are close to the gripper.
**File**: `src/gazebo_ros_vacuum_gripper.cpp` (lines ~197-201)

### 4. Incomplete Velocity Synchronization
**Problem**: When objects were very close (< 1cm), velocities weren't properly synchronized after setting pose.
**Fix**: Added explicit velocity synchronization after setting world pose.
**File**: `src/gazebo_ros_vacuum_gripper.cpp` (lines ~217-219)

### 5. Incorrect Inertia Matrix in Robot URDF
**Problem**: The vacuum gripper link had an incorrect ixy value of 0.1 instead of 0.
**Fix**: Corrected ixy to 0 for a proper diagonal inertia tensor.
**File**: `auboi5/urdf/auboi5.urdf.xacro` (line ~239)

## Testing

### Method 1: Using the Test Script
```bash
# Terminal 1: Launch Gazebo
roslaunch gazebo_ros_vacuum_gripper_debugger test_gazebo.launch

# Terminal 2: Run the test script
rosrun gazebo_ros_vacuum_gripper_debugger test_vacuum_gripper.py
```

### Method 2: Manual Testing
```bash
# Terminal 1: Launch Gazebo
roslaunch gazebo_ros_vacuum_gripper_debugger test_gazebo.launch

# Terminal 2: Activate the gripper
rosservice call /vacuum_gripper/on

# Terminal 2: Check status
rostopic echo /vacuum_gripper/grasping

# Terminal 2: Deactivate the gripper
rosservice call /vacuum_gripper/off
```

### Expected Behavior
1. When the gripper is activated (using the `/on` service), it should apply suction forces to objects within 5cm
2. The gripper should pull objects towards it and lock their position when within 1cm
3. The `/grasping` topic should publish `true` when an object is being held
4. When deactivated (using the `/off` service), the gripper releases the object

## How the Vacuum Gripper Works

The plugin works by:
1. Checking the distance between the gripper and all objects in the scene
2. For objects within 5cm, it applies a suction force: F = 100/distance (max 100N)
3. For objects within 1cm, it locks their position and synchronizes velocities
4. The suction force pulls objects towards the gripper center

## Parameters

Key parameters can be adjusted in the URDF:
- `<maxDistance>`: Maximum distance for suction force (not currently used in code, hardcoded to 5cm)
- The actual working distance is hardcoded in the C++ code as 0.05 meters (5cm)
- The lock distance is 0.01 meters (1cm)

## Additional Notes

- The gripper works best with objects that have proper collision shapes
- The object should have realistic mass and inertia properties
- Very heavy objects may require increased suction force
- The gripper plugin currently ignores the `<maxForce>`, `<maxDistance>`, and `<minDistance>` parameters from the URDF (they're not read in the Load function)
