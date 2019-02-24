# GeometricOS-Perception

Takes synchronised sensor data as input and outputs processed visual data.

- Outputs detected object labels.
- Outputs positions and orientations for each detected object
- Generates 3D reconstruction.


TODO Next

- Initiate Dense 3D model with Elastic Fusion
- For each new frame, write code for object detection (both in 2D and 3D)
- Set-up transformations that link each Camera pose with object poses i.e. track object
    and camera poses over time

TODO Later
-Propogate object detections both backward and forwards in time.
-(Object level Kalman Filtering)