**Vision-Based-Pose-and-Velocity-Estimation-for-Quadrotor-Navigation**

# **Project Objective:**

Developed a vision-based system for accurately estimating the 3D pose and velocity of a Nano+ quadrotor using AprilTag markers for localization and optical flow for motion estimation. This project focuses on robust outlier rejection using RANSAC to enhance estimation accuracy, making it suitable for autonomous robotic navigation and control.

# **Project Overview:**

The project addresses key challenges in vision-based localization and velocity estimation for drones operating in dynamic environments. By leveraging camera-based observations and fusing them with robust estimation techniques it enables precise tracking of the quadrotor’s position and velocity even in the presence of noise and outliers.

# **Key Contributions:**

- **Accurate Pose Estimation:** Used AprilTag markers to extract pose information, transforming camera-based measurements into the robot’s frame of reference through homography and calibration techniques.
- **Velocity Estimation via Optical Flow:** Implemented a sparse optical flow algorithm to estimate motion between consecutive frames. Robust RANSAC-based outlier rejection was applied to refine velocity estimates and improve robustness.
- **Visualization and Performance Evaluation:** Visualized pose and velocity estimates alongside Vicon ground truth, providing insights into translational and rotational errors.

# **Methodology:**

**1. Pose Estimation:**

- Detected AprilTag corners in the camera image and computed the homography matrix relating the image coordinates to known world coordinates.
- Decomposed the homography to estimate the quadrotor’s 3D pose (position and orientation).
- Transformed the pose estimates from the camera frame to the robot’s frame using calibration data.

**2. Velocity Estimation:**

- Tracked sparse corner points between consecutive frames using optical flow.
- Computed pixel displacements and converted them into velocity estimates.
- Used RANSAC to reject outliers and enhance the robustness of velocity measurements, ensuring accurate and noise-resistant estimates.

**Challenges and Solutions:**

- **Noisy Sensor Data:** Applied RANSAC for outlier rejection and noise filtering, which significantly improved velocity accuracy.
- **Coordinate Transformation:** Accurately transformed pose estimates from the camera frame to the quadrotor’s frame using intrinsic and extrinsic calibration parameters.
- **Handling Real-World Variability:** Tuned the EKF models and optical flow algorithms to handle varying environmental conditions and motion scenarios.

**Key Outcomes:**

- Achieved high accuracy in pose estimation by leveraging AprilTag markers and robust homography-based transformations.
- Demonstrated reliable velocity estimation with minimal errors, as validated by comparisons with Vicon ground truth data.
- Improved robustness against noisy data through the use of RANSAC in the velocity estimation pipeline.

# **Technologies and Tools:**

Matlab

- **Pose Estimation:** AprilTag markers, Homography-based transformations
- **Velocity Estimation:** Sparse optical flow, RANSAC
- **Programming and Visualization:** MATLAB
- **Data Sources:** Pre-parsed image datasets synchronized with Vicon ground truth

# **Future Improvements:**

- Extend velocity estimation to handle real-time video streams.
- Incorporate visual odometry to enable full SLAM capabilities.
- Explore advanced noise-robust estimation techniques for further accuracy improvements in dynamic and outdoor environments.

This project demonstrates a robust framework for vision-based localization and velocity estimation, providing a foundation for applications in autonomous UAV navigation and real-time robotic control.
