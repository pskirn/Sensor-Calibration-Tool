1. Problem Statement 

For a any Robot that need to percieve the world arround, it needs atleast one sensor which Camera or Lidar or both in some cases. In a case of needing both sensors, both needs to be calibrated to know the pose of an object correctly.

The Camera is an 2D sensor and a Lidar is a 3D sensor and to fuse both of them, we need a transformation matrix(T) which has a Rotation(R) and translation(t) matrices, which converts the LiDAR frame to camera frame. This is a 6 dof (3 - rotation, 3 - translation).

For a point P in real world, with lidar it is P_L point in its own co-ordinate frame and camera it is P_C point in its own co-ordinate frame.

P_C = T * P_L;

where T = [R t]
          [0 1]

R is a 3x3 matrix, t is 3x1 matrix, T is the 4x4 matrix




2. Planar Target 


