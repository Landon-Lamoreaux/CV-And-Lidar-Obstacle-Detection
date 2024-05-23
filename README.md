# CV-And-Lidar-Obstacle-Detection
This is my obstacle detection code I wrote to detect white lane lines and 3D obstacles. This code was developed for the Intelligent Ground Vehicle Competition (IGVC) 2023 competition. 
I wrote this code for the South Dakota School of Mines and Technology Rocker Robotics team. Computer vision was used to look at 2 camera feeds and only keep the pixels that are white and belong to a lane line. These locations were used to create pointclouds of points we want to avoid. 
A coordinate transformation was then done to convert the pointclouds from local coordinates to world coordinates. Theses were then combined with the pointcloud of 3D obstacles created by our Lidar, this pointcloud was also transformed into world points first. 
The Lidar data was also filtered to remove all the points that we don't care about or don't want, these were set to very large numbers. Once all the pointclouds were merged the final pointcloud was sent out on a ROS topic as a list of all the obstacles in the world we want to avoid. Our path planning algorithm then used this to plan a path.