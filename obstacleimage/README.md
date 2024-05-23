# Obstacle Image

Need to install the pcl library and pyrealsense to get everything to work:

## pcl
`sudo apt install ros-humble-pcl-ros`

## pyrealsense
`git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./vcpkg install realsense2`

and

`pip3 install pyrealsense2`