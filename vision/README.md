# The Computer Vision Package

## If you are on a NVIDIA Jetson or another ARM machine.
To run this module go to this link: ```https://jetsonhacks.com/2019/12/22/install-realsense-camera-in-5-minutes-jetson-nano/```
and follow the instuctions on that page to download the repo. Then run the buildlibrealsense script to build the library on the jetson.
Once it has built sucessfully run this command: ```echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/OFF' >> ~/.bashrc```
This will allow python to find the pyrealsense2 library.

## If you are not on an ARM machine.
`pip3 install pyrealsense2`
