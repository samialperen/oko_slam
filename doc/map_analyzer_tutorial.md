# MAP_ANALYZER INSTALLATION GUIDE & TUTORIALS


## Installation

### Installation Using Source
[Here](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html) is a reference source.
[Here](https://www.learnopencv.com/install-opencv3-on-ubuntu/) is an another reference source that I followed.

First install dependencies
```
$ sudo apt-get install build-essential
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
$ sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```

Install OpenCV 3.3.1 
```
$ cd ~/your_desired_location
$ git clone https://github.com/opencv/opencv.git
$ cd opencv 
$ git checkout 3.3.1 
$ cd ..
$ git clone https://github.com/opencv/opencv_contrib.git
$ cd opencv_contrib
$ git checkout 3.3.1
$ cd ..
```

Build using CMake
```
$ cd ~/your_desired_location/opencv
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
```

**NOTE:** If above cmake code does not work, try this one:
```
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
```

Install libraries (takes very long time!)
```
$ sudo make install
```


