# LiDAR Point Cloud  Segmentation 
>Real-time Semantic Segmentation & Object Detection in LiDAR Point Clouds

# Benchmark


| Version         | Total Runtime (ms) | Runtime - grid creation (ms)  | Runtime - segmentation (ms) | Memory Consuption (kbyte) | Test Machine                                      |
| :-------------: | :-------------:    | :-------------:               |  :-------------:            |  :-------------:          |          :-------------:                          | 
| v0.0            |         95         |              84               |           11                |          92600            | CPU: Intel Xeon(R) E3-1270 v6 @ 3.80GHz x 8       |
| v0.0            |         156        |              141              |           15                |          92600            | CPU: Intel Xeon(R) E5-1620 v4 @ 3.50GHz × 8       |
| v0.1            |         10         |              6.7              |           3.2               |          51200            | CPU: Intel Xeon(R) E3-1270 v6 @ 3.80GHz x 8	     |
| v0.1            |         11.2       |              7.6              |           3.6               |          51200            | CPU: Intel Xeon(R) E5-1620 v4 @ 3.50GHz x 8       |				         

### Setting up the Framework

Third party Dependencies
  * Cmake 
  * Point Cloud Library 1.7+
  * OpenCV 2.4+

Coding policy: C++11

```
#cmake
sudo apt-get install cmake 

#pcl
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
sudo apt-get install pcl-tools

#opencv
sudo apt-get install libopencv-dev
```

## Compile & Run 

Compile 

* cd lidar_segmentation
* chmod +x startup.sh 
* sh startup.sh


Run 
* cd build
* chmod +x lidarsegm
* ./lidarsegm


## Modules in master under development

* point cloud semantic segmentation 
* object detection
 
## Branches / Release history

* master
  * Branch for current development including:
    * runtime optimization (optimize-runtime)
    * integrate object detection (hk_br)
    * include your branch here boi

# Meta

Please contact Attila Börcs for more info. (attila.borcs@hu.bosch.com)

# Trouble Shooting

Trouble during startup.sh?

Use the bugfix below 
```
sudo apt-get install libproj-dev
ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so
```

## Contributing

Clone the repository in the opened folder

Switch to proper branch, develop, commit and push it to server.

