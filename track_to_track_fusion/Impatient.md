# Introduction for the impatient

## Check out the fusion and its dependencies
```
$ cd <your_catkin_workspace>/src
$ git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/track_to_track_fusion.git
$ git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/perception_kit_msgs.git
```

### Optionally check out visualization
```
$ cd <your_catkin_workspace>/src
$ git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/perception_kit_visu.git
```

### Optionally check out evaluation package
```
$ cd <your_catkin_workspace>/src
$ git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/track_to_track_fusion_evaluation.git
```

## Build the fusion
```
$ catkin build track_to_track_fusion
```

## Run the fusion on some example measurement files (visualization and evaluation package required)
```
$ mkdir -p /srv/marv/perception_kit/
$ scp -r your-name@asy-ci-ape:/srv/marv/perception_kit/track_to_track_fusion_evaluation  /srv/marv/perception_kit/
$ roslaunch track_to_track_fusion_evaluation demo_rviz.launch test_name:=DST_001
```
