# Indoor Navigation via Vision-Inertial Data Fusion

This is the code for the following paper:

Farnoosh, A., Nabian, M., Closas, P., & Ostadabbas, S. (2018, April). First-person indoor navigation via vision-inertial data fusion. In Position, Location and Navigation Symposium (PLANS), 2018 IEEE/ION (pp. 1213-1222). IEEE.

![Algorithm Result](figs/hallway_results.PNG)


Contact: 
[Amirreza Farnoosh](farnoosh.a@husky.neu.edu),

[Sarah Ostadabbas](ostadabbas@ece.neu.edu)


## Contents   
* [1. Requirement](#1-requirement)
* [2. iPhone APP for Collecting Video-IMU](#2-iPhone-APP-for-Collecting-Video-IMU)
* [3. Running Code for Hallway Video](#3-Running-Code-for-Hallway-Video)
* [Citation](#citation)
* [License](#license)


## 1. Requirement 

This code is written with MATLAB R2016b

## 2. iPhone APP for Collecting Video-IMU

Contact [Sarah Ostadabbas](ostadabbas@ece.neu.edu) to request access to our iPhone App for collecting synchronous video and IMU data with adjustable frequency   

## 2. Sample Video 

The original video of the hallway used for experiments in the paper along with its IMU measurements collected with out iPhone APP is included in `./sample_video/` directory.  

## 3. Running Code for Hallway Video  

Run `demo_vpdetect_modular.m`

This code contains the following sections:

* Read entire video
* Read IMU data
* Synchonize IMU and video (if not)
* Apply GMM Method On each frame
* Straight line grouping
* Find aLpha, beta and amma for each frame from vanishing directions
* Kalman filter fusion of IMU and video
* Horizon line detection
* Plane detection & depth/width inference
* Step counting & finding step locations
* 2D-map generation

## Citation 
If you find our work useful in your research please consider citing our paper:
```
@inproceedings{farnoosh2018first,
  title={First-person indoor navigation via vision-inertial data fusion},
  author={Farnoosh, Amirreza and Nabian, Mohsen and Closas, Pau and Ostadabbas, Sarah},
  booktitle={Position, Location and Navigation Symposium (PLANS), 2018 IEEE/ION},
  pages={1213--1222},
  year={2018},
  organization={IEEE}
}

```

## License 
* This code is for non-commercial purpose only. For other uses please contact ACLab of NEU. 
* No maintenance service



