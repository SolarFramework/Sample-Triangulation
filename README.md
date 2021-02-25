# SolAR Sample Triangulation

[![License](https://img.shields.io/github/license/SolARFramework/Sample-triangulation?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

The SolAR **Triangulation sample** shows a SolAR pipeline that sparsely reconstructs a 3D scene given to viewpoints.


| ![](./StandAlone/matching.jpg) | ![](./StandAlone/cloudpoint.jpg) |
|:-:|:-:|
| Matching | Cloudpoint | 


## How to run

* If you want to change the calibration parameters of the camera, edit the [camera_calibration.yml](./Mono/camera_calibration.yml).

* To change properties of the components of the Triangulation pipeline, edit the [conf_Triangulation.xml](./Mono/conf_Triangulation.xml) file.


### StandAlone

To run it, launch:

> #### Windows
>
	SolARTriangulationSample.exe

> #### Linux
>
	./run.sh ./SolARTriangulationSample

You will visualize the 3D point cloud built from two viewpoints.

* Press `escape` to quit the application.

## Contact 
Website https://solarframework.github.io/

Contact framework.solar@b-com.com

