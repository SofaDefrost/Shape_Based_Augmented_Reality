# Augmented Reality

This repository provides Python code for augmented reality algorithms. With this repository you will be able to overlay a 3D model to its 3D object in the view of a camera for pictures or real time videos. The algorithms are based on depth cameras, especially the ones developed by Intel: Realsense cameras. For the development of this repository, a D405 Realsense camera has been used.

Here is a quick explanation of the pipeline of the project:

1. Acquisition of the view of the camera. Allow to obtain a point cloud of the scene.
1. Filtering of this data (based on the hue, saturation, brightness ...) to extract the 3D shape of the object.
1. Resize of the 3D model, to match with the 3D shape extracted.
1. Center the 3D shape on the 3D model.
1. Find the best rotation (among given values) that best align the shape and the model.
1. Find the exact rotation that aligns the shape and the model (using ICP algorithms).
1. Compute the projection matrix.
1. Display the results.

## Usage

To run the code, you can :

- Execute the Python script *ar_image.py*. This Python script is for one image only. You can choose (in the top of the file) if you want to load a pre-saved file (there are somes provided in the 'example/input' folder), or do a new acquisition:
```console
python3 ar_image.py
```

- Execute the Python script *ar_video.py*. This Python script is for video only. You can choose (in the top of the file) if you want to load a pre-saved file (there is one provided in the 'example/input' folder named 'sofa.mply'), or do new acquisitions. This pre-saved file is an *.mply*: this format (defined in the "Python 3D ToolBox for Realsense") allows storing data of multiples *.ply* files. However, you will notice that this file only generates one small video (2 frames). This is because *.mply* files that represent long videos are usually too heavy for the GitHub repository. We recommend you to create your own file or to check our youtube channel to see demos of possible outputs (https://www.youtube.com/watch?v=dQw4w9WgXcQ).
```console
python3 ar_video.py
```

All outcomes will be stored in the 'example/output' folder.


## Prerequisites

This repository has been created using Python 3.8.10. Using another version may result in some problems. 

Python libraries required for the entire repository:

```console
pip3 install numpy
pip3 install scipy
pip3 install opencv-python
```

## Python 3D ToolBox for Realsense

The code has been developed by using the "Python 3D ToolBox for Realsense" repository. It is hence needed.

You will find all the details on the Github repository of the project : https://github.com/SofaDefrost/Python_3D_Toolbox_for_Realsense

## Important Notes

For the *ar_image.py* code and for the first image generated with *ar_video.py* script, the 3D object must be in the same orientation of its 3D model for the view of the camera (only rotations around the axis of the camera are allowed).
This decision has been taken to improve performances of the algorithm and especially to be able to make quick tests and demos.
However, you can easily change this behaviour by selecting other range of angles in the ```find_the_best_pre_rotation_to_align_points``` function (for instance ```[-180, 180, 10]``` on each axis allow to place the 3D object freely).

Authors: Thibaud Piccinali, Tinhinane Smail

Supervision: Paul Chaillou