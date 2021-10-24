This is the Git repo for task 2 and 3, outlined in this [document](https://docs.google.com/document/d/144RnQbVfqP_-Lkm2pNSkxNoAAIMNT4Cj5_UVTuXWZlY/edit?usp=sharing)

## Task 2
- The main first thing to do, will be to extend the triangulation function so that it can handle up to four detections. At the moment, we use the [mid-point triangulation](https://en.wikipedia.org/wiki/Triangulation_(computer_vision)#Mid-point_method) method. Feel free to extend the method as it already is, or to build a new triangulation function using a new algorithm. You can assume that we have the camera's real world position in 3D coordinates, and that we also have the camera's parameters (these can be found in the documentation for task 1) 
- At this moment in time, we do not have test cases for four cameras, but you can create simplified scenarios for testing 
- The main piece of code is the *def triangulate* function which is in the camera_merger.py file

## Task 3
- This task is concerned with tracking the football - we will update info here in the coming days

