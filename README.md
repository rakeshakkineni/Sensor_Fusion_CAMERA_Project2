# SFND 2D Feature Tracking

In this project methods to track objects across multiple camera images were explored. Various detector / descriptor combinations were tested to see which ones perform best. This mid-term project consists of four parts:

* First, implementing ring buffer to optimize memory load. 
* Second, keypoint detectors such as HARRIS, FAST, BRISK and SIFT were integrated and they were compared with respect to number of keypoints and speed. 
* Third, various descriptor extraction routines like BRIEF, ORB, FREAK, AKAZE, SIFT were integrated and descriptors were matched using brute force and also the FLANN approach we discussed in the previous lesson. 
* Last part, once the code framework is complete, the various algorithms were tested in different combinations and they were compared with regard to some performance measures. 

## Ring Buffer 
As requested in first task a ring buffer was implemented. Implemented logic is 
 - Until buffer is full inserting new images at the beginning of the buffer. 
 - Once the buffer is full , shift all the buffers right remove last buffer and insert the new image in the beginning. 

## Filtering only preceding vehicle keypoints
- Coordinates were placed in a rect object.
- Every detected keypoint is checked if it in the rectangular that fits the coordinates, all the points that are outside are removed.

## Performance Measurement 
### Keypoints and Neighbourhood Statistics
Different detection routines were implemented. Then a test code was implemented, it prints out keypoint size , Neighbourhood size min , Neighbourhood size max , Neighbourhood mean , Neighbourhood Standard Deviation. The test code is commented out in the final project. 

### Descriptor Match Size
Different descriptors , matchers were implemented. A test code to write the matcher output size for each image into a file was implemented. Code was modified to accept descriptor and detector types from command prompt using cin. Code was executed with different combinations of detectors and descriptors. In the final project all the mentioned modifications were commented out. 

### Excution Time measurement 
Different descriptors , matchers were implemented. A test code to write the execution time of detection time and descriptor time for each image into a file was implemented.  Code was modified to accept descriptor and detector types from command prompt using cin. Code was executed with different combinations of detectors and descriptors. In the final project all the mentioned modifications were commented out. 

## Output
All the measurements are placed in Output/Performance_Analaysis_Updated.xlsx

## Performance analysis
Based on the execution time measurements i consider the following to be the best 3 combinations. 
1) FAST Detection and ORB Descriptor
2) FAST Detection and BRIEF Descriptor
3) ORB Detection and BRIEF Descriptor
