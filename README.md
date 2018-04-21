# SDCND-P4

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.
---
[//]: # (Image References)

[image1]: ./output_images/calibration.JPG 
[image2]: ./output_images/pipeline_figure.jpg 



## Rubric Points Discussion
---

### Camera Calibration

#### 1.Camera matrix and distortion coefficient calculation 

* [The first part of the code](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding.py#L18-L31) was dedicated to calibration and calculation of objectpoints and imagepoints array with the helpe of the `cv2.findChessboardCorners()` function  
   * "object points" are the (x, y, z) coordinates of the chessboard corners in the real world. HereIt is assumed that it is planar  fixed on the (x, y) at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  
   * `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection. 
   
To visualize the corners detected, the function `cv2.drawChessboardCorners()` was used and `cv2.imwrite()` generated the following pictures   


![alt text][image1]


### Pipeline (single images)

#### 1.distortion-corrected image.
Then`objpoints` and `imgpoints` previously generated were used to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function. Distortion correction to the test image was applied using the `cv2.undistort()` in the [code](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding.py#L34-L39)

#### 2. color transforms and binary images

[In the code](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding.py#L66-L91) a combination of color and gradient thresholds to generate a binary image:
* First img is converted to HLS color space
* Sobel edge detection was applied to the L channel
* Thresholding was applied to S channel 
* Then the two latter were combined


#### 3. perspective transform 

[The code for perspective transform](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding.py#L41-L62) was based on `src` and `dst` points were generated roughly to cover a bottom cemtered trapezoid and then the transform was applied using `cv2.getPerspectiveTransform()` and `cv2.warpPerspective`

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 0, 720        | 25,695        | 
| 512, 480      | 25, 25        |
| 768, 480      | 1255, 695     |
| 1280, 720     | 1255, 25      |

Prespective transform is verified and plotted below.


#### 4. Sliding window search to find lines 
 [This part](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding.py#L93-L176) is performed as follows:
* a histogram was constructed adding up pixels vertically showing positional information on the X axis 
* the highest left and right peaks were used as areference starting point for the slide window search to begin
* windows are slid and are shifting centers if a certain number of high value pixels were found around 
* all lane lines were marked and their indices registered 
* a polyfit line, using `np.polyfit()` was drawn between the lane points at each side 

*Through this part a startegy to use history of fit line and search around it as shown in [here](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding.py#L120-L122)*
*Consequently, a [condition](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding.py#L139) is added to check how parallel the fit curves are and if a limit is broken window search is initiated again to ensure a good accuracy* 


#### 5. the radius of curvature of the lane and the position of the vehicle with respect to center.

[In the code](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding.py#L251-L267)
* a conversion from pixels to real world dimensions is defined from the classroom 
* then the polyfit is redefined accordingly 
* and the radius of curvature is calculated [mathematically](http://mathworld.wolfram.com/RadiusofCurvature.html) 
* Theoretically the camera is fixed at the car center, so that the center of the image corresponds to the center of the car, so that the X position of the car relative to the frame is exactly at the midpoint
* lane center position is exactly the middle of both lane curves 


#### 6. Drawing pipeline results 

All results of the pipeline were plotted with [this code attached](https://github.com/verena-yacoub/SDCND-P4/blob/master/output_images/plotting_pipeline.py) and shown below 

![alt text][image2]

---

### Pipeline (video)

Here's a [link to my video result](./project_output.mp4)

---

### Discussion
* Experimenting the code with the challenge video falsly detected edges appeared which implies that probably a change in the preprocessing step and binary thresholding is needed (may me color detection combined integrated in pipeline)
* Also, Hough transform may be useful for further experiments with the project 

### Notes for resubmission
* mistake of plotting on the original frame image instead of the undistorted one was fixed [here](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding_resubmission.py#L339)
* mistake of not considering the histogram bases of each frame in curvature calculation was fixed [here](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding_resubmission.py#L35)
* unnecessary recalculation of camera calibration is avoided by reallocating [this](https://github.com/verena-yacoub/SDCND-P4/blob/master/Advanced_lane_finding_resubmission.py#L35) line

*here is the reprocessed [video](https://github.com/verena-yacoub/SDCND-P4/blob/master/project_output_resubmission.mp4)* 

### References 
* Udacity classroom
* *https://github.com/k4jeremy-shannon/CarND-Advanced-Lane-Lines*
