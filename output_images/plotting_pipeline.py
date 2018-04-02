# -*- coding: utf-8 -*-
"""
Created on Mon Apr  2 13:20:42 2018

@author: Verena Maged
"""

# -*- coding: utf-8 -*-
"""
Created on Fri Mar 30 17:08:09 2018

@author: Verena Maged
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Mar 28 15:36:17 2018

@author: Verena Maged
"""


"""
This code was mostly taken from udacity carnd classroom Lesson 15 Advanced lane finding 

And the functionalities were tested and compared with the code in the following repository 
https://github.com/jeremy-shannon/CarND-Advanced-Lane-Lines/blob/master/project.ipynb
"""
#------------------- Importing Libraries ------------------------------------------------------------------------------------------------------------#
import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
from moviepy.editor import VideoFileClip

#------------------- Calibrate Camera Once  ---------------------------------------------------------------------------------------------------#

objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane.

objp = np.zeros((6*9,3), np.float32) # first we construct a 3d array of all zeros, in our 2D image target we always have Z=0
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1,2) # so, we keep Z=0 as it is and we generate a grid corresponding to our x,y corners preknown numbers 

images = glob.glob('camera_cal/calibration*.jpg')
for i, filename in enumerate(images):
    img = cv2.imread(filename)    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert image to gray in order for the next function to work properly
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None) # function to find the chessboard-corener in gray image
    if ret == True: # if corners were found
            imgpoints.append(corners) # add corners point to the image point array
            objpoints.append(objp) # add the prepared object points to the object points array     
    
#------------------- Defining necessary functions ---------------------------------------------------------------------------------------------------#
def Undistort (image,newobjpoints, newimgpoints): # function to undistort image based on object points and image points already determined
    
    img_size = (image.shape[1], image.shape[0]) # get image shape in the format Width, Height
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(newobjpoints, newimgpoints, img_size, None, None) #calibrate camera and calculate camera matrix 
    cv2.undistort(image, mtx, dist, None, mtx) # undistort images remap the original image with distortion compensation 
    return image

def Birdeye (image):
    
    # defining checkpoints relative to image size 
    midpoint = image.shape[1]//2 
    farOffset = image.shape[1]//10
    nearOffset = image.shape[1]//2
    length = image.shape[0]//3
    bottom_margin = 0
    
    # defining source points and destination points which take a trapezoid of interest and map it to the flat image 
    src = np.float32([[midpoint-nearOffset, image.shape[0]-bottom_margin], [midpoint+nearOffset+30, image.shape[0]-bottom_margin],
                     [midpoint-farOffset,image.shape[0]-length], [midpoint+farOffset, image.shape[0]-length]])

    dst = np.float32([[25, image.shape[0]-25], [image.shape[1]-25, image.shape[0]-25],
                     [25,25], [image.shape[1]-25, 25]])
    
    img_size = (image.shape[1], image.shape[0]) # get image shape in the format Width, Height
    
    M = cv2.getPerspectiveTransform(src, dst) # calculating the linear transformation matrix to be applied for perspective shift
    Minv= np.linalg.inv(M) # calculating the inverse transformation matrix for remapping drawing to original perspective afterwards
    warped= cv2.warpPerspective(image, M, img_size) # map the images according to the transformation matrix
    return warped,Minv 



def pipeline(img, s_thresh=(90, 255), sx_thresh=(20, 100)):
    img = np.copy(img)
    
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)# Convert to HLS color space and separate the V channel
    l_channel = hls[:,:,1] # saving L channel
    s_channel = hls[:,:,2] # saving S channel
   
    # Sobel x
    sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx)) # scale back the image values
    
    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
    # Threshold color channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    
    # Stack each channel and output an image value where one of both channels have a value
    color_binary = np.dstack(( np.zeros_like(sxbinary), sxbinary, s_binary)) #* 255
    color_binary[(s_binary == 1) | (sxbinary ==1)]=1 
    #color_binary=color_binary*255
    
    return color_binary
    
def Findlines (binary_warped):
    
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(binary_warped.shape[0]//nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    margin = 80
    # Set minimum number of pixels found to recenter window
    minpix = 40
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Draw the windows on the visualization image
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(255,255,0), 2) 
        cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(255,255,0), 2) 
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    # Fit a second order polynomial to each
    
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    
    
    
    y_eval = np.max(ploty)
    
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/700 # meters per pixel in x dimension
    
    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters
    h = binary_warped.shape[0]
    if right_fit is not None and left_fit is not None:
        car_position = binary_warped.shape[1]/2
        l_fit_x_int = left_fit[0]*h**2 + left_fit[1]*h + left_fit[2]
        r_fit_x_int = right_fit[0]*h**2 + right_fit[1]*h + right_fit[2]
        lane_center_position = (r_fit_x_int + l_fit_x_int) /2
        center_dist = (car_position - lane_center_position) * xm_per_pix
    return left_fitx,right_fitx, ploty,left_curverad, right_curverad, center_dist, left_fit, right_fit,left_lane_inds,right_lane_inds,out_img,histogram,leftx,lefty,rightx,righty
    


    
def draw_lane(original_img, binary_img, l_fit, r_fit, Minv):
    new_img = np.copy(original_img)
    
    if l_fit is None or r_fit is None:
        return original_img
    # Create an image to draw the lines on
    warp_zero=np.copy(binary_img)*0
    warp_zero =  np.zeros_like(binary_img[:,:,0].astype(np.uint8))
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    
    h,w,z = binary_img.shape
    ploty = np.linspace(0, h-1, num=h)# to cover same y-range as image
    left_fitx = l_fit[0]*ploty**2 + l_fit[1]*ploty + l_fit[2]
    right_fitx = r_fit[0]*ploty**2 + r_fit[1]*ploty + r_fit[2]

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    colorwarp= cv2.fillPoly(color_warp, np.int_([pts]),color=(0,255,0))
    cv2.polylines(colorwarp, np.int32([pts_left]), False,(int(255),int(0), int(0)), 15)
    cv2.polylines(colorwarp, np.int32([pts_right]),False,(int(0),int(0), int(255)), 15)
    
    
    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp= cv2.warpPerspective(colorwarp, Minv, (w, h))
    #plt.imshow(newwarp)
    
    
    # Combine the result with the original image
    result = cv2.addWeighted(new_img, 1, newwarp, 0.8, 0)
    return result, colorwarp

def draw_data(original_img, curv_rad, center_dist):
    new_img = np.copy(original_img)
    font = cv2.FONT_HERSHEY_DUPLEX
    text = 'Curve radius: ' + '{:04.2f}'.format(curv_rad) + 'm'
    cv2.putText(new_img, text, (40,70), font, 1.5, (200,255,155), 2, cv2.LINE_AA)
    direction = ''
    if center_dist > 0:
        direction = 'right'
    elif center_dist < 0:
        direction = 'left'
    abs_center_dist = abs(center_dist)
    text = '{:04.3f}'.format(abs_center_dist) + 'm ' + direction + ' of center'
    cv2.putText(new_img, text, (40,120), font, 1.5, (200,255,155), 2, cv2.LINE_AA)
    return new_img



 #--------------------------------------------------------------------------------------------------   

frame=cv2.imread ('test1.jpg')

undist= Undistort (frame,objpoints,imgpoints) #undistort
warped,Minv= Birdeye(undist) # warp prespective
combined_binary1=pipeline(warped) # make it binary
combined_binary2= combined_binary1[:,:,0] # make the binary 2D 
left_fitx,right_fitx, ploty,left_curverad, right_curverad, center_dist, left_fit, right_fit,left_lane_inds,right_lane_inds,out_img,histogram,leftx,lefty,rightx,righty= Findlines (combined_binary2) # findlines on binary
curv_rad= (left_curverad+right_curverad)/2 # calculate average curvature of both lines
result, newwarp= draw_lane(frame, combined_binary1,left_fit, right_fit, Minv) # draw lane on frame
newimg= draw_data(result, curv_rad, center_dist) # draw measurment data on frame    


cv2.imwrite( 'ot.jpg',out_img) 
out_img1= np.asarray(out_img, dtype='uint8')

f, ([ax1,ax2],[ax3,ax4],[ax5,ax6],[ax7,ax8]) = plt.subplots(4,2, figsize=(20,20))
ax1.set_title('original', fontsize=20)
ax1.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

ax2.set_title('undistorted', fontsize=20)
ax2.imshow(cv2.cvtColor(undist, cv2.COLOR_BGR2RGB))

ax3.set_title('birdseye', fontsize=20)
ax3.imshow(cv2.cvtColor(warped, cv2.COLOR_BGR2RGB))

ax4.set_title('binary', fontsize=20)
ax4.imshow(combined_binary2, cmap='gray')

ax5.set_title('histogram', fontsize=20)
ax5.plot(histogram)       

ax6.set_title('Window search', fontsize=20)      
ax6.imshow(cv2.cvtColor(out_img1, cv2.COLOR_BGR2RGB))


ax7.set_title('Lines drawn', fontsize=20)
ax7.imshow(combined_binary2,cmap='gray')
ax7.plot(left_fitx, ploty, color='red',linewidth=5.0)
ax7.plot(right_fitx, ploty, color='red',linewidth=5.0) 
   

ax8.set_title('final image', fontsize=20)
ax8.imshow(cv2.cvtColor(newimg, cv2.COLOR_BGR2RGB))   
    