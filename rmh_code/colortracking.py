#! /usr/bin/env python
# import cv2
# import numpy as np

# lowerBound=np.array([33,80,40])
# upperBound=np.array([102,255,255])

# cam= cv2.VideoCapture(0)
# kernelOpen=np.ones((5,5))
# kernelClose=np.ones((20,20))

# #font=cv2.cv.InitFont(cv2.cv.CV_FONT_HERSHEY_SIMPLEX,2,0.5,0,3,1)

# while True:
#     ret, img=cam.read()
#     img=cv2.resize(img,(340,220))

#     #convert BGR to HSV
#     imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#     # create the Mask
#     mask=cv2.inRange(imgHSV,lowerBound,upperBound)
#     #morphology
#     maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
#     maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

#     maskFinal=maskClose
#     conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
#     cv2.drawContours(img,conts,-1,(255,0,0),3)
#     for i in range(len(conts)):
#         x,y,w,h=cv2.boundingRect(conts[i])
#         cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255), 2)
#         cv2.cv.PutText(cv2.cv.fromarray(img), str(i+1),(x,y+h),font,(0,255,255))
#     cv2.imshow("maskClose",maskClose)
#     cv2.imshow("maskOpen",maskOpen)
#     cv2.imshow("mask",mask)
#     cv2.imshow("cam",img)
#     cv2.waitKey(10)
# python dynamic_color_tracking.py --filter HSV --webcam
 
import cv2
import argparse
import numpy as np
 
 
def callback(value):
    pass
 
 
def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)
 
    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255
 
        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)
 
 
def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filter', required=True,
                    help='Range filter. RGB or HSV')
    ap.add_argument('-w', '--webcam', required=False,
                    help='Use webcam', action='store_true')
    args = vars(ap.parse_args())
 
    if not args['filter'].upper() in ['RGB', 'HSV']:
        ap.error("Please speciy a correct filter.")
 
    return args
 
 
def get_trackbar_values(range_filter):
    values = []
 
    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)
    return values
 
 
def main():
    args = get_arguments()
 
    range_filter = args['filter'].upper()
 
    camera = cv2.VideoCapture(2)
 
    setup_trackbars(range_filter)
 
    while True:
        if args['webcam']:
            ret, image = camera.read()
 
            if not ret:
                break
 
            if range_filter == 'RGB':
                frame_to_thresh = image.copy()
            else:
                frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)
 
        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
 
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
 
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
 
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(image, center, 3, (0, 0, 255), -1)
                cv2.putText(image,"centroid", (center[0]+10,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
                cv2.putText(image,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
 
        # show the frame to our screen
        cv2.imshow("Original", image)
        cv2.imshow("Thresh", thresh)
        cv2.imshow("Mask", mask)
 
        if cv2.waitKey(1) & 0xFF is ord('q'):
            break
 
 
if __name__ == '__main__':
    main()