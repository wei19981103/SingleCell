import cv2
import sys
import numpy as np

image = cv2.imread("/home/mengting/Wei/221205/04_12_2023_16_47_47.png")

#cv2.imshow("img1", image)
thresh_par1 = 45
thresh_par2 = 10
maxRadius = 25
minRadius = 10
dp =1.3
circle_par1 = 60
circle_par2 = 0.6
iterations=2
bboxes = []

# grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# apply binary thresholding

thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, thresh_par1, thresh_par2)

# detect the contours on the binary image
contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image=thresh, contours=contours, contourIdx=-1, color=(0, 0, 0), thickness=1, lineType=cv2.LINE_AA)

# dilation
el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
dil = cv2.dilate(thresh, el, iterations=iterations)

# apply a blur using the Gaussian filter
blr = cv2.GaussianBlur(dil,(3,3), cv2.BORDER_DEFAULT)
cv2.imshow("Blr", cv2.resize(blr, ((int(blr.shape[1]/3),int(blr.shape[0]/3)))))

# find circles using the Hough transform
circles = cv2.HoughCircles(image=blr, method=cv2.HOUGH_GRADIENT_ALT, dp=dp, minDist=15, 
                    param1=circle_par1, param2=circle_par2, minRadius=minRadius, maxRadius=maxRadius)

# determine ROI
if np.any(circles):

    circles = (np.rint(circles)).astype(int)
    for i in circles[0,:]:
        # bounding box (radius:i[2], (i[0],i[1])=center)
        bbox = (i[0]-i[2]-2, i[1]-i[2]-2, 2*i[2]+4)
        bboxes.append(bbox)
        # draw the outer circle with radius i[2] in green, (i[0],i[1])=center
        cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center as a circle with radius 2 in red
        cv2.circle(image,(i[0],i[1]),1,(0,0,255),1)

cv2.putText(image, str(len(bboxes)), (300,300), cv2.FONT_HERSHEY_SIMPLEX, 6,(255,0,0),10)
cv2.imshow("Processed", cv2.resize(image, ((int(image.shape[1]/3),int(image.shape[0]/3)))))

# Wait for the user to press a key
cv2.waitKey(0)
cv2.destroyAllWindows()
# if cv2.waitKey(1) & 0xFF == 27:  # Esc pressed
#     cv2.destroyAllWindows()
