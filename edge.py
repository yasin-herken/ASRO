import cv2
import numpy as np
# Read the original img
cv2.startWindowThread()
img = cv2.imread('test.jpeg')
print(img) 
blur = cv2.blur(img,(5,5))
blur0=cv2.medianBlur(blur,5)
blur1= cv2.GaussianBlur(blur0,(5,5),0)
blur2= cv2.bilateralFilter(blur1,9,75,75)
hsv = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)

light_orange = (1, 190, 200)
dark_orange = (18, 255, 255)
mask = cv2.inRange(hsv, light_orange, dark_orange)
res = cv2.bitwise_and(img,img, mask= mask)
edges = cv2.Canny(res,100,200)
cv2.imshow("Edges",edges)
cv2.waitKey(0)
cv2.destroyAllWindows()