import cv2
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

flags = [i for i in dir(cv2) if i.startswith('COLOR_')]

len(flags)
flags[40]
nemo = cv2.imread('./test.jpeg')
nemo = cv2.cvtColor(nemo, cv2.COLOR_BGR2RGB)
hsv_nemo = cv2.cvtColor(nemo, cv2.COLOR_RGB2HSV)
light_orange = (1, 190, 200)
dark_orange = (18, 255, 255)
mask = cv2.inRange(hsv_nemo, light_orange, dark_orange)
result = cv2.bitwise_and(nemo, nemo, mask=mask)
plt.subplot(1, 2, 1)
plt.imshow(result, cmap="gray")
plt.show()
blur_img = cv2.GaussianBlur(hsv_nemo, (3,3), 0)
img_edges = cv2.Canny(image=blur_img, threshold1=50, threshold2=155)

'''cv2.imshow('Canny Edge Detection', img_edges)
cv2.waitKey(0)
cv2.destroyAllWindows()'''
