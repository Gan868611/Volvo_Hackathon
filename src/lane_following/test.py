#%%
# import imp
import cv2 
import numpy as np

# video = cv2.VideoCapture('/home/aman/projects/compVision/lane_detection/test_video/Highway - 10364.mp4')


        # Fill lane detection algotithm
img = cv2.imread('/home/volvo2/catkin_ws/src/lane_following/images/output_039.jpg') 
blur = cv2.GaussianBlur(img,(5,5),0)
edges = cv2.Canny(blur,100,200)

mask = np.zeros_like(edges)   
ignore_mask_color = 255

imshape = img.shape
vertices = np.array([[(0,imshape[0]),(450, 290), (490, 290), (imshape[1],imshape[0])]], dtype=np.int32)

cv2.fillPoly(mask, vertices, ignore_mask_color)
masked_edges = cv2.bitwise_and(edges, mask)
#%%
rho = 1
theta = np.pi/270
threshold = 1
min_line_length = 10
max_line_gap = 1

line_image = np.copy(masked_edges)*0 

lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]),
                        min_line_length, max_line_gap)

for line in lines:
    for x1,y1,x2,y2 in line:
        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)

combo = cv2.addWeighted(masked_edges, 0.2, line_image, 1, 0) 
save_path = 'saved_image.jpg'  # Specify the file path and name for saving
success = cv2.imwrite(save_path, combo)




# %%
