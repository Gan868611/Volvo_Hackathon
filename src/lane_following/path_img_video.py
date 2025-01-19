
# %%
import numpy as np
# from tqdm import tqdm
import cv2
import os
import sys
sys.path.append("/home/volvo2/catkin_ws/src/lane_following/transformations")
from camera import transform_img, eon_intrinsics
from model import medmodel_intrinsics
import torch
sys.path.append("/home/volvo2/catkin_ws/src/lane_following/Efficient_net")
from Efficient_net.model import EfficientNet
# from model_2 import GRU
import os

from camera import img_from_device, denormalize, view_frame_from_device_frame
# %%
def draw_path(device_path, img, width=1, height=1.2, fill_color=(128,0,255), line_color=(0,255,0), isTest = False):
    device_path_l = device_path + np.array([0, 0, height])                                                                    
    device_path_r = device_path + np.array([0, 0, height])  
    
    if isTest:
        width = width - 0.5
    device_path_l[:,1] -= width                                                                                               
    device_path_r[:,1] += width

    img_points_norm_l = img_from_device(device_path_l)
    img_points_norm_r = img_from_device(device_path_r)
    img_pts_l = denormalize(img_points_norm_l)
    img_pts_r = denormalize(img_points_norm_r)

    # filter out things rejected along the way
    valid = np.logical_and(np.isfinite(img_pts_l).all(axis=1), np.isfinite(img_pts_r).all(axis=1))
    img_pts_l = img_pts_l[valid].astype(int)
    img_pts_r = img_pts_r[valid].astype(int)
    
    if isTest:
        fill_color=None
        line_color=(0,0,250)

    for i in range(1, len(img_pts_l)):
        u1,v1,u2,v2 = np.append(img_pts_l[i-1], img_pts_r[i-1])
        u3,v3,u4,v4 = np.append(img_pts_l[i], img_pts_r[i])
        pts = np.array([[u1,v1],[u2,v2],[u4,v4],[u3,v3]], np.int32).reshape((-1,1,2))
        cv2.fillPoly(img,[pts],fill_color)
        cv2.polylines(img,[pts],True,line_color)
# %%


def frames_to_tensor(frames):                                                                                               
        H = (frames.shape[1]*2)//3                                                                                                
        W = frames.shape[2]                                                                                                       
        in_img1 = np.zeros((frames.shape[0], 6, H//2, W//2), dtype=np.uint8)                                                      
                                                                                                                                
        in_img1[:, 0] = frames[:, 0:H:2, 0::2]                                                                                    
        in_img1[:, 1] = frames[:, 1:H:2, 0::2]                                                                                    
        in_img1[:, 2] = frames[:, 0:H:2, 1::2]                                                                                    
        in_img1[:, 3] = frames[:, 1:H:2, 1::2]                                                                                    
        in_img1[:, 4] = frames[:, H:H+H//4].reshape((-1, H//2,W//2))                                                              
        in_img1[:, 5] = frames[:, H+H//4:H+H//2].reshape((-1, H//2,W//2))
        return in_img1

#%%
# path = 'C:/Users/yzgan/Desktop/Kommu.ai/msia_video/'
# video_list = []
# for (root,dirs,filelist) in os.walk(path):
#     for filename in filelist:
#         if filename[-5:] == '.hevc':
#             video_list.append(os.path.join(root,filename))

# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# out = cv2.VideoWriter('msia_video_gru.mp4',fourcc,20, (1164,874))

# TOTAL_FRAMES = 1200


device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
#%%
model = EfficientNet('b0',100).to(device)
checkpoint = torch.load('/home/volvo2/catkin_ws/src/lane_following/Efficient_net/model_best.pth.tar',map_location=torch.device('cpu'))
model.load_state_dict(checkpoint['state_dict'])
model.eval()
#%%

# model = GRU(1280,512,1,100).to(device)
# checkpoint = torch.load('./model__gru_best.pth.tar')
# model.load_state_dict(checkpoint['state_dict'])
# model.eval()


# for VIDEO_PATH in tqdm(video_list):
#     vid = cv2.VideoCapture(VIDEO_PATH)
#     bgr_imgs = []
#     yuv_imgs = []
#     for frame_number in range(TOTAL_FRAMES):
       
#         ret, frame = vid.read()
#         if (frame_number % 5 == 0):
#             try:
input_image = cv2.imread('/home/volvo2/catkin_ws/src/lane_following/images/output_018.jpg') 
input_image = input_image[:, 0:320] 
frame = cv2.resize(input_image,(1164,874))
bgr_imgs = frame
img_yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)
yuv_img = img_yuv


    
imgs_med_model = np.zeros((384, 512), dtype=np.uint8)
imgs_med_model = np.expand_dims(imgs_med_model, axis=0)

imgs_med_model = transform_img(yuv_img, from_intr=eon_intrinsics, to_intr=medmodel_intrinsics, yuv=True,
                                output_size=(512,256))
imgs_med_model = np.expand_dims(imgs_med_model, axis=0)
frame_tensors = frames_to_tensor(np.array(imgs_med_model)).astype(np.float32)/128.0 - 1.0


#%%

    
input_tensor = torch.from_numpy(frame_tensors).to(device)



# img_shift = np.roll(frame_tensors,1)
# input_tensor = np.stack((img_shift,frame_tensors),1)
# input_tensor = torch.from_numpy(input_tensor).to(device)

with torch.no_grad():
    outs = model(input_tensor)
    

outs = outs.detach().cpu().numpy()
print(outs)

#%%
for i in range(len(frame_tensors)):
    verification_img = bgr_imgs
    forward_data = np.linspace(0,99,100)
    down_data = np.linspace(0,6,100)
    draw_path(np.transpose([forward_data,outs[i],down_data]), img=verification_img, isTest = True)
    save_path = 'saved_image.jpg'  # Specify the file path and name for saving
    success = cv2.imwrite(save_path, verification_img)

    # out.write(verification_img)

    
# %%
# %%





# cv2.destroyAllWindows()
# out.release()


# %%
