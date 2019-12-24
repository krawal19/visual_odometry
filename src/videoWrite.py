import cv2
import numpy as np
#Path of the stored image frames
path = "D:\\673\Project_5\Oxford_dataset\output\\"


#Path to store the video
pathToVideo = "D:\\673\Project_5\Oxford_dataset\output\FinalVideoOutput.avi"
img_list = []
for i in range(0,507):
    pathImg = path + str(i) + ".png"
    img = cv2.imread(pathImg)  
    img_list.append(img)
    
    out = cv2.VideoWriter(pathToVideo,cv2.VideoWriter_fourcc(*'DIVX'), 10, (img.shape[1],img.shape[0]))
    
    for i in range(len(img_list)):
        out.write(img_list[i])
        
out.release() 
