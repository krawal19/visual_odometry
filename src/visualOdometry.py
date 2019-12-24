import numpy as np
import cv2
import glob
import random
import math
import matplotlib.pyplot as plt
from ReadCameraModel import*
from mpl_toolkits.mplot3d import Axes3D

# Find skew matrix
def skew(imagePoint0,imagePoint1):
    a = imagePoint0[0]
    b = imagePoint0[1]
    c = imagePoint0[2] 
    skew1 = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
    a = imagePoint1[0]
    b = imagePoint1[1]
    c = imagePoint1[2]
    skew2 = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
    return skew1,skew2

# Calculate Linear Triangulation
def linearTriangulate(K,P0,P1,P2,P3,P4,imagePoint0,imagePoint1):
    X1 = []
    X2 = []
    X3 = []
    X4 = []
    pose1 = np.dot(K,P0) #3X4
    pose21 = np.dot(K,P1) #3X4
    pose22 = np.dot(K,P2) #3X4
    pose23 = np.dot(K,P3) #3X4
    pose24 = np.dot(K,P4) #3X4
    for i in range(0,imagePoint0.shape[0]):
        skew1,skew2 = skew(imagePoint0[i],imagePoint1[i]) #3X3
        A1 = np.dot(skew1,pose1) #3X4
        A21 = np.dot(skew2,pose21) #3X4
        A22 = np.dot(skew2,pose22)
        A23 = np.dot(skew2,pose23)
        A24 = np.dot(skew2,pose24)
        A_1 = np.vstack((A1,A21)) #6X4
        A_2 = np.vstack((A1,A22))
        A_3 = np.vstack((A1,A23))
        A_4 = np.vstack((A1,A24))
        u1,s1,v1 = np.linalg.svd(A_1) #6X6,6X4,4X4
        u2,s2,v2 = np.linalg.svd(A_2)
        u3,s3,v3 = np.linalg.svd(A_3)
        u4,s4,v4 = np.linalg.svd(A_4)
        X1.append(v1[-1]) 
        X2.append(v2[-1])
        X3.append(v3[-1])
        X4.append(v4[-1])
    X1 = np.array(X1)
    X1 = X1.T[:3,:]/X1.T[3,:]  #Xt is 3xn
    X2 = np.array(X2)
    X2 = X2.T[:3,:]/X2.T[3,:]  #Xt is 3xn
    X3 = np.array(X3)
    X3 = X3.T[:3,:]/X3.T[3,:]  #Xt is 3xn
    X4 = np.array(X4)
    X4 = X4.T[:3,:]/X4.T[3,:]  #Xt is 3xn
    return X1, X2, X3, X4

# Find Projection Matrix
def findPMatrix(C,R,K):
    T = np.array([[1.0,0,0,-C[0][0]],[0,1.0,0,-C[1][0]],[0,0,1.0,-C[2][0]]])
    P = np.matmul(K,np.matmul(R,T))
    return P

# Check Cheirality
def checkChieralty(X,R,C):
    sub = X - C
    r3 = R[2,:]
    r3 = r3.reshape((1,3))
    triangu = np.matmul(r3 ,sub)
    test = triangu.T
    numberOfpoints = len(np.where(test > 0)[0])
    return numberOfpoints
    
# Finds L1 Norm of a Matrix
def findL1Norm(F):
    norm = np.max(F.sum(axis= 0))
    return norm

# Implements Zhang's Method of Fundamental Matrix
def zhangsMethod(imageW, imageH, points1, points2):
    dw = int(imageW/8)
    dh = int(imageH/8)
    gridPoints1 = []
    for i in range(0,8):
        for j in range(0,8):
            points1X1 = np.where(points1[:,0] < (i+1)*dw)
            testX1 = points1[points1X1]
            points1X2 = np.where(testX1[:,0] > i*dw)
            testX2 = testX1[points1X2]
            points2X1 = np.where(testX2[:,1] < (j+1)*dh)
            testX3 = testX2[points2X1]
            points2X2 = np.where(testX3[:,1] > j*dh)
            testX4 = testX3[points2X2]
            gridPoints1.append(testX4)
    gridPoints1N = list(filter(lambda x: x.size != 0, gridPoints1))
    return gridPoints1N


# Function to Normalize the data for the 8-point Algorithm
def normalizeData(points):
    meanX = np.mean(points[:,0])
    meanY = np.mean(points[:,1])
    d = np.sqrt((points[:,0] - meanX)**2 + (points[:,1] - meanY)**2)
    dMean = np.mean(d)
    scale = math.sqrt(2)/dMean
    T = np.array([[scale, 0 , -meanX*scale],[0, scale, -meanY*scale],[0, 0, 1]])
    normalizedPoints = np.matmul(T, points.T)
    return normalizedPoints.T, T  #returns points as nx3 

# Function implementing Normalized F Matrix Calculation Method     
def normalizeFMethod(points1, points2):
    points1Normalize, T1 = normalizeData(points1)
    points2Normalize, T2 = normalizeData(points2)
    in1, in2, in3, in4, in5, in6, in7, in8, in9 = points2Normalize[:,0]*points1Normalize[:,0],points2Normalize[:,0]*points1Normalize[:,1],points2Normalize[:,0],points2Normalize[:,1]*points1Normalize[:,0],points2Normalize[:,1]*points1Normalize[:,1],points2Normalize[:,1],points1Normalize[:,0], points1Normalize[:,1], np.ones(len(points1Normalize[:,1]))
    a = np.vstack((in1, in2, in3, in4, in5, in6, in7, in8, in9))
    A = a.T
    u, s, V = np.linalg.svd(A)
    Fmatrix = np.reshape(V[8,:], (3, 3))
    u1,s1,v1 = np.linalg.svd(Fmatrix)
    avg = (s1[1]+s1[0])/2
    s1[0], s1[1] = avg, avg
    s1[2] = 0
    fmat = np.matmul(u1,np.matmul(np.diag(s1),v1))
    F = np.matmul(T2.T, np.matmul(fmat, T1))
    Fnormal = F/findL1Norm(F)
    Fnormal = Fnormal/Fnormal[2][2]
    if Fnormal[2,2] < 0:
        Fnormal = -1 * Fnormal
    return Fnormal

# Finding FMatrix using RANSAC
def findRANSAC(points1, points2, imageW, imageH):
    gridPoints1N = zhangsMethod(imageW, imageH, points1, points2)
    it = 0
    numb = 0
    while(it < 250):

        blockNumber = []
        i = 0
        if(len(gridPoints1N) <=8):
            while(i < 8):
                b = random.randint(0,len(gridPoints1N)-1)
                blockNumber.append(b)
                i += 1
        else:
            while(i < 8):
                b = random.randint(0,len(gridPoints1N)-1)
                if not b in blockNumber:
                    blockNumber.append(b)
                else:
                    i = i - 1 
                i += 1
        pnts1 = []
        pnts2 = []
        for i in blockNumber:
            itr = random.randint(0, len(gridPoints1N[i])-1)
            pnts1.append(list(gridPoints1N[i][itr,:]))
            pos = 0
            for p in range(0 , points1.shape[0]):
                if(points1[p][0] == gridPoints1N[i][itr,0] and points1[p][1] == gridPoints1N[i][itr,1]):
                    pos = p
            pnts2.append(list(points2[pos]))
        pnts1 = np.array(pnts1)
        pnts2 = np.array(pnts2)
        F = normalizeFMethod(pnts1, pnts2)
        checkInliner = np.matmul(points2 , np.matmul(F, points1.T))
        diagonalOfInliners = checkInliner.diagonal()
        inliers = np.where(abs(diagonalOfInliners) <= 0.05)[0]
        numberOfInliners = len(inliers)
        if(numberOfInliners > numb):
            numb = numberOfInliners
            Ffinal = F
            inliersPoints1 = points1[inliers]
            inliersPoints2 = points2[inliers]
        it += 1    
    Fn = normalizeFMethod(inliersPoints1, inliersPoints2)
    return Fn 

# Finding essential matrix
def findEssentialMatrix(F, Ml, Mr):
    E = np.matmul(Mr, np.matmul(F, Ml))
    u,s,v = np.linalg.svd(E)
    s = np.eye(3)
    s[2][2] = 0
    E = np.dot(u,np.dot(s,v))
    E = E/findL1Norm(E)
    return E

# Getting Camera Poses
def getCameraPose(E):
    u, s, v = np.linalg.svd(E)
    W = np.array([[0, -1, 0],[1, 0, 0],[0, 0, 1]]) 
    a = u[:,2]
    C1 = np.reshape(a,(3,1))
    R1 = np.matmul(u, np.matmul(W, v))    
    a = -u[:,2]
    C2 = np.reshape(a,(3,1))
    R2 = np.matmul(u, np.matmul(W, v))
    a = u[:,2]
    C3 = np.reshape(a,(3,1))
    R3 = np.matmul(u, np.matmul(W.T, v))
    a = -u[:,2]
    C4 = np.reshape(a,(3,1))
    R4 = np.matmul(u, np.matmul(W.T, v))
    if np.linalg.det(R1) < 0:
        R1 = -1 * R1
        C1 = -1 * C1
    if np.linalg.det(R2) < 0:
        R2 = -1 * R2
        C2 = -1 * C2
    if np.linalg.det(R3) < 0:
        R3 = -1 * R3
        C3 = -1 * C3
    if np.linalg.det(R4) < 0:
        R4 = -1 * R4
        C4 = -1 * C4
    C1 = np.dot(-R1.T,C1)
    C2 = np.dot(-R2.T,C2)
    C3 = np.dot(-R3.T,C3)
    C4 = np.dot(-R4.T,C4)
    return C1, R1, C2, R2, C3, R3, C4, R4

# Function to disambiguate the R and Cs
def  disambiguateChoices(C1, R1, C2, R2, C3, R3, C4, R4):
    cSet = np.hstack((C1,C2,C3,C4)) # 3X4
    rSet = np.dstack((R1,R2,R3,R4))
    ind = []
    for i in range(0,4):
        if cSet[2][i] > 0 :
            ind.append(i)
    
    ind_2 = []
    for i in ind:
        R_test = rSet[:,:,i]
        if(R_test[1][1] > 0.9 and abs(R_test[0][1]) < 0.1 and abs(R_test[1][0]) < 0.1 and abs(R_test[1][2]) < 0.1 and abs(R_test[2][1]) < 0.1):
            ind_2.append(i)
    tF = []
    RF = []
    if len(ind_2) > 0:
        t_min = 1000
        for i in ind_2:
            R = rSet[:,:,i]
            t = cSet[:,i]
            tN = np.array([[t[0]],[0],[t[2]]])
            if(abs(t[1]) < t_min):
                t_min = abs(t[2])
                tF = tN
                RF = R
        RF[0,1] = 0 
        RF[1,0] = 0
        RF[2,1] = 0
        RF[1,2] = 0

        if(abs(RF[0,2]) < 0.001):
            RF[0,2] = 0
        if(abs(RF[2,0]) < 0.001):
            RF[2,0] = 0
        if(RF[0,0] > 0.99):
            tF[0] = 0
    else:
        RF = np.identity(3)
        tF = np.zeros((3,1))

    return RF, tF 


# Reading images
images = glob.glob('undistorted\Oxford_dataset\stereo\centre\*.png')
# Extract the camera parameters usingReadCameraModel.py
fx, fy, cx, cy, G_camera_image, LUT = ReadCameraModel('Oxford_dataset/model')
# Essential Matrix 
calibrationMatrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])

T_t  = np.array([0,0,0])
T_t = T_t.reshape((3,1))
R_t = np.eye(3)
Tcv  = np.array([0,0,0])
Tcv = Tcv.reshape((3,1))
Rcv = np.eye(3)
f1 = plt.figure()
ax1 = f1.add_subplot(111)
count = 0
for i in range(30,len(images)-1):#len(images) 
    img1 = cv2.imread(images[i],0)
    img2 = cv2.imread(images[i+1],0)
    h,w = img1.shape
    # Initiate ORB detector
    orb = cv2.ORB_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)
    
    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors.
    matches = bf.match(des1, des2)
    
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)
    list1 =[]
    list2 =[]
    for i in range(0,len(matches)):
        list1.append(kp1[matches[i].queryIdx].pt)
        list2.append(kp2[matches[i].trainIdx].pt)

    # List to array conversion
    list1,list2 = np.array(list1), np.array(list2)    
    onesMatrix = np.ones(len(list1[:,0]))
    point1 = np.vstack((list1[:,0],list1[:,1],onesMatrix))
    point2 = np.vstack((list2[:,0],list2[:,1],onesMatrix))
    #Custom Function
    fmatrix = findRANSAC(point1.T, point2.T, w, h)
    E1 = findEssentialMatrix(fmatrix, calibrationMatrix, calibrationMatrix.T)
    C1, R1, C2, R2, C3, R3, C4, R4 = getCameraPose(E1)
    R,t = disambiguateChoices(C1, R1, C2, R2, C3, R3, C4, R4)
    T_t = T_t + np.dot(R_t , t)
    R_t = np.dot(R_t,R)
    #Opencv
    E,mask1 = cv2.findEssentialMat(list1,list2,calibrationMatrix, cv2.RANSAC,0.999,1.0)
    points1, newRcv, tcv, mask = cv2.recoverPose(E, list1,list2,calibrationMatrix,mask=mask1)
    Tcv = Tcv + np.dot(-1*Rcv,np.dot(newRcv.T,tcv))
    Rcv = np.dot(Rcv,newRcv.T)
    plt.ion()
    ax1.scatter(-T_t[0][0],T_t[2][0],s=10, c= 'b', label="Custom VO")
    ax1.scatter(Tcv[0][0],Tcv[2][0],s=10,c = 'r', label="OpenCV VO")
    ax1.set_title('Visual Odometry Output')
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
    plt.savefig("withouttriangulation/"+str(count)+".png")
    print (count)
    count = count+1