#------Yellow ball tracking with NAO----------------
#------WORKING!!!!-------------------------
#------19/10/2016--------------------------
#------Santos Orozco-----------------------
#------This code contains a proportional control for NAO humanoid which tracks a yellow
#------ball using visual feedback
import cv2
import cv2.cv as cv
import numpy as np
from naoqi import ALProxy
from vision_definitions import *
import time
import math

#ipnao="127.0.0.1"
ipnao="169.254.172.97"
puerto=9559
tts=ALProxy('ALTextToSpeech',ipnao,puerto)
camProxy = ALProxy('ALVideoDevice',ipnao,puerto)
motionProxy = ALProxy('ALMotion',ipnao,puerto)
memoryProxy = ALProxy('ALMemory',ipnao,puerto)
postureProxy = ALProxy('ALRobotPosture',ipnao,puerto)

nameId = "python_client"
  
width,height = 320,240
actuators = ['HeadYaw','HeadPitch']
maxSpeed=0.05
dyaw=0.1
def nothing(x):
    pass

def main():
    count=0
    connectCamera(0)
    fractionMaxSpeed = 0.1
    motionProxy.wakeUp()
    while(1):
        rame = getImage(0)
        frame = cv2.cvtColor(rame,cv2.COLOR_BGR2HSV)
        #yellow ball segmentation
        lower = np.array([44,126,155])
        upper = np.array([96,245,232])
        seg = cv2.inRange(frame, lower, upper)
        #filtering
        blurred = cv2.medianBlur(seg,7)
        kernel = np.ones((5,5),np.uint8)
        kernel1 = np.ones((1,1),np.uint8)
        dilation = cv2.dilate(blurred,kernel,iterations = 10)
        #contours of the yellow object
        contours, hierarchy = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (0,255,0), 2)
        #image center
        u,v = seg.shape
        #moment of the yellow object
        M = cv2.moments(contours[0])
        #yellow object centroid
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (v/2, u/2), 7, (0, 255, 0), -1)
        cv2.circle(frame, (cX, cY), 7, (255, 0,0), -1)
        #error signals
        eX = v/2-cX
        eY = u/2-cY
        pan = memoryProxy.getData('Device/SubDeviceList/HeadYaw/Position/Sensor/Value')
        tilt = memoryProxy.getData('Device/SubDeviceList/HeadPitch/Position/Sensor/Value')
        if(eX > 10):
                changesx = 0.0005*abs(eX)
        elif(eX < -10):
                changesx = -0.0005*abs(eX)
        else:
                changesx = 0
        #print changesx
        motionProxy.changeAngles("HeadYaw", changesx, fractionMaxSpeed)
        if(eY > 10):
                changesy = -0.0003*abs(eY)
        elif(eY < -10):
                changesy = 0.0003*abs(eY)
        else:
                changesy = 0
        #print changesy
        motionProxy.changeAngles("HeadPitch", changesy, fractionMaxSpeed)        
		  # controller
        if abs(eY) < 10:
                if abs(eX) < 10:
                    pan = memoryProxy.getData('Device/SubDeviceList/HeadYaw/Position/Sensor/Value')
                    tilt = memoryProxy.getData('Device/SubDeviceList/HeadPitch/Position/Sensor/Value')
                    h = 38;
                    theta_deg = 90-math.degrees(tilt)
                    phi_deg = math.degrees(tilt)
                    theta = math.radians(theta_deg)
                    phi = math.radians(phi_deg)
                    disx = h/(1-math.tan(phi))+170
                    cv2.putText(frame,'distance {} mm'.format(disx), (cX - 20, cY - 20),
						  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)            

        cv2.imshow('Capture Images',frame)
        cv2.imshow('Filtered Image',dilation)
        ch = 0xFF & cv2.waitKey(1)

        #print("tecla=%c",ch)
        if ch == 27:
            break
        if ch == 112:
            count+=1
            nombre='trackyellow'+ str(count)+'.jpg'
            cv2.imwrite(nombre, frame)
            print 'Imagen Guardada {}'.format(count)
        if count == 5:
            break
    cv2.destroyAllWindows()
    disconnectCamera()
  
# camera selection
def connectCamera(nCamera):
    resolution = kVGA
    spaceColor = kBGRColorSpace
    fps = 30
    try:
        camProxy.subscribeCamera(nameId,nCamera,resolution,spaceColor,fps)
        camProxy.setParam(18,nCamera)
        time.sleep(1.0)
    except BaseException, err:
        print ("ERR: connectCamera: catching error: %s!" % err)

 # disconnect camera       
def disconnectCamera():
    try:
        camProxy.unsubscribe(nameId) 
    except BaseException, err:
        print ("ERR: disconnectCamera: catching error: %s!" % err)

# caputurin image        
def getImage(nCamera):
    try:
        i = camProxy.getImageRemote(nameId)
        if i != None:
            image = (np.reshape(np.frombuffer(i[6],dtype='%iuint8' % i[2]),(i[1],i[0],i[2])))
            elim_dist(image,nCamera)
            return image
    except BaseException, err:
        print ("ERR: getImage: catching error: %s!" % err)
    return None;

# eliminating distortion with camera parameters
def elim_dist(img, cam):
    #cam==0 cam_up, 
    if cam==0:
        fx= 770.62891039
        fy=767.76331171
        cx = 348.51844174
        cy=264.31492807
        kc = [0.33982982, -1.2359181,0.00400206,0.01930419, 1.2398553]
        kc=np.array(kc)
        cam_mat=[fx, 0, cx ],[0,fy,cy],[0, 0,1]
        cam_mat=np.array(cam_mat)
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cam_mat,kc,(w,h),1,(w,h))
        dst = cv2.undistort(img, cam_mat, kc, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
    elif cam==1:
        kc=[4.54670020e-01 , -2.35928947, -2.82117633e-03, 4.25070591e-03, 4.75239956]#OpenCV
        kc=np.array(kc)
        cam_mat=[795.80585538,0,326.03217012],[0, 796.41452639, 230.2267145],[0,0,1]#OpenCV
        cam_mat=np.array(cam_mat)
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cam_mat,kc,(w,h),1,(w,h))
        dst = cv2.undistort(img, cam_mat, kc, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
    else:
        print "Error: no camera"
    return dst

if __name__=="__main__":
    main()
