from controller import Robot
try:
    import cv2
    import mediapipe as mp
    import math
    import time
 
except ImportError:
    print("Warning: 'cv2' or 'mediapipe' or 'time' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")




#  The video source  (0) and (n) is for, to using the builtin camera of laptop and to using the nth external camera source.


#cap=cv2.VideoCapture("C:/Users/lenovo/Videos/Captures/D__Webots_Project1_RobotArm.wbt (No Project) - Webots R2021b 2022-02-27 15-25-36.mp4")

class poseDetector():

    def __init__(self, mode = False, upBody = False, smooth = True, detectionCon = 0.5, trackCon = 0.5):
        self.mode = mode
        self.upBody = upBody
        self.smooth = smooth
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(self.mode,1,self.upBody,self.smooth,self.detectionCon,self.trackCon)



    def findPose(self,img,draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)

        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img,self.results.pose_landmarks,self.mpPose.POSE_CONNECTIONS)

        return img


    def findPosition(self, img, draw=True):
        lmList=[]

        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c, = img.shape
                cx, cy = int(lm.x*w), int(lm.y*h)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx,cy), 5, (255, 0, 0), cv2.FILLED)

        return lmList





def elbow_ang(points):
    elbow = points[1][1], points[1][2]
    shoulder = points[0][1], points[0][2]
    wrist = points[2][1], points[2][2]
    AB = (int(shoulder[0]) - int(elbow[0])), (int(shoulder[1]) - int(elbow[1]))
    AC = (int(wrist[0]) - int(elbow[0])), (int(wrist[1]) - int(elbow[1]))
    ab = ((AB[0] ** 2) + (AB[1] ** 2)) ** 0.5
    ac = ((AC[0] ** 2) + (AC[1] ** 2)) ** 0.5
    theta_elbow = 180 / math.pi * (math.acos((AB[0] * AC[0] + AB[1] * AC[1]) / (ab * ac)))
    return theta_elbow




def shoulder_ang(points):
    elbow = points[0][1], points[0][2]
    shoulder = points[1][1], points[1][2]
    hip = points[2][1], points[2][2]

    hyp = (int(elbow[0]) - int(shoulder[0])), (int(elbow[1]) - int(shoulder[1]))
    base = (int(hip[0]) - int(shoulder[0])), (int(hip[1]) - int(shoulder[1]))

    ab = ((hyp[0] ** 2) + (hyp[1] ** 2)) ** 0.5
    ac = ((base[0] ** 2) + (base[1] ** 2)) ** 0.5

    theta_shoulder = 180 / math.pi * (math.acos((hyp[0] * base[0] + hyp[1] * base[1]) / (ab * ac)))
    return theta_shoulder





def right_shoulder_2_chest_ang(points):
    wrist = points[0][1], points[0][2]
    rightshoulder = points[1][1], points[1][2]
    leftshoulder = points[2][1], points[2][2]

    hyp = (int(rightshoulder[0]) - int(wrist[0])), (int(rightshoulder[1]) - int(wrist[1]))
    base = (int(rightshoulder[0]) - int(leftshoulder[0])), (int(rightshoulder[1]) - int(leftshoulder[1]))

    ab = ((hyp[0] ** 2) + (hyp[1] ** 2)) ** 0.5
    ac = ((base[0] ** 2) + (base[1] ** 2)) ** 0.5

    theta_right_shoulder_2_chest = 180 / math.pi * (math.acos((hyp[0] * base[0] + hyp[1] * base[1]) / (ab * ac)))
    return theta_right_shoulder_2_chest






def rotate(m,angle):
    m.setVelocity(angle)
    
    




def main():
    cap = cv2.VideoCapture(0)
    pTime = 0
    detector=poseDetector()
  
    robot=Robot()
    timestep=64
    m=robot.getDevice("elbow_joint")
    initPos=0#-2.67
    m.setPosition(initPos)
    m.setVelocity(0.0)
    pSensor=robot.getDevice("elbow_joint_sensor")
    pSensor.enable(timestep)
    l4=robot.getDevice("wrist_joint")
    
    speed=0
    k=0  #to get position sensor reading
    shoulder_lift_joint=robot.getDevice("shoulder_lift_joint")
    shoulder_lift_joint.setPosition(float('inf'))
    shoulder_lift_joint.setPosition(0) #-1.5708=90deg
    
    shoulder_pan_joint=robot.getDevice("shoulder_pan_joint")
    shoulder_pan_joint.setPosition(float('inf'))
    shoulder_pan_joint.setPosition(0)
    

    while robot.step(timestep)!=-1:
        success, img = cap.read()
        img=cv2.flip(img,1)
        img=detector.findPose(img,draw=False)
       # rotate(m,1)
        
        # m.setVelocity(speed)
        # k=pSensor.getValue()
        # if (k>1.57):
            # if (speed==1):
                # speed=-1
        # if (k<0):
            # if (speed==-1):
                # speed=1
                
                
                
        lmList=detector.findPosition(img, False)
        
        
        if len(lmList) != 0:
            print(lmList[12], (lmList[14]), (lmList[16]),lmList[11], (lmList[13]), (lmList[15]))

            """For Left Hand"""
            # cv2.circle(img, (lmList[12][1], lmList[12][2]), 15, (255, 0, 0), cv2.FILLED)
            # cv2.circle(img, (lmList[14][1], lmList[14][2]), 15, (255, 0, 0), cv2.FILLED)
            # cv2.circle(img, (lmList[16][1], lmList[16][2]), 15, (255, 0, 0), cv2.FILLED)
            # cv2.line(img, (lmList[12][1],lmList[12][2]),(lmList[14][1],lmList[14][2]),(0,255,0),2)
            # cv2.line(img, (lmList[14][1],lmList[14][2]),(lmList[16][1],lmList[16][2]),(0,255,0),2)
            # points = [lmList[12], lmList[14], lmList[16]]
            # theta_elbow=elbow_ang(points)
            # cv2.putText(img, (str(int(theta_elbow))+"'"), (lmList[14][1], lmList[14][2]+50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 3)
           
            # AngleRadian=((theta_elbow)*(.01745329252))
           
            
            
            # #angle=(AngleRadian)-(pSensor.getValue())
            # print(theta_elbow)
            # print(AngleRadian)
            # #rotate(m,1)
            # m.setPosition(AngleRadian)
            # m.setVelocity(1)
            



            """For Right Hand"""

            """For Right Hand"""
            cv2.circle(img, (lmList[11][1], lmList[11][2]), 15, (255, 0, 0), cv2.FILLED)
            cv2.circle(img, (lmList[13][1], lmList[13][2]), 15, (255, 0, 0), cv2.FILLED)
            cv2.circle(img, (lmList[15][1], lmList[15][2]), 15, (255, 0, 0), cv2.FILLED)
            cv2.line(img, (lmList[11][1], lmList[11][2]), (lmList[13][1], lmList[13][2]), (0, 255, 0), 2)
            cv2.line(img, (lmList[13][1], lmList[13][2]), (lmList[15][1], lmList[15][2]), (0, 255, 0), 2)

            theta_elbow=elbow_ang([lmList[11],lmList[13],lmList[15]])
            theta_shoulder=shoulder_ang([lmList[13],lmList[11],lmList[23]])
            #theta_right_shoulder_2_chest=right_shoulder_2_chest_ang([lmList[15],lmList[11],lmList[12]])

            cv2.putText(img, (str(int(theta_elbow))+"'"), (lmList[13][1], lmList[13][2]+50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 3)
            cv2.putText(img, (str(int(theta_shoulder))+"'"), (lmList[11][1], lmList[11][2]+40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 3)
            #cv2.putText(img, (str(int(theta_right_shoulder_2_chest))+"'"), (lmList[11][1]-120, lmList[11][2]), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 3)
            
            AngleRadianElbow=((theta_elbow)*(.01745329252))
            AngleRadianShoulder=(theta_shoulder)*(.01745329252)
            #AngleRadian_right_shoulder_2_chest=(theta_right_shoulder_2_chest)*(.01745329252)
            
            
            
            #angle=(AngleRadian)-(pSensor.getValue())
            print(theta_elbow)
            print(AngleRadianElbow)
            
            print(theta_shoulder)
            print(AngleRadianShoulder)
            
            # print(theta_right_shoulder_2_chest)
            # print(AngleRadian_right_shoulder_2_chest)
            
            
            #rotate(m,1)
            # nAngleRad= -(3.14+AngleRadian)
            m.setPosition(-6.38+(3.14+AngleRadianElbow))
            m.setVelocity(1)
            
            shoulder_lift_joint.setPosition((-AngleRadianShoulder))
            shoulder_lift_joint.setVelocity(1)
            
            # shoulder_pan_joint.setPosition((AngleRadianShoulder))
            # shoulder_pan_joint.setVelocity(1)
            
            # m1.setPosition(-6.38+(3.14+AngleRadian))
            # m1.setVelocity(1)
            
         
        
        
        
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        cv2.putText(img, str(int(fps)), (70, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
        cv2.imshow("Image", img)
        
        check = cv2.waitKey(30) & 0xff
        if check == 27:
            break



if __name__=="__main__":
    main()
