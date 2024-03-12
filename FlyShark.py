import cv2
import RPi.GPIO as GPIO
from time import sleep
import time
import math
from cvzone.HandTrackingModule import HandDetector
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import imutils
import pickle

detector=HandDetector(staticMode=False, maxHands=2, modelComplexity=0, detectionCon=0.7, minTrackCon=0.5)

# Pins for Motor Driver Inputs  (Fin)
DC_Motor_Pin1_A = 11                 # Define PIN for DC Motor
DC_Motor_Pin2_A = 13                 # Define PIN for DC Motor
PWM_Pin_A = 15

# Pins for Motor Driver Inputs (Ballast)
DC_Motor_Pin1_B = 29                 # Define PIN for DC Motor
DC_Motor_Pin2_B = 31                 # Define PIN for DC Motor
PWM_Pin_B = 33

def check_face(cam):
    #Initialize 'currentname' to trigger only when a new person is identified.
    currentname = "unknown"
    #Determine faces from encodings.pickle file model created from train_model.py
    encodingsP = "encodings.pickle"

    # load the known faces and embeddings along with OpenCV's Haar
    # cascade for face detection
    print("[INFO] loading encodings + face detector...")
    data = pickle.loads(open(encodingsP, "rb").read())

    # initialize the video stream and allow the camera sensor to warm up
    cap = cam
    start_time = time.time()

    # start the FPS counter
    fps = FPS().start()

    # loop over frames from the video file stream
    while True:
        # grab the frame from the threaded video stream and resize it
        ret, frame = cam.read()
        scale_percent = 60 # percent of original size
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        frame = cv2.resize(frame, dim)
        
        # Detect the fce boxes
        boxes = face_recognition.face_locations(frame)
        # compute the facial embeddings for each face bounding box
        encodings = face_recognition.face_encodings(frame, boxes)
        names = []

        # loop over the facial embeddings
        for encoding in encodings:
            # attempt to match each face in the input image to our known
            # encodings
            matches = face_recognition.compare_faces(data["encodings"],
                encoding)
            name = "Unknown" #if face is not recognized, then print Unknown

            # check to see if we have found a match
            if True in matches:
                # find the indexes of all matched faces then initialize a
                # dictionary to count the total number of times each face
                # was matched
                matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                counts = {}

                # loop over the matched indexes and maintain a count for
                # each recognized face face
                for i in matchedIdxs:
                    name = data["names"][i]
                    counts[name] = counts.get(name, 0) + 1

                # determine the recognized face with the largest number
                # of votes (note: in the event of an unlikely tie Python
                # will select first entry in the dictionary)
                name = max(counts, key=counts.get)

                #If someone in your dataset is identified, print their name on the screen
                if currentname != name:
                    currentname = name
                    print(currentname)

            # update the list of names
            names.append(name)

        # loop over the recognized faces
        for ((top, right, bottom, left), name) in zip(boxes, names):
            # draw the predicted face name on the image - color is in BGR
            cv2.rectangle(frame, (left, top), (right, bottom),
                (0, 255, 225), 2)
            y = top - 15 if top - 15 > 15 else top + 15
            cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                .8, (0, 255, 255), 2)

        # display the image to our screen
        cv2.imshow("Facial Recognition is Running", frame)
        key = cv2.waitKey(1) & 0xFF

        # quit when 'q' key is pressed
        if key == ord("q"):
            break
        
        # quit after 5s
        if (time.time() - start_time > 5):
            break

        # update the FPS counter
        fps.update()

    # stop the timer and display FPS information
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    
    if (len(names) < 1):
        names.append("Unkown")

    return names

def setup():
    #Setup GPIO pins on Pi
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)    # Consider complete raspberry-pi board
    GPIO.setwarnings(False)     # To avoid same PIN use warning
    GPIO.setup(DC_Motor_Pin1_A,GPIO.OUT)   # Set pin function as output
    GPIO.setup(DC_Motor_Pin2_A,GPIO.OUT)   # Set pin function as output
    GPIO.setup(PWM_Pin_A, GPIO.OUT) 
    GPIO.setup(DC_Motor_Pin1_B,GPIO.OUT)   # Set pin function as output
    GPIO.setup(DC_Motor_Pin2_B,GPIO.OUT)   # Set pin function as output
    GPIO.setup(PWM_Pin_B, GPIO.OUT) 
    
def idle():
    # Idle the shark forwards slowly
    forwards(10, 3, .5, 10)

def forwards(pwm, delay, rev_time, loop_count):
    # Move shark forward
    print("Goin' forwards")
    
    i = 0
    
    while (i<loop_count):
        # Going forwards
        GPIO.output(DC_Motor_Pin1_A,GPIO.LOW)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.HIGH)   #Send low signal on motor pin 2
        sleep(rev_time*0.75)
        fstop()
        
        GPIO.output(DC_Motor_Pin1_A,GPIO.HIGH)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.LOW)   #Send low signal on motor pin 2
        sleep(rev_time*1)
        fstop()
        
        i += 1
 

def turn_right(pwm, delay, rev_time, loop_count):
    # Move shark right
    print("Turnin' Right")
    
    i = 0
    
    while (i<loop_count):
        GPIO.output(DC_Motor_Pin1_A,GPIO.LOW)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.HIGH)   #Send low signal on motor pin 2
        sleep(rev_time*0.75)
        fstop()
        
        GPIO.output(DC_Motor_Pin1_A,GPIO.HIGH)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.LOW)   #Send low signal on motor pin 2
        sleep(rev_time*1)
        fstop()
        
        GPIO.output(DC_Motor_Pin1_A,GPIO.LOW)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.HIGH)   #Send low signal on motor pin 2
        sleep(rev_time*0.75)
        fstop()
        
        GPIO.output(DC_Motor_Pin1_A,GPIO.HIGH)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.LOW)   #Send low signal on motor pin 2
        sleep(rev_time*1.25)
        fstop()
        sleep(2)
        
        i += 1
    
def turn_left(pwm, delay, rev_time, loop_count):
    # Move shark left
    print("Turnin' Left")
    
    i = 0
    
    while (i<loop_count):
        GPIO.output(DC_Motor_Pin1_A,GPIO.HIGH)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.LOW)   #Send low signal on motor pin 2
        sleep(rev_time*1)
        fstop()
        
        GPIO.output(DC_Motor_Pin1_A,GPIO.LOW)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.HIGH)   #Send low signal on motor pin 2
        sleep(rev_time*0.75)
        fstop()
        
        GPIO.output(DC_Motor_Pin1_A,GPIO.HIGH)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.LOW)   #Send low signal on motor pin 2
        sleep(rev_time*1)
        fstop()
        
        GPIO.output(DC_Motor_Pin1_A,GPIO.LOW)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2_A,GPIO.HIGH)   #Send low signal on motor pin 2
        sleep(rev_time*1.25)
        fstop()
        sleep(2)
        
        i += 1
        
def go_down(pwm, delay, rev_time, loop_count):
    # Move ballast forward (shark down)
    print("Goin' Down")
    
    # Move ballast down
    GPIO.output(DC_Motor_Pin1_B,GPIO.HIGH)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2_B,GPIO.LOW)   #Send low signal on motor pin 2
    sleep(rev_time*1.5)
    fstop()
    
    forwards(pwm, delay, rev_time, loop_count)
    
    # Move ballast back
    GPIO.output(DC_Motor_Pin1_B,GPIO.LOW)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2_B,GPIO.HIGH)   #Send low signal on motor pin 2
    sleep(rev_time*1.5)
    fstop()
    
def go_up(pwm, delay, rev_time, loop_count):
    # Move ballast back (shark up)
    print("Goin' Up")
    
    # Move ballast up
    GPIO.output(DC_Motor_Pin1_B,GPIO.LOW)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2_B,GPIO.HIGH)   #Send low signal on motor pin 2
    sleep(rev_time*1.5)
    fstop()
    
    forwards(pwm, delay, rev_time, loop_count)
    
    # Move ballast back
    GPIO.output(DC_Motor_Pin1_B,GPIO.HIGH)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2_B,GPIO.LOW)   #Send low signal on motor pin 2
    sleep(rev_time*1.5)
    fstop()
 
def fstop():
    # stop the shark's motors
    GPIO.output(DC_Motor_Pin1_A,GPIO.LOW)  #Motor off
    GPIO.output(DC_Motor_Pin2_A,GPIO.LOW)  #Motor off
    
    GPIO.output(DC_Motor_Pin1_B,GPIO.LOW)  #Motor off
    GPIO.output(DC_Motor_Pin2_B,GPIO.LOW)  #Motor off
    
    print("Stop")
    
def destroy():  
    GPIO.cleanup()
    
def loop(cap):
    start_time = time.time()
    
    # Assign value of current user of Autoshark
    current_user = check_face(cap)
    
    #Friendly = True # For testing without facial recognition
    
    if (current_user[0] == "Carson"):
        Friendly = True
    else:
        Friendly = False
        
    while Friendly:
        
        # Loop through facial recognition at set interval
        duration = time.time() - start_time
        if (abs(duration % 10 - 0) < 0.2):
            current_user = check_face(cap)
            Friendly = False
            
        if (current_user[0] == "Carson"):
            Friendly = True
            
        # Fin Parameters
        loop_count = 10
        delay = .1
        rev_time = 0.9
        
        # Ballast Parameters
        loop_count_b = 1
        delay_b = .1
        rev_time_b = 0.625
        
        ret,frame=cap.read()
        #frame=cv2.flip(frame,1)
        hands,frame=detector.findHands(frame, flipType=True)
        
        # Capture video 
        #result.write(frame) # for recording video during motion
        
        duration = time.time() - start_time
        if (abs(duration % 2 - 0) < 0.2):
            if hands:
                
                # Initialize counts
                count2 = 0
                count = 0
                
                hands1=hands[0]
                
                # Assign first count to right hand
                if hands1["type"] == "Right":
                    fingers1=detector.fingersUp(hands1)
                    count=fingers1.count(1)
                    count2 = 0
                else:
                    fingers1=detector.fingersUp(hands1)
                    count2=fingers1.count(1)
                
                if len(hands) == 2:
                    # Alternative additional hand for up/down control
                    hands2=hands[1]
                    if hands2["type"] == "Left":
                        fingers2=detector.fingersUp(hands2)
                        count2=fingers1.count(1)
                    else:
                        fingers1=detector.fingersUp(hands1)
                        count=fingers1.count(1)
                    
                print("Left count", count2)
                print("Right count", count)
                
                if count == 1:
                    forwards(pwm_a, delay, rev_time, loop_count)
                    #sleep(wait)
                elif count==2:
                    turn_right(pwm_a, delay, rev_time, loop_count)
                    #sleep(wait)
                elif count==3:
                    turn_left(pwm_a, delay, rev_time, loop_count)
                    #sleep(wait)
                elif count2 == 1:
                    go_down(pwm_b, delay_b, rev_time_b, loop_count_b)
                elif count2 == 2:
                    go_up(pwm_b, delay_b, rev_time_b, loop_count_b)
                elif count==5:
                    fstop()
                    sleep(1)
                else:
                    fstop()

            else:
                print("nohands")
                setup()
                fstop()
                
        #frame=cv2.imshow("FRAME",frame)
    
        if cv2.waitKey(1)&0xFF==27:
            break
        
    if (~Friendly):
        #Idle shark if user not approved
        idle()
    
if __name__ == '__main__':     # Program start from here
    
    cap=cv2.VideoCapture(0)
    setup()

    #Fin Motor Params
    pwm_a=GPIO.PWM(PWM_Pin_A, 100)
    pwm_a.start(0)
    pwm_a.ChangeDutyCycle(20)
    
    #Ballast Motor Params
    pwm_b=GPIO.PWM(PWM_Pin_B, 100)
    pwm_b.start(0)
    pwm_b.ChangeDutyCycle(8) 
    
    # Filename 
    filename = 'SharkPOV.jpg'
    
    sleep(5)
    ret,frame=cap.read()
  
    # Capture what shark sees upon startup
    cv2.imwrite(filename, frame) 
    
    # Initialize video 
    frame_width = int(cap.get(3)) 
    frame_height = int(cap.get(4)) 
    
    size = (frame_width, frame_height) 
    
    # Below VideoWriter object will create 
    # a frame of above defined The output  
    # is stored in 'filename.avi' file. 
    result = cv2.VideoWriter('POV.avi',  
                            cv2.VideoWriter_fourcc(*'MJPG'), 
                            10, size) 
    
    # Set Wait duration
    wait = 5
    try:
        loop(cap)
    except KeyboardInterrupt:
        destroy()
cap.release()
cv2.destroyAllWindows()




