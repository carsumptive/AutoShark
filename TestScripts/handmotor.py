import cv2
import RPi.GPIO as GPIO
from time import sleep
from cvzone.HandTrackingModule import HandDetector
detector=HandDetector(staticMode=False, maxHands=1, modelComplexity=0, detectionCon=0.5, minTrackCon=0.5)

# Pins for Motor Driver Inputs Fin
DC_Motor_Pin1 = 11                 # Define PIN for DC Motor
DC_Motor_Pin2 = 13                 # Define PIN for DC Motor
PWM_Pin = 15

# Pins for Motor Driver Inputs Ballast
DC_Motor_Pin1_B = 29                 # Define PIN for DC Motor
DC_Motor_Pin2_B = 31                 # Define PIN for DC Motor
PWM_Pin_B = 33


def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)    # Consider complete raspberry-pi board
    GPIO.setwarnings(False)     # To avoid same PIN use warning
    GPIO.setup(DC_Motor_Pin1,GPIO.OUT)   # Set pin function as output
    GPIO.setup(DC_Motor_Pin2,GPIO.OUT)   # Set pin function as output
    GPIO.setup(PWM_Pin, GPIO.OUT)
    GPIO.setup(DC_Motor_Pin1_B,GPIO.OUT)   # Set pin function as output
    GPIO.setup(DC_Motor_Pin2_B,GPIO.OUT)   # Set pin function as output
    GPIO.setup(PWM_Pin_B, GPIO.OUT) 

def forwards(pwm, delay, rev_time):
    print("Goin' forwards")
    
    # Going forwards
    GPIO.output(DC_Motor_Pin1,GPIO.LOW)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2,GPIO.HIGH)   #Send low signal on motor pin 2
    
    GPIO.output(DC_Motor_Pin1_B,GPIO.LOW)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2_B,GPIO.HIGH)   #Send low signal on motor pin 2
    sleep(rev_time*0.75)
    fstop()
    
    #pwm.ChangeDutyCycle(13) 
    GPIO.output(DC_Motor_Pin1,GPIO.HIGH)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2,GPIO.LOW)   #Send low signal on motor pin 2
    
    GPIO.output(DC_Motor_Pin1_B,GPIO.HIGH)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2_B,GPIO.LOW)   #Send low signal on motor pin 2
    sleep(rev_time*1)
    fstop()
 

def backwards():
    # Going backwards
    GPIO.output(DC_Motor_Pin1,GPIO.LOW)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2,GPIO.HIGH)   #Send low signal on motor pin 2
    print("Goin' backwards")
 
def fstop():
    # fstop
    GPIO.output(DC_Motor_Pin1,GPIO.LOW)  #Motor off
    GPIO.output(DC_Motor_Pin2,GPIO.LOW)  #Motor off
    
    GPIO.output(DC_Motor_Pin1_B,GPIO.LOW)  #Motor off
    GPIO.output(DC_Motor_Pin2_B,GPIO.LOW)  #Motor off
    print("Stop")
    
def destroy():  
    GPIO.cleanup()    

cap=cv2.VideoCapture(0)
setup()

#Fin Motor Params
pwm=GPIO.PWM(PWM_Pin, 100)
pwm.start(0)
pwm.ChangeDutyCycle(13)

pwm_B=GPIO.PWM(PWM_Pin_B, 100)
pwm_B.start(0)
pwm_B.ChangeDutyCycle(5) 
    
while True:
    
    #Fin Parameters
    i = 0
    loop_count = 2
    delay = .1
    rev_time = 0.625
    
    ret,frame=cap.read()
    #frame=cv2.flip(frame,1)
    hands,frame=detector.findHands(frame, flipType=True)
    if hands:
        hands1=hands[0]
        fingers1=detector.fingersUp(hands1)
        count=fingers1.count(1)
        print(count)
        if count == 1:
             forwards(pwm, delay, rev_time)
        elif count==2:
            print("nada")
            #backwards()
        elif count==3:
             fstop()
        else:
             fstop()
    else:
        print("nohands")
        setup()
        fstop()
            
    #frame=cv2.imshow("FRAME",frame)
   
    if cv2.waitKey(1)&0xFF==27:
        break
cap.relase()
cv2.destroyAllWindows()
