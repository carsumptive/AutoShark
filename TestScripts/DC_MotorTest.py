import RPi.GPIO as GPIO     # Import Library to access GPIO PIN
import time                 # To access delay function
GPIO.setmode(GPIO.BOARD)    # Consider complete raspberry-pi board
GPIO.setwarnings(False)     # To avoid same PIN use warning
DC_Motor_Pin1 = 11                 # Define PIN for DC Motor
DC_Motor_Pin2 = 13                 # Define PIN for DC Motor
PWM_Pin = 15
GPIO.setup(DC_Motor_Pin1,GPIO.OUT)   # Set pin function as output
GPIO.setup(DC_Motor_Pin2,GPIO.OUT)   # Set pin function as output
GPIO.setup(PWM_Pin, GPIO.OUT)

pwm=GPIO.PWM(PWM_Pin, 100)
pwm.start(0)

i = 0

while (i<3):
    print("Motor clockWise 10%")
    pwm.ChangeDutyCycle(30)  
    GPIO.output(DC_Motor_Pin1,GPIO.HIGH)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2,GPIO.LOW)   #Send low signal on motor pin 2
    time.sleep(10)                  # Give 5 second delay
    print("Motor AnticlockWise")   
    pwm.ChangeDutyCycle(20)  
    GPIO.output(DC_Motor_Pin1,GPIO.LOW)  #Send low signal on motor pin 1
    GPIO.output(DC_Motor_Pin2,GPIO.HIGH)  #Send high signal on motor pin2
    time.sleep(10)                   # Give 5 second delay
    i += 1
    
GPIO.output(DC_Motor_Pin1,GPIO.LOW)  #Motor off
GPIO.output(DC_Motor_Pin2,GPIO.LOW)  #Motor off