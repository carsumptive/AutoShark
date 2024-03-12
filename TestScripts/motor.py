import RPi.GPIO as GPIO
from time import sleep

# Pins for Motor Driver Inputs 
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
 
def loop():
    pwm=GPIO.PWM(PWM_Pin, 100)
    pwm.start(0)
    pwm.ChangeDutyCycle(5) 
    
    i = 0
    loop_count = 1
    delay = .1
    rev_time = 0.625
    
    while (i<loop_count):
        
        # at duty cycle 1, 1.6 revolutions in a second.
        # should be about .625 seconds for one revolution
        
        # Going backwards
        GPIO.output(DC_Motor_Pin1,GPIO.LOW)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2,GPIO.HIGH)   #Send low signal on motor pin 2
        print("Goin' backwards")
     
        sleep(rev_time*0.75)
        stop(delay)
    
        # Going forwards
        pwm.ChangeDutyCycle(13) 
        GPIO.output(DC_Motor_Pin1,GPIO.HIGH)  #Send high signal on motor pin1
        GPIO.output(DC_Motor_Pin2,GPIO.LOW)   #Send low signal on motor pin 2
        print("Goin' forwards")
     
        sleep(rev_time*1)
        stop(delay)

        
        i += 1

def destroy():  
    GPIO.cleanup()
    
def home():
    pwm=GPIO.PWM(PWM_Pin, 100)
    pwm.start(0)
    pwm.ChangeDutyCycle(13) 
    
    stop(1)
    GPIO.output(DC_Motor_Pin1,GPIO.LOW)  #Send high signal on motor pin1
    GPIO.output(DC_Motor_Pin2,GPIO.HIGH)   #Send low signal on motor pin 2
    print("Goin' Home")
     
    sleep(.625*0.1)
    
def stop(seconds):
    GPIO.output(DC_Motor_Pin1,GPIO.LOW)  #Motor off
    GPIO.output(DC_Motor_Pin2,GPIO.LOW)  #Motor off
    print("Stop")
    sleep(seconds)

if __name__ == '__main__':     # Program start from here
    setup()
    try:
            loop()
            home()
    except KeyboardInterrupt:
        destroy()
