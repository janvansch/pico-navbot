# Import Modules:
from machine import Pin, PWM, Timer
import utime

#analogvalue = machine.ADC(28)

distance = 0

# Define timer object
timer = Timer()

# Setup PWM for servo
pwm = PWM(Pin(0))
pwm.freq(50)

# Define onboard LED object
led = Pin(25, Pin.OUT)

# Set IR sensor pins
front_centre = Pin(2, Pin.IN)
left_front = Pin(4, Pin.IN)
right_front = Pin(5, Pin.IN)

# Set sonic sensor pins 
trigger = Pin(14, Pin.OUT)
echo = Pin(15, Pin.IN) 

# Set pins for motors
motor_1a = Pin (6, Pin.OUT) # L298N IN4 orange
motor_1b = Pin (7, Pin.OUT) # L298N IN3 yellow
motor_2a = Pin (8, Pin.OUT) # L298N IN2 green
motor_2b = Pin (9, Pin.OUT) # L298N IN1 blue
# Remember to connect Pico GND to L298N GND

# Define drive control functions
def stop():
    print("*** Full Stop")
    motor_1a.low()
    motor_1b.low()
    motor_2a.low()
    motor_2b.low()
    
def forward():
    print("*** Forward: Heading: 000, Distance: 000")
    motor_1a.low()
    motor_1b.high()
    motor_2a.high()
    motor_2b.low()
    
def reverse():
    print("*** Reverse")
    motor_1a.high()
    motor_1b.low()
    motor_2a.low()
    motor_2b.high()

def left():
    print("*** Spin Left - right wheels forward & left wheels reverse")
    motor_1a.low()
    motor_1b.high()
    motor_2a.low()
    motor_2b.high()

def right():
    print("*** Spin Right - left wheels forward & right wheels reverse")
    motor_1a.high()
    motor_1b.low()
    motor_2a.high()
    motor_2b.low()

# Define interrupt handlers
def irq_front_centre(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Center")
        stop()
        setServoCycle()
    else:
        led.off()

def irq_left_front(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Left")
        stop()
        setServoCycle()
    else:
        led.off()

def irq_right_front(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Right")
        stop()
        setServoCycle()
    else:
        led.off()

# Cycle servo
def setServoCycle():
    for position in range (800, 8000, 50):
        pwm.duty_u16(position)
        utime.sleep(0.01)
        
    for position in range (8000, 800, -50):
        pwm.duty_u16(position)
        utime.sleep(0.01)

# Process sonic sensor data 
def get_distance(timer):
    global distance
    #Pause for two milliseconds to ensure the previous setting has completed 
    utime.sleep_us(2) 
    trigger.high()
    # After pulling the high potential,
    # wait for 5 milliseconds,
    # and immediately set to Low
    utime.sleep_us(5)   
    trigger.low()
    
    # Create a while loop to check whether the echo pin is 0
    # and record the time
    while echo.value() == 0:          
        signaloff = utime.ticks_us()
        
    # Create a while loop to check Whether the echo pin value is 1
    # and then record the time    
    while echo.value() == 1:         
        signalon = utime.ticks_us()
        #Calculate the time difference between sending and receiving
    
    duration = signalon - signaloff

    #Sonic travel time x speed of sound
    # (343.2 m/s , Which is 0.0343 cm per microsecond),
    # and the back-and-forth distance is divided by 2 
    distance = (duration * 0.0343) / 2            
    print('The distance is： ', distance, 'cm')
    if distance < 5:
            led.on()
            print('Obstical within 5cm')
            stop()
            utime.sleep(2)
            led.off()

def main():
    # Main loop
    run = True
    test_cycles = 2
    test_time = 5
    count = 0
    while run:
        #reading = analogvalue.read_u16()
        #pwm.duty_u16(int(reading/6))
            
        if distance < 5:
            led.on()
            stop()
            utime.sleep(2)
            led.off()
            while distance < 5:
                pass
                
        else:
            led.off()
            # Forward Test
            forward()
            utime.sleep(test_time)
            stop()
            utime.sleep(1)
            # Reverse Test
            reverse()
            utime.sleep(test_time)
            stop()
            utime.sleep(1)
            # Left Test
            left()
            utime.sleep(test_time)
            stop()
            utime.sleep(1)
            # Right Test
            right()
            utime.sleep(test_time)
            stop()
            utime.sleep(1)
            
            count = count + 1  
            if count > test_cycles:
                run = False
    stop()
    timer.deinit()
    print ('*** Test Completed ***')

# Setup interrupts
front_centre.irq(handler=irq_front_centre, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
left_front.irq(handler=irq_left_front, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
right_front.irq(handler=irq_right_front, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)

# Configure timer
timer.init(freq=1, mode=Timer.PERIODIC, callback=get_distance)

try:
    # execute main loop
    main()
    
except KeyboardInterrupt:
    # Stop bot
    stop()
    print ('CTRL-C received')
    
finally:
    # Cleanup
    timer.deinit()
    print ('*** END ***')