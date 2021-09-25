# Import Modules:
from machine import Pin, PWM, Timer
import utime

#analogvalue = machine.ADC(28)

# ------------------------
#  Initialisation Section
# ------------------------

# Initialise some global variables
distance = 0
left_rear_encoder_count = 0

# Define timer object
timer = Timer()

# Setup PWM object for servo
pwm = PWM(Pin(0))
pwm.freq(50)

# Onboard LED object
led = Pin(25, Pin.OUT)

# IR sensor interface
ir_front_centre = Pin(2, Pin.IN)
ir_left_front = Pin(4, Pin.IN)
ir_right_front = Pin(5, Pin.IN)

# Sonic sensor interface 
trigger = Pin(14, Pin.OUT)
echo = Pin(15, Pin.IN)

# Motor speed encoder interface
#encoder_front_right = Pin(10, Pin.IN)
#encoder_front_left = Pin(11, Pin.IN)
#encoder_rear_right = Pin(12, Pin.IN)
encoder_rear_left = Pin(13, Pin.IN)

# Motor control interface 
motor_1a = Pin(6, Pin.OUT) # L298N IN4 orange
motor_1b = Pin(7, Pin.OUT) # L298N IN3 yellow
motor_2a = Pin(8, Pin.OUT) # L298N IN2 green
motor_2b = Pin(9, Pin.OUT) # L298N IN1 blue
# Remember to connect Pico GND to L298N GND

# ---------------------------------------
#  Interrupt Handlers Definition Section
# ---------------------------------------
#
# Define speed interrupt handlers
#
def left_rear_encoder_counter(pin):
    global left_rear_encoder_count
    left_rear_encoder_count += 1
#
# Define obstical interrupt handlers
#
def obstical_front_centre(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Center")
        stop()
        setServoCycle()
    else:
        led.off()

def obstical_left_front(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Left")
        stop()
        setServoCycle()
    else:
        led.off()

def obstical_right_front(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Right")
        stop()
        setServoCycle()
    else:
        led.off()

# -------------------------
#  Servo Control Utilities
# -------------------------
def setServoCycle():
    for position in range (800, 8000, 50):
        pwm.duty_u16(position)
        utime.sleep(0.01)
        
    for position in range (8000, 800, -50):
        pwm.duty_u16(position)
        utime.sleep(0.01)

# -------------------------   
#  Motor Control Utilities
# -------------------------
def stop():
    print("### Full Stop")
    motor_1a.low()
    motor_1b.low()
    motor_2a.low()
    motor_2b.low()
    
def forward():
    motor_1a.low()
    motor_1b.high()
    motor_2a.high()
    motor_2b.low()
    
def reverse():
    print("<<< REVERSE <<<")
    motor_1a.high()
    motor_1b.low()
    motor_2a.low()
    motor_2b.high()

def left():
    print("<.> SPIN LEFT <.>")
    # Spin Left - right wheels forward & left wheels reverse")
    motor_1a.low()
    motor_1b.high()
    motor_2a.low()
    motor_2b.high()

def right():
    print(">.< SPIN RIGHT >.<") 
    # Spin Right - left wheels forward & right wheels reverse")
    motor_1a.high()
    motor_1b.low()
    motor_2a.high()
    motor_2b.low()

# -----------------------------------
# Sonic Obstical Avoidance Utilities
# -----------------------------------
def get_distance():
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
    # print('The distance is： ', distance, 'cm')
    return distance

def sonic_sense(timer):
    # Determine current free distance
    distance = get_distance()
    print('---> Sensor distance is： ', distance, 'cm')
    # If free distance is limited stop
    if distance < 5:
        stop()
        #suspend timer
        timer.deinit()
        led.on()
        print('Obstical within 5cm')
        utime.sleep(2)
        led.off()
        while distance < 5:
            led.on()
            utime.sleep(1)
            led.off()
            utime.sleep(1)
            distance = get_distance()
        #start timer again
        timer.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)

# ------------------------
#  Move Control Utilities
# ------------------------
def reset_left_rear_encoder_counter():
    global left_rear_encoder_count
    left_rear_encoder_count = 0
    
def calc_clicks(leg_distance):
   # wheel diameter in mm
    wheel_diameter = 65
    # speed encoder slots
    slots = 20
    wheel_circumference = 2 * 3.1415 * (wheel_diameter / 2)
    distance_per_click = wheel_circumference / (slots * 2)
    clicks = int(leg_distance * 10 / distance_per_click)
    return clicks

def move_forward(forward_distance):
    #global left_rear_encoder_count
    reset_left_rear_encoder_counter()
    # Why does this work here but not at the end?????
    
    clicks = calc_clicks(forward_distance)
    # Move forward while counter less than clicks
    print(">>> FORWARD >>>")
    print("--- Heading: 000, Distance: ", forward_distance)
    print("--- Clicks: ", clicks, "Encoder Count: ", left_rear_encoder_count)
    #forward()
    while left_rear_encoder_count < clicks:
        forward()
        pass    
    # Stop wheel when counter = clicks
    stop()
    print("==>Encoder Count: ", left_rear_encoder_count)

# -------------------------
#  Main Navigation Control
# -------------------------
# The definition of a route. A route has one or more legs
# A leg is defined as a heading and a distance
# A route can be in one of four states:
# - Tracking : driving on a heading towards the target
# - Detour : driving on a heading that is off target to avoid an obstical
# - Suspended: stopped can't find a way to the target; waiting for operator input
# - Complete: arrived at calculated target point; waiting for operator input

def main():
    # Main loop
    run = True
    test_cycles = 2
    test_time = 2
    count = 1
    while run:
        #reading = analogvalue.read_u16()
        #pwm.duty_u16(int(reading/6))
        led.off()
        # Forward Test (move_forward(distance in cm)
        move_forward(150)
        utime.sleep(test_time)
        # stop()
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
    # timer.deinit()
    print ('*** Test Completed ***')

# -------------------------------
#  Interrupts Definition Section
# -------------------------------
#
# Setup IR obstical sensor interrupts
#
ir_front_centre.irq(handler=obstical_front_centre, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
ir_left_front.irq(handler=obstical_left_front, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
ir_right_front.irq(handler=obstical_right_front, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
#
# Set speed encoder interrupts
#
encoder_rear_left.irq(handler=left_rear_encoder_counter, trigger=Pin.IRQ_RISING)
#
# Configure timer interrupt event
#
timer.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)

# -------------------
#  Execution Control
# -------------------
try:
    # start main loop
    main()
    timer.deinit()
    
except KeyboardInterrupt:
    # Abort, stop bot
    print ('CTRL-C received, Abort')
    stop()
    
finally:
    # Cleanup
    timer.deinit()
    print ('*** END ***')