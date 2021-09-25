# Import Modules:
from machine import Pin, PWM, Timer
import utime

# ------------------------
#  Initialisation Section
# ------------------------

# Initialise some global variables
distance = 0
front_left_encoder_count = 0
front_right_encoder_count = 0
rear_left_encoder_count = 0
rear_right_encoder_count = 0

# Onboard LED object
led = Pin(25, Pin.OUT)

# Define timer object
timer = Timer()

# Setup PWM object for servo
pwm = PWM(Pin(16))
pwm.freq(50)

# UART communication with Bluetooth module
# Connect GP0 (UART0 Tx) to Rx of HC-05/06 Bluetooth module (brown)
# Connect GP1 (UART0 Rx) to Tx of HC-05/06 Bluetooth module (white)
# Create UART object connect to UART channel 0 at a baud rate of 9600
uart = UART(0, 9600)

# IR sensor interface
ir_front_left = Pin(5, Pin.IN) # orange
ir_front_centre = Pin(4, Pin.IN) # yellow
ir_front_right = Pin(3, Pin.IN) # green
ir_rear_centre = Pin(2, Pin.IN) # blue
# ir_mid_left = Pin(7, Pin.IN) # light grey
# ir_mid_right = Pin(6, Pin.IN) # purple

# Sonic sensor interface 
trigger = Pin(14, Pin.OUT)
echo = Pin(15, Pin.IN)

# Motor speed encoder interface
# encoder_front_left = Pin(10, Pin.IN) # blue
encoder_front_right = Pin(11, Pin.IN) # brown
encoder_rear_left = Pin(12, Pin.IN) # white
# encoder_rear_right = Pin(13, Pin.IN) # purple

# Motor control interface 
motor_1a = Pin(18, Pin.OUT) # L298N IN4 orange
motor_1b = Pin(19, Pin.OUT) # L298N IN3 yellow
motor_2a = Pin(20, Pin.OUT) # L298N IN2 green
motor_2b = Pin(21, Pin.OUT) # L298N IN1 blue
# Remember to connect Pico GND to L298N GND

# ---------------------------------------
#  Interrupt Handlers Definition Section
# ---------------------------------------
#
# Define speed interrupt handlers
#

def front_left_encoder_counter(pin):
    global front_left_encoder_count
    front_left_encoder_count += 1

def front_right_encoder_counter(pin):
    global front_right_encoder_count
    front_right_encoder_count += 1

def rear_left_encoder_counter(pin):
    global rear_left_encoder_count
    rear_left_encoder_count += 1

def rear_right_encoder_counter(pin):
    global rear_right_encoder_count
    rear_right_encoder_count += 1

#
# Define obstical interrupt handlers
#

def obstical_front_left(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Left")
        stop()
        setServoCycle()
    else:
        led.off()
        
def obstical_front_centre(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Center")
        stop()
        setServoCycle()
    else:
        led.off()

def obstical_front_right(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Front Right")
        stop()
        setServoCycle()
    else:
        led.off()

def obstical_rear_centre(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Rear Centre")
        stop()
        setServoCycle()
    else:
        led.off()

def obstical_mid_left(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Mid left")
        stop()
        setServoCycle()
    else:
        led.off()

def obstical_mid_right(pin):
    if (pin.value() == 0):
        led.on()
        print("*** Obsticale - Mid Right")
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
        utime.sleep(0.5)
        led.off()
        while distance < 5:
            led.on()
            utime.sleep(0.5)
            led.off()
            utime.sleep(0.5)
            distance = get_distance()
        #start timer again
        timer.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)

# ------------------------
#  Move Control Utilities
# ------------------------
def reset_front_left_encoder_counter():
    global front_left_encoder_count
    front_left_encoder_count = 0
def reset_front_right_encoder_counter():
    global front_right_encoder_count
    front_right_encoder_count = 0
def reset_rear_left_encoder_counter():
    global rear_left_encoder_count
    rear_left_encoder_count = 0
def reset_rear_right_encoder_counter():
    global rear_right_encoder_count
    rear_right_encoder_count = 0
    
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
    reset_rear_left_encoder_counter()
    # Why does this work here but not at the end?????
    reset_front_right_encoder_counter()
    
    clicks = calc_clicks(forward_distance)
    # Move forward while counter less than clicks
    print(">>> FORWARD >>>")
    print("--- Heading: 000, Distance: ", forward_distance)
    print("--- Clicks: ", clicks, "Encoder Count: ", rear_left_encoder_count)
    #forward()
    while rear_left_encoder_count < clicks:
        forward()
        pass    
    # Stop wheel when counter = clicks
    stop()
    print("==>Front Right Encoder Count: ", front_right_encoder_count)
    print("==>Rear Left Encoder Count: ", rear_left_encoder_count)

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
    run_time = 2
    stop_time = .5
    count = 1
    while run:
        led.off()
        # Forward Test (move_forward(distance in cm) and stop
        move_forward(100)
        utime.sleep(stop_time)
        # Reverse Test
        reverse()
        utime.sleep(run_time)
        stop()
        utime.sleep(stop_time)
        # Left Test
        left()
        utime.sleep(run_time)
        stop()
        utime.sleep(stop_time)
        # Right Test
        right()
        utime.sleep(run_time)
        stop()
        utime.sleep(stop_time)
        
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
# - Front:
ir_front_left.irq(handler=obstical_front_left, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
ir_front_centre.irq(handler=obstical_front_centre, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
ir_front_right.irq(handler=obstical_front_right, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
# - Rear:
ir_rear_centre.irq(handler=obstical_rear_centre, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
# - Middle:
# ir_mid_left.irq(handler=obstical_mid_left, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
# ir_mid_right.irq(handler=obstical_mid_right, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)

#
# Set speed encoder interrupts
#
# - Front
# encoder_front_left.irq(handler=front_left_encoder_counter, trigger=Pin.IRQ_RISING)
encoder_front_right.irq(handler=front_right_encoder_counter, trigger=Pin.IRQ_RISING)
# - Rear
encoder_rear_left.irq(handler=rear_left_encoder_counter, trigger=Pin.IRQ_RISING)
# encoder_rear_right.irq(handler=rear_right_encoder_counter, trigger=Pin.IRQ_RISING)
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
    #timer.deinit()
    
except KeyboardInterrupt:
    # Abort, stop bot
    print ('CTRL-C received, Abort')
    stop()
    
finally:
    # Cleanup
    # timer.deinit()
    print ('*** END ***')