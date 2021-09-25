# =================
#  Import Modules:
# =================
from machine import Pin, PWM, Timer, UART
# from machine import WDT
import utime
import ujson

# ========================
#  Initialisation Section
# ========================

# Set Timeout
#wdt = WDT(id=0, timeout=5000) 
# Onboard LED object

led = Pin(25, Pin.OUT)

# UART communication channel for Bluetooth module

# Connect GP0 (UART0 Tx) to Rx of HC-05/06 Bluetooth module (brown)
# Connect GP1 (UART0 Rx) to Tx of HC-05/06 Bluetooth module (white)
# Create UART object connect to UART channel 0 at a baud rate of 9600
uart = UART(0, 9600)

# IR sensor interface

# ir_mid_left = Pin(4, Pin.IN) # light grey
# ir_mid_right = Pin(5, Pin.IN) # purple
ir_front_left = Pin(6, Pin.IN) # orange
ir_front_centre = Pin(7, Pin.IN) # yellow
ir_front_right = Pin(8, Pin.IN) # green
ir_rear_centre = Pin(9, Pin.IN) # blue

# Motor speed encoder interface
# encoder_front_left = Pin(10, Pin.IN) # blue
encoder_front_right = Pin(11, Pin.IN) # brown
encoder_rear_left = Pin(12, Pin.IN) # white
# encoder_rear_right = Pin(13, Pin.IN) # purple

# Sonic sensor interface 

trigger = Pin(14, Pin.OUT) # brown
echo = Pin(15, Pin.IN) # white

# Servo PWM interface

pwm = PWM(Pin(16))
pwm.freq(50)

# Motor control interface 

motor_1a = Pin(18, Pin.OUT) # L298N IN4 orange
motor_1b = Pin(19, Pin.OUT) # L298N IN3 yellow
motor_2a = Pin(20, Pin.OUT) # L298N IN2 green
motor_2b = Pin(21, Pin.OUT) # L298N IN1 blue
# Remember to connect Pico GND to L298N GND

# Define timer object
timer = Timer()

# Initialise global variables

detour_mode = False
distance = 0
heading = 0
current_heading = 0
detour_heading = 0
front_left_encoder_count = 0
front_right_encoder_count = 0
rear_left_encoder_count = 0
rear_right_encoder_count = 0

# ====================
#  Interrupt Handlers 
# ====================

#
# Process data received on UART-0 channel received dataprocess_rx,
#

def process_rx():
    pass

#
# Speed encoder pulse counters
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
# Handlers for obstical detection
#

def obstical_front_left(pin):
    global detour_mode
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Front Left")
        global detour_mode
        detour_mode = True
        print("===> Detour mode")
    else:
        led.off()
        
def obstical_front_centre(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Front Center")
        global detour_mode
        detour_mode = True
        print("===> Detour mode")
    else:
        led.off()

def obstical_front_right(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Front Right")
        global detour_mode
        detour_mode = True
        print("===> Detour mode")
    else:
        led.off()

def obstical_rear_centre(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Rear Centre")
        global detour_mode
        detour_mode = True
        print("===> Detour mode")
    else:
        led.off()

def obstical_mid_left(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Mid left")
        global detour_mode
        detour_mode = True
        print("===> Detour mode")
    else:
        led.off()

def obstical_mid_right(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Mid Right")
        global detour_mode
        detour_mode = True
        print("===> Detour mode")
    else:
        led.off()

# ==================
#  Hardware Control
# ==================
# -----------------------------------
#  Bluetooth Communication Functions
# -----------------------------------

#     weather_data = {
#         "temp" : temp,
#         "humid" : hum,
#         "baro" : pres
#     }
#     weather_data_json = ujson.dumps(weather_data)
#     
#     time.sleep(8)
#     
#     LED.value(1)
#     uart.write(weather_data_json)
#     LED.value(0)

def send_data(tx_data):
    tx_data_json = ujson.dumps(tx_data)
    uart.write(tx_data_json)

def receive_data():
    data = uart.readline()
    return data

# ------------------------
#  Sonic Sensor Functions
# ------------------------

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
    tx_data = { "distance" : distance }
    send_data(tx_data)
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

# -------------------------   
#  Motor Control Functions
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

def turn_left():
    print("<.> TURN LEFT <.>")
    # Spin Left - right wheels forward & left wheels reverse")
    motor_1a.low()
    motor_1b.high()
    motor_2a.low()
    motor_2b.high()

def turn_right():
    print(">.< TURN RIGHT >.<") 
    # Spin Right - left wheels forward & right wheels reverse")
    motor_1a.high()
    motor_1b.low()
    motor_2a.high()
    motor_2b.low()
    
# ------------------------
#  Servo Control Function
# ------------------------

def cycle_servo():
    for position in range (800, 8000, 50):
        pwm.duty_u16(position)
        utime.sleep(0.01)
        
    for position in range (8000, 800, -50):
        pwm.duty_u16(position)
        utime.sleep(0.01)

def angle_servo(deg):
    
    # Set servo parameter values
    max_deg = 180
    min_deg = 0
    max_duty = 8000
    min_duty = 50
    
    # Calculate PWM Duty Cycle for deg
    #position = max(min(max_duty, (deg - min_deg) * (max_duty - min_duty) // (max_deg - min_deg) + min_duty), min_duty)
    position = int(min_duty + ((max_duty - min_duty) / (max_deg - min_deg) * deg))
    
    print("Position ", position)
    
    # Set the duty cycle of the PWM defined as pwm
    pwm.duty_u16(position)
    
    # Wait for servo to set
    utime.sleep(2)

# ===============
#  Drive Control
# ===============

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

def drive_forward(forward_distance):
    
    # Turn sonic sensor forward
    angle_servo(90)
    
    # Reset encoder counters
    reset_front_left_encoder_counter() # Why does this work here but not at the end?????
    reset_front_right_encoder_counter()
    reset_rear_left_encoder_counter()
    reset_rear_right_encoder_counter()
    
    # Convert distance into encoder pulses
    clicks = calc_clicks(forward_distance)
    
    # Move forward while counter less than clicks
    print(">>> FORWARD >>>")
    print("--- Heading: 000, Distance: ", forward_distance)
    print("--- Clicks: ", clicks, "Encoder Count: ", rear_left_encoder_count)
    forward()
    while rear_left_encoder_count < clicks:
        if detour_mode:
            stop()
            break
        
    # Stop drive when counter = clicks
    stop()
    
    print("==>Front Right Encoder Count: ", front_right_encoder_count)
    print("==>Rear Left Encoder Count: ", rear_left_encoder_count)
    
def drive_detour(slow, side):
    # If for the detour the bot turned right drive until left is not blocked
    # If for the detour the bot turned left drive until right is not blocked
    print("Drive detour")
    forward() # dummy test code
    utime.sleep(2)
    stop()

# ====================
#  Navigation Control
# ====================

def read_compass():
    bearing = 90
    return bearing

def calc_target_heading():
    new_target_heading = 90
    return new_target_heading

def calc_target_distance():
    new_target_distance = 100
    return new_target_distance

def best_turn_angle():
    # Use sonic sensor to scan for free space
    # in a 180 degee arc at 30 degree intervals
    # to avoid an obstical
    
    # Stop Sonic Scan IRQ Timer
    print("--- Stop Timer")
    timer.deinit()
    
    # Scan 90 degrees right
    print("--- scan R90")
    angle_servo(0)
    #utime.sleep(2)
    distance_R90 = get_distance()
    print("--- R90 open distance = ", distance_R90)
    
    # Scan 60 degrees right
    print("--- scan R60")
    angle_servo(30)
    #utime.sleep(2)
    distance_R60 = get_distance()
    print("--- R60 open distance = ", distance_R60)
    
    # Scan 30 degrees right
    print("--- scan R30")
    angle_servo(60)
    #utime.sleep(2)
    distance_R30 = get_distance()
    print("--- R30 open distance = ", distance_R30)
    
    # Scan Front
    print("--- scan Forward")
    angle_servo(90)
    #utime.sleep(2)
    distance_F0 = get_distance()
    print("--- Forward open distance = ", distance_F0)
    
    # Scan 30 degrees left
    print("--- scan L30")
    angle_servo(120)
    #utime.sleep(2)
    distance_L30 = get_distance()
    print("--- L30 open distance = ", distance_L30)
    
    # Scan 60 degrees left
    print("--- scan L60")
    angle_servo(150)
    #utime.sleep(2)
    distance_L60 = get_distance()
    print("--- L60 open distance = ", distance_L60)
    
    # Scan 90 degrees left
    print("--- scan L90")
    angle_servo(180)
    #utime.sleep(2)
    distance_L90 = get_distance()
    print("--- L90 open distance = ", distance_L90)
    
    # Determine best detour angle
    free = distance_F0
    turn_angle = 0 # keep going forward
    if distance_R90 > free:
        free = distance_R90
        turn_angle = 90 # turn 90 degrees right
    if distance_R60 > free:
        free = distance_R60
        turn_angle = 60 # turn 60 degrees right
    if distance_R30 > free:
        free = distance_R30
        turn_angle = 30 # turn 30 degrees right
    if distance_L30 > free:
        free = distance_L30
        turn_angle = -30 # turn 30 degrees left
    if distance_L60 > free:
        free = distance_L60
        turn_angle = -60 # turn 60 degrees left
    if distance_L90 > free:
        free = distance_L90
        turn_angle = -90 # turn 90 degrees left

    print("--- Best turn angle: ", turn_angle, " with free distance of ", free, "cm")    
    
    # set ultra sound sensor to scan forward
    angle_servo(90)
    
    # Start Sonic Scan IRQ Timer
    print("--- Start Timer")
    timer.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)
    
    return turn_angle

def calc_detour_heading():
    current_heading = read_compass()
    turn_angle = best_turn_angle()
    if (current_heading + turn_angle) > 360:
        detour_heading = (current_heading + turn_angle) - 360
    elif (current_heading + turn_angle) < 0:
        detour_heading = 360 + (current_heading + turn_angle)
    else:
        detour_heading = current_heading + turn_angle

    print("Detour heading: ", detour_heading)
    return detour_heading
        
def turn_to_heading(req_heading):
    #
    # For testing only
    #
    req_heading = 90
    
    # Execute turn
    current_heading = read_compass()
    if current_heading > req_heading:
        if req_heading < current_heading - 180:
            turn_right()
            while read_compass() != req_heading:
                pass
            stop()
        else:
            turn_left()
            while read_compass() != req_heading:
                pass
            stop()
    elif current_heading < req_heading:
        if req_heading > current_heading + 180:
            turn_left()
            while read_compass() != req_heading:
                pass
            stop()
        else:
            turn_right()
            while read_compass() != req_heading:
                pass
            stop()
    else:
        print("On course")
            
def verify_heading(req_heading, distance_remaining):
    pass
    heading = read_compass()
    deviance = req_heading
    if deviance > 1 and distance > 10:
        stop()
        turn_to_heading(req_heading)
        move_forward(distance_remaining)
    
def detour():
    # Current heading to target is blocked, determine a detour
    detour_heading = calc_detour_heading()
    turn_to_heading(detour_heading)
    drive_detour("speed", "side")
    # obsticle avoided, resume drive to target
    target_heading = calc_target_heading()
    target_distance = calc_target_distance()
    turn_to_heading(detour_heading)
    drive_forward(target_distance)
    # Problem:
    #  - how to return to route() to execute additional legs of original route

# ==================
#  Route Definition
# ==================

# The definition of a route. A route has one or more legs
# A leg is defined as a heading and a distance
# A route can be in one of four states:
# - Tracking : driving on a heading towards the target
# - Detour : driving on a heading that is off target to avoid an obstical
# - Suspended: stopped can't find a way to the target; waiting for operator input
# - Complete: arrived at calculated target point; waiting for operator input

def test_route():
    leg_heading = 90 # degrees
    leg_distance = 100 # cm
    # current_heading = leg_heading
    turn_to_heading(leg_heading)
    drive_forward(leg_distance)
    if detour_mode:
        detour()
    else:
        print ('*** Route Test Completed ***')
    
# Motor control test
def test_setup():
    run = True
    test_cycles = 2
    run_time = 2
    stop_time = .5
    count = 1
    # Turn sonic sensor forward
    angle_servo(90)
        
    while run:
        led.off()
        # Forward Test
        forward()
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
    
    print ('*** Setup Test Completed ***')
    
def run_test():
    # test_setup()
    test_route()
    stop()
    # timer.deinit()

# ===============================
#  Interrupts Definition Section
# ===============================
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

#
# UART RX interrupt
#
#uart.irq(UART.RX_ANY, priority=5, handler=process_rx, wake=machine.IDLE)

# ===================
#  Execution Control
# ===================
try:
    cycle_servo()
    # start main loop
    # main()
    run_test()
    #timer.deinit()
    
except KeyboardInterrupt:
    # Abort, stop bot
    print ('CTRL-C received, Abort')
    machine.reset()
    #stop()
    
finally:
    # Cleanup
    # timer.deinit()
    print ('*** END ***')