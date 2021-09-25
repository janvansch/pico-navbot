# NavBot control code
# MicroPython
# By Johannes van Schalkwyk, all rights reserved

# =================
#  Import Modules:
# =================

from machine import Pin, UART, I2C, PWM, Timer
import _thread
import utime
import math
import ujson

# ========================
#  Initialisation Section
# ========================

# --------------------
#  Onboard LED object
# --------------------

led = Pin(25, Pin.OUT)

# -----------------------------------
#  UART channel for Bluetooth module
# -----------------------------------
# Setup UART channel 0 (TX = GP0 & RX = GP1), with baud rate of 9600
# Connect GP0 (UART0 Tx) to Rx of HC-05/06 Bluetooth module (brown)
# Connect GP1 (UART0 Rx) to Tx of HC-05/06 Bluetooth module (orange)
# Module powerred by 5v bus, but RX and TX powerred by internal 3.3v regulator

uart = UART(0, 9600)

# --------------------------------
#  I2C channel for Compass module
# --------------------------------
# Powerred by 3.3v bus
# Does not require Logic Level Converter to convert sda to 3.3v
# sda = green
# scl = grey
# Compass module address = 0x1e

i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)

# ------------------------
#  Sonic sensor interface
# ------------------------
# Powerred by 5v bus
# Requires Logic Level Converter to convert echo signal to 3.3v (1 channel)

trigger = Pin(4, Pin.OUT) # brown
echo = Pin(5, Pin.IN) # white

# -----------------------------------
#  Wheel rotation sensor interface
# -----------------------------------
# Powerred by 5v bus of L298N
# Requires Logic Level Converter to convert signal to 3.3v (4 channels)

front_left_wheel = Pin(6, Pin.IN) #  blue	
front_right_wheel = Pin(7, Pin.IN) # dark purple
#rear_left_wheel = Pin(8, Pin.IN) # grey
#rear_right_wheel = Pin(9, Pin.IN) # white

# ---------------------
#  IR sensor interface
# ---------------------
# Powerred by 5v bus
# Requires Logic Level Converter to convert signal to 3.3v (6 channels)

ir_front_left = Pin(10, Pin.IN) # orange
ir_front_centre = Pin(11, Pin.IN) # yellow
ir_front_right = Pin(12, Pin.IN) # green
ir_rear_centre = Pin(13, Pin.IN) # blue
ir_mid_left = Pin(14, Pin.IN) # purple
ir_mid_right = Pin(15, Pin.IN) # light grey

# -----------------------------------
#  Motor Controler (L298N) interface
# -----------------------------------
# Remember to connect L298N GND to Pico GND
# Pico output signal (3.3v), no Logic Level Converter required

# Speed control - PWM
motor_1_pwm = PWM(Pin(16)) # L298N ENA blue left
motor_1_pwm.freq(50)
motor_2_pwm = PWM(Pin(17)) # L298N ENB brown right
motor_2_pwm.freq(50)

# Forward & reverse control
motor_1a = Pin(18, Pin.OUT) # L298N IN4 red right
motor_1b = Pin(19, Pin.OUT) # L298N IN3 orange right
motor_2a = Pin(20, Pin.OUT) # L298N IN2 yellow left
motor_2b = Pin(21, Pin.OUT) # L298N IN1 green left

# -----------------------
#  Servo interface - PWM
# -----------------------
# Powered by 5v bus of L298N
# Pico output signal (3.3v), no Logic Level Converter required

servo_pwm = PWM(Pin(22)) # yellow
servo_pwm.freq(50)

# ---------------------
#  Define timer object
# ---------------------

timer = Timer()

# -----------------------------
#  Initialise global variables
# -----------------------------

avg_heading = 0
second_thread = True
detour_mode = False
detour_mode_count = 0
distance = 0
heading = 0
current_heading = 0
detour_heading = 0
front_left_wheel_count = 0
front_right_wheel_count = 0
rear_left_wheel_count = 0
rear_right_wheel_count = 0

# ------------
#  Constants:
# ------------

# Required MPU6050 Registers and their Address

# Address of Configuration register A
REGISTER_A = 0x00

# Address of configuration register B
REGISTER_B = 0x01

# Address of mode register
REGISTER_MODE = 0x02

# Address of X-axis MSB data register
X_AXIS_H = 0x03

# Address of Z-axis MSB data register
Z_AXIS_H = 0x05

# Address of Y-axis MSB data register
Y_AXIS_H = 0x07

# Declination angle in radians of location
DECLINATION = -0.4459       
# -25degrees 33minutes 25.55 -0.4459 radians
                                
# Define pi value
PI = 3.14159265359     

# HMC5883L magnetometer device address
DEVICE_ADDRESS = 0x1e

# -----------------------------------
#  Initialise HMC5883L magnetometer:
# -----------------------------------
# Write to Configuration Register A
i2c.writeto_mem(DEVICE_ADDRESS, REGISTER_A, b'\x70')

# Write to Configuration Register B for gain
i2c.writeto_mem(DEVICE_ADDRESS, REGISTER_B, b'\xa0')

# Write to mode Register for selecting mode
i2c.writeto_mem(DEVICE_ADDRESS, REGISTER_MODE, b'0')
    
# =========
#  Helpers
# =========

# Start heading monitor as second thread
def start_heading_monitor():
    global second_thread
    second_thread = True
    print("--- Start heading monitor")
    _thread.start_new_thread((monitor_heading), ())

# Trigger second thread stop
def stop_heading_monitor():
    global second_thread
    second_thread = False

# Navigation state

def switch_nav_mode():
    pass

#     if detour_mode:
#         global detour_mode
#         detour_mode = False
#         print("--- Target mode set!")
#         global nav_state
#         nav_state = "target"
#     else:
#         global detour_mode
#         detour_mode = True
#         print(">>> Detour mode set!")
#         global nav_state
#         nav_state = "detour"

def set_detour_mode():
    global detour_mode_count
    global detour_mode
    global nav_state
    
    if detour_mode:
        # If already in detour mode then an obstical was encountered
        # in the detour drive heading. Increase counter by 1.
        # This will suspend the current detour drive.
        detour_mode_count +=1
    else:
        detour_mode_count = 1
        detour_mode = True
        print(">>> Detour mode set!")
        nav_state = "detour"

def set_target_mode():
    global detour_mode
    detour_mode = False
    print("--- Target mode set!")
    global nav_state
    nav_state = "target"
    global detour_mode_count
    detour_mode_count = 0

def read_raw_data(addr):
    
    # Read raw 16-bit value
    byte_high = i2c.readfrom_mem(DEVICE_ADDRESS, addr, 1)
    byte_low = i2c.readfrom_mem(DEVICE_ADDRESS, addr+1, 1)
    
    # Convert to integer
    high = int.from_bytes(byte_high, "big")
    low = int.from_bytes(byte_low, "big")
    
    # Concatenate higher and lower value (only works with integers)
    value = ((high << 8) | low)
      
    # To get signed value from module
    if(value > 32768):
        value = value - 65536
    
    return value

# ====================
#  Interrupt Handlers
# ====================

#
# Process data received on UART-0 channel received dataprocess_rx,
#

def process_rx():
    pass

# ----------------------
#  wheel pulse counters
# ----------------------

def front_left_wheel_counter(pin):
    global front_left_wheel_count
    front_left_wheel_count += 1

def front_right_wheel_counter(pin):
    global front_right_wheel_count
    front_right_wheel_count += 1

# def rear_left_wheel_counter(pin):
#     global rear_left_wheel_count
#     rear_left_wheel_count += 1
# 
# def rear_right_wheel_counter(pin):    
#     global rear_right_wheel_count
#     rear_right_wheel_count += 1

# ---------------------------------
#  Handlers for obstical detection
# ---------------------------------

# When an obstical is detected make a detour

def obstical_front_left(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Front Left")
        set_detour_mode()
    else:
        led.off()

def obstical_front_centre(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Front Center")
        set_detour_mode()
    else:
        led.off()

def obstical_front_right(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Front Right")
        set_detour_mode()
    else:
        led.off()

def obstical_rear_centre(pin):
    if (pin.value() == 0):
        stop()
        led.on()
        print("*** Obsticale - Rear Centre")
        set_detour_mode()
    else:
        led.off()

# def obstical_mid_left(pin):
#     if (pin.value() == 0):
#         stop()
#         led.on()
#         print("*** Obsticale - Mid left")
#         set_detour_mode()
#     else:
#         led.off()
# 
# def obstical_mid_right(pin):
#     if (pin.value() == 0):
#         stop()
#         led.on()
#         print("*** Obsticale - Mid Right")
#         set_detour_mode()
#     else:
#         led.off()

# ========================
#  Second Thread Function
# ========================
def monitor_heading():
    
    global avg_heading
    
    count = 0
    sum_heading_angles = 0

    while second_thread:
            
        # Read Accelerometer raw value
        x = read_raw_data(X_AXIS_H)
        z = read_raw_data(Z_AXIS_H)
        y = read_raw_data(Y_AXIS_H)
        
        #print("X: ", x)
        #print("Y: ", y)

        heading = math.atan2(y, x) + DECLINATION
        
        # Due to declination check for >360 degree
        if(heading > 2*PI):
            heading = heading - 2*PI

        # Check for sign
        if(heading < 0):
            heading = heading + 2*PI

        # Convert into angle
        heading_angle = heading * 180/PI
            
        count += 1
        sum_heading_angles = sum_heading_angles + heading_angle
        
        
        if count == 50:
            
            #print(sum_heading_angles, " ", round(sum_heading_angles/count, 1))
            #print ("Heading Angle = %d°" %(sum_heading_angles/count))
            avg_heading = round(sum_heading_angles/count, 1)
            count = 0
            sum_heading_angles = 0
            
            start_time = utime.ticks_ms()
            interval = 500
            while utime.ticks_ms() - start_time < interval:
                pass
            # utime.sleep(0.5) is this blocking, e.g. irq
    _thread.exit()

# ==================
#  Hardware Control
# ==================

# -----------------------------------
#  Bluetooth Communication Functions
# -----------------------------------

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
    duration = 0
    distance = 0
    signaloff = 0
    signalon = 0
    
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

    utime.sleep(0.2)
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
#  Servo Control Functions
# -------------------------

def cycle_servo():
    
    print("--- Test servo")

    for position in range (1150, 8650, 50):
        servo_pwm.duty_u16(position)
        utime.sleep(0.02)

    for position in range (8650, 1150, -50): # range(start, stop, step)
        servo_pwm.duty_u16(position)
        utime.sleep(0.02)
        
    angle_servo(90)
    
    print("--- Servo test complete")

def angle_servo(deg):

    # Set servo parameter values
    max_deg = 180
    min_deg = 0
    max_duty = 8650
    min_duty = 1150

    # Calculate PWM Duty Cycle for deg
    position = max(min(max_duty, (deg - min_deg) * (max_duty - min_duty) // (max_deg - min_deg) + min_duty), min_duty)
   
    print("Position ", position)

    # Set the duty cycle of the PWM defined as pwm
    servo_pwm.duty_u16(position)

    # Wait for servo to set
    utime.sleep(0.5)

# -------------------------
#  Motor Control Functions
# -------------------------

def stop():
    print("--- STOP ---")
    
    motor_1a.low()
    motor_1b.low()
    motor_2a.low()
    motor_2b.low()

def forward():
    #print(">>> Set FORWARD >>>")
    motor_1a.low()
    motor_1b.high()
    motor_2a.high()
    motor_2b.low()
    #print ("--- FORWARD set")

def reverse():
    print("<<< Set REVERSE <<<")
    
    motor_1a.high()
    motor_1b.low()
    motor_2a.low()
    motor_2b.high()

def turn_left():
    # Static left turn - left wheels forward & right wheels reverse
    print("<<< TURN LEFT ---")
    
    set_speed('slow')
        
    motor_1a.low()
    motor_1b.high()
    motor_2a.low()
    motor_2b.high()

def turn_right():
    # Static right turn - left wheels forward & right wheels reverse
    print("--- TURN RIGHT >>>")
    
    set_speed('slow')
    
    motor_1a.high()
    motor_1b.low()
    motor_2a.high()
    motor_2b.low()
    
def duty_cycle(level):
    # The motor operating voltage is 3-6 volts
    # The max duty cycle is 65025, power is on 100% off the cycle, 6v. 
    # A 50% duty cycle is 32513 rounded, power is on 50% of the time, 3v.
    levels = 10
    max_duty = 65025 # 1111111111111111 = 65535?
    min_duty = 32513
    duty_range = max_duty - min_duty
    duty = int((level/levels) * duty_range) + min_duty
    return duty
    

def set_speed(speed):
    # Forward drive speed settings are slow, medium, economy and fast
    
    duty = 0
            
    if speed == 'slow':
        duty = duty_cycle(2)
        
    elif speed == 'medium':
        duty = duty_cycle(5)
        
    elif speed == 'economy':
        duty = duty_cycle(8)
        
    elif speed == 'full':
        duty = duty_cycle(10)
        
    if duty > 0:
        motor_1_pwm.duty_u16(duty)
        motor_2_pwm.duty_u16(duty)
       #print ("--- PWM duty set for required speed")
        
    else:
        print ("*** ERROR - invalid speed ***")
        
def fwd_left_curve(speed='medium', turn_rate='gentle'):
    # A forward drive speed settings are slow, medium, economy and full
    # Turn can be gentle or sharp
    # Note that if speed is fast only gentle turns will be executed
    
    if speed == 'slow':
        
        if turn_rate == 'gentle':
            left_duty_cycle = duty_cycle(1)
            right_duty_cycle = duty_cycle(2)
            
        elif turn_rate == 'sharp':
            left_duty_cycle = duty_cycle(1)
            right_duty_cycle = duty_cycle(3)
        
    elif speed == 'medium':
        
        if turn_rate == 'gentle':
            left_duty_cycle = duty_cycle(4)
            right_duty_cycle = duty_cycle(5)
            
        elif turn_rate == 'sharp':
            left_duty_cycle = duty_cycle(4)
            right_duty_cycle = duty_cycle(6)
            
    elif speed == 'economy':
        
        if turn_rate == 'gentle':
            left_duty_cycle = duty_cycle(7)
            right_duty_cycle = duty_cycle(8)
            
        elif turn_rate == 'sharp':
            left_duty_cycle = duty_cycle(7)
            right_duty_cycle = duty_cycle(9)
            
    elif speed == 'full':
        
        if turn_rate == 'gentle':
            left_duty_cycle = duty_cycle(9)
            right_duty_cycle = duty_cycle(10)
            
        elif turn_rate == 'sharp':
            left_duty_cycle = duty_cycle(9)
            right_duty_cycle = duty_cycle(10)
            
            print("NOTE - only gentle turns at full speed")
    
    motor_1_pwm.duty_u16(left_duty_cycle)
    motor_2_pwm.duty_u16(right_duty_cycle)
    
def fwd_right_curve(speed='medium', turn_rate='gentle'):
    # A forward drive speed settings are slow, medium, economy and full
    # Turn can be gentle or sharp
    # Note that if speed is fast only gentle turns will be executed
                                      
    if speed == 'slow':
        
        if turn_rate == 'gentle':
            left_duty_cycle = duty_cycle(1)
            right_duty_cycle = duty_cycle(2)
            
        elif turn_rate == 'sharp':
            left_duty_cycle = duty_cycle(1)
            right_duty_cycle = duty_cycle(3)
        
    elif speed == 'medium':
        
        if turn_rate == 'gentle':
            left_duty_cycle = duty_cycle(4)
            right_duty_cycle = duty_cycle(5)
            
        elif turn_rate == 'sharp':
            left_duty_cycle = duty_cycle(4)
            right_duty_cycle = duty_cycle(6)
            
    elif speed == 'economy':
        
        if turn_rate == 'gentle':
            left_duty_cycle = duty_cycle(7)
            right_duty_cycle = duty_cycle(8)
            
        elif turn_rate == 'sharp':
            left_duty_cycle = duty_cycle(7)
            right_duty_cycle = duty_cycle(9)
            
    elif speed == 'full':
        
        if turn_rate == 'gentle':
            left_duty_cycle = duty_cycle(9)
            right_duty_cycle = duty_cycle(10)
            
        elif turn_rate == 'sharp':
            left_duty_cycle = duty_cycle(9)
            right_duty_cycle = duty_cycle(10)
            
            print("NOTE - only gentle turns at full speed")
   
    motor_1_pwm.duty_u16(right_duty_cycle)
    motor_2_pwm.duty_u16(left_duty_cycle)

# def set_duty(motor, speed):
#         
#     duty = calc_duty(duty_level)
# 
#     if motor == 'left':
#         motor_1_pwm.duty_u16(duty)
# 
#     elif motor == 'right':
#         motor_2_pwm.duty_u16(duty)
# 
#     elif motor == 'both':
#         motor_1_pwm.duty_u16(duty)
#         motor_2_pwm.duty_u16(duty)
# 
#     else:
#         stop()
#         print("ERROR - Invalid motor selection, stop executed")
#         nav_state = 'stop'

# ===============
#  Drive Control
# ===============
#
# Reset drive distance counters -
# This must be done before the start of a drive while the bot is still stationary
# in order to avoid conflicts between this and the IRQ of the motor turn pulse counter
#
def reset_front_left_wheel_counter():
    global front_left_wheel_count
    front_left_wheel_count = 0

def reset_front_right_wheel_counter():
    global front_right_wheel_count
    front_right_wheel_count = 0

def reset_rear_left_wheel_counter():
    global rear_left_wheel_count
    rear_left_wheel_count = 0

def reset_rear_right_wheel_counter():
    global rear_right_wheel_count
    rear_right_wheel_count = 0

def calc_click_distance():
    # Wheel diameter in mm
    wheel_diameter = 65
    # Encoder slots
    slots = 20
    
    wheel_circumference = 2 * 3.1415 * (wheel_diameter / 2)
    
    distance_per_click = wheel_circumference / (slots)
    #print ("--- Click distance = ", distance_per_click)
    
    return distance_per_click

#
#  Convert distance (cm) into slots count
#
def calc_clicks(distance_cm):
    #print ("--- Convert distance to clicks")
    distance_mm = distance_cm * 10
    clicks = int(distance_mm / calc_click_distance())
    #print ("=== The distance of ", distance_cm, "cm equals ", clicks, " sensor clicks")
    return clicks

#
#  Convert clicks counted to distance travelled
#
def calc_distance(clicks):
    distance_mm = clicks * calc_click_distance()
    distance_cm = distance_mm / 10
    return distance_cm

#
# Monitor state of target side IR sensor
#
def side_sensor_state(side):
    side_blocked = False
    return side_blocked

#
#  Drive on target heading for required distance
#

def drive_target(target_distance):
    
    print("--- Start drive to target")
    
    distance_remaining = 0
    
    # Reset wheel counters
    reset_front_left_wheel_counter()
    reset_front_right_wheel_counter()
    reset_rear_left_wheel_counter()
    reset_rear_right_wheel_counter()

    # Convert distance into encoder slots
    clicks = calc_clicks(target_distance)
        
    print("--- Distance to target: ", target_distance, " / Clicks: ", clicks)
    
    # Set speed
    #print ("--- Set speed")
    speed = "full"
    set_speed(speed)
    print ("--- Speed set to ", speed)
    
    # Drive forward
    print ("--- Drive forward")
    forward()
    
    near_target = False
    # Continue forward while clicks counter is less than distance clicks
    while front_left_wheel_count < clicks:
                        
        if detour_mode:
            print("--- Detour mode set, target drive suspended")
            # Software interrupt - detour mode has been set, stop target drive
            stop()
            # A detour from original couse is required to avoid an obstical
            # How must of the drive was completed?
            # Calculate average clicks
            average_clicks = (front_right_wheel_count + front_left_wheel_count) / 2
            # calculate and record remaining distance to target
            distance_remaining = target_distance - calc_distance(average_clicks)
            break
        
        ##############################
        # if course_deviation_right:
        #     slow_left()
        # if course_deviation_left:
        #     slow_right()
        ##############################
        
        # Slow down when 80% complete        
        if (front_left_wheel_count / clicks) * 100 > 80.0 and not near_target:
            near_target = True
            set_speed('medium')
            print("--- slow down, almost at target")

    # Stop destination reached, counter = clicks
    stop()

    print("==> Front Right wheel Count: ", front_right_wheel_count)
    print("==> Front Left wheel Count: ", front_left_wheel_count)

    return distance_remaining

def drive_detour(side, count):
    # If the bot made a turn to the right the target is on the left.
    # Drive untill the left side is not blocked.
    # If the bot made a turn to the left the target is on the right.
    # Drive untill the right side is not blocked.
    
    target_blocked = True

    print("*** Drive detour ***")
    print( ":--> Detour count: ", count)
    
    # Reset wheel counters to record detour drive distance
    reset_front_left_wheel_counter()
    reset_front_right_wheel_counter()
    reset_rear_left_wheel_counter()
    reset_rear_right_wheel_counter()
    
    # Slowly drive forward
    set_speed('slow')
                                             
    forward()

    while target_blocked and count == detour_mode_count:
        # If the detour count increases it means an obstacle was encountered
        # in the detour route. Stop current detour drive
        if detour_mode_count != count:
            # An obstacle event was triggered during a detour drive
            break
        
        # Code must be inserted here to read the state of the relevant side
        # sensor. IRQ event? maybe not this ia a small loop
        # target_blocked = side_sensor_state(side)
        
    utime.sleep(10)
    stop()
    
    return target_blocked
    

# ====================
#  Navigation Control
# ====================

def read_compass():
    bearing = 90
    return bearing

# def calc_target_heading():
#     new_target_heading = 90
#     return new_target_heading
# 
# def calc_target_distance():
#     new_target_distance = 100
#     return new_target_distance



def calc_heading(turn_angle):
    
    # Read current compass heading
    current_heading = read_compass()
    
    # Calculate detour compass heading
    if (current_heading + turn_angle) > 360:
        heading = (current_heading + turn_angle) - 360
    elif (current_heading + turn_angle) < 0:
        heading = 360 + (current_heading + turn_angle)
    else:
        heading = current_heading + turn_angle

    print("Detour heading: ", detour_heading)
    
    return heading

def best_detour_angle():
    # To avoid and obstical use the sonic sensor to scan for open space
    # in a 180 degee arc at 10 degree intervals

    # Stop Sonic Scan Timer
    print("--- Stop Timer")
    timer.deinit()

    scan_arc = 180
    scan_angle = -90
    free_distance = 0
    best_distance = 0
    correction = 10 # the actual scan arc is not 180 on the servo, don't know why
    
    while scan_arc >= 0:
        angle_servo(scan_arc)
        print("--- scan", scan_angle)
        free_distance = get_distance()
        print("--- Free distance = ", free_distance)
        
        if free_distance > best_distance:
            best_distance = free_distance
            if scan_angle < correction:
                best_angle = scan_angle + 10
            elif scan_angle > correction:
                best_angle = scan_angle - 10
            else:
                best_angle = 0 # i.e. forward
            
        scan_arc -= 10
        scan_angle += 10
    
    print( "--- The best turn angle is ", best_angle, "deg with a free distance of: ", best_distance, "cm")

    # set ultra sound sensor to scan forward
    angle_servo(90)

    # Start Sonic Scan Timer
    print("--- Start Timer")
    timer.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)

    return best_angle

def turn_to_heading(req_heading):
    #
    # For testing only
    #
    req_heading = 90

    # Execute turn
    print ("--- Turn to heading")
    current_heading = read_compass()
    if current_heading > req_heading:
        if req_heading < current_heading - 180:
            turn_right('slow')
            while read_compass() != req_heading:
                pass
            stop()
        else:
            turn_left('slow')
            while read_compass() != req_heading:
                pass
            stop()
    elif current_heading < req_heading:
        if req_heading > current_heading + 180:
            turn_left('slow')
            while read_compass() != req_heading:
                pass
            stop()
        else:
            turn_right('slow')
            while read_compass() != req_heading:
                pass
            stop()
    else:
        print("--- On course, no turn required")

def verify_heading(req_heading, distance_remaining, current_speed):

    # This function must be called by drive_target() while driving
    # to verify drive heading. If there is a deviation correct heading
    # by slowing either the left wheels for a left correction or right
    # wheels for a right correction. This should happen smoothly without
    # stopping
    # maybe this should be continious and part of the drive functions

    # dummy code
    heading = read_compass()
    deviance = req_heading - heading
    
    while deviance > 1 and distance > 10:
        
        if deviance < 5:
            speed = current_speed
            turn = 'gentle'
            
        elif deviance > 5:
            speed = 'slow'
            turn = 'sharp'
            
        if req_heading < heading:
            fwd_left_turn(speed, turn)
            
        elif req_heading > heading:
            fwd_right_turn(speed, turn)
    
    set_speed(current_speed)

def calc_target_position(deviation_angle, detour_distance, target_distance, obstacle_side):
    
    # a) Calculate target position (heading & distance) from current position.
    #    This will be the new dead reconning point.
    # b) If the detour is successfull in avoided the obstacle the NavBot will set
    #    target mode and drive to target as per the target data from this point.
    # c) If not, a new detour will start from this point and the deviation
    #    from target heading will be calculated from this point.
    
    # For the detour angle and travel distance how far did the navbot deviate
    # from the original target heading? This is side of the detour triangle that
    # is opposite the detour angle. The triangle is formed by the dead reconning
    # point distance to target, the detour angle, the detour travel distance and
    # the distance of the detour position from the target. The oposite side will
    # be the new distance to target.
    # The law of cosines will be used to calculate this distance and the turn
    # angle the NavBot must execute to point towards the target
    #
    #                   new target
    #                    distance
    #   target position____________detour position
    #                  \          /
    #                   \        /
    #    target distance \      / detour distance &
    #  & target heading   \    /  detour heading
    #                      \  /
    #                       \/
    #                detour point (Last dead reconning point)
    #                this is also the detour deviation angle
    
    if deviation_angle > 180:
        # angle outside target triangle
        deviation_angle = 360 - deviation_angle
            
    if deviation_angle == 180:
        
        # Target heading won't change. Only the target distance which would
        # equal current target distance plus detour drive distance
        # so the calculation of a new target heading is not required
        new_target_distance = target_distance + detour_distance
    
    else:
        # target distance = distance to target from detour position
        new_target_distance = math.sqrt(target_distance**2 + detour_distance**2 - 2 * target_distance * detour_distance * math.cos(deviation_angle)) 
    
    print(":--> Target distance: ", new_target_distance)
          
    # calc detour position angle
    detour_position_angle = math.acos((new_target_distance**2 + detour_distance**2 - target_distance**2) / (2 * new_target_distance * detour_distance))
    
    print(":--> Detour position angle: ", detour_position_angle)
    
    # turn_angle = the degrees to turn from the current detour heading to
    # new target heading as calculated at the current detour position -
    # detour heading is a straight line, 180 degrees
    target_turn_angle = 180 - detour_position_angle
    
    # But in which direction?
    # If obstacle is to the left it means the NavBot turned to the right
    # to avoid the obstacle. That means the target heading is to the left
    if obstacle_side == 'left':
        # that is counter clock wise on compass, make angle negative
        target_turn_angle = target_turn_angle * -1
        
#         new_detour_heading = current_heading + detour_angle
#         
#         if new_detour_heading > 360:
#             new_detour_heading = new_detour_heading - 360
#         
#     else:
#         new_detour_heading = target_heading - detour_angle
#         
#         if detour_heading < 0:
#             new_detour_heading = new_detour_heading + 360
                
    print(":--> Target turn angle: ", target_turn_angle)
    
    # calc target heading using current compass heading    
    new_target_heading = calc_heading(target_turn_angle)
    
    print(":--> New target heading: ", new_target_heading)
        
        # calc target heading without using compass heading
    #         if detour_angle > 0:
    #             new_target_heading_2 = detour_heading - target_turn_angle
    #         
    #             if new_target_heading_2 < 0:
    #                 new_target_heading_2 = new_target_heading_2 + 360
    #         else:
    #             target_heading_2 = detour_heading + target_turn_angle
    #         
    #             if target_heading_2 > 360:
    #                 target_heading_2 = target_heading_2 - 360

    target_position = {
        "heading": new_target_heading,
        "distance": new_target_distance}
    
    return target_position

def do_detour(target_heading, target_distance):
    
    # Current heading to target is blocked, make a detour
    print ('*** Detour Started ***')
    
    while detour_mode:
    
        # Determine turn angle with best free distance
        # if turn angle > 0 turn right else turn to the left
        detour_angle = best_detour_angle()
        # the deviation angle = the detour angle
        
        # Determine detour heading, current heading + detour angle
        # Use compass to determine current heading
        detour_heading = calc_heading(detour_angle)
        
        print(" :--> Detour heading: ", detour_heading)
        
        turn_to_heading(detour_heading)
        
        # deviation_angle = target_turn_angle + detour_angle
        # deviation angle is detour heading minus current target heading
        deviation_angle = detour_heading - target_heading
        # The angle by which the detour deviates from the target's heading
        # This angle is required to calculate the distance to target using
        # Cosine Rule when calculating target position after detour
        
        print( ":--> Deviation angle: ", deviation_angle)
        
        # Determine obstacle side
        if detour_angle > 0:
            # turning right, obstacle on left
            obstacle_side = "left"
        else:
            # turning left, obstacle 
            obstacle_side = "right"
        
        print( ":--> Obstacle side: ", obstacle_side)
        
        # Drive detour heading
        target_blocked = drive_detour(obstacle_side, detour_mode_count)
        # If the detour drive ends and the target direction is still blocked an
        # obstical interrupt was triggered during the detour drive.
            
        # What happens when an obstical is encountered in a detour drive?
        #  1. detour_mode_count is incremented
        #  2. this will stop current detour
        #  3. new dead reconning point is calculated
        #  4. start a new detour
        
        # Calculate average clicks
        average_clicks = (front_right_wheel_count + rear_left_wheel_count) / 2
            
        # calculate and record detour distance travelled
        detour_distance = calc_distance(average_clicks)
        
        print( ":--> Detour distance: ", detour_distance)
        
        # At the current detour location of the NavBot what is the heading and
        # distance to the target? If the detour was successful this will be the
        # target position information used to drive to the target. If not this
        # will be the dead reconning point form which the next detour drive will
        # start
        target_position = calc_target_position(deviation_angle, detour_distance, target_distance, obstacle_side)
        
        target_heading = target_position['heading']    
        target_distance = target_position['distance']
        
        print( ":--> Target heading: ", target_heading, " and distance: ", target_distance)
            
        if (not target_blocked):
            set_target_mode()
    
    # Obstical avoided, start driving towards target        
    
    return target_position

def go_target(target_position):
    
    target_heading = target_position['heading']
    target_distance = target_position['distance']
    
    target_state = "executing"

    while target_state == "executing":

        # Turn to required heading
        print("=== Current heading: ", avg_heading)
        turn_to_heading(target_heading)

        # Drive to target
        #print ('=== Drive to target')
        remaining_distance = drive_target(target_distance)

        # Process drive outcome:
        if nav_state == "detour":
            # Obstical encounterred, plan and drive detour
            # Return target position (heading and distance) from the current
            # position, dead reconning detour position of the NavBot
            new_target_position = do_detour(target_heading, remaining_distance)
            
            if nav_state == "target":
                print ('*** Detour Successful ***')
                target_heading = new_target_position['heading']
                target_distance = new_target_position['distance']
                global detour_mode_count
                detour_mode_count = 0
                
            elif nav_state == "stop":
                print ('*** Detour Unsuccessful ***')
                target_state = "halt"
                
        else:
            print ('*** Target Reached ***')
            target_state = 'complete'

    return target_state

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

def router(route):
    # For each way point in the route extract heading & distance data as targets
    print ("--- Router Started")
    legs_in_route = 1 # for testing

    if legs_in_route > 0:
        leg_count = 0
        route_state = 'executing'
        while route_state == 'executing':
            #print ("--- Processing leg number: ", leg_count + 1)
            target_data = load_route()
            #target_data = route[leg_count] #=======>
            target_state = go_target(target_data)
            #target_state = target(route['heading'], route['distance'])

            if target_state == 'complete':
                leg_count +=1
                print ("--- Completed: ", leg_count)

                if leg_count >= legs_in_route:
                    route_state = 'complete'

            elif target_state == 'halt':
                print ("*** HALT - could not reach leg target")
                route_state = 'halt'

            else:
                print ("*** ERROR - Target() returned invalid state")
    else:
        print ("*** HALT - No route information")
        route_state = 'halt'

    return route_state

def manual_ctrl():
    pass

def select_mode():
    mode = 'auto'
    return mode

def load_route():

    # display saved routes

    # select route or capture new route

    route = {
        "heading" : 90,
        "distance" : 400
    }

    return route

def get_start():

    # wait for user to give start or cancel command
    auto_state = "execute"

    return auto_state

def main():

    #
    # Configure timer interrupt event for sonic sensor
    #
    timer.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)
    
    # Set Mode - auto or manual
    mode = select_mode()

    # Self navigation mode selected
    if mode == "auto":
        #print ('*** Self Navigation Mode Selected ***')

        route = load_route()

        auto_state = get_start()

        # Start Auto Navigation
        if auto_state == "execute":

            # Set Navigation State
            global nav_state
            nav_state = "target"

            # Start executing route
            route_state = router(route)

            # Process route result
            if route_state == "complete":
                print ('*** Route Completed ***')

            elif route_state == "halt":
                print ('*** Failed to reach objective ***')

            else:
                print ('*** ERROR - unexpected end to route ***')

        else:
            # User cancelled drive
            print ('*** Cancelled ***')

    # Manual control selected
    elif mode == "manual":
        print ('*** Manual Navigation Mode Selected ***')
        manual_ctrl()

    print ('*** Main END ***')

# ===============================
#  Interrupts Definition Section
# ===============================

# -------------------------------------
#  Setup IR obstical sensor interrupts
# -------------------------------------

# - Front:
#ir_front_left.irq(handler=obstical_front_left, trigger=Pin.IRQ_FALLING) # obstacle
ir_front_left.irq(handler=obstical_front_left, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)

#ir_front_centre.irq(handler=obstical_front_centre, trigger=Pin.IRQ_FALLING) # obstacle
ir_front_centre.irq(handler=obstical_front_centre, trigger=Pin.IRQ_RISING)
#ir_front_centre.irq(handler=obstical_front_centre, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)

ir_front_right.irq(handler=obstical_front_right, trigger=Pin.IRQ_FALLING) # obstacle
# ir_front_right.irq(handler=obstical_front_right, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)

# - Rear:
ir_rear_centre.irq(handler=obstical_rear_centre, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
#ir_rear_centre.irq(handler=obstical_rear_centre, trigger=Pin.IRQ_FALLING) # obstacle

# - Middle:
#ir_mid_left.irq(handler=obstical_mid_left, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
#ir_mid_right.irq(handler=obstical_mid_right, trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING)
#ir_mid_left.irq(handler=obstical_mid_left, trigger=Pin.IRQ_FALLING) # obstacle
#ir_mid_right.irq(handler=obstical_mid_right, trigger=Pin.IRQ_FALLING) # obstacle
#ir_mid_left.irq(handler=obstical_mid_left, trigger=Pin.IRQ_RISING) # no obstacle
#ir_mid_right.irq(handler=obstical_mid_right, trigger=Pin.IRQ_RISING) # no obstacle

# ----------------------------------
#  Set wheel turn sensor interrupts
# ----------------------------------

# - Front
front_left_wheel.irq(handler=front_left_wheel_counter, trigger=Pin.IRQ_RISING)
front_right_wheel.irq(handler=front_right_wheel_counter, trigger=Pin.IRQ_RISING)

# - Rear
# rear_left_wheel.irq(handler=rear_left_wheel_counter, trigger=Pin.IRQ_RISING)
# rear_right_wheel.irq(handler=rear_right_wheel_counter, trigger=Pin.IRQ_RISING)

# ---------------------------------
#  Configure timer interrupt event
# ---------------------------------

# timer.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)

# -------------------
#  UART RX interrupt
# -------------------

# uart.irq(UART.RX_ANY, priority=5, handler=process_rx, wake=machine.IDLE)

# ===================
#  Execution Control
# ===================
try:
    
    # Start reading compass to monitor heading 
    start_heading_monitor()
    
    # Test servo
    cycle_servo()
    
    # Start main loop
    main()
    # test_setup()
    #timer.deinit()
    print('*** Execution Complete ***')
    timer.deinit()
    stop_heading_monitor()
    usys.exit()
    

except KeyboardInterrupt:
    # Abort, stop motors
    stop()
    print('CTRL-C received, Abort')
    print ('*** Reset ***')
    timer.deinit()
    stop_heading_monitor()
    machine.reset()

except Exception as ex:
    # Exception, stop motors
    stop()
    print('An exception as occurred!')
    print(ex)
    print('*** End Execution ***')
    timer.deinit()
    stop_heading_monitor()
    usys.exit()

except:
    # Exception, stop motors
    stop()
    print('An error has occurred.')

finally:
    # Cleanup, stop motors
    stop()
    timer.deinit()
    stop_heading_monitor()
    usys.exit()
