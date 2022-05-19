import micropython
micropython.alloc_emergency_exception_buf(100)

# ================================================
#  NavBot control code
#  MicroPython
#  By Johannes van Schalkwyk, all rights reserved
# ================================================

# =================
#  Import Modules:
# =================

from machine import Pin, UART, I2C, PWM, Timer
import _thread
import utime
import math
import json
import sys

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

uart = UART(0, baudrate=9600, parity=None, stop=1, bits=8, tx=Pin(0), rx=Pin(1))
# init() method was dropped in the rp2 port.
# uart = UART(uart_num, 9600, parity=None, stop=1, bits=8, rx=rxPin, tx=txPin)
# init is performed at instantiation

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

# ---------------------------------
#  Motor rotation sensor interface
# ---------------------------------
# Powerred by 5v bus of L298N
# Requires Logic Level Converter to convert signal to 3.3v (4 channels)

front_left_motor = Pin(6, Pin.IN) #  blue	
front_right_motor = Pin(7, Pin.IN) # dark purple
rear_left_motor = Pin(8, Pin.IN) # white
rear_right_motor = Pin(9, Pin.IN) # grey

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
left_pwm = PWM(Pin(16)) # L298N ENA blue left 
left_pwm.freq(50) # below 100Hz recommended for best performance
right_pwm = PWM(Pin(17)) # L298N ENB brown right
right_pwm.freq(50)

# Forward & reverse control
right_reverse = Pin(18, Pin.OUT) # L298N IN4 red right reverse
right_forward = Pin(19, Pin.OUT) # L298N IN3 orange right forward
left_forward = Pin(20, Pin.OUT) # L298N IN2 yellow left forward
left_reverse = Pin(21, Pin.OUT) # L298N IN1 green left reverse

# -----------------------
#  Servo interface - PWM
# -----------------------
# Powered by 5v bus of L298N
# Pico output signal (3.3v), no Logic Level Converter required

servo_pwm = PWM(Pin(22)) # yellow
servo_pwm.freq(50)

# ----------------------
#  Define timer objects
# ----------------------

timer_1 = Timer()
timer_2 = Timer()

# -----------------------------
#  Initialise global variables
# -----------------------------

# Note:
# Variables declared outside of a function are global by default.
# Variables declared inside a function are local by default.
# Use the "global" keyword to read and write a global variable inside a function.

second_thread = True
obstacle = False
front_left_motor_count = 0
front_right_motor_count = 0
rear_left_motor_count = 0
rear_right_motor_count = 0
avg_heading = 0
leg_num = 0
drive_state = ''

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
    
def reset_obstacle():
    global obstacle
    obstacle = False

# # Navigation state

# def switch_nav_mode():
#     pass

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
# Process data received on UART-0 channel.
#

# def process_rx():
    
#     global route_cmd
#     global start_cmd
#     global stop_cmd
#     bt_rx_cmd = {}

#     bt_rx_cmd = receive_data()

#     print("Command received: ", bt_rx_cmd)
    
#     # msg_body = {
#     #   'type' : 'NAV',
#     #   'data' : [{'leg' : 99, 'head' : 999, 'dist' : 999} ... ]
#     # }
#     # msg_body = {'type' : 'GO','data' : []}
#     # msg_body = {'type' : 'STOP','data' : []}

#     if bt_rx_cmd['type'] == 'NAV':
#         route_cmd = bt_rx_cmd['data']
#         print("Route data: ", route_cmd)
        
#     elif bt_rx_cmd['type'] == 'GO':
#         start_cmd = True
#         stop_cmd = False
#         print("Go: start - ", start_cmd, "stop: ", stop_cmd)

#     elif bt_rx_cmd['type'] == 'STOP':
#         stop_cmd = True
#         start_cmd = False
#         print("STOP: start - ", start_cmd, "stop: ", stop_cmd)
    
# -------------------------------------
#  Wheel rotation sensor slot counters
# -------------------------------------

def front_left_motor_counter(pin):
    global front_left_motor_count 
    front_left_motor_count += 1

def front_right_motor_counter(pin):
    global front_right_motor_count
    front_right_motor_count += 1

def rear_left_motor_counter(pin):
    global rear_left_motor_count
    rear_left_motor_count += 1

def rear_right_motor_counter(pin):    
    global rear_right_motor_count
    rear_right_motor_count += 1

# ------------------------------------
#  Handlers for IR obstacle detection
# ------------------------------------

# When an obstacle is detected set obstacle flag

def set_obstacle(pin):
    global obstacle
    obstacle = True
    print("*** IRQ source: ", pin)

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

# -----
#  LED
# -----
def blink(num, on, off):
    for i in range(num):
        led.value(1)
        utime.sleep(on)
        led.value(0)
        if i < num - 1:
            utime.sleep(off)

# ---------------------
#  Bluetooth Functions
# ---------------------

def send_data(tx_data):
    tx_data_json = json.dumps(tx_data)
    # bt_data = b'{$tx_data_json} + \r\n'
    bytes_sent = uart.write(tx_data_json)
    # write(buf) = write a buffer of bytes to bus 
    # 7 or 8 bit characters use one byte
    # 9 bit characters use two bytes
    # buf must have an even number of bytes
    # Returns the number of bytes writen

def receive_data():
    # read() = read all available characters
    # read(10) = read 10 characters, returns a bytes object
    # readline() = reads a line, ending in a newline character
    # readinto(buf) = read and store into given buffer
    # any() = returns number of characters waiting
    # on timeout none is returned
    
    rx_data = uart.readline()
    print("rx_data: ", rx_data, type(rx_data))
    
    if rx_data is None:
        return ''
    
    return json.loads(rx_data)

def send_progress(timer_2):
    progress = {}
    progress = {
        'leg_n' : leg_num,
        #'leg_h' : leg_heading, # why? As Command knows the route all it is the leg to know the headingneeds
        #'leg_d' : leg_distance, # why?
        'state' : drive_state,
        't_head' : avg_heading,
        #'t_dist' : avg_distance, # why? can work this out from the counters
        'lf_c' : front_left_motor_count,
        'rf_c' : front_right_motor_count,
        'lr_c' : rear_left_motor_count,
        'rr_c' : rear_right_motor_count,
        's_dist' : sonic_distance
    }

    send_data(progress)
    print("Progress message: ", progress)

# ------------------------
#  Sonic Sensor Functions
# ------------------------

def get_distance():
    
    global distance
    duration = 0
    distance = 0
    signaloff = 0
    signalon = 0
    
    #Pause for two milliseconds to ensure 
    # the previous setting has completed
    utime.sleep_us(2)
    
    trigger.high()
    # After pulling the high potential,
    # wait for 5 milliseconds,
    # and immediately set to Low
    utime.sleep_us(5)
    trigger.low()

    # Create a while loop to check whether the  
    # echo pin is 0 and record the time
    while echo.value() == 0:
        signaloff = utime.ticks_us()

    # Create a while loop to check Whether the echo 
    # pin value is 1 and then record the time
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

def sonic_sense(timer_1):
        
    # Determine current free distance
    global sonic_distance
    sonic_distance = get_distance()
    print('---> Sensor distance is: ', sonic_distance, 'cm')
    
    # If free distance is limited stop
    if sonic_distance < 5:
        stop()
        #suspend timer
        timer_1.deinit()
        led.on()
        print('obstacle within 5cm')
        utime.sleep(0.5)
        led.off()
        while sonic_distance < 5:
            led.on()
            utime.sleep(0.5)
            led.off()
            utime.sleep(0.5)
            distance = get_distance()
        #start timer again
        timer_1.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)

# -------------------------
#  Servo Control Functions
# -------------------------

def cycle_servo():
    
    print("--- Test servo")

    for position in range (1150, 8650, 50): # range(start, stop, step)
        servo_pwm.duty_u16(position)
        utime.sleep(0.02)

    for position in range (8650, 1150, -50): # range(start, stop, step)
        servo_pwm.duty_u16(position)
        utime.sleep(0.02)
        
    angle_servo(90)
    
    print("--- Servo test complete")

def angle_servo(angle):

    # Set servo parameter values
    max_degree = 180
    min_degree = 0
    # Pico PWM can be set between 0 and 65536 (16-bit logical level)
    # The SG90 servo uses 1000 to 9000 for all the available angles
    max_duty = 9000
    min_duty = 1000
    # But it does not go through the full 180 degrees
    # experimented with max_duty = 8650 min_duty = 1150
    
    # Expanded calculation of duty cycle required for the angle specified in degrees
    duty_range = max_duty - min_duty
    deg_range = max_degree - min_degree
    duty_per_deg = duty_range // deg_range
    angle_duty = (duty_per_deg * angle) + min_duty
       
    print("Position ", angle, angle_duty)

    # Set servo angle according the calculated duty cycle
    servo_pwm.duty_u16(angle_duty)

    # Wait for servo to set
    utime.sleep(0.5)

# -------------------------
#  Motor Control Functions
# -------------------------

# Speed Control

def set_speed(speed):
    
    # Forward drive speed settings are slow, medium, economy and full
    # Define speed levels:
    speeds = {
        'slow' : 2,
        'medium' : 5,
        'economy' : 8,
        'full' : 10
    }
    
    # Determine speed level:
    level = speeds[speed]
    
    
    # The motor operating voltage is 3-6 volts
    # The max duty cycle is 65025, power is on 100% of the cycle, 6v. 
    # A 50% duty cycle is 32513 rounded, power is on 50% of the time, 3v.
    # levels = 10
    # max_duty = 65025 # 1111111111111111 = 65535?
    # min_duty = 32513
    # duty_range = max_duty - min_duty
    # duty = int((level/levels) * duty_range) + min_duty
    
    # Calc duty:
    duty = int((level/10) * (65025 - 32513)) + 32513
    
    # Set duty:
    left_pwm.duty_u16(duty)
    right_pwm.duty_u16(duty)
    
    print ("--- Motors PWM duty set")
    
    return duty

# Drive states

def stop():
    print("--- STOP ---")
    right_reverse.low()
    left_reverse.low()
    left_forward.low()
    right_forward.low()

def forward(speed):
    print(">>> Set FORWARD >>>")
    set_speed(speed)
    right_reverse.low()
    left_reverse.low()
    left_forward.high()
    right_forward.high()
    
def reverse(speed):
    print("<<< Set REVERSE <<<")
    set_speed(speed)
    right_forward.low()
    left_forward.low()
    left_reverse.high()
    right_reverse.high()
    
# Turn states

def turn_left():
    # Static left pivot - left motors reverse & right motors forward
    print("<<< TURN LEFT ---")
    stop()
    set_speed('slow')
    right_reverse.low()
    left_forward.low()
    left_reverse.high()
    right_forward.high()

def turn_right():
    # Static right turn - left motors forward & right motors reverse
    print("--- TURN RIGHT >>>")
    stop()
    set_speed('slow')
    right_forward.low()
    left_reverse.low()
    right_reverse.high()
    left_forward.high()
     
# ===============
#  Drive Control
# ===============
#
# Reset drive distance counters -
# This must be done before the start of a drive while the bot is still stationary
# in order to avoid conflicts between this and the IRQ of the motor turn pulse counter
#
# def reset_front_left_motor_counter():
#     global front_left_motor_count
#     front_left_motor_count = 0
# 
# def reset_front_right_motor_counter():
#     global front_right_motor_count
#     front_right_motor_count = 0
# 
# def reset_rear_left_motor_counter():
#     global rear_left_motor_count
#     rear_left_motor_count = 0
# 
# def reset_rear_right_motor_counter():
#     global rear_right_motor_count
#     rear_right_motor_count = 0

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
# def calc_clicks(distance_cm):
#     #print ("--- Convert distance to clicks")
#     distance_mm = distance_cm * 10
#     clicks = int(distance_mm / calc_click_distance())
#     #print ("=== The distance of ", distance_cm, "cm equals ", clicks, " sensor clicks")
#     return clicks

# #
# #  Convert clicks counted to distance travelled
# #
# def calc_distance(clicks):
#     distance_mm = clicks * calc_click_distance()
#     distance_cm = distance_mm / 10
#     return distance_cm


def target_drive(target_distance):
    #
    #  Drive on target heading for required distance
    #
    # Note:- Distance is in clicks
    
    print("--- Start drive to target")
    
    near_target = False
    avg_distance = 0
    
    # Reset motor counters
    global front_left_motor_count
    front_left_motor_count = 0
 
    global front_right_motor_count
    front_right_motor_count = 0

    global rear_left_motor_count
    rear_left_motor_count = 0
 
    global rear_right_motor_count
    rear_right_motor_count = 0
    
    print ("--- Counters reset")
    
    # Drive forward
    forward('economy')
    print ("--- Forward drive started")
        
    # Continue while average distance is less than target distance
    while avg_distance < target_distance:
        
        if obstacle:
            
            # Obstacle flag set, stop target drive
            stop()
            
            # A detour from original couse is required to avoid an obstacle
            print("--- Obstacle encountered, target drive suspended")
            reset_obstacle()
            break
        
        if (avg_distance / target_distance) * 100 > 80.0 and not near_target:
            
            # Slow down when 80% complete  
            near_target = True
            forward('medium')
            print("--- Slowed down, almost at target")
        
        avg_distance = (
                front_left_motor_count +
                front_right_motor_count +
                rear_left_motor_count +
                rear_right_motor_count) / 4

    # End of while, destination reached
    stop()
    
    print("==> Average click count: ", avg_distance)
    
    return avg_distance

def detour_drive(side):
    # If the bot made a turn to the right the target is on the left.
    # Drive untill the left side is not blocked.
    # If the bot made a turn to the left the target is on the right.
    # Drive untill the right side is not blocked.
    
    target_blocked = True

    print("=== Detour Drive Started ===")
        
    # Reset motor counters to record detour drive distance
    global front_left_motor_count
    front_left_motor_count = 0
 
    global front_right_motor_count
    front_right_motor_count = 0

    global rear_left_motor_count
    rear_left_motor_count = 0
 
    global rear_right_motor_count
    rear_right_motor_count = 0
                          
    forward('slow')

    while target_blocked:
        
        if obstacle:
            # IRQ - obstacle flag set, stop detour drive
            # Another detour required to avoid an obstacle
            print("--- Obstacle encountered, suspend current detour")
            reset_obstacle()
            
            break
        
        # Read side sensor state
        # Sensor output:
        #   0 = low, obstacle
        #   1 = high, no obstacle (5v on sensor line)
        
        if side == "left" and ir_mid_left.value() == 1:
            target_blocked = False
            
        if side == "right" and ir_mid_right.value() == 1:
            target_blocked = False    
        
    stop()
    # Record distance travelled on detour
    avg_distance = (
                front_left_motor_count +
                front_right_motor_count + 
                rear_left_motor_count +
                rear_right_motor_count) / 4
        
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

def deviation_heading(turn_angle):
    
    # Read current compass heading
    current_heading = read_compass()
    
    # Calculate detour compass heading
    if (current_heading + turn_angle) > 360:
        heading = (current_heading + turn_angle) - 360
    elif (current_heading + turn_angle) < 0:
        heading = 360 + (current_heading + turn_angle)
    else:
        heading = current_heading + turn_angle

    print("Detour heading: ", heading)
    
    return heading

def best_deviation_angle():
    
    # The deviation angle is the angle of change in
    # in the current heading direction
    
    # Stop scan timer
    timer_1.deinit()
    
    print("---> Sonic scan timer stopped")

    free_distance = 0
    best_distance = 0
    best_angle = 0
    
    # Find best detour direction using the sonic sensor to scan
    # turn angle with most open space in steps of 10 degrees
    for scan_angle in range ( 10, 170, 10):
        # the actual scan arc is not 180 on the servo, don't know why
        
        print("Scan angle: ", scan_angle)
        
        angle_servo(scan_angle)
        free_distance = get_distance()
        deviation_angle = scan_angle - 90
        
        print("Deviation angle: ", deviation_angle, "Free distance: ", free_distance)
        
        if free_distance > best_distance:
            best_angle = deviation_angle
            best_distance = free_distance
                
    print(f"The best deviation angle is {best_angle} deg, with a free distance of {best_distance} cm")

    # set to scan straight ahead
    angle_servo(90)

    # Start scan timer again
    timer_1.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)
    
    print("--- Sonic scan timer started")

    return best_angle

def turn_to_heading(req_heading):
    
    print ("--- Start turn to heading ---")
    
    ####################
    # For testing only #
    ####################
    req_heading = 90

    # Execute turn
    
    current_heading = read_compass()
    print ("---> Compass heading: ", current_heading)
    
    if current_heading > req_heading:
        
        if req_heading < current_heading - 180:
            while read_compass() != req_heading:
                turn_right()
            stop()
            
        else:
            while read_compass() != req_heading:
                turn_left()
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
        
        print("--- On course, no turn required")
    
    return

def verify_heading(req_heading, distance_remaining, current_speed):

    # This function must be called by drive_target() while driving
    # to verify drive heading. If there is a deviation correct heading
    # by slowing either the left motors for a left correction or right
    # motors for a right correction. This should happen smoothly without
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
            turn_left()
            
        elif req_heading > heading:
            turn_right()
    
    set_speed(current_speed)

def target_position(deviation_angle, detour_distance, target_distance, obstacle_side):
    
    # a) Calculate target position (heading & distance) from detour position.
    #    This will be the new dead reconning point.
    # b) If the detour is successfull in avoided the obstacle the NavBot will set
    #    target mode and drive to target as per the target data from this point.
    # c) If not, a new detour will start from this point and the deviation
    #    from target heading will be calculated from this point.
    
    # For the detour angle and travel distance how far did the navbot deviate
    # from the original target heading? This is the side opposite the detour angle 
    # of the detour triangle. This triangle is formed by the dead reconning point's
    # distance to target, the detour angle, the detour travel distance and the 
    # distance of the detour position from the target. The oposite side will be
    # the new distance to target. The law of cosines will be used to calculate this
    # new distance and heading. The new heading will be used to calculate the turn
    # angle the NavBot must execute to point towards the target.
    #
    #                new target heading
    #                    and distance
    #  target position ________________ detour position
    #                  \              /
    #                   \            /
    #                    \          /
    #  Distance remaining \        /  detour heading
    #  to target & heading \      /    & distance
    #                       \    /
    #                        \  /
    #                         \/
    #              detour point (Last dead reconning point)
    #                  and the detour deviation angle
    #
        
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
        new_target_distance = math.sqrt(target_distance**2 + detour_distance**2 - 2 * target_distance * detour_distance * math.cos(math.radians(deviation_angle))) 
    
    print(":--> Target distance: ", new_target_distance)
          
    # calc detour position angle
    detour_position_angle = math.degrees(math.acos((new_target_distance**2 + detour_distance**2 - target_distance**2) / (2 * new_target_distance * detour_distance)))
    
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
    new_target_heading = deviation_heading(target_turn_angle)
    
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
        'heading': new_target_heading,
        'distance': new_target_distance}
    
    return target_position

def detour(target_heading, target_distance):
    
    print ('*** Detour Started ***')

    global front_left_motor_count
    global front_right_motor_count
    global rear_left_motor_count
    global rear_right_motor_count

    target_pos = {}

    # Plan and drive detour
    target_blocked = True
    
    while target_blocked:
               
        # Determine direction with best free distance
        print("Determine best detour heading")
        
        deviation_angle = best_deviation_angle()
        # The angle by which the detour will deviate from the current heading
        # This angle is required to calculate the distance to target using
        # the Cosine rule
        
        print(":--> Deviation angle: ", deviation_angle)
                
        # Determine detour heading, current heading + detour angle
        # Use compass to determine current heading
        detour_heading = deviation_heading(deviation_angle)
        
        print(":--> Best detour heading: ", detour_heading)
        
        turn_to_heading(detour_heading)
        
        print(":--> Turned to detour heading")
                
        # Determine obstacle side
        if deviation_angle > 0:
            # turning right, target would be to the left
            target_side = "left"
        else:
            # turning left, target would be to the right 
            target_side = "right"
        
        print( ":--> Target side: ", target_side)
        
        # Drive on detour heading
        target_blocked = detour_drive(target_side)
        
        # If the detour drive ends and the target side is still blocked an
        # obstacle was encountered during the detour drive.
        # Calculate a new target heading and start a new detour
        
        # Calculate average clicks recorded during detour drive
        detour_distance = (
            front_left_motor_count +
            front_right_motor_count + 
            rear_left_motor_count +
            rear_right_motor_count) / 4
            
        # What is the heading and distance to the target at
        # the current detour location?
        # If the detour was successful this will be the target
        # position information used to drive to the target.
        # If not this will be the dead reconning point form
        # which the next detour drive will start from
        target_pos = target_position(
                deviation_angle,
                detour_distance,
                target_distance,
                target_side)
                
        target_heading = target_pos['heading']    
        target_distance = target_pos['distance']
        
        print( ":--> New target heading: ", target_heading, " and distance: ", target_distance)
            
    print( ":--> Detour avoided, resume drive to target")    
    
    return target_pos

def drive_leg(target):
    
    global drive_state
    drive_state = 'target'
    
    heading = target['head']
    distance_cm = target['dist']
    leg_state = ''
        
    # Convert distance in cm to distance in clicks
    # From here onward distances will be in clicks
    # ============================================
    distance = int((distance_cm * 10) / calc_click_distance())
    
    while leg_state != 'complete':

        # Turn to required heading
        print("---> Current heading: ", avg_heading)
        turn_to_heading(heading)

        # Drive to target
        print ('---> Drive to target')
        drive_dist = target_drive(distance)
                
        if drive_dist != distance:
            
            # If distance is greater then target was not reached
            # because an obstacle was encountered.
            # A detour action is required.
                        
            print( "---> Target direction obstructed, detour required")
            drive_state = 'detour'
            
            # The remaining distance is required to calculate
            # target heading and distance from detour point
            remaining_dist = distance - drive_dist
            
            # Execute detour and return heading and distance
            # to target once obstacle has been avoided. This
            # will be used to start a new target drive
            new_target = detour(heading, remaining_dist)
            
            heading = new_target['heading']
            distance = new_target['distance']
            drive_state = 'target'
            
        else:
            print ('--- Target Reached ---')
            leg_state = 'complete'

    return leg_state



def main():
    
    # When Navbot starts it waits for connection message
    # from the Command module. After the Command module
    # connects it will send this message to the Control
    # module running on the NavBot.
    #  
    # After receiving the connected message the Control
    # module will wait for route data. After receiving
    # the route data message from the Command module the
    # Control module will wait for a go message to start
    # the drive.
    # 
    # On receiving the go command the Control module will
    # start executing the routing instructions received 
    # driving the NavBot towards the target.
    # 
    # During the drive the Navbot will transmit progress
    # data back to command module. The Command module
    # will display this data.
    #
    # The stop command is not implemented at this stage
    # as MicroPython does not provide for an UART IRQ
    
    # Types of Command Messages:
    # (a) Connection msg_body = {'type' : 'CON','data' : []}
    # (b) Navigation data msg_body = {
    #       'type' : 'NAV',
    #       'data' : [{'leg' : 99, 'head' : 999, 'dist' : 999} ... ]
    #     }
    # (c) Go msg_body = {'type' : 'GO','data' : []}
    # (d) Stop msg_body = {'type' : 'STOP','data' : []}

    print ('--- Main START ---')

    utime.sleep(4)

    global bot_state
    bot_state = ''
    con_cmd = False
    nav_cmd = False
    go_cmd = False
    command_cmd = {}
    route_cmd = {}
            
    # Ready for connection from command module
    bot_state = 'Command'
    print("Ready for command connection")
    print("Waiting for connection confirmation command")

    while con_cmd == False:
        # Indicate by slow/fast blinking LED - 
        # blink(num, time on, time off)
        blink(1, 0.3, 0)
        
        command_cmd = receive_data()

        if command_cmd != '':
            print("Command received: ", command_cmd)

            if command_cmd['type'] == 'CON':
                con_cmd = True
        else:    
            utime.sleep(2)

    # Ready to receive route data
    bot_state = 'Connected'
    print("Command module connected")  
    print("Waiting for route data")

    while nav_cmd == False:
        # Indicate by slow/fast blinking LED
        blink(2, 0.3, 0.2)
        
        command_cmd = receive_data()

        if command_cmd != '':
            print("Command received: ", command_cmd)

            if command_cmd['type'] == 'NAV':
                nav_cmd = True
                route_cmd = command_cmd['data']
                print("Route Data Received: ", route_cmd)
        
        else:    
            utime.sleep(2)
    
    # Ready to receive go command
    bot_state = 'Ready'
    print("Route data received")  
    print("Ready for GO command")

    while go_cmd == False:
        # Indicate by slow/fast blinking LED
        blink(3, 0.3, 0.2)
        
        command_cmd = receive_data()

        if command_cmd != '':
            print("Command received: ", command_cmd)

            if command_cmd['type'] == 'GO':
                go_cmd = True
                        
        else:    
            utime.sleep(2)
    
    # Start executing route
    bot_state = 'go'
    print("Go command received")  
    print("Drive route")
    
    # Start reading compass to monitor heading 
    start_heading_monitor()
    print('---> Heading monitor started')
    utime.sleep(1)
    print('---> Heading: ', avg_heading)
    
    # Test servo
    cycle_servo()
    
    # Configure timer interrupt event for sonic sensor
    timer_1.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)
    print('---> Sonic timer started')
    utime.sleep(2)

    # Configure timer interrupt event for progress update
    timer_2.init(freq=1, mode=Timer.PERIODIC, callback=send_progress)
    print('---> Progress timer started')
    utime.sleep(2)
    
    # Start navigating drive
    for leg in route_cmd:

        print('---> Leg: ', leg)
        
        global leg_num #, leg_heading, leg_distance
        leg_num = leg['leg']
        # leg_heading = leg['head'] From leg Command knows this as it knows the route
        # leg_distance = leg['dist'] From leg Command knows this as it knows the route
        
        # Drive to leg target
        leg_state = drive_leg(leg)

        # Process leg drive result
        if leg_state == "complete":
            print (f'---> Leg {leg_num} complete')

        elif leg_state == "halt":
            print (f'--- Failed to reach leg {leg_num} objective ---')
            break

        else:
            print ('--- ERROR - unexpected end to route ---')
            break

    print ('--- Main END ---')

# ===================
#  Execution Control
# ===================
def navbotMain():
    print('=== NavBot Start ===')
    try:
        # Start main loop
        print('--- Start Main Loop ---')
        main()
        print('--- Main Loop Complete ---')
        timer_1.deinit()
        timer_2.deinit()
        stop_heading_monitor()
        sys.exit()

    except Exception as ex:
        # Exception, stop motors
        stop()
        print('An exception as occurred!')
        print(ex)
        print('=== End Execution ===')
        timer_1.deinit()
        timer_2.deinit()
        stop_heading_monitor()
        sys.exit()

    except:
        # Exception, stop motors
        stop()
        print('An error has occurred.')

    finally:
        # Cleanup, stop motors
        stop()
        timer_1.deinit()
        timer_2.deinit()
        stop_heading_monitor()
        #sys.exit()

def direct(): 
    while True:
        print('=== NavBot Start ===')
        try:
            # Start main loop
            print('--- Start Main Loop ---')
            main()
            print('--- Main Loop Complete ----')
            timer_1.deinit()
            timer_2.deinit()
            stop_heading_monitor()
            sys.exit()

        except:
            # Exception, stop motors
            print('An error has occurred.')

        finally:
            # Cleanup, stop motors
            stop()
            timer_1.deinit()
            timer_2.deinit()
            print('--- Timers 1 & 2 stopped ---')
            stop_heading_monitor()
            print('--- Compass monitor stopped ---')
            sys.exit()

# ===============================
#  Interrupts Definition Section
# ===============================

# -------------------------------------
#  Setup IR obstacle sensor interrupts
# -------------------------------------
# Monitor front obstacle sensors only.
# The mid sensors are monitored during a detour drive and
# the rear sensors will be monitored when reversing
ir_front_left.irq(handler=set_obstacle, trigger=Pin.IRQ_FALLING, hard=True) # obstacle
ir_front_centre.irq(handler=set_obstacle, trigger=Pin.IRQ_FALLING, hard=True) # obstacle
ir_front_right.irq(handler=set_obstacle, trigger=Pin.IRQ_FALLING, hard=True) # obstacle

# ----------------------------------
#  Set motor turn sensor interrupts
# ----------------------------------

# - Front
front_left_motor.irq(handler=front_left_motor_counter, trigger=Pin.IRQ_RISING, hard=True)
front_right_motor.irq(handler=front_right_motor_counter, trigger=Pin.IRQ_RISING, hard=True)

# - Rear
rear_left_motor.irq(handler=rear_left_motor_counter, trigger=Pin.IRQ_RISING, hard=True)
rear_right_motor.irq(handler=rear_right_motor_counter, trigger=Pin.IRQ_RISING, hard=True)

# -------------------
#  UART RX interrupt - not available
# -------------------
# uart.irq(UART.RX_ANY, priority=5, handler=process_rx, wake=machine.IDLE)
# MicroPyton does not implement this for the Pico. Only availabe for WiPy.

if __name__ == "__main__":
    print("%s is being run directly" %__name__)
    direct()
else:
    print("%s is being imported" %__name__)