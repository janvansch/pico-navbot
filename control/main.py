import micropython

micropython.alloc_emergency_exception_buf(100)

# ================================================
#  NavBot control code
#  NavBot is an auto-navigating experiment
#  MicroPython v1.18
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

# Connect GP0 (UART0 Tx) to Rx of HC-05/06 Bluetooth module (brown)
# Connect GP1 (UART0 Rx) to Tx of HC-05/06 Bluetooth module (orange)
# Module powered by 5v bus, but RX and TX powered by internal 3.3v regulator

# Setup UART channel 0 (TX = GP0 & RX = GP1), with baud rate of 9600
# uart = UART(uart_num, 9600, parity=None, stop=1, bits=8, rx=rxPin, tx=txPin)
uart = UART(0, baudrate=9600, parity=None, stop=1, bits=8, tx=Pin(0), rx=Pin(1))

# The init() method was dropped in the rp2 port.
# init is performed at instantiation

# --------------------------------
#  I2C channel for Compass module
# --------------------------------
# Powered by 3.3v bus
# Does not require Logic Level Converter to convert sda to 3.3v
# sda = green
# scl = grey
# Compass module address = 0x1e

i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)

# ------------------------
#  Sonic sensor interface
# ------------------------
# Powered by 5v bus
# Requires Logic Level Converter to convert echo signal to 3.3v (1 channel)

trigger = Pin(4, Pin.OUT)  # brown
echo = Pin(5, Pin.IN)  # white

# ---------------------------------
#  Motor rotation sensor interface
# ---------------------------------
# Powered by 5v bus of L298N
# Requires Logic Level Converter to convert signal to 3.3v (4 channels)

rear_left_motor = Pin(8, Pin.IN)  # white
rear_right_motor = Pin(9, Pin.IN)  # grey
front_left_motor = Pin(14, Pin.IN)  #  blue
front_right_motor = Pin(15, Pin.IN)  # dark purple

# ---------------------
#  IR sensor interface
# ---------------------
# Powered by 5v bus
# Requires Logic Level Converter to convert signal to 3.3v (6 channels)

ir_mid_left = Pin(6, Pin.IN)  # purple
ir_mid_right = Pin(7, Pin.IN)  # light grey
ir_rear_centre = Pin(10, Pin.IN)  # blue
ir_front_left = Pin(11, Pin.IN)  # orange
ir_front_centre = Pin(12, Pin.IN)  # yellow
ir_front_right = Pin(13, Pin.IN)  # green

# -----------------------------------
#  Motor Controller (L298N) interface
# -----------------------------------
# Remember to connect L298N GND to Pico GND because it is
# powered by a separate battery but signal wires are shared
# Pico output signal (3.3v), no Logic Level Converter required

# Speed control - PWM
left_pwm = PWM(Pin(16))  # L298N ENA blue left
left_pwm.freq(50)  # below 100Hz recommended for best performance
right_pwm = PWM(Pin(17))  # L298N ENB brown right
right_pwm.freq(50)

# Forward & reverse control
right_forward = Pin(18, Pin.OUT)  # L298N IN4 red right forward
right_reverse = Pin(19, Pin.OUT)  # L298N IN3 orange right reverse
left_reverse = Pin(20, Pin.OUT)  # L298N IN2 yellow left reverse
left_forward = Pin(21, Pin.OUT)  # L298N IN1 green left forward

# -----------------------
#  Servo interface - PWM
# -----------------------
# Powered by 5v bus of L298N
# Pico output signal (3.3v), no Logic Level Converter required

servo_pwm = PWM(Pin(22))  # yellow
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

second_thread = False
front_obstacle = False
rear_obstacle = False
left_obstacle = False
right_obstacle = False
front_left_motor_count = 0
front_right_motor_count = 0
rear_left_motor_count = 0
rear_right_motor_count = 0

leg_num = 0
sonic_distance = 0
wheel_diameter_cm = 6.5  # cm
encoder_slots = 20
drive_state = ""
# Define speed levels
speeds = {"crawl": 1, "slow": 3, "half": 5, "economy": 7, "full": 9, "flank": 10}

# ---------------------------------------------
#  Initialise HMC5883L magnetometer constants:
# ---------------------------------------------

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
# -25degrees 33minutes 25.55 = -0.4459 radians

# Set pi value
PI = 3.14159265359

# HMC5883L magnetometer device address
DEVICE_ADDRESS = 0x1E

# -----------------------------------
#  Initialise HMC5883L magnetometer:
# -----------------------------------
# Write to Configuration Register A
i2c.writeto_mem(DEVICE_ADDRESS, REGISTER_A, b"\x70")

# Write to Configuration Register B for gain
i2c.writeto_mem(DEVICE_ADDRESS, REGISTER_B, b"\xa0")

# Write to mode Register for selecting mode
i2c.writeto_mem(DEVICE_ADDRESS, REGISTER_MODE, b"0")

# ========================
#  Second Thread Function
# ========================
# def monitor_heading():

#     global avg_heading

#     count = 0
#     sum_heading_angles = 0

#     while second_thread:
#         # Read Accelerometer raw value
#         x = read_raw_data(X_AXIS_H)
#         # z = read_raw_data(Z_AXIS_H)
#         y = read_raw_data(Y_AXIS_H)

#         heading = math.atan2(y, x) + DECLINATION

#         # Due to declination check for >360 degree
#         if heading > 2 * PI:
#             heading = heading - 2 * PI

#         # Check for sign
#         if heading < 0:
#             heading = heading + 2 * PI

#         # Convert into angle
#         heading_angle = heading * 180 / PI

#         count += 1

#         sum_heading_angles = sum_heading_angles + heading_angle

#         if count == 10:
#             avg_heading = round(sum_heading_angles / count, 1)

#             count = 0
#             sum_heading_angles = 0

#             # start_time = utime.ticks_ms()
#             # interval = 500
#             # while utime.ticks_ms() - start_time < interval:
#             #    pass
#         utime.sleep(0.2) #is this blocking, e.g. irq

# =========
#  Helpers
# =========

# # Start heading monitor as second thread
# def start_heading_monitor():
#     print("---> Start heading monitor in second thread")
#     global second_thread
#     second_thread = True
#     _thread.start_new_thread((monitor_heading), ())


# # Trigger second thread stop
# def stop_heading_monitor():
#     global second_thread
#     second_thread = False
#     _thread.exit()
#     print("---> Second thread with heading monitor stopped")


# Reset obstacle flags
def reset_front_obstacle():
    global front_obstacle
    front_obstacle = False


def reset_rear_obstacle():
    global rear_obstacle
    rear_obstacle = False


def reset_left_obstacle():
    global left_obstacle
    left_obstacle = False


def reset_right_obstacle():
    global right_obstacle
    right_obstacle = False


# Reset travel distance counters
def reset_counters():
    global front_left_motor_count
    front_left_motor_count = 0

    global front_right_motor_count
    front_right_motor_count = 0

    global rear_left_motor_count
    rear_left_motor_count = 0

    global rear_right_motor_count
    rear_right_motor_count = 0

# ====================
#  Interrupt Handlers
# ====================

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

def set_front_obstacle(pin):
    global front_obstacle

    if front_obstacle == False:
        front_obstacle = True
        print("---> IRQ source: ", pin)

def set_rear_obstacle(pin):
    global rear_obstacle

    if rear_obstacle == False:
        rear_obstacle = True
        print("---> IRQ source: ", pin)

def clear_left_obstacle(pin):
    global left_obstacle
    if left_obstacle == True:
        left_obstacle = False
        print("---> IRQ source: ", pin)

def clear_right_obstacle(pin):
    global right_obstacle

    if right_obstacle == True:
        right_obstacle = False
        print("---> IRQ source: ", pin)

# ==================
#  Hardware Control
# ==================

# -----------------------
#  Onboard LED Functions
# -----------------------
def blink(num, on, off):
    led.value(0)
    utime.sleep(0.2)
    for i in range(num):
        led.value(1)
        utime.sleep(on)
        led.value(0)
        utime.sleep(off)
        # if not i < num - 1:
        #    utime.sleep(off)
    utime.sleep(0.2)

# ---------------------------------
#  HMC5883L Nagnetometer Functions
# ---------------------------------
def read_raw_data(addr):
    # Read raw 16-bit value
    byte_high = i2c.readfrom_mem(DEVICE_ADDRESS, addr, 1)
    byte_low = i2c.readfrom_mem(DEVICE_ADDRESS, addr + 1, 1)

    # Convert to integer
    high = int.from_bytes(byte_high, "big")
    low = int.from_bytes(byte_low, "big")

    # Concatenate higher and lower value (only works with integers)
    value = (high << 8) | low

    # To get signed value from module
    if value > 32768:
        value = value - 65536

    return value

def heading():
    # Return current heading of nav-bot
    # Heading is the average of 10 readings

    count = 0
    sum_heading_angles = 0

    while count <= 10:
        count += 1
        # Read compass raw value
        x = read_raw_data(X_AXIS_H)
        z = read_raw_data(Z_AXIS_H)
        y = read_raw_data(Y_AXIS_H)

        heading = math.atan2(y, x) + DECLINATION
        # Check for >360 degree
        if heading > 2 * PI:
            heading = heading - 2 * PI
        # Check for sign
        if heading < 0:
            heading = heading + 2 * PI
        # Convert into angle
        heading_angle = heading * 180 / PI
        
        # Sum angles to enable average heading calculation
        sum_heading_angles = sum_heading_angles + heading_angle
        utime.sleep(0.2)
        # start_time = utime.ticks_ms()
        # interval = 500
        # while utime.ticks_ms() - start_time < interval:
        #    pass
    
    # Calculate average heading angle
    avg_heading = round(sum_heading_angles / count, 1)
        
    return avg_heading

# ---------------------
#  Bluetooth Functions
# ---------------------

def send_data(tx_data):
    tx_data_json = json.dumps(tx_data)
    bytes_sent = uart.write(tx_data_json)

    # write(buf) = write a buffer of bytes to bus
    # 7 or 8 bit characters use one byte
    # 9 bit characters use two bytes
    # buf must have an even number of bytes
    # Returns the number of bytes written
    print("Bytes sent", bytes_sent)

def receive_data():
    # read() = read all available characters
    # read(10) = read 10 characters, returns a bytes object
    # readline() = reads a line, ending in a newline character
    # readinto(buf) = read and store into given buffer
    # any() = returns number of characters waiting
    # on timeout none is returned

    rx_data = uart.readline()
    # print("rx_data: ", rx_data, type(rx_data))

    if rx_data is None:
        return ""

    return json.loads(rx_data)

def send_progress(timer_2):
    progress = {}
    progress = {
        "leg_n": leg_num,
        #'leg_h' : leg_heading, # why? As Command knows the route all it needs is the leg to know the heading
        #'leg_d' : leg_distance, # why?
        "state": drive_state,
        "t_head": avg_heading,
        #'t_dist' : avg_distance, # why? can work this out from the counters
        "lf_c": front_left_motor_count,
        "rf_c": front_right_motor_count,
        "lr_c": rear_left_motor_count,
        "rr_c": rear_right_motor_count,
        "s_dist": sonic_distance,
    }

    send_data(progress)
    print("Progress message: ", progress)

# ------------------------
#  Sonic Sensor Functions
# ------------------------


def get_sonic_distance():
    duration = 0
    distance = 0
    signal_off = 0
    signal_on = 0

    # Pause for two milliseconds to ensure
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
        signal_off = utime.ticks_us()

    # Create a while loop to check Whether the echo
    # pin value is 1 and then record the time
    while echo.value() == 1:
        signal_on = utime.ticks_us()

    utime.sleep(0.2)
    # Calculate the time difference between sending and receiving
    duration = signal_on - signal_off

    # Sonic travel time x speed of sound
    # (343.2 m/s , Which is 0.0343 cm per microsecond),
    # and the back-and-forth distance is divided by 2
    distance = (duration * 0.0343) / 2

    return distance


def sonic_sense(timer_1):
    # Determine current free distance
    global sonic_distance
    sonic_distance = get_sonic_distance()
    # print("---> Sensor distance is: ", sonic_distance, "cm")

    # If free distance is limited stop
    if sonic_distance < 5:
        stop()
        # suspend timer
        timer_1.deinit()
        led.on()
        print("obstacle within 5cm")
        utime.sleep(0.5)
        led.off()
        while sonic_distance < 5:
            led.on()
            utime.sleep(0.5)
            led.off()
            utime.sleep(0.5)
            sonic_distance = get_sonic_distance()
        # start timer again
        timer_1.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)


# -------------------------
#  Servo Control Functions
# -------------------------

def cycle_servo():
    print("--- Servo cycle test")

    for position in range(1150, 8650, 50):  # range(start, stop, step)
        servo_pwm.duty_u16(position)
        utime.sleep(0.1)

    for position in range(8650, 1150, -50):  # range(start, stop, step)
        servo_pwm.duty_u16(position)
        utime.sleep(0.1)

    angle_servo(90)
    utime.sleep(0.2)

    print("--- Servo cycle test complete")


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

# Motor speed control


def calc_duty(level):
    print("--- Calculate Duty ---")

    if level < 0:
        level = 1

    if level > 10:
        level = 10

    # Convert level to duty:
    # Recommended operating voltage of motors are 3-6 volts
    # Power pack is 3.6V x 3 = 10.8V
    # Voltage drop of motor driver is ?
    # The max duty cycle is 65025 i.e. power is on 100% of the cycle. lets say 9v
    # Duty cycle for 6v would be 65025 x 6 / 9 = 43350
    # Duty cycle for 3v would be 65025 x 3 / 9 = 21675
    # Duty range is therefore 43350 - 21675 = 21675
    # Say 10 power levels is required
    # The duty cycle for level 1 (crawl) would be:
    #     (21675 x 1 / 10) + 21675 = 23842 which is 9v x 23842 / 65025 = 3.3v which is perfect as motor will stall at 3v
    # The duty cycle for level 10 would be:
    #     (21675 x 10 / 10) + 21675 = 43350 which is 9v x 43350 / 65025 = 6v
    # Need to confirm this with testing.
    # Lithium nominal 3.6v and 3.7v and 4.2v after charge
    #
    motor_max_v = 6
    motor_min_v = 3
    supply_max_v = 9  # regulated, Nominal battery supply is 10.8v with a max of 12.6
    v_drop = 1.5  # L298N
    levels = 10
    full_duty = 65025  # 1111111111111111 = 65535?
    max_duty = full_duty * motor_max_v / (supply_max_v - v_drop)
    min_duty = full_duty * motor_min_v / (supply_max_v - v_drop)
    duty_range = max_duty - min_duty
    duty = int((level * duty_range / levels) + min_duty)
    return duty


def set_speed(speed):
    # Determine forward duty level to set PWM

    print("--- Set Speed ---")

    if not speed in speeds:
        print("ERROR - Invalid speed selection. Defaulted to 'slow'")
        speed = "slow"

    # PWM level of speed:
    level = speeds[speed]

    print("---> Speed: ", speed, " Level: ", level)

    if level < 0:
        level = 1

    if level > 10:
        level = 10

    # Determine duty_cycle of level
    duty_cycle = calc_duty(level)

    print("---> Duty: ", duty_cycle)

    # Set forward duty cycle
    left_pwm.duty_u16(duty_cycle)
    right_pwm.duty_u16(duty_cycle)


# Define motor drive states


def forward(speed):
    set_speed(speed)

    # Initiate motors to turn forward
    right_reverse.low()
    left_reverse.low()
    left_forward.high()
    right_forward.high()

    print(">>> FORWARD set >>>")


def reverse(speed):
    set_speed(speed)

    right_forward.low()
    left_forward.low()
    left_reverse.high()
    right_reverse.high()

    print("<<< REVERSE set <<<")


def stop():
    right_reverse.low()
    left_reverse.low()
    left_forward.low()
    right_forward.low()

    print("--- STOPPED ---")

# Define motor turn states

def pivot_left():
    # Static left pivot - left motors reverse & right motors forward
    print("<<< PIVOT LEFT ---")
    right_reverse.low()
    left_forward.low()
    left_reverse.high()
    right_forward.high()


def pivot_right():
    # Static right turn - left motors forward & right motors reverse
    print("--- PIVOT RIGHT >>>")
    right_forward.low()
    left_reverse.low()
    right_reverse.high()
    left_forward.high()


# ======================
#  Navigation Functions
# ======================


# -------------------------
#  Calc index of data item
# -------------------------
def scan_angle_idx(scan_angle, start, width):
    return int((scan_angle - start) / width)


# ------------------------
#  Find a wide enough gap
# ------------------------
def find_gap(scan_data, side, gap_width, scan_depth):
    scan_count = 0
    total_dist = 0
    result = False
    avg_distance = 0
    last_scan_angle = 0
    scan_start = 0
    scan_end = 0
    scan_width = 7
    range_start = 6

    if side == "left":
        scan_start = 97
        scan_end = 174 + 1

    if side == "right":
        scan_start = 6
        scan_end = 83 + 1

    for scan_angle in range(scan_start, scan_end, scan_width):
        index = scan_angle_idx(scan_angle, range_start, scan_width)

        if scan_data[index]["distance"] >= scan_depth:
            scan_count += 1
            total_dist = total_dist + scan_data[index]["distance"]

        else:
            scan_count = 0
            total_dist = 0

        # print( "Distance Running Count: ", total_dist)
        if scan_count >= gap_width:
            result = True
            avg_distance = total_dist / scan_count
            last_scan_angle = scan_angle

            return result, avg_distance, last_scan_angle

    print("No gap found")
    return result, 0, 0


# -------------------------
#  Collect free space data
# -------------------------


def sonic_scan(scan_start, scan_end, scan_width):
    scan_data = []  # List of dictionary items with scan data
    count = 0  # Scan count

    for scan_angle in range(scan_start, scan_end + 1, scan_width):
        angle_servo(scan_angle)
        distance = get_sonic_distance()
        scan_data.append({"scan_point": scan_angle, "distance": distance})

    return scan_data


def best_deviation_angle():
    #
    # Determine the best angle to deviate from the current heading to avoid an obstacle.
    # If no direction is found, 0 is returned.
    # The strategy would then be to turn 90 degrees and to scan again.
    # The turn will be made in the direction with most free space.
    # If the four quarters are scanned and no gap was found then abort,
    # target can't be reached.
    #
    # Stop periodic forward scanning timer
    timer_1.deinit()

    # -------
    #  Setup
    # -------
    scan_start = 6  # degrees
    scan_end = 174  # degrees
    scan_width = 7  # degrees
    vehicle_width = 10  # number of scan widths
    # The number of scan that would cover a gap the vehicle would fit through
    scan_depth = 30  # The distance that is obstacle free
    # How far must vehicle be able to move before it can turn again

    best_angle = 0
    gap_found = False

    while gap_found != True:
        scan_data = []  # List of dictionary items with scan data
        left_outcome = []
        right_outcome = []

        # ---------------
        #  Get scan data
        # ---------------
        scan_data = sonic_scan(scan_start, scan_end, scan_width)

        # -------------------------
        #  Analyse right scan data
        # -------------------------

        print("Scan Right")

        right_outcome = find_gap(scan_data, "right", vehicle_width, scan_depth)
        # result, avg_distance, last_scan_angle

        print("Right scan result: ", right_outcome)

        # ------------------------
        #  Analyse left scan data
        # ------------------------

        print("Scan Left")

        left_outcome = find_gap(scan_data, "left", vehicle_width, scan_depth)
        # outcome = result, avg_distance, last_scan_angle

        print("Left scan result: ", left_outcome)
        if (left_outcome[0] == True) and (right_outcome[0] == True):
            if right_outcome[1] >= left_outcome[1]:
                best_angle = right_outcome[2]
                gap_found = True

            else:
                best_angle = left_outcome[2]
                gap_found = True

        elif left_outcome[0] == True and right_outcome[0] == False:
            best_angle = left_outcome[2]
            gap_found = True

        elif left_outcome[0] == False and right_outcome[0] == True:
            best_angle = right_outcome[2]
            gap_found = True

        else:
            # No gap found
            best_angle = 0
            return 0

    # -----------------
    #  Calc turn angle
    # -----------------
    turn_angle = 90 - (best_angle - ((vehicle_width / 2) * scan_width))

    # set to scan straight ahead
    angle_servo(90)

    # Restart periodic forward scanning timer
    timer_1.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)

    print("--- Sonic scan timer restarted")

    return turn_angle


# ---------------------------------------
#  Calculate new heading from turn angle
# ---------------------------------------
def calc_heading(turn_angle):
    #
    # This function calculates a new heading from
    # the current heading and the turn angle.
    #
    # The current heading is taken as the average
    # of multiple readings of the compass module.
    current_heading = avg_heading

    # Calculate heading
    heading = current_heading + turn_angle

    if heading > 360:
        heading = heading - 360

    if heading < 0:
        heading = 360 + heading

    print("New heading: ", heading)

    return heading


def calc_target_pos(deviation_angle, detour_distance, remaining_distance, obstacle_side):
    # a) Calculate target position (heading & distance) from current detour position.
    #    This will be the new dead reconning point.
    # b) If the detour is successful in avoiding the obstacle the NavBot will
    #    set target mode and drive to the calculated target position.
    # c) If not, a new detour will start from this point and the deviation
    #    from target heading will be calculated from current detour position.

    # Form the detour angle and detour travel distance how far did the NavBot deviate
    # from the original target heading? This is the side opposite the detour angle
    # of the detour triangle. This triangle is formed by the dead reconning point's
    # distance to target, the detour angle, the detour travel distance and the
    # distance of the detour position from the target. The opposite side will be
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
    #  remaining distance \        /  detour heading
    #  to target & heading \      /    & distance
    #                       \    /
    #                        \  /
    #                         \/
    #              detour point (Last dead reconning point)
    #                  and the detour deviation angle
    #

    # This function returns the target position data from the detour position at the end of the detour drive
    current_target_position = {}

    if deviation_angle > 180:
        # angle outside target triangle
        deviation_angle = 360 - deviation_angle

    if deviation_angle == 180:
        # Target heading won't change. Only the target distance which would
        # equal current target distance plus detour drive distance
        # so the calculation of a new target heading is not required
        current_target_distance = remaining_distance + detour_distance

    else:
        # target distance = distance to target from detour position
        current_target_distance = math.sqrt(
            remaining_distance**2
            + detour_distance**2
            - 2
            * remaining_distance
            * detour_distance
            * math.cos(math.radians(deviation_angle))
        )

    print(":--> Target distance: ", current_target_distance)

    # calc detour position angle
    detour_position_angle = math.degrees(
        math.acos(
            (
                current_target_distance**2
                + detour_distance**2
                - remaining_distance**2
            )
            / (2 * current_target_distance * detour_distance)
        )
    )

    print(":--> Detour position angle: ", detour_position_angle)

    # The degrees to turn, the target_turn_angle, from the current detour heading
    # to the new target heading calculated from the current detour position -
    # detour heading is a straight line, 180 degrees
    target_turn_angle = 180 - detour_position_angle

    # But in which direction?
    # If obstacle is to the left it means the NavBot turned to the right
    # to avoid the obstacle. That means the target heading is to the left
    if obstacle_side == "left":
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
    current_target_heading = calc_heading(target_turn_angle)

    print(":--> New target heading: ", current_target_heading)

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

    current_target_position["heading"] = (current_target_heading,)
    current_target_position["distance"] = current_target_distance

    return current_target_position


# =================
#  Drive Functions
# =================

def correction(side, correction):
    # Correct small heading deviations while driving by setting different left and right
    # motor speeds based on the degree of deviation.
    # If deviation is to large trigger detour mode which will do a new heading calculation

    left_duty_cycle = 0
    right_duty_cycle = 0
    speed = ""
    turn = ""

    if correction > 5:
        # Course correction required. Stop and create dead reconning point
        stop()
        global obstacle
        obstacle = True

    else:
        if correction < 3:
            speed = "half"
            turn = "gentle"

        elif correction >= 3 or correction <= 5:
            speed = "slow"
            turn = "sharp"

        # PWM level of speed:
        level = speeds[speed]

        if turn == "gentle":
            # Gentle turns are done at half speed
            if side == "left":
                left_duty_cycle = calc_duty(level - 1)
                right_duty_cycle = calc_duty(level)
            if side == "right":
                left_duty_cycle = calc_duty(level)
                right_duty_cycle = calc_duty(level - 1)

        elif turn == "sharp":
            # Sharp turns are done at slow speed
            if side == "left":
                left_duty_cycle = calc_duty(level - 1)
                right_duty_cycle = calc_duty(level + 1)
            if side == "right":
                left_duty_cycle = calc_duty(level + 1)
                right_duty_cycle = calc_duty(level - 1)

        left_pwm.duty_u16(left_duty_cycle)
        right_pwm.duty_u16(right_duty_cycle)


def verify_heading(heading, speed, clicks_remaining):
    #
    # This function must be called by target_drive() & detour_drive()
    # while driving to verify heading. If there is a deviation correct
    # heading by setting different speeds for the left motors and the
    # right motors.
    # As this is not considered to be a course correction this should happen
    # smoothly as part of the drive function without stopping. But if there
    # is a major difference then the drive should be stopped and a course
    # correction must be made.

    curr_heading = heading()
    deviance = abs(heading - curr_heading)

    while deviance > 1 and clicks_remaining > 10:
        if heading < curr_heading:
            correction("left", deviance)

        elif heading > curr_heading:
            correction("right", deviance)

    set_speed(speed)


def turn_to_heading(target_heading):
    #
    # Course correction at waypoint or at detour point.
    # This becomes a dead reconning point.
    #

    print("--- Turn to Heading ---")

    # Determine current heading
    curr_heading = heading()

    #
    # Big step turn when far from required heading.
    #
    # pulse = 150  # ms
    # set_speed("slow")
    # utime.sleep_ms(100)

    while abs(curr_heading - target_heading) > 0:
        
        if abs(curr_heading - target_heading) > 20:
            print("*** Big Step ***")
            pulse = 150  # ms
            set_speed("slow")
        else:
            print("--- Small Step ---")
            pulse = 25  # ms
            set_speed("crawl")
        #
        # Shortest turn direction
        #
        if curr_heading > target_heading:
            if curr_heading - target_heading > 180:
                pivot_right()
            else:
                pivot_left()

        elif curr_heading < target_heading:
            if target_heading - curr_heading > 180:
                pivot_left()
            else:
                pivot_right()

        utime.sleep_ms(pulse)

        stop()
        
        print(
            "Heading - Current: ",
            curr_heading,
            " - Target: ",
            target_heading,
            " - Deviation: ",
            abs(curr_heading - target_heading),
        )

        blink(2, 0.1, 0.3)

        # Get heading after turn
        curr_heading = heading()
        utime.sleep(2)

        #
        # Small step turn when close to required heading.
        #
        # pulse = 25  # ms
        # set_speed("crawl")
        #utime.sleep_ms(100)

#     while abs(curr_heading - target_heading) > 0:
#         print("Small Step")
#         print(
#             "Heading - Current: ",
#             curr_heading,
#             "Target: ",
#             target_heading,
#             "Deviation: ",
#             abs(curr_heading - target_heading),
#         )
#         #
#         # Determine shortest turn direction
#         #
#         if curr_heading < target_heading:
#             if curr_heading - target_heading > 180:
#                 print(">>> Right")
#                 turn_right()
#             else:
#                 print("<<< Left")
#                 turn_left()
# 
#         else:
#             if curr_heading - target_heading < -180:
#                 print("<<< Left")
#                 turn_left()
#             else:
#                 print(">>> Right")
#                 turn_right()

#         utime.sleep_ms(pulse)
# 
#         stop()
#         
#         blink(1, 0.1, 0.3)
# 
#         # Get heading after turn
#         curr_heading = heading()
#         utime.sleep(3)

    print("---> Compass Heading: = %d°" % (heading()))
    stop()


# -------------------------------------------
#  Drive detour heading to avoid an Obstacle
# -------------------------------------------


def detour_drive(side):
    #
    # If the bot made a turn to the right the target is on the left.
    # Drive until the left side is not blocked.
    # If the bot made a turn to the left the target is on the right.
    # Drive until the right side is not blocked.

    target_blocked = True

    # front_obstacle = False
    # rear_obstacle = False
    # if side == "left":
    #     left_obstacle = True
    # else:
    #     left_obstacle = False
    # if side == "right":
    #     right_obstacle = True
    # else:
    #     right_obstacle = False

    forward("slow")

    print("--- Detour Drive Started ---")

    while target_blocked and not front_obstacle:
        # Read side sensor state
        # Sensor output:
        #   0 = low, obstacle
        #   1 = high, no obstacle (5v on sensor line)

        # === Potential problem:
        # === The sensor may be clear but how big is the gap?

        if side == "left" and ir_mid_left.value() == 1:
            target_blocked = False  # target clear, resume drive to target

        if side == "right" and ir_mid_right.value() == 1:
            target_blocked = False  # target clear, resume drive to target

    stop()

    if front_obstacle:
        # IRQ - obstacle flag set, stop detour drive
        # Another detour required to avoid an obstacle in the detour path
        print("--- Obstacle encountered in detour path, suspend detour")

        return "obstacle"

    return "clear"


# --------------------------------------------------------------
#  Control detour and return new heading and distance to target
# --------------------------------------------------------------


def make_detour(target_heading, target_distance):
    print("=== Detour Started ===")

    detour_count = 0
    target_blocked = True
    global drive_state

    while target_blocked:
        print(":--> Start detour number: ", detour_count)

        # Determine direction with best free distance
        print("Determine best detour heading")

        deviation_angle = best_deviation_angle()
        # The angle by which the detour will deviate from the current heading
        # This angle is required to calculate the distance to target using
        # the Cosine rule

        print(":--> Deviation angle: ", deviation_angle)

        # Determine obstacle side
        if deviation_angle > 0:
            # turning right, target would then be on the left
            target_side = "left"
        elif deviation_angle < 0:
            # turning left, target would then be on the right
            target_side = "right"
        else:
            print("Target side error")
            target_side = "err"

        print(":--> Target side: ", target_side)

        # Determine detour heading, current heading + detour angle
        # Use compass to determine current heading
        detour_heading = calc_heading(deviation_angle)

        print(":--> Detour heading: ", detour_heading)

        # Execute turn to the heading that will avoid the obstacle
        turn_to_heading(detour_heading)

        print(":--> Turned to detour ", detour_count, " heading")

        # Reset obstacle flag after turning away from obstacle
        reset_front_obstacle()

        # Reset distance counters in order to determine detour drive distance
        reset_counters()

        print(":--> Distance counters reset for detour: ", detour_count)

        # Execute detour drive in direction of detour heading
        detour_result = detour_drive(target_side)

        print(":--> Detour ", detour_count, "result :", detour_result)

        # Calculate average clicks recorded during detour drive
        avg_detour_distance = (
            front_left_motor_count
            + front_right_motor_count
            + rear_left_motor_count
            + rear_right_motor_count
        ) / 4

        # Determine heading and distance to the target at
        # the current detour location?
        target_position = calc_target_pos(
            deviation_angle, avg_detour_distance, target_distance, target_side
        )

        target_heading = target_position["heading"]
        target_distance = target_position["distance"]

        print(
            ":--> New target heading: ",
            target_heading,
            " and distance: ",
            target_distance,
        )

        if detour_result == "clear":
            # Obstacle cleared. The calculated target position is
            # the target for the target drive routine

            print(":--> Obstacle avoided, resume drive to target")
            target_blocked = False

            # Return the position of the target from the detour position. This
            # will be used by the target drive routine
            return target_position

        elif detour_result == "obstacle":
            # The target position is the dead reconning point from
            # which the next detour drive will start from.
            # Determine a new detour heading and start next detour
            print(":-- Detour blocked by another obstacle")
            target_blocked = True
            detour_count += 1
            drive_state = "detour" + str(detour_count)

        else:
            print("--- ERROR - STOP ---")
            sys.exit()


# --------------------------------
#  Drive directly to target point
# --------------------------------


def target_drive(target_heading, target_clicks):
    #
    #  Drive on target heading for required distance
    #
    # Note:- Distance is in clicks

    print("--- Start drive to target")

    near_target = False
    avg_clicks = 0

    # Reset distance counters
    reset_counters()

    print("--- Counters reset")

    # Drive forward
    forward("economy")

    print("--- Forward drive started")

    # Continue drive while average distance is less
    # than target distance and no obstacle detected
    while avg_clicks < target_clicks and not front_obstacle:
        if (avg_clicks / target_clicks) * 100 > 80.0 and not near_target:
            # Slow down when 80% complete
            near_target = True
            set_speed("half")
            print("--- Slowed down, almost at target")

        avg_clicks = (
            front_left_motor_count
            + front_right_motor_count
            + rear_left_motor_count
            + rear_right_motor_count
        ) / 4

        clicks_remaining = target_clicks - avg_clicks

        verify_heading(target_heading, "economy", clicks_remaining)

    # End of while

    # Stop drive, destination reached or obstacle detected
    stop()

    if obstacle:
        # A detour from original course is required to avoid an obstacle
        print("--- Obstacle encountered, target drive suspended")
        return "obstacle"

    print(">>> Average click count: ", avg_clicks)

    return "complete"


# -----------------------------
#  Control drive to leg target
# -----------------------------


def drive_leg(target_heading, target_clicks):
    print("--- Drive Leg  ---")

    global drive_state

    drive_state = "target"

    leg_state = ""

    while leg_state != "complete":
        print("---> Current heading: ", heading())

        # Turn to required heading
        turn_to_heading(target_heading)

        print("---> Heading after turn: ", heading())

        # Drive to target
        print("---> Drive to target")
        drive_result = target_drive(target_heading, target_clicks)

        # Wait for data send to catch up
        utime.sleep(1)

        if drive_result == "obstacle":
            # An obstacle was encountered on current target heading
            # A detour action is required.
            drive_state = "detour"

            print("---> Target direction obstructed, detour required")

            # Calculate distance driven to target
            avg_clicks_completed = (
                front_left_motor_count
                + front_right_motor_count
                + rear_left_motor_count
                + rear_right_motor_count
            ) / 4

            # Calculate remaining distance. It is required to calculate
            # target heading and distance from detour point
            remaining_clicks = target_clicks - avg_clicks_completed

            # Execute detour and return target heading and distance from current detour position.
            # This will be used to start a new target drive
            target_position = make_detour(target_heading, remaining_clicks)
            print("--- New Target: ", target_position)

            # Update target position with heading and distance to target from current detour position
            target_heading = target_position.get("heading")
            target_clicks = target_position.get("distance")

            # Initiate drive towards target again
            drive_state = "target"

        elif drive_result == "complete":
            print("--- Target Reached ---")
            leg_state = "complete"

        else:
            print("--- ERROR - STOP ---")
            leg_state = "error"

    return leg_state


# ========================
#  Main Execution Control
# ========================

# 1. When NavBot starts it waits for connection message
# from the Command module. After the Command module
# connects it will send this message to the Control
# module running on the NavBot.
#
# 2. After receiving the connected message the Control
# module will wait for route data. After receiving
# the route data message from the Command module the
# Control module will wait for a go message to start
# driving the route.
#
# 3. On receiving the go command the Control module will
# start executing the routing instructions received
# driving the NavBot towards the target.
#
# 4. During the drive the NavBot will transmit progress
# data back to command module. The Command module
# will display this data.
#
# 5. The stop command is not implemented at this stage
# as MicroPython does not provide for an UART IRQ

# Types of Command Messages:
# (a) Connection msg_body = {'type' : 'CON','data' : []}
# (b) Navigation data msg_body = {
#       'type' : 'NAV',
#       'data' : [{'leg' : 99, 'head' : 999, 'dist' : 999} ... ]
#     }
# (c) Go msg_body = {'type' : 'GO','data' : []}
# (d) Stop msg_body = {'type' : 'STOP','data' : []}


def get_route():
    # The command module will connect to the control module. The control module
    # will then receive the route and the go command from the command module

    print("--- Get Target ---")

    utime.sleep(3)

    global bot_state
    bot_state = ""
    con_cmd = False  # connection state
    nav_cmd = False  # nav data state
    go_cmd = False  # execute drive
    route_cmd = {}

    # Ready for connection from command module
    bot_state = "Command"

    print("--- Ready for command module connection")

    print(">>> Waiting for connection confirmation command")

    while con_cmd == False:
        # Indicate by slow/fast blinking LED -
        # blink(num, time on, time off)

        blink(1, 0.3, 0)

        command_cmd = receive_data()

        if command_cmd != "":
            if command_cmd["type"] == "CON":
                con_cmd = True  # Commander has connected via Bluetooth
                bot_state = "Connected"

            else:
                utime.sleep(1)

    print("--- Command module connected")

    # Commander connected, ready to receive route data

    print(">>> Waiting for route data")

    while nav_cmd == False:
        # Indicate by slow/fast blinking LED
        blink(2, 0.3, 0.2)

        # Read route data
        command_cmd = receive_data()

        if command_cmd != "":
            if command_cmd["type"] == "NAV":
                nav_cmd = True
                route_cmd = command_cmd["data"]
                print("Route Data Received: ", route_cmd)

            else:
                utime.sleep(1)

    print("--- Route data received")

    # Ready to receive go command

    bot_state = "Ready"

    print(">>> Ready for GO command")

    while go_cmd == False:
        # Indicate by slow/fast blinking LED
        blink(3, 0.3, 0.3)

        command_cmd = receive_data()

        if command_cmd != "":
            if command_cmd["type"] == "GO":
                go_cmd = True

            else:
                utime.sleep(1)

    print("--- Go command received")

    # Start executing route
    bot_state = "go"

    print("--- Ready to start drive to target")

    return route_cmd


def go_target(target_route):
    # The control module will attempt to drive the NavBot to the target after
    # receiving the route and the go command from the command module.

    # -------------------------------
    #  Start drive support resources
    # -------------------------------

    print("--- Start Drive Resources")

    # # Start reading compass to monitor heading
    # print("---> Start heading monitor")
    # blink(1, 0.5, 0.4)
    # start_heading_monitor()
    # print("---> Heading monitor started")
    # utime.sleep(1)
    # print("---> Current heading: ", avg_heading)

    # # Configure timer interrupt event for sonic sensor
    # print("---> Start sonic timer")
    # blink(3, 0.5, 0.3)
    # timer_1.init(freq=1, mode=Timer.PERIODIC, callback=sonic_sense)
    # print("---> Sonic timer started")

    # Configure timer interrupt event for progress update
    # Period is in milliseconds; 1 second = 1000
    # timer_2.init(period=500, mode=Timer.PERIODIC, callback=send_progress)
    # But freq=1 just seems to work better. Why? Need to find this out!
    #     print("---> Start progress feedback timer")
    #     blink(4, 0.5, 0.4)
    #     timer_2.init(freq=1, mode=Timer.PERIODIC, callback=send_progress)
    #     print("---> Progress feedback timer started")

    # --------------------------
    #  Calculate click distance
    # --------------------------

    print("---> Calculate click distance")
    # From wheel diameter given calculate wheel circumference
    wheel_circumference = 2 * PI * (wheel_diameter_cm / 2)
    print(">>> Wheel circumference: ", wheel_circumference, " cm.")  # 21.67699

    # Calculate click distance from wheel circumference and the number of slots in sensor wheel
    click_distance = int(wheel_circumference / encoder_slots)  # 1.0838495cm
    print(">>> Click distance = ", click_distance, " cm")

    # ------------------------------
    #  Start driving route received
    # ------------------------------

    print("--- Drive Route ---")

    for leg in target_route:
        print("---> Leg detail: ", leg)

        global leg_num  # , leg_heading, leg_distance
        leg_num = leg["leg"]

        leg_heading = leg["head"]
        leg_distance_cm = leg["dist"]

        print(
            "--> Leg number: ",
            leg_num,
            " Heading: ",
            leg_heading,
            " Distance: ",
            leg_distance_cm,
            " cm",
        )

        # Convert distance in cm to distance in clicks
        leg_distance = int(leg_distance_cm / click_distance)  # clicks
        print("--- Leg distance = ", leg_distance, " clicks")

        # +++++++++++++++++++++++++++++++++++++++++++++++
        #  NB - From here onward distances are in clicks
        # +++++++++++++++++++++++++++++++++++++++++++++++

        # Drive to leg target point
        blink(5, 0.5, 0.4)
        leg_state = drive_leg(leg_heading, leg_distance)

        utime.sleep(0.5)

        # Process leg drive result
        if leg_state == "complete":
            print(f"---> Leg {leg_num} complete")

        elif leg_state == "halt":
            print(f"--- Failed to reach leg {leg_num} objective ---")
            break

        else:
            print("--- ERROR - unexpected end to route ---")
            break

    print("--- Route Done ---")


# ===================
#  Execution Control
# ===================


def main():
    print("=== Main NavBot Control ===")

    try:
        print("--- Main Start ---")

        # Test servo
        print("--- Test servo ---")
        blink(2, 0.5, 0.4)
        # cycle_servo()
        print("--- Servo tested ---")
        
        print("--- Test motors ---")
        print("<<< Left")
        pivot_left()
        utime.sleep_ms(50)
        stop()
        blink(1, 0.1, 0.3)
        print(">>> Right")
        pivot_right()
        utime.sleep_ms(50)
        stop()
        blink(1, 0.1, 0.3)
        print("--- Motors tested ---")

        # Get route data
        print("--- Get the route")
        # route = get_route() # get route from Command module
        # For testing without Command module
        route = [{"leg": 1, "head": 90, "dist": 150}]
        print("--- Route received")

        # Drive route
        print("--- Start drive to target")
        go_target(route)
        print("--- Drive to target Complete ---")

        print("--- Main END ---")

    except Exception as ex:
        stop()  # Exception, stop motors

        print("An exception has occurred!")
        print(ex)
        print("=== End Execution ===")
        
    except keyboardInterrupt:
        stop()  # stop motors
        print("Keyboard Interrupt - ctrl + c")

    except:
        stop()  # stop motors
        print("Stop, exception")

    finally:
        # Cleanup
        # stop_heading_monitor()
        timer_1.deinit()
        timer_2.deinit()
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
ir_front_left.irq(
    handler=set_front_obstacle, trigger=Pin.IRQ_FALLING, hard=True
)  # obstacle
ir_front_centre.irq(
    handler=set_front_obstacle, trigger=Pin.IRQ_FALLING, hard=True
)  # obstacle
ir_front_right.irq(
    handler=set_front_obstacle, trigger=Pin.IRQ_FALLING, hard=True
)  # obstacle

# ----------------------------------
#  Set motor turn sensor interrupts
# ----------------------------------

# - Front
front_left_motor.irq(
    handler=front_left_motor_counter, trigger=Pin.IRQ_RISING, hard=True
)
front_right_motor.irq(
    handler=front_right_motor_counter, trigger=Pin.IRQ_RISING, hard=True
)

# - Rear
rear_left_motor.irq(handler=rear_left_motor_counter, trigger=Pin.IRQ_RISING, hard=True)
rear_right_motor.irq(
    handler=rear_right_motor_counter, trigger=Pin.IRQ_RISING, hard=True
)

# -------------------
#  UART RX interrupt - not available
# -------------------
# uart.irq(UART.RX_ANY, priority=5, handler=process_rx, wake=machine.IDLE)
# Micropython does not implement this for the Pico. Only available for WiPy.

if __name__ == "__main__":
    print("%s is being run directly" % __name__)
    main()

else:
    print("%s is being imported" % __name__)
