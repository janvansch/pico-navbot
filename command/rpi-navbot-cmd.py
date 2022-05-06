# NavBot Commander
# NavBot is an auto-navigating experiment
# This comand app will be used to -
#   1. Input navigation info to the robot
#   2. To monitor the progress of the robot while navigating

# Developed by Johannes van Schalkwyk
# All rights are reserved

# Written in Python 3.9.2 for execution on Raspberry Pi running Raspberry OS 64-bit 
# Uses:
#      Bluez for Linux
#      tkinter

import bluetooth
import tkinter as tk
from tkinter import ttk
from tkinter import font
from tkinter import messagebox
import threading
import json
import time
import sys

# ===============================
#  Define & set golbal variables
# ===============================
app_name = "NavBot Commander"
window_width = 550
window_height = 760
selected = "none"
primary_color = 'lightblue'
secondary_color = 'white'
highlight_color = '#347083'
routes = []

# When a window is resizable, you can specify the minimum and maximum sizes
# min_width = 465
# min_height = 620
# max_width = 600
# max_height = 900

# window transparency, from 0.0 (fully transparent) to 1.0 (fully opaque):
transparency = 1

def log(event):
    print(event)

# ============================
#  Bluetooth connection setup
# ============================

def bt_connect():
    #
    # Connect to NavBot Bluetooth interface 
    #
    bd_addr = "00:20:10:08:63:A3" # used rpi-findmyphone.py to get address
    port = 1
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((bd_addr, port))
    return sock

def read_nav_data(sock):
    #
    # Retrieve data from NavBot
    #
    bt_data = sock.recv(1024)
    nav_data = json.loads(bt_data)
    return nav_data

def update_data():
    #
    # This runs as a thread
    #
    
    # Connect Bluetooth
    sock = bt_connect()
    
    # Continiously read NavBot data over connection
    while True:
        bot_data = read_nav_data(sock)
        print(nav_data)
        
        # Update GUI with data received
        nav_data[0].set(bot_data['leg_num'])
        nav_data[1].set(bot_data['leg_heading'])
        nav_data[2].set(bot_data['leg_distance'])
        nav_data[3].set(bot_data['nav_status'])
        nav_data[4].set(bot_data['travel_head'])
        nav_data[5].set(bot_data['travel_dist'])
        nav_data[6].set(bot_data['sonic_dist'])
        nav_data[7].set(bot_data['lf_count'])
        nav_data[8].set(bot_data['rf_count'])
        nav_data[9].set(bot_data['lr_count'])
        nav_data[10].set(bot_data['rr_count'])
        
        
        # Pause for GUI to update and user to read data
        time.sleep(1)

# ---------------------------------
#  Create thread for updating data
# ---------------------------------
data_task = threading.Thread(target = update_data, daemon = True)

# ===================
#  Control Functions
# ===================

def connect_bot():
    global bot_sock
    bot_sock = bt_connect()
    
def send_route():
    
    # Get data for selected route
    
    # Convert to dictionary
    # Send json data to NavBot via Bluetooth
    pass

def send_start():
    
    # This function will:
    #  - First activate the data collection thread
    #  - Then send the instruction to the NavBot to start the drive.
    # Navbot will only execute if a route was loaded.
    
    # Activate data collection thread
    #data_task.start()
    
    # Send drive route command
    
    pass

def send_stop():
    
    # Emergency stop
    pass

# ====================
#  Routes load & save
# ====================

def load_routes():
    
    global routes
    
    try:
        with open('routes.data', 'r') as openfile:
            routes = json.load(openfile)
            
    except IOError:
        print("File not found")
        messagebox.showerror("Error", "ERROR: Routes data file not found!")

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Temp for testing, to be removed
        routes = [
            {
                'num' : 1,
                'desc' : "Single leg",
                'legs' : [{'leg' : 1, 'head' : 10, 'dist' : 150}]
            },
            {
                'num' : 2,
                'desc' : "Multi Leg",
                'legs' : [
                    {
                        'leg' : 1, 'head' : 100, 'dist' : 150
                    },
                    {
                        'leg' : 2, 'head' : 190, 'dist' : 50
                    }
                ]
            },
            {
                'num' : 3, # the dog led ally test
                'desc' : "Obstacle Navigation Test",
                'legs' : [{'leg' : 1, 'head' : 350, 'dist' : 150}]
            }
        ]
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        print("Loaded dummy routes")
        messagebox.showinfo("INFO", "Loaded dummy data.")

    else:
        print("Routes loaded")

    finally:
        print(routes)
        print(type(routes))
        print("Routes Load End")

    #return routes
          
def save_routes(routes):
    
    try:
        with open('routes.data', 'w') as outfile:
            json.dump(routes, outfile)
        
        result = 'ok'
            
    except IOError:
        print("Data not saved")
        result = 'ERR'
        messagebox.showerror("ERROR", "Routes data NOT saved!")    
        
    else:
        print("Data saved")

    finally:
        print("Routes Save End")
    
    return result

# =========================
#  Routes data maintenance
# ========================= 

# -----------------------------
#  Route maintenance functions
# ----------------------------- 

def add_route():
        
    print("Add route clicked")

    global routes
    
    # Add dummy route entry to routes list
    new_route_num = (len(routes)) + 1
    new_route = { 'num' : new_route_num, 'desc' : '<new route>', 'legs' : []}
    routes.append(new_route)

    # Write routes list to file
    save_routes(routes)

    # Update routes display
    redisplay_routes(new_route_num - 1) # list index starts from 0

    print(routes)
    print(type(routes))

def update_route():

    print("Update route clicked")

    # Get index of selected route
    route_idx = route_tree.focus()
    
    # Update route description
    routes[int(route_idx)]['desc'] = route_name_entry.get()

    # Write routes list to file
    save_routes(routes)

    # Update routes display
    redisplay_routes(route_idx)

    # Update legs display
    display_route_legs('e')

def delete_route():

    print("Delete route clicked")

    # Selected route
    route_idx = route_tree.focus()
    
    # Pop route off routes list
    routes.pop(int(route_idx))
    
    # Renumber routes
    for item in range(0, len(routes)):
       routes[item]['num'] = item + 1 # routes numberred from 1

    # Write routes list to file
    save_routes(routes)
    
    # Update routes display
    redisplay_routes(0) # original selected route deleted

    # Update legs display
    display_legs(0) # original selected route deleted

# ---------------------------------
#  Route leg maintenance functions
# --------------------------------- 

def add_leg():

    print("Add leg clicked")

    # Selected route
    route_num = route_tree.focus()
    print("route: ", route_num, type(route_num))

    # Append dummy leg legs list
    if len(routes[int(route_num)]['legs']) > 0:
        # Add to end of list
        new_leg = { 'leg' : len(routes[int(route_num)]['legs']) + 1, 'head' : 0, 'dist' : 0}
        routes[int(route_num)]['legs'].append(new_leg)
    
    else:
        # Add first leg
        new_leg = { 'leg' : 0, 'head' : 0, 'dist' : 0}
        routes[int(route_num)]['legs'].append(new_leg)
    
    # Write routes list to file
    save_routes(routes)

    # Update legs display
    display_route_legs('e')
    
def update_leg():

    print("Update leg clicked")

    # Leg and route reference
    route_idx = route_tree.focus()
    leg_idx = leg_tree.focus()
    
    #Check that a leg was selected
    if leg_idx == '':
        messagebox.showinfo("INFO", "Select a leg to update.")

    else:
        # Update route description
        routes[int(route_idx)]['legs'][int(leg_idx)]['head'] = leg_head_entry.get()
        routes[int(route_idx)]['legs'][int(leg_idx)]['dist'] = leg_dist_entry.get()

        # Write routes list to file
        save_routes(routes)

        # Update legs display
        display_route_legs('e')

def delete_leg():
    print("Delete leg clicked")

    # Check that a leg was selected    
    leg_idx_str = leg_tree.focus()
    if leg_idx_str == '':
        messagebox.showinfo("INFO", "Select a leg to delete.")

    else:
        # selected leg and route reference
        route_idx = int(route_tree.focus())
        leg_idx = int(leg_idx_str)

        # Pop leg from legs list of the selected route
        routes[route_idx]['legs'].pop(leg_idx)

        # Renumber legs
        legs_len = len(routes[route_idx]['legs'])
        for item in range(0, legs_len):
            routes[route_idx]['legs'][item]['leg'] = item + 1 # legs are numberred from 1

        # Write routes list to file
        save_routes(routes)
        
        # Update legs display
        display_route_legs('e') # original selected route deleted

# ===================
#  Create GUI Object
# ===================

# -----------------------
#  Create GUI Components
# -----------------------

def create_styles(win):

    # Set default font
    def_font_detail = font.nametofont("TkDefaultFont").actual()
    print(def_font_detail)
    default_font = def_font_detail['family']
        
    # Add Style Object
    win.style = ttk.Style(win)

    # Style configuration
    win.style.configure('TLabel', font = (default_font, 10))
    win.style.configure('TEntry', font = (default_font, 10, 'bold'), foreground = '#0000ff')
    win.style.configure('TButton', font = (default_font, 10))

    # Styles forControl buttons
    win.style.configure('Connect.TButton', font = (default_font, 9, 'bold'), foreground = '#bbbbff', background = '#4444ff')
    win.style.configure('SendRoute.TButton', font = (default_font, 9, 'bold'), foreground = '#4444ff', background = '#55ffff')
    win.style.configure('Start.TButton', font = (default_font, 9, 'bold'), foreground = '#bb3333', background = '#55ff55')
    win.style.configure('Stop.TButton', font = (default_font, 9, 'bold'), foreground = '#ffbb00', background = '#aa2222')

    win.style.configure('Heading.TLabel', font = (default_font, 16))
    win.style.configure('SubHeading.TLabel', font = (default_font, 14), justify = 'left')
    
    # Configure the Treeview Colors
    win.style.configure(
        'Treeview',
        background = '#D3D3D3',
        foreground = 'black',
        rowheight = 25,
        fieldbackground = '#D3D3D3'
    )
    
    # Change selected row color #347083
    win.style.map('Treeview',
        background = [('selected', '#347083')]
    )
    
def create_data_vars(win):
    
    # GUI data variables
    
    curr_leg_num = tk.IntVar(win, value = 99)
    curr_leg_heading = tk.IntVar(win, value = 360)
    curr_leg_distance = tk.IntVar(win, value = 999)
    nav_status = tk.StringVar(win, 'not set')
    travel_head = tk.IntVar(win, 360)
    travel_dist = tk.IntVar(win, 999)
    sonic_dist = tk.IntVar(win, 999)
    lf_count = tk.IntVar(win, 9999)
    rf_count = tk.IntVar(win, 9999)
    lr_count = tk.IntVar(win, 9999)
    rr_count = tk.IntVar(win, 9999)
    
    global nav_data
    nav_data = [
        curr_leg_num,
        curr_leg_heading,
        curr_leg_distance,
        nav_status,
        travel_head,
        travel_dist,
        sonic_dist,
        lf_count,
        rf_count,
        lr_count,
        rr_count
    ]
    
def create_control_frame(win):
    
    # Create frame object
    frame = ttk.Frame(win)
    
    # Define grid columns
    frame.columnconfigure(0, weight=3)
    frame.columnconfigure(1, weight=2)
    frame.columnconfigure(2, weight=0)
    frame.columnconfigure(3, weight=1)
    frame.columnconfigure(4, weight=1)
    
    ttk.Button(
        frame,
        text = 'Connect NavBot',
        style = 'Connect.TButton',
        command = bt_connect
    ).grid(column = 0, row = 0)
    
    ttk.Button(
        frame, 
        text = 'Send Route',
        style = 'SendRoute.TButton',
        command = send_route()
        ).grid(column = 1, row = 0)
        
    #ttk.Button(frame, text = ' ').grid(column = 2, row = 0)

    ttk.Button(
        frame, 
        text = 'Start',
        style = 'Start.TButton',
        command = send_start()
        ).grid(column = 3, row = 0)
    
    ttk.Button(
        frame, 
        text = 'Stop',
        style = 'Stop.TButton',
        command = send_stop()
        ).grid(column = 4, row = 0)
    
    for widget in frame.winfo_children():
        widget.grid(padx = 5, pady = 5)
    
    return frame

def create_progress_frame(win):
    
    # Create frame object
    prog_frame = ttk.Frame(win)
    
    # Define grid columns
    prog_frame.columnconfigure(0, weight=2)
    prog_frame.columnconfigure(1, weight=2)
    prog_frame.columnconfigure(2, weight=2)
    prog_frame.columnconfigure(3, weight=1)
    
    # Abandoned this because disabled/readonly state does not display value
    #entry_cleg = ttk.Entry(prog_frame, width = 3, foreground='red')
    #entry_cleg.grid(column=0, row=1)
    #entry_cleg.insert(0, '1')
    
    # Current leg data
    curr_leg_head = ttk.Label(prog_frame, text = 'Current Leg #')
    curr_leg_head.grid(column = 0, row = 0, padx = 15, pady = 5)
    curr_leg_val = ttk.Label(prog_frame, foreground='red', textvariable = nav_data[0])
    curr_leg_val.grid(column = 0, row = 1)
    
    leg_heading_head = ttk.Label(prog_frame, text = 'Leg Heading')
    leg_heading_head.grid(column = 1, row = 0, padx = 15, pady = 5)
    leg_heading_val = ttk.Label(prog_frame, foreground='red', textvariable = nav_data[1])
    leg_heading_val.grid(column = 1, row = 1)
    
    leg_dist_head = ttk.Label(prog_frame, text = 'Leg Distance')
    leg_dist_head.grid(column = 2, row = 0, padx = 15, pady = 5)
    leg_dist_val = ttk.Label(prog_frame, foreground='red', textvariable = nav_data[2])
    leg_dist_val.grid(column = 2, row = 1)
    
    # Current status data
    curr_stat_head = ttk.Label(prog_frame, text = 'Current Status')
    curr_stat_head.grid(column = 0, row = 10, padx = 15, pady = 5)
    curr_stat_val = ttk.Label(prog_frame, textvariable = nav_data[3])
    curr_stat_val.grid(column = 0, row = 11)

    heading_head = ttk.Label(prog_frame, text = 'Heading')
    heading_head.grid(column = 1, row = 10, padx = 15, pady = 5)
    heading_val = ttk.Label(prog_frame, textvariable = nav_data[4])
    heading_val.grid(column = 1, row = 11)

    trav_dist_head = ttk.Label(prog_frame, text = 'Travel Distance')
    trav_dist_head.grid(column = 2, row = 10, padx = 15, pady = 5)
    trav_dist_val = ttk.Label(prog_frame, textvariable = nav_data[5])
    trav_dist_val.grid(column = 2, row = 11)
    
    # Sonic Distance
    son_dist_head = ttk.Label(prog_frame, text = 'Sonic Distance')
    son_dist_head.grid(column = 3, row = 10, pady = 5)
    son_dist_val = ttk.Label(prog_frame, textvariable = nav_data[6])
    son_dist_val.grid(column = 3, row = 11)
    
    # Counter data
    count_header = ttk.Label(prog_frame, text = 'Slot Counters:')
    count_header.grid(column = 0, row = 20, pady = 5)
    
    left_front_head = ttk.Label(prog_frame, text = 'Left Front')
    left_front_head.grid(column = 0, row = 30)
    left_front_val = ttk.Label(prog_frame, textvariable = nav_data[7])
    left_front_val.grid(column = 0, row = 31)

    right_front_head = ttk.Label(prog_frame, text = 'Right Front')
    right_front_head.grid(column = 1, row = 30)
    right_front_val = ttk.Label(prog_frame, textvariable = nav_data[8])
    right_front_val.grid(column = 1, row = 31)

    left_rear_head = ttk.Label(prog_frame, text = 'Left Rear')
    left_rear_head.grid(column = 2, row = 30)
    left_rear_val = ttk.Label(prog_frame, textvariable = nav_data[9])
    left_rear_val.grid(column = 2, row = 31)

    right_rear_head = ttk.Label(prog_frame, text = 'Right Rear')
    right_rear_head.grid(column = 3, row = 30)
    right_rear_val = ttk.Label(prog_frame, textvariable = nav_data[10])
    right_rear_val.grid(column = 3, row = 31)
    
    return prog_frame
    
def create_routes_frame(container):
    
    # -------------------------------------------------
    #  Create a route treeview with scrollbar in frame
    # -------------------------------------------------
    
    # Create frame
    route_frame = ttk.Frame(container)
    
    # Create scrollbar
    route_scroll = ttk.Scrollbar(route_frame)
    route_scroll.pack(side = tk.RIGHT, fill=tk.Y)
        
    # Create the treeview
    global route_tree
    route_tree = ttk.Treeview(
        route_frame,
        yscrollcommand = route_scroll.set,
        selectmode = "extended"
    )

    # Display
    route_tree.pack()
    
    # Configure the scrollbar
    route_scroll.config(command = route_tree.yview)
    
    # Configure treeview
    route_tree['columns'] = ("Route #", "Route Name")
    
    # Format route Columns
    route_tree.column("#0", width=0, stretch=tk.NO)
    route_tree.column("Route #", anchor=tk.W, width=65)
    route_tree.column("Route Name", anchor=tk.W, width=180)
    
    # Create column headings
    route_tree.heading("#0", text="", anchor=tk.W)
    route_tree.heading("Route #", text="Route #", anchor=tk.CENTER)
    route_tree.heading("Route Name", text="Route Name", anchor=tk.CENTER)
    
    # Create striped rows
    route_tree.tag_configure('oddrow', background=secondary_color)
    route_tree.tag_configure('evenrow', background=primary_color)
      
    return route_frame

def create_route_entry_frame(win):
    
    # ------------------------
    #  Create route entry box 
    # ------------------------
    
    global route_entry_frame
    route_entry_frame = ttk.LabelFrame(win, text="Route Maintenance")
    
    # Define entry frame columns
    route_entry_frame.columnconfigure(0, weight=1)
    route_entry_frame.columnconfigure(1, weight=2)
        
    route_num_label = ttk.Label(route_entry_frame, text="Route #")
    route_num_label.grid(row=0, column=0, padx=5, pady=5)
    
    global route_num_entry
    route_num_entry = ttk.Entry(route_entry_frame, width = 4, state = "readonly")
    route_num_entry.grid(row=1, column=0, padx=5, pady=5)

    route_name_label = ttk.Label(route_entry_frame, text="Route Name")
    route_name_label.grid(row=0, column=1, padx=5, pady=5)
    
    global route_name_entry
    route_name_entry = ttk.Entry(route_entry_frame, width = 20)
    route_name_entry.grid(row=1, column=1, padx=5, pady=5)

    # Create buttons sub-frame object
    entry_buttons_frame = ttk.Frame(route_entry_frame )
    entry_buttons_frame.grid(column=0, row=2, columnspan=2)

    # Define buttons sub-frame columns
    entry_buttons_frame.columnconfigure(0, weight=1)
    entry_buttons_frame.columnconfigure(1, weight=1)
    entry_buttons_frame.columnconfigure(2, weight=1)
       
    ttk.Button(
        entry_buttons_frame,
        width = 4,
        text = 'Add',
        #style = 'Connect.TButton',
        command = add_route
    ).grid(column = 0, row = 3)
    
    ttk.Button(
        entry_buttons_frame,
        width = 6,
        text = 'Update',
        command = update_route
    ).grid(column = 1, row = 3)

    ttk.Button(
        entry_buttons_frame,
        width = 6,
        text = 'Delete',
        command = delete_route
    ).grid(column = 2, row = 3)
       
    for widget in entry_buttons_frame.winfo_children():
        widget.grid(padx = 5, pady = 5) 
    
    return route_entry_frame
    
def create_legs_frame(win):
    
    # -------------------------------------------------
    #  Create a leg treeview with scrollbar in frame
    # -------------------------------------------------
    
    # Create frame
    leg_frame = ttk.Frame(win)
    
    # Create scrollbar
    leg_scroll = ttk.Scrollbar(leg_frame)
    leg_scroll.pack(side = tk.RIGHT, fill=tk.Y)
        
    # Create the treeview
    global leg_tree
    leg_tree = ttk.Treeview(
        leg_frame,
        yscrollcommand = leg_scroll.set,
        selectmode = "extended"
    )

    # Display
    leg_tree.pack()
    
    # Configure the scrollbar
    leg_scroll.config(command = leg_tree.yview)
    
    # Configure treeview
    leg_tree['columns'] = ("Leg #", "Heading", "Distance")
    
    # Format leg Columns
    leg_tree.column("#0", width=0, stretch=tk.NO)
    leg_tree.column("Leg #", anchor=tk.W, width=65)
    leg_tree.column("Heading", anchor=tk.W, width=80)
    leg_tree.column("Distance", anchor=tk.W, width=80)
    
    # Create column headings
    leg_tree.heading("#0", text="", anchor=tk.W)
    leg_tree.heading("Leg #", text="Leg #", anchor=tk.CENTER)
    leg_tree.heading("Heading", text="Heading", anchor=tk.CENTER)
    leg_tree.heading("Distance", text="Distance", anchor=tk.CENTER)
    
    # Create striped rows
    leg_tree.tag_configure('oddrow', background=secondary_color)
    leg_tree.tag_configure('evenrow', background=primary_color)
    
    return leg_frame

def create_leg_entry_frame(win):
    
    # ----------------------
    #  Create leg entry box 
    # ----------------------
    
    leg_entry_frame = ttk.LabelFrame(win, text="Route Leg Maintenance")
    
    # Define entry frame columns
    leg_entry_frame.columnconfigure(0, weight=1)
    leg_entry_frame.columnconfigure(1, weight=2)
    leg_entry_frame.columnconfigure(2, weight=2)
    
    leg_num_label = ttk.Label(leg_entry_frame, text="Leg #")
    leg_num_label.grid(row=0, column=0, padx=2, pady=5)
    
    global leg_num_entry
    leg_num_entry = ttk.Entry(leg_entry_frame, width = 3, state = "readonly")
    leg_num_entry.grid(row=1, column=0, padx=2, pady=5)

    leg_head_label = ttk.Label(leg_entry_frame, text="Heading (deg)")
    leg_head_label.grid(row=0, column=1, padx=2, pady=5)
    
    global leg_head_entry
    leg_head_entry = ttk.Entry(leg_entry_frame, width = 8)
    leg_head_entry.grid(row=1, column=1, padx=2, pady=5)
    
    leg_dist_label = ttk.Label(leg_entry_frame, text="Distance (cm)")
    leg_dist_label.grid(row=0, column=2, padx=2, pady=5)
    
    global leg_dist_entry
    leg_dist_entry = ttk.Entry(leg_entry_frame, width = 8)
    leg_dist_entry.grid(row=1, column=2, padx=2, pady=5)

    # Create buttons sub-frame object
    entry_buttons_frame = ttk.Frame(leg_entry_frame )
    entry_buttons_frame.grid(column=0, row=2, columnspan=3)

    # Define buttons sub-frame columns
    entry_buttons_frame.columnconfigure(0, weight=1)
    entry_buttons_frame.columnconfigure(1, weight=1)
    entry_buttons_frame.columnconfigure(2, weight=1)
       
    ttk.Button(
        entry_buttons_frame,
        width = 4,
        text = 'Add',
        #style = 'Connect.TButton',
        command = add_leg
    ).grid(column = 0, row = 3)
    
    ttk.Button(
        entry_buttons_frame,
        width = 6,
        text = 'Update',
        command = update_leg
    ).grid(column = 1, row = 3)

    ttk.Button(
        entry_buttons_frame,
        width = 6,
        text = 'Delete',
        command = delete_leg
    ).grid(column = 2, row = 3)
       
    for widget in entry_buttons_frame.winfo_children():
        widget.grid(padx = 5, pady = 5)

    return leg_entry_frame

# -----------------------------
#  GUI Data Display Management
# -----------------------------

def display_routes():
    
    # ------------------------------ 
    #  Add route data to the screen
    # ------------------------------
    
    global route_count
    route_count = 0
    for route in routes:
        if route_count % 2 == 0:
            route_tree.insert(
                parent='',
                index='end',
                iid=route_count,
                text='',
                values=(
                    route['num'],
                    route['desc']
                ),
                tags=('evenrow',)
            )

        else:
            route_tree.insert(
                parent='',
                index='end',
                iid=route_count,
                text='',
                values=(
                    route['num'],
                    route['desc']                   
                ),
                tags=('oddrow',))
		
        route_count += 1

def redisplay_routes(route_idx):

    # Clear entry boxes
    clear_route_entry()
    clear_leg_entry()

    # Clear the treeviews
    route_tree.delete(*route_tree.get_children())
    leg_tree.delete(*leg_tree.get_children())

    # Display updated routes
    display_routes()

    # Remove current selection and focus
    for item in route_tree.selection():
        route_tree.selection_remove(item)

    # Set selection and focus to added route
    route_tree.selection_set(route_idx)
    route_tree.focus(route_idx)

    # Display new route in entry box
    fill_route_entry('e')

def display_legs(route_idx):

    selected_route_legs = routes[route_idx]['legs']
    
    print(selected_route_legs)
        
    global leg_count
    leg_count = 0
    
    for leg in selected_route_legs:
        
        if leg_count % 2 == 0: # odd or even
            leg_tree.insert(
                parent='',
                index='end',
                iid=leg_count,
                text='',
                values=(
                    leg['leg'],
                    leg['head'],
                    leg['dist']
                ),
                tags=('evenrow',)
            )

        else:
            leg_tree.insert(
                parent='',
                index='end',
                iid=leg_count,
                text='',
                values=(
                    leg['leg'],
                    leg['head'],
                    leg['dist']
                ),
                tags=('oddrow',))
		
        leg_count += 1

def display_route_legs(e):
    
    selected_route = int(route_tree.focus())
    
    if selected_route == '':
        selected_route = 0
    
    # clear entry box
    clear_leg_entry()

    # Clear leg treeview table
    leg_tree.delete(*leg_tree.get_children())
        
    # Display legs of selected route
    display_legs(selected_route)

def fill_route_entry(e):
    
    # ----------------------
    #  Fill route entry box
    # ----------------------
        
	# Clear route entry box
	route_num_entry.delete(0, tk.END)
	route_name_entry.delete(0, tk.END)
	
	# Get route number
	selected = route_tree.focus()
	
	# Get route values
	values = route_tree.item(selected, 'values')

	# Display in route data entry box
	route_num_entry.insert(0, values[0])
	route_name_entry.insert(0, values[1])
		
def fill_leg_entry(e):
    
    # --------------------
    #  Fill leg entry box
    # --------------------
    
	# Clear leg entry box
	leg_num_entry.delete(0, tk.END)
	leg_head_entry.delete(0, tk.END)
	leg_dist_entry.delete(0, tk.END)
	
	# Get leg number
	selected = leg_tree.focus()
	
	# Get leg values
	values = leg_tree.item(selected, 'values')

	# Display in leg data entry box
	leg_num_entry.insert(0, values[0])
	leg_head_entry.insert(0, values[1])
	leg_dist_entry.insert(0, values[2])

# Clear route entry boxes
def clear_route_entry():
	
	leg_num_entry.delete(0, tk.END)
	leg_head_entry.delete(0, tk.END)
	leg_dist_entry.delete(0, tk.END)

# Clear leg entry box
def clear_leg_entry():
	
	leg_num_entry.delete(0, tk.END)
	leg_head_entry.delete(0, tk.END)
	leg_dist_entry.delete(0, tk.END)
	
# --------------------
#  Create Main Window
# --------------------

def create_main_window():
    
    # ----------------------------------
    #  Create & Configure Window Object
    # ----------------------------------
    
    root = tk.Tk()
    root.title(app_name)
    #root.iconbitmap('temp.ico')

    # Set window position to the center of the screen
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    center_x = int(screen_width/2 - window_width / 2)
    center_y = int(screen_height/2 - window_height / 2)
    root.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')

    # The window is resizable by default,
    # to prevent resizing use resizable() method
    root.resizable(False, False)

    # If a window is resizable, minimum and maximum sizes can be set
    # root.minsize(min_width, min_height)
    # root.maxsize(max_width, max_height)

    # Set transparency
    root.attributes('-alpha', transparency)

    # window stacking - on top
    root.attributes('-topmost', 1)
    
    # --------------------------
    #  Configure window content
    # --------------------------
    
    # Window layout
    root.columnconfigure(0, weight = 1)
    root.columnconfigure(1, weight = 1)
    
    # Set content styles
    create_styles(root)
        
    # Setup data variables
    create_data_vars(root)
    
    # Main header
    lbl_header = ttk.Label(root, text = app_name, style = 'Heading.TLabel')
    lbl_header.grid(column = 0, row = 0, columnspan = 2, padx = 2, pady = 15)
    
    # ----------------------
    #  Control Buttons View
    # ----------------------

    control_frame = create_control_frame(root)
    control_frame.grid(column = 0, row = 1, columnspan = 2)
    
    # ---------------
    #  Progress View
    # ---------------
    
    # Progress view sub-heading
    lbl_progress = ttk.Label(root, text = 'Route Progress:', style = 'SubHeading.TLabel')
    lbl_progress.grid(column = 0, row = 2, sticky = tk.W, columnspan = 2, padx = 15, pady = 10)
    
    # Create progress view
    progress_frame = create_progress_frame(root)
    progress_frame.grid(column = 0, row = 3, columnspan = 2)
    
    # --------------------
    #  Route Planner View
    # --------------------
    
    # Route planner view sub-heading
    lbl_planner = ttk.Label(root, text = 'Route Planner:', style = 'SubHeading.TLabel')
    lbl_planner.grid(column = 0, row = 4, sticky = tk.W, columnspan = 2, padx = 15, pady = 10)
    
    # Create routes list view
    #routes_frame = create_routes_frame(root, routes)
    routes_frame = create_routes_frame(root)
    routes_frame.grid(column = 0, row = 5)
    
    # Create legs list view
    #legs_frame = create_legs_frame(root, routes)
    legs_frame = create_legs_frame(root)
    legs_frame.grid(column = 1, row = 5)
    
    # Create route entry form
    #routes_entry_frame = create_route_entry_frame(root, routes)
    routes_entry_frame = create_route_entry_frame(root)
    routes_entry_frame.grid(column = 0, row = 6, pady = 5)
    
    # Create leg entry form
    #legs_entry_frame = create_leg_entry_frame(root, routes)
    legs_entry_frame = create_leg_entry_frame(root)
    legs_entry_frame.grid(column = 1, row = 6, pady = 5)
    
    # --------------
    #  Display data
    # --------------

    # load routes data from file
    load_routes()

    # Display routes loaded
    display_routes()

    # Select first route, index = 0
    route_tree.selection_set(0)
    route_tree.focus(0)

    # Fill route entry box
    fill_route_entry('e')

    # Display legs of first route
    display_legs(0)
    
    # ----------
    #  Bindings
    # ----------

    route_tree.bind("<ButtonRelease-1>", fill_route_entry)
    route_tree.bind("<ButtonRelease-1>", display_route_legs, add='+')
    leg_tree.bind("<ButtonRelease-1>", fill_leg_entry)

    # ----------------
    #  Display Window
    # ----------------

    root.mainloop()

    
if __name__ == "__main__":
    
    #load_routes()
#     if routes != []:
#         # Start GUI display
#         create_main_window()
    create_main_window()










