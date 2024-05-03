#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import RPi.GPIO as GPIO
from threading import Thread
import time

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor configuration for 3 motors
motor_pins = [
    {'step': 27, 'direction': 21, 'enable': 4},
    {'step': 26, 'direction': 23, 'enable': 13},
    {'step': 12, 'direction': 20, 'enable': 22}
]

motor_speed = [100, 100, 100]
motor_threads = [None] * len(motor_pins)
motor_direction = [GPIO.HIGH] * len(motor_pins)

# Setup GPIO pins
for pin_set in motor_pins:
    for pin in pin_set.values():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

motor_on_duration = 0.5
motor_off_duration = 0.5
ramp_up_time = 0.3
ramp_down_time = 0.3
notch_pause_duration = 0.05  # Default notch pause duration
notch_delay = 0.1  # Default delay as a fraction of the ramp-down phase

# Function Definitions for Slider Updates
def update_ramp_time(value):
    global ramp_up_time
    ramp_up_time = float(value)

def update_on_duration(value):
    global motor_on_duration
    motor_on_duration = float(value)

def update_off_duration(value):
    global motor_off_duration
    motor_off_duration = float(value)

def update_notch_pause(value):
    global notch_pause_duration
    notch_pause_duration = float(value)

def update_notch_delay(value):
    global notch_delay
    notch_delay = float(value)

def start_motor(index):
    if motor_threads[index] is None or not motor_threads[index].is_alive():
        motor_threads[index] = Thread(target=run_motor, args=(index,))
        motor_threads[index].start()

def stop_motor(index):
    GPIO.output(motor_pins[index]['enable'], GPIO.HIGH)
    if motor_threads[index] is not None:
        motor_threads[index].join()
        motor_threads[index] = None

def stop_all_motors():
    for index in range(len(motor_pins)):
        stop_motor(index)

def quit_program():
    stop_all_motors()
    root.destroy()

def run_motor(index):
    step_pin = motor_pins[index]['step']
    dir_pin = motor_pins[index]['direction']
    en_pin = motor_pins[index]['enable']

    GPIO.output(en_pin, GPIO.LOW)
    GPIO.output(dir_pin, motor_direction[index])

    while GPIO.input(en_pin) == GPIO.LOW:
        ramp_steps = int(ramp_up_time / (1.0 / motor_speed[index]))
        ramp_down_steps = int(ramp_down_time / (1.0 / motor_speed[index]))
        initial_speed = motor_speed[index] * 0.1
        current_speed = initial_speed
        speed_increment = (motor_speed[index] - initial_speed) / ramp_steps
        speed_decrement = (motor_speed[index] - current_speed) / ramp_down_steps

        # Ramp-up phase
        for step in range(ramp_steps):
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(1.0 / current_speed / 2)
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(1.0 / current_speed / 2)
            current_speed += speed_increment

        # Calculate when to apply the notch during the ramp-down phase
        notch_step = int(notch_delay * ramp_down_steps)

        # Ramp-down phase
        for step in range(ramp_down_steps):
            if step == notch_step:  # Apply notch based on the calculated step
                time.sleep(notch_pause_duration)
            current_speed -= speed_decrement
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(1.0 / current_speed / 2)
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(1.0 / current_speed / 2)

        # Final off duration
        time.sleep(motor_off_duration)

def toggle_direction(index):
    motor_direction[index] = GPIO.LOW if motor_direction[index] == GPIO.HIGH else GPIO.HIGH
    if motor_threads[index] is not None and motor_threads[index].is_alive():
        GPIO.output(motor_pins[index]['direction'], motor_direction[index])

def create_motor_control(frame, motor_index):
    motor_frame = tk.Frame(frame, bg='black')
    motor_frame.pack(pady=5)

    def update_speed(index, value):
        motor_speed[index] = int(value)
        if motor_threads[index] is not None and motor_threads[index].is_alive():
            stop_motor(index)
            start_motor(index)

    start_button = tk.Button(motor_frame, text=f"Start \nMotor {motor_index+1}", command=lambda idx=motor_index: start_motor(idx), bg='green', fg='white', font=("Arial", 14, "bold"))
    start_button.pack(side='left', padx=5)

    stop_button = tk.Button(motor_frame, text=f"Stop \nMotor {motor_index+1}", command=lambda idx=motor_index: stop_motor(idx), bg='red', fg='white', font=("Arial", 14, "bold"))
    stop_button.pack(side='left', padx=5)

    direction_button = tk.Button(motor_frame, text=f"Direction \n{motor_index+1}", command=lambda idx=motor_index: toggle_direction(idx), bg='purple', fg='white', font=("Arial", 14, "bold"))
    direction_button.pack(side='left', padx=5)

    speed_slider = tk.Scale(motor_frame, from_=1, to=255, orient=tk.HORIZONTAL, command=lambda value: update_speed(motor_index, int(value)), font=("Arial", 12), length=200, sliderlength=50)
    speed_slider.set(motor_speed[motor_index])
    speed_slider.pack(side='left', padx=5)

def setup_tabs():
    notebook = ttk.Notebook(root)
    main_frame = tk.Frame(notebook, bg='black')
    adjustments_frame = tk.Frame(notebook, bg='black')

    notebook.add(main_frame, text='Main Controls')
    notebook.add(adjustments_frame, text='Adjustments')
    notebook.pack(expand=True, fill='both')

    setup_main_controls(main_frame)
    add_adjustment_widgets(adjustments_frame)

def setup_main_controls(frame):
    welcome_label = tk.Label(frame, text="Welcome to PULSE PiClyde!", bg='black', fg='red', font=("Arial", 18, "bold"))
    welcome_label.pack(pady=5)

    for i in range(3):
        create_motor_control(frame, i)

    stop_all_motors_button = tk.Button(frame, text="Stop All Motors", command=stop_all_motors, bg='blue', fg='white', font=("Arial", 20, "bold"))
    stop_all_motors_button.pack(pady=10)

    quit_program_button = tk.Button(frame, text="Quit Program", command=quit_program, bg='orangered', fg='white', font=("Arial", 18, "bold"))
    quit_program_button.pack(pady=10)

def add_adjustment_widgets(frame):
    left_column = tk.Frame(frame, bg='black')
    right_column = tk.Frame(frame, bg='black')

    left_column.pack(side='left', fill='both', expand=True, padx=10, pady=10)
    right_column.pack(side='left', fill='both', expand=True, padx=10, pady=10)

    create_slider(left_column, "Ramp Up Time:", 0.1, 1.0, ramp_up_time, update_ramp_time)
    create_slider(left_column, "Motor On Duration:", 0.1, 2.0, motor_on_duration, update_on_duration)
    create_slider(left_column, "Motor Off Duration:", 0.1, 2.0, motor_off_duration, update_off_duration)

    create_slider(right_column, "Notch Pause Duration:", 0.01, 0.1, notch_pause_duration, update_notch_pause)
    create_slider(right_column, "Notch Delay Time:", 0.0, 1.0, notch_delay, update_notch_delay)

def create_slider(frame, label_text, from_val, to_val, initial_val, command_func):
    label = tk.Label(frame, text=label_text, bg='black', fg='white', font=("Arial", 14))
    label.pack(pady=(10, 0))
    slider = tk.Scale(frame, from_=from_val, to=to_val, resolution=(to_val - from_val) / 100, orient=tk.HORIZONTAL, font=("Arial", 12), length=200, sliderlength=50, command=command_func)
    slider.set(initial_val)
    slider.pack(pady=(0, 10))

root = tk.Tk()
root.title("PULSE Stepper Motor Control")
root.geometry("1024x600")
root.configure(bg='black')

setup_tabs()

root.mainloop()
