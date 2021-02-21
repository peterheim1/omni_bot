#!/usr/bin/env python3

import tkinter as tk
import tk_tools

from tk_tools.images import rotary_gauge_volt
#from SerialDataGateway import SerialDataGateway
root = tk.Tk()
led = tk_tools.Led(root, size=50,)
led.to_red
led.grid(row=0, column=2)
p1 = tk_tools.RotaryScale(root,
                          max_value=180.0,
                          size=100,
                          needle_thickness=3,
                          needle_color='red',
                          img_data=rotary_gauge_volt)
p1.grid(row=1, column=0)

p2 = tk_tools.RotaryScale(root,
                          max_value=180.0,
                          size=100,
                          needle_thickness=3,
                          needle_color='green',
                          img_data=rotary_gauge_volt)

p2.grid(row=1, column=1)

p3 = tk_tools.RotaryScale(root,
                          max_value=180.0,
                          size=100,
                          needle_thickness=3,
                          needle_color='blue',
                          img_data=rotary_gauge_volt)

p3.grid(row=2, column=0)

p4 = tk_tools.RotaryScale(root,
                          max_value=180.0,
                          size=100,
                          needle_thickness=3,
                          needle_color='black',
                          img_data=rotary_gauge_volt)

p4.grid(row=2, column=1)

increment = 10.0
value = 90.0
front_value = 90.0
rear_value = 90.0
status = "dis"


def Set():
    global value
    global front_value
    global rear_value
    front_value = 90
    rear_value = 90
    value = 90

    p1.set_value(90)
    p2.set_value(90)
    p3.set_value(90)
    p4.set_value(90)


def inc():
    global value
    value += increment

    p1.set_value(value)
    p2.set_value(value)
    p3.set_value(value)
    p4.set_value(value)


def dec():
    global value
    value -= increment

    p1.set_value(value)
    p2.set_value(value)
    p3.set_value(value)
    p4.set_value(value)

def rot_inc():
    global front_value
    global rear_value
    front_value -= increment
    rear_value += increment

    p1.set_value(front_value)
    p2.set_value(front_value)
    p3.set_value(rear_value)
    p4.set_value(rear_value)

def rot_dec():
    global front_value
    global rear_value
    front_value += increment
    rear_value -= increment

    p1.set_value(front_value)
    p2.set_value(front_value)
    p3.set_value(rear_value)
    p4.set_value(rear_value)

def con():
    print("conected")
    led.to_green


inc_btn = tk.Button(root,
                    text='increment by {}'.format(increment),
                    command=inc)

inc_btn.grid(row=3, column=0, columnspan=2, sticky='news')

dec_btn = tk.Button(root,
                    text='decrement by {}'.format(increment),
                    command=dec)

dec_btn.grid(row=4, column=0, columnspan=2, sticky='news')

rot_i_btn = tk.Button(root,
                    text='rotate inc {}'.format(increment),
                    command=rot_inc)

rot_i_btn.grid(row=5, column=0, columnspan=2, sticky='news')

rot_d_btn = tk.Button(root,
                    text='rotate dec {}'.format(increment),
                    command=rot_dec)

rot_d_btn.grid(row=6, column=0, columnspan=2, sticky='news')

set_btn = tk.Button(root,
                    text='reset {}'.format(90),
                    command=Set)

set_btn.grid(row=7, column=0, columnspan=2, sticky='news')

con_btn = tk.Button(root,
                    text='conect',
                    command=led.to_green)


con_btn.grid(row=0, column=0, sticky='news')

dis_btn = tk.Button(root,
                    text='disconect',
                    command=led.to_red)


dis_btn.grid(row=0, column=1, sticky='news')


root.mainloop()
