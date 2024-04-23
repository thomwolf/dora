#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dora import Node

import serial
import pyarrow as pa
import numpy as np

COM_PORT = '/dev/tty.usbserial-120'

POT_MIN_MAX = [(500, 850),
               (0, 1000),
               (0, 1000),
               (0, 1000),
               (0, 1000),
               (0, 1000)]

SERVO_MIN_MAX = [(0.0, 1.0),
                 (0.0, 1.0),
                 (0.0, 1.0),
                 (0.0, 1.0),
                 (0.0, 1.0),
                 (0.0, 1.0)]

arduino = serial.Serial(port=COM_PORT, baudrate=115200, timeout=.1)
dora_node = Node()

def read_positions(line_length=13, max_tests=100) -> pa.Array:
    """ Read a line from the serial port and return it. If the line is not the expected length, try again up to max_tests times.

    :param line_length: the expected length of the line
    :param max_tests: the maximum number of times to try reading the line
    :return: the line read from the serial port
    """
    arduino.reset_input_buffer()  # flush the input buffer to get the most recent data
    _ = arduino.readline()  # dump the first reading
    data, tests = "", 0
    while len(data) != line_length and tests < max_tests:
        data = arduino.readline()  # read a line

    # unparse the data (it's an array of uint16_t positions)
    arr = np.frombuffer(data[:-1], dtype=np.int16)
    print(f"arr original: {arr}", flush=True)
    arr = arr.astype(np.float32)
    for i in range(6):
        arr[i] = servo_convert(arr[i], *POT_MIN_MAX[i], *SERVO_MIN_MAX[i])
    print(f"arr converted: {arr}", flush=True)
    arrow_array = pa.array(arr)
    # print(f"arr: {arr}", flush=True)
    return arrow_array


def servo_convert(pot_post: float,
                  pot_min: float = 0,
                  pot_max: float = 1000,
                  servo_min: float = 0,
                  servo_max: float = 1000,
                  flip: bool = False) -> float:
    """ Convert the position of Hiwonder sync arm potentiometer to the normalized position of a servo
    Args:

    """
    pot_post = float(pot_post)
    pot_min = float(pot_min)
    pot_max = float(pot_max)
    servo_min = float(servo_min)
    servo_max = float(servo_max)

    # Between 0 and 1
    normalized_pos = (pot_post - pot_min) / (pot_max - pot_min)
    if flip:
        p = 1.0 - p

    p = normalized_pos * (servo_max - servo_min) + servo_min

    # Clip the value to the servo range
    if p < servo_min:
        p = servo_min
    elif p > servo_max:
        p = servo_max
    return p

print("looping", flush=True)

while True:
    event = dora_node.next()
    if event["type"] == "INPUT":
        data = read_positions()
        dora_node.send_output("positions", data, event["metadata"])
    elif event["type"] == "STOP" or event["type"] == "ERROR":
        break

