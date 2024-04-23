#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dora import Node

import serial
import pyarrow as pa
import numpy as np

COM_PORT = '/dev/tty.usbserial-120'

POT_MIN = [500] + [0.0 for _ in range(5)]
POT_MAX = [800] + [1000 for _ in range(5)]
SERVO_MIN = [-3.14, -3.14,  -3.14,  -3.14,  -3.14,  -3.14]
SERVO_MAX = [3.14,  0.0,   0.0,   3.14,   0.0,    0.0]
FLIP =      [False, False,  True,   False,  True,   False]

arduino = serial.Serial(port=COM_PORT, baudrate=115200, timeout=.1)
dora_node = Node()

def read_positions(line_length=15, max_tests=100):
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
    # print(f"data: {data}", flush=True)
    np_arr = np.frombuffer(data[:-3], dtype=np.int16).astype(np.float32)
    key_1 = bool(data[-3])
    key_2 = bool(data[-2])
    print(f"np_arr: {np_arr}", flush=True)
    print(f"key_1: {key_1}", flush=True)
    print(f"key_2: {key_2}", flush=True)
    return np_arr, key_1, key_2


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
        normalized_pos = 1.0 - normalized_pos

    p = normalized_pos * (servo_max - servo_min) + servo_min

    # Clip the value to the servo range
    if p < servo_min:
        p = servo_min
    elif p > servo_max:
        p = servo_max
    return p

print("looping", flush=True)

is_calibration = False
pot_min = POT_MIN
pot_max = POT_MAX

# is_calibration = True
# pot_min = np.array([np.inf] * 6, dtype=float)
# pot_max = np.array([-np.inf] * 6, dtype=float)

clean_data = np.array([0.0] * 6, dtype=float)
while True:
    event = dora_node.next()
    if event["type"] == "INPUT":
        data, key_1, key_2 = read_positions()

        if is_calibration:
            for i in range(6):
                if data[i] > pot_max[i]:
                    pot_max[i] = data[i]
                if 0 <= data[i] < pot_min[i]:  # ignore negative values (-1) – bad readings
                    pot_min[i] = data[i]
            print(f"pot_min: {pot_min}", flush=True)
            print(f"pot_max: {pot_max}", flush=True)
        else:
            # Remove -1 values – bad readings
            for i in range(6):
                if data[i] == -1:
                    data[i] = clean_data[i]
                else:
                    clean_data[i] = data[i]

            for i in range(6):
                data[i] = servo_convert(data[i], pot_min[i], pot_max[i], SERVO_MIN[i], SERVO_MAX[i], FLIP[i])
            print(f"arr converted: {data}", flush=True)
            dora_node.send_output("positions", pa.array(data), event["metadata"])
        if key_1 or key_2:
            is_calibration = False

    elif event["type"] == "STOP" or event["type"] == "ERROR":
        break

