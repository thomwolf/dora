#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dora import Node

import serial
import pyarrow as pa
import struct

COM_PORT = '/dev/tty.usbserial-120'

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

    print(data, flush=True)
    
    # unparse the data (it's an array of uint16_t positions)
    buf = pa.py_buffer(data[:-1])  # remove the newline
    print(f"buf: {buf}", flush=True)
    arr = pa.Int16Array.from_buffers(type=pa.int16(), length=6, buffers=[None, buf])
    # print(f"arr: {arr}", flush=True)
    return arr


def servo_convert(pot_post: int,
                pot_min: int = 0,
                pot_max: int = 1000,
                servo_min: int = 0,
                servo_max: int = 1000,
                flip: bool = False) -> int:
    """ Convert the position of Hiwonder sync arm potentiometer to the normalized position of a servo
    Args:

    """
    normalized_pos = (pot_post - pot_min) / (pot_max - pot_min)
    p = normalized_pos * (servo_max - servo_min) + servo_min
    p = int(p)
    
    if flip:
        p = pot_max - p
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
        # time.sleep(0.05)
        print(f"Arduino data: {data.to_numpy()}", flush=True)
        dora_node.send_output("positions", data, event["metadata"])
    elif event["type"] == "STOP" or event["type"] == "ERROR":
        break

