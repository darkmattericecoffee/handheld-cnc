import os
import time
import serial
import numpy as np
from pathlib import Path
import pandas as pd
import serial.tools.list_ports

# TODO:
# - Add windows compatability (does not have usbmodem in name for get_serial_ports)


def get_serial_ports():
    ports = serial.tools.list_ports.comports()
    logger_ports = [port.device for port in ports if "usbmodem" in port.device]
    if len(logger_ports) != 2:
        print("Error! Dual Serial Ports not found!")
        return None
    else:
        return logger_ports


def establish_serial(baud_rate=9600):
    serial_ports = get_serial_ports()
    ser_data = serial.Serial(serial_ports[0], baud_rate)
    ser_command = serial.Serial(serial_ports[1], baud_rate)
    if not ser_data.is_open:
        ser_data.open()
    if not ser_command.is_open:
        ser_command.open()

    ser_data.reset_input_buffer()
    ser_command.reset_output_buffer()

    _flush_all(ser_data, ser_command)

    ser_command.write(b"u")
    ser_data.write(b"u")

    while ser_data.in_waiting < 1 and ser_command.in_waiting < 1:
        pass

    data_response = ser_data.read(ser_data.in_waiting).decode("utf-8")
    command_response = ser_command.read(ser_command.in_waiting).decode("utf-8")

    if command_response == "x":
        ser_data, ser_command = ser_command, ser_data
        # print("Flip!")
    elif data_response == "x":
        # print("No flip necessary")
        pass
    else:
        print("ERROR! No response from teensy!")
        return None

    _flush_all(ser_data, ser_command)
    return ser_data, ser_command


def _flush_all(ser_data, ser_command):
    ser_data.flushInput()
    ser_data.flush()
    ser_command.flushInput()
    ser_command.flush()


def put_device_ID(device_ID):
    ser_data, ser_command = establish_serial()
    ser_command.write(b"p")  # eeprom put
    # time.sleep(1)
    device_ID_OG = device_ID
    device_ID += "x"
    print("device_ID to write = ", device_ID)
    ser_data.write(bytes(device_ID, "utf-8"))
    while ser_data.in_waiting < len(device_ID) - 1:
        pass  # wait til device_ID is written back

    check_device_ID = ser_data.read_all().decode("utf-8")
    print("Device ID is: ", check_device_ID)

    _close_serial(ser_data, ser_command)
    if check_device_ID == device_ID_OG:  # TODO: error handling
        return True
    else:
        return False


def _close_serial(ser_data, ser_command):
    ser_data.close()
    ser_command.close()


def _open_serial(ser_data, ser_command):
    ser_data.open()
    ser_command.open()


def get_device_ID():
    ser_data, ser_command = establish_serial()
    ser_command.write(b"g")
    device_ID = b""
    while b"x" not in device_ID:
        if ser_data.in_waiting > 0:
            device_ID += ser_data.read(ser_data.in_waiting)
    device_ID = device_ID.decode("utf-8")

    _close_serial(ser_data, ser_command)

    return device_ID[:-1]


def put_fw_ver(fw_ver):
    ser_data, ser_command = establish_serial()
    _flush_all(ser_data, ser_command)
    ser_command.write(b"b")  # eeprom put device

    fw_ver_OG = fw_ver
    fw_ver += "x"
    ser_data.write(bytes(fw_ver, "utf-8"))
    while ser_data.in_waiting < len(fw_ver) - 1:
        pass  # wait til fw_ver is written back

    check_fw_ver = ser_data.read_all().decode("utf-8")
    print(fw_ver)

    _close_serial(ser_data, ser_command)

    if check_fw_ver == fw_ver_OG:
        return True
    else:
        return False


def get_fw_ver():
    ser_data, ser_command = establish_serial()
    _flush_all(ser_data, ser_command)
    ser_command.write(b"a")
    fw_ver = b""
    while b"x" not in fw_ver:
        if ser_data.in_waiting > 0:
            fw_ver += ser_data.read(ser_data.in_waiting)
    fw_ver = fw_ver.decode("utf-8")
    ser_data.close()
    return fw_ver[:-1]
