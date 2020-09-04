
# ----------------------------------------------
# Title:   gp2y0e02b_burner.py
# Author:  Alp Sayin
# Translated from: GP2Y0E02B_E-Fuse_Burner/E_FUSE.ino
# Originally from: Martin Pålsson
# Description: E-Fuse programmer for SHARP GP2Y0E02B, GP2Y0E03
# -----------------------------------------------


import random
import time
import traceback
import argparse
from math import nan as NAN
from pathlib import Path
import smbus2 as smbus
import RPi.GPIO as GPIO

I2C_CHANNEL = 1
VPP_PIN = 17
CURRENT_ADDRESS = 0x40
SETADDR = 0x70 << 1  # 4 MSB, will bitshift when used
ENABLE_VERIFICATION = False  # apparently this didnt quite work for the original author

wire = None

class Wire(object):
    def __init__(self, dev=1):
        self.dev = dev
        self.bus = smbus.SMBus(1)  # open serial port

    def write_sequence(self, devAddr, regAddr, data):
        self.bus.write_byte_data(devAddr, regAddr, data)

    def read_sequence(self, devAddr, regAddr):
        return self.bus.read_byte_data(devAddr, regAddr)

    def scan_bus(self):
        i2c_devices = list()
        for address in range(0, 127):
            try:
                try:
                    self.bus.read_byte(address)
                    i2c_devices.append(address)
                except OSError as ose:
                    if ose.errno == 121:
                        # print(f'No device in {address}: {ose}')
                        pass
                    else:
                        raise ose
            except Exception as ex:
                print(f'Unexpected exception caught: {ex}')
                traceback.print_exc()
        return i2c_devices


def EFuseSlaveID(newID):
    global wire

    print("Stage 1 started.")

    wire.write_sequence(CURRENT_ADDRESS, 0xEC, 0xFF)
    print("Data = 0xFF is set in Address = 0xEC")

    GPIO.output(VPP_PIN, GPIO.HIGH)
    print("3.3V is applied in the Vpp terminal")

    print("Stage 2 started.")
    wire.write_sequence(CURRENT_ADDRESS, 0xC8, 0x00)
    print("Data = 0x00 is set in Address = 0xC8")

    print("Stage 3 started.")
    wire.write_sequence(CURRENT_ADDRESS, 0xC9, 0x45)
    print("Data = 0x45 is set in Address = 0xC9")

    # THIS IS WHERE THE ADDRESS WILL BE SET! */
    print("Stage 4 started.")
    wire.write_sequence(CURRENT_ADDRESS, 0xCD, newID >> 4)
    print("Data = SETADDR >> 4 is set in Address = 0xCD")

    print("Stage 5 started.")
    wire.write_sequence(CURRENT_ADDRESS, 0xCA, 0x01)
    print("Data = 0x01 is set in Address = 0xCA")
    print("Wait for 500 us")
    time.sleep(1e-6*500)

    print("Stage 6 started.")
    wire.write_sequence(CURRENT_ADDRESS, 0xCA, 0x00)
    print("Data = 0x00 is set in Address = 0xCA")
    GPIO.output(VPP_PIN, GPIO.LOW)
    print("Vpp terminal grounded.")

    print("Stage 7 started.")
    wire.write_sequence(CURRENT_ADDRESS, 0xEF, 0x00)
    print("Data = 0x00 is set in Address = 0xEF")
    wire.write_sequence(CURRENT_ADDRESS, 0xC8, 0x40)
    print("Data = 0x40 is set in Address = 0xC8")
    wire.write_sequence(CURRENT_ADDRESS, 0xC8, 0x00)
    print("Data = 0x00 is set in Address = 0xC8")

    print("Stage 8 started.")
    wire.write_sequence(CURRENT_ADDRESS, 0xEE, 0x06)
    print("Data = 0x06 is set in Address = 0xEE")

    print("Stage 9 started.")
    wire.write_sequence(CURRENT_ADDRESS, 0xEC, 0xFF)
    print("Data = 0xFF is set in Address = 0xEC")
    wire.write_sequence(CURRENT_ADDRESS, 0xEF, 0x03)
    print("Data = 0x03 is set in Address = 0xEF")
    print("Read out the data in Address = 0x27.")
    print("Data: 0B")
    x27Val = wire.read_sequence(CURRENT_ADDRESS, 0x27)
    print(f'{x27Val}')
    wire.write_sequence(CURRENT_ADDRESS, 0xEF, 0x00)
    print("Data = 0x00 is set in Address = 0xEF")
    wire.write_sequence(CURRENT_ADDRESS, 0xEC, 0x7F)
    print("Data = 0x7F is set in Address = 0xEC")

    checkSum = wire.read_sequence(CURRENT_ADDRESS, 0x27)
    if (checkSum & 0b11111) != 1:
        print(f"Checksum? is 0b{checkSum:0b}")

    if ENABLE_VERIFICATION:
        checkSum = wire.read_sequence(CURRENT_ADDRESS, 0x27)
        # if (checkSum & 0b11111) != 1:
        if checkSum == 0b10001:
            print("ERROR!   >:(\nE-fuse probably broken.")
            print("Data: 0B")
            x27Val = wire.read_sequence(CURRENT_ADDRESS, 0x27)
            print(f'{x27Val}')

            print("Stage 10 - 1 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xEC, 0xFF)
            print("Data = 0xFF is set in Address = 0xEC")
            GPIO.output(VPP_PIN, GPIO.HIGH)
            print("3.3V is applied in Vpp terminal")

            print("Stage 10 - 2 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xC8, 0x37)
            print("Data = 0x37 is set in Address = 0xC8")

            print("Stage 10 - 3 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xC9, 0x74)
            print("Data = 0x74 is set in Address = 0xC9")

            print("Stage 10 - 4 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xCD, 0x04)
            print("Data = 0x04 is set in Address = 0xCD")

            print("Stage 10 - 5 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xCA, 0x01)
            print("Data = 0x01 is set in Address = 0xCA")
            time.sleep(1e-6*500)
            print("Wait for 500 us.")

            print("Stage 10 - 6 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xCA, 0x00)
            print("Data = 0x00 is set in Address = 0xCA")
            GPIO.output(VPP_PIN, GPIO.LOW)
            print("Vpp terminal is grounded.")

            print("Stage 10 - 1' started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xEC, 0xFF)
            print("Data = 0xFF is set in Address = 0xEC")
            GPIO.output(VPP_PIN, GPIO.HIGH)
            print("3.3V is applied in Cpp terminal")

            print("Stage 10 - 2' started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xC8, 0x3F)
            print("Data = 0x3F is set in Address = 0xC8")

            print("Stage 10 - 3' started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xC9, 0x04)
            print("Data = 0x04 is set in Address = 0xC9")

            # THIS IS WHERE THE CURRENT_ADDRESS IS PROGRAMMED */
            print("Stage 10 - 4' started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xCD, newID >> 4)
            print("Data = 0x08 is set in Address = 0xCD")

            print("Stage 10 - 5' started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xCA, 0x01)
            print("Data = 0x01 is set in Address = 0xCA")
            time.sleep(1e-6*500)
            print("Wait for 500 us.")

            print("Stage 10 - 6' started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xCA, 0x00)
            print("Data = 0x00 is set in Address = 0xCA")
            GPIO.output(VPP_PIN, GPIO.LOW)
            print("Vpp terminal is grounded.")

            print("Stage 10 - 7 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xEF, 0x00)
            print("Data = 0x00 is set in Address = 0xEF")

            wire.write_sequence(CURRENT_ADDRESS, 0xC8, 0x40)
            print("Data = 0x40 is set in Address = 0xC8")

            wire.write_sequence(CURRENT_ADDRESS, 0xC8, 0x00)
            print("Data = 0x00 is set in Address = 0xC8")

            print("Stage 10 - 8 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xEE, 0x06)
            print("Data = 0x06 is set in Address = 0xEE")

            print("Stage 10 - 9 started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xEC, 0xFF)
            print("Data = 0xFF is set in Address = 0xEC")
            wire.write_sequence(CURRENT_ADDRESS, 0xEF, 0x03)
            print("Data = 0x03 is set in Address = 0xEF")

            x18Val = wire.read_sequence(CURRENT_ADDRESS, 0x18)
            x19Val = wire.read_sequence(CURRENT_ADDRESS, 0x19)

            print(f"0x18 = {x18Val:0x}")
            print(f"\tx19Val = {x19Val:0x}")

            if x18Val != 0x82 or x19Val != 0x00:
                print("Not possible to correct error.")
            else:
                print("E-Fuse programming finished with bit replacement.")
        else:
            print("E-Fuse programming finished.")
        pass

def setup(dev):
    global wire
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(VPP_PIN, GPIO.OUT)
    wire = Wire(dev=dev)

def loop():
    global wire
    scan_results = wire.scan_bus()
    print(f'I2C Bus scan results:')
    for dev in scan_results:
        print(f'0x{dev:0x} ({dev}d)')
    if scan_results:
        EFuseSlaveID(SETADDR)
    else:
        print(f'Problem scanning i2c bus')
    return

def main():
    setup(dev=I2C_CHANNEL)
    loop()
    return 0

if __name__ == "__main__":
    try:
        main()
        GPIO.cleanup()
    except Exception as ex:
        GPIO.cleanup()
        raise ex
