
# ----------------------------------------------
# Title:   gp2y0e02b_burner.py
# Author:  Alp Sayin
# Translated from: GP2Y0E02B_E-Fuse_Burner/E_FUSE.ino
# Originally from: Martin Pålsson
# Description: E-Fuse programmer for SHARP GP2Y0E02B, GP2Y0E03
# -----------------------------------------------

import sys
import time
import traceback
import argparse
import smbus2 as smbus
import RPi.GPIO as GPIO

I2C_CHANNEL = 1
VPP_PIN = 17
CURRENT_ADDRESS = 0x40
SETADDR = 0x70 << 1  # 4 MSB, will bitshift when used
DRY_RUN = False
# apparently this didnt quite work for the original author
ENABLE_VERIFICATION = False

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

    if not DRY_RUN:
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
    if not DRY_RUN:
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
            if not DRY_RUN:
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
            if not DRY_RUN:
                GPIO.output(VPP_PIN, GPIO.LOW)
            print("Vpp terminal is grounded.")

            print("Stage 10 - 1' started.")
            wire.write_sequence(CURRENT_ADDRESS, 0xEC, 0xFF)
            print("Data = 0xFF is set in Address = 0xEC")
            if not DRY_RUN:
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
            if not DRY_RUN:
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
    GPIO.output(VPP_PIN, GPIO.LOW)
    wire = Wire(dev=dev)

def loop():
    global wire
    scan_results = wire.scan_bus()
    print(f'I2C Bus scan results:')
    for dev in scan_results:
        print(f'0x{dev:0x} ({dev}d)')
    if scan_results:
        if CURRENT_ADDRESS in scan_results:
            if SETADDR not in scan_results:
                EFuseSlaveID(SETADDR)
            else:
                print(f'Problem scanning i2c bus: {CURRENT_ADDRESS} found, but {SETADDR} also exists')
        else:
            print(f'Problem scanning i2c bus: {CURRENT_ADDRESS} not found')
    else:
        print(f'Problem scanning i2c bus: no i2c devices found')
    return


def auto_int(x):
    '''From https://stackoverflow.com/a/25513044'''
    return int(x, 0)

def clean_exit(*args, **kwargs):
    GPIO.cleanup()
    sys.exit(*args, **kwargs)

def main():
    global I2C_CHANNEL, CURRENT_ADDRESS, SETADDR, DRY_RUN
    parser = argparse.ArgumentParser()
    parser.add_argument('--dev', '--channel', '-d', '-c', dest='dev', type=int, default=1, help='I2C Channel Number')
    parser.add_argument('--current-address', '-ca', dest='sharp_address', type=auto_int, default=CURRENT_ADDRESS, help='Current sensor I2C address')
    parser.add_argument('--new-address', '-na', dest='new_address', type=auto_int, default=SETADDR, help='Desired sensor I2C address')
    parser.add_argument('--dry-run', '-dr', action='store_true', dest='dry_run', help='Desired sensor I2C address')
    args = vars(parser.parse_args())
    print(f'args: {args}')

    I2C_CHANNEL = args['dev']
    CURRENT_ADDRESS = args['sharp_address']
    SETADDR = args['new_address']
    DRY_RUN = args['dry_run']

    if CURRENT_ADDRESS == SETADDR:
        print(f'Target address=0x{SETADDR:0x} is the same as current address=0x{CURRENT_ADDRESS:0x}')
        print(f'Nothing to do; exiting...')
        clean_exit(1)

    if DRY_RUN:
        print(f'*** Running in dry-run mode. Nothing will be written. ***')

    setup(dev=I2C_CHANNEL)
    loop()

if __name__ == "__main__":
    try:
        main()
        clean_exit(0)
    except Exception as ex:
        GPIO.cleanup()
        raise ex
