"""
Manual hardware smoke test — run on RPi 5 with Teensy 4.1 connected.
Requires: Teensy flashed with firmware/mower_firmware/mower_firmware.ino
          udev rule installed, /dev/mower symlink exists
"""
from mower.hal.serial_driver import SerialDriver
from mower.hal.hardware_interface import HardwareInterface
import time

driver = SerialDriver(port='/dev/mower', baud=921600)
hw = HardwareInterface(driver=driver)
hw.on_sensors = lambda d: print("Sensors:", d)
hw.on_soc = lambda d: print("SOC:", d)
hw.on_status = lambda d: print("Status:", d)
hw.start()  # starts Serial-Driver + automatic PING sender (10 Hz)

time.sleep(1.0)  # stabilise Teensy connection

print("Driving forward for 2 seconds...")
hw.drive(speed=0.3, steering=0.0)
time.sleep(2.0)
hw.drive(speed=0.0, steering=0.0)
time.sleep(0.5)

print("ESTOP")
hw.estop()
hw.stop()
print("Done.")
