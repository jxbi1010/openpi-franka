import time
import serial
import contextlib
import os
from pyrobotiqgripper import RobotiqGripper

class RobotiqInterface:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port

        with open(os.devnull, 'w') as f, contextlib.redirect_stdout(f), contextlib.redirect_stderr(f):
            self.gripper = RobotiqGripper(portname=self.port)

        self.gripper.connect()
        self.activate()

    def reset(self):
        with serial.Serial(self.port, 115200, timeout=1) as ser:
            ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
            time.sleep(0.5)

    def activate(self):
        try:
            self.gripper.activate()
        except Exception as e:
            print(f"Activation failed: {e}, resetting and trying again")
            self.reset()
            self.activate()

    @property
    def is_active(self):
        """Checks if the gripper is actually activated and fault-free."""
        status = self.gripper.readStatus()
        return status and status[1] == 3 and status[5] == 0

    @property
    def position(self):
        """Returns current position 0-255."""
        return self.gripper.getPosition()

    def move(self, pos, speed=255, force=100):
        pos = max(10, min(pos, 245))
        self.gripper.move(pos, speed=speed, force=force)

    def open(self):
        self.gripper.open()

    def close(self):
        self.gripper.close()

    def disconnect(self):
        self.gripper.disconnect()

# --- Refined Usage ---
if __name__ == "__main__":

    gripper = RobotiqInterface('/dev/ttyUSB0')
    print(f"Gripper is active: {gripper.is_active}")
    print(f"Gripper position: {gripper.position}")

    target = 100
    print(f"Moving to {target}...")
    gripper.move(target)
    
    # Wait until movement is finished or object detected
    while True:
        pos = gripper.position
        print(f"Position: {pos}", end="\r")
        
        if abs(pos - target) < 2:
            break
        time.sleep(0.1)
        
    print(f"\nMotion finished. Object status: {gripper.position}")
    
    time.sleep(1)
    gripper.open()
    gripper.disconnect()
