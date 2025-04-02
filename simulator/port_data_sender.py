import serial
import struct
import time
import random

# 使用 /dev/pty/2 作为接收数据串口，/dev/pty/3 作为发送数据串口
ser = serial.Serial(
    port="/dev/pts/3",
    baudrate=9600,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=None,
)

if not ser.is_open:
    ser.open()


# 生成数据
class Emulator:
    def __init__(self):
        self.T = time.time()

        # car position
        self.start = 0x3A
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.color = 1
        self.mode = 1
        self.shoot = 0
        self.end = 0xAA

    def __call__(self):
        if time.time() - self.T >= 10:  # re-generate
            self.T = time.time()
            self.roll = random.uniform(-180, 180)
            self.pitch = random.uniform(-180, 180)
            self.yaw = random.uniform(-180, 180)
            self.mode = random.randint(0, 255)
            self.shoot = random.randint(0, 255)

        packs = struct.pack(
            "=BfffBBBB",
            self.start,
            self.roll,
            self.pitch,
            self.yaw,
            self.color,
            self.mode,
            self.shoot,
            self.end,
        )
        print(struct.calcsize("=BfffBBBB"))
        return packs


data = Emulator()
while True:
    x = data()
    ser.write(x)
    print(data.roll, data.pitch, data.yaw)
    time.sleep(5)
