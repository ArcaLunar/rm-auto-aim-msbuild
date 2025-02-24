import struct
import serial

ser = serial.Serial(
    port="/dev/pts/6",
    baudrate=9600,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=None,
)

if not ser.is_open:
    ser.open()

while True:
    data = ser.read(17)
    if len(data) == 17:
        start, roll, pitch, yaw, color, mode, shoot, end = struct.unpack(
            "=BfffBBBB", data
        )
        print(
            f"start: {start}, roll: {roll}, pitch: {pitch}, yaw: {yaw}, color: {color}, mode: {mode}, shoot: {shoot}, end: {end}"
        )
