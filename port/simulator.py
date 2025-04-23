import serial, time, random, struct, threading

port = serial.Serial(
    port="/home/arca/port1",
    baudrate=460800,
    bytesize=8,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=None,
)

if not port.is_open:
    port.open()


# 生成数据
class Simulator:
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
        # if time.time() - self.T >= 10:  # re-generate
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

    def __str__(self):
        return f"roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}, color: {self.color}, mode: {self.mode}, shoot: {self.shoot}"

def send_data():
    simulator = Simulator()
    T = time.time()
    while True:
        data = simulator()
        port.write(data)
        if time.time() - T >= 1:
            print(simulator)
            T = time.time()
        time.sleep(0.1)
        
def receive():
    T = time.time()
    while True:
        if port.in_waiting > 0:
            data = port.read(port.in_waiting)
            if time.time() - T >= 1:
                print("Received:", data)
                T = time.time()
            time.sleep(0.1)
            
threading.Thread(target=send_data).start()
threading.Thread(target=receive).start()