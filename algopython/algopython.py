import serial.tools.list_ports
import os
import threading
import queue
import time
import serial
import math

__all__ = ['move', 'light', 'playSound', 'wait', 'listAvailableSounds','moveStop','wait_sensor',
           'lightStop','soundStop','rotations','get_sensor_value','FOREVER']

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)

ser = None
serial_lock = threading.Lock()
serial_command_queue = queue.Queue()
serial_worker_running = False
status_thread = None
status_thread_running = False

CMD_REPLY_MAP = {
    0x10: 0x80,  # MOVE_REQ         -> MOVE_REP
    0x11: 0x81,  # LIGHT_REQ        -> LIGHT_REP
    0x12: 0x82,  # PLAY_SOUND_REQ   -> PLAY_SOUND_REP
    0x13: 0x83,  # MOVE_STOP_REQ    -> MOVE_STOP_REP
    0x14: 0x84,  # LIGHT_STOP_REQ   -> LIGHT_STOP_REP
    0x15: 0x85,  # SOUND_STOP_REQ   -> SOUND_STOP_REP 
    0x16: 0x86,  # LIGHT12_REQ      -> LIGHT12_REP (not implemented yet)
    0x17: 0x87,  # WAIT_SENSOR_REQ  -> WAIT_SENSOR_REP
    0x18: 0x88,  # GET_SENSOR_REQ   -> GET_SENSOR_REP
    0x19: 0x89,  # GET_STATUS_REQ   -> GET_STATUS_REP
}

class SerialCommand:
    def __init__(self, cmd, payload, expect_reply=True):
        self.cmd = cmd
        self.payload = payload
        self.expect_reply = expect_reply
        self.response = None
        self.done = threading.Event()

def start_serial_worker():
    global serial_worker_running
    if not serial_worker_running:
        serial_worker_running = True
        threading.Thread(target=serial_worker_loop, daemon=True).start()

def serial_worker_loop():
    global ser
    while serial_worker_running:
        try:
            command = serial_command_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        with serial_lock:
            result = send_packet(
                command.cmd,
                command.payload,
                wait_done=True,
                verbose=True
            )
            command.response = result
            command.done.set()

def stop_status_monitor():
    global status_thread_running
    status_thread_running = False
    print("Status monitor stopped.")

class DeviceStatus:
    def __init__(self):
        # Motors
        self.motor1 = False 
        self.motor2 = False 
        self.motor3 = False 

        # LEDs
        self.led1 = False
        self.led2 = False

        # Sound
        self.sound = False

        # Sensors (state and values)
        self.sensor1 = False
        self.sensor2 = False
        self.sensor1_value = 0.0
        self.sensor2_value = 0.0


g_algopython_system_status = DeviceStatus()

def serial_thread_task():
    last_status_time = time.time()
    while True:
        # Check if it's time to fetch the brain status (every 10ms)
        now = time.time()

        # if (now - last_status_time) >= 1:  # 10ms = 0.01 seconds
        if (now - last_status_time) >= 0.05:  # 10ms = 0.01 seconds
            serial_get_brain_status()
            last_status_time = now

        # Process queued commands (non-blocking)
        try:
            command = serial_command_queue.get_nowait()
            serial_send_next_command(command)
        except queue.Empty:
            pass

        # Short sleep to avoid busy-loop hogging CPU
        time.sleep(0.001)  # 1ms pause


def serial_thread_start():
    threading.Thread(target=serial_thread_task, daemon=True).start()

def serial_send_next_command(command):
    result = send_packet(
            command.cmd,
            command.payload,
            wait_done=command.expect_reply,
            verbose=True
            )
    command.response = result
    command.done.set()

def serial_get_brain_status():
    global g_algopython_system_status
    response = serial_send_command(0x19, b"", expect_reply=True)

    if not response or len(response) < 10:
        return "?, ?, ?, ?, ?, ?, ?, ?, ?, ?"

    g_algopython_system_status.motor1 = response[0]
    g_algopython_system_status.motor2 = response[1]
    g_algopython_system_status.motor3 = response[2]
    g_algopython_system_status.led1 = bool(response[3])
    g_algopython_system_status.led2 = bool(response[4])
    g_algopython_system_status.sound = bool(response[5])
    g_algopython_system_status.sensor1 = bool(response[6])
    g_algopython_system_status.sensor2 = bool(response[7])
    g_algopython_system_status.sensor1_value = response[8]
    g_algopython_system_status.sensor2_value = response[9]

    s = g_algopython_system_status
    print(
        f"Motors: {s.motor1}, {s.motor2}, {s.motor3} | "
        f"LEDs: {int(s.led1)}, {int(s.led2)} | "
        f"Sound: {int(s.sound)} | "
        f"Sensors: Trig1={int(s.sensor1)}, Trig2={int(s.sensor2)}, "
        f"Value1={s.sensor1_value}, Value2={s.sensor2_value}"
    )
def serial_queue_command(cmd, payload, expect_reply=True):
    command = SerialCommand(cmd, payload, expect_reply)
    serial_command_queue.put(command)
    command.done.wait()
    return command.response

def serial_send_command(cmd, payload, expect_reply=True):
    command = SerialCommand(cmd, payload, expect_reply)
    serial_tx_command(command)
    command.done.wait()
    return command.response

def serial_tx_command(command):
    result = send_packet(
            command.cmd,
            command.payload,
            wait_done=command.expect_reply,
            verbose=True
            )
    command.response = result
    command.done.set()

def serial_send_next_command(command):
    result = send_packet(
            command.cmd,
            command.payload,
            wait_done=command.expect_reply,
            verbose=True
            )
    command.response = result
    command.done.set()


def algopython_init(port: str = None):
    time.sleep(2) # Allow time for the system to start up and establish the serial connection
    os.system('cls' if os.name == 'nt' else 'clear')
    global ser
    if not port:
        port = find_usb_serial_port()
        if not port:
            print("USB port not found. Please connect the device and try again.")
            return False
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        print(f"Serial port: {port}")
        serial_thread_start();
        time.sleep(2)
        return True
    except serial.SerialException as e:
        print(f"\nError when opening port: {port}: {e}\n")
        return False

def find_usb_serial_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "USB" in p.description or "CP210" in p.description or "ttyUSB" in p.device:
            return p.device
    return None

def build_packet(cmd: int, payload: bytes) -> bytes:
    if not isinstance(payload, (bytes, bytearray)):
        payload = bytes(payload)
    header = bytes([0xA5, cmd, len(payload)])
    crc = sum(header) % 256
    return header + payload + bytes([crc])

def send_packet(cmd, payload, wait_done=True, delay_after=0.01, retries=2, verbose=True):
    global ser
    if ser is None:
        print("[Error] Serial port is not initialized.")
        return None

    packet = build_packet(cmd, payload)
    expected_reply_cmd = CMD_REPLY_MAP.get(cmd)
    # print(f"Sending packet: {packet.hex()} (CMD: 0x{cmd:02X}, Expected Reply: 0x{expected_reply_cmd:02X})")
    for attempt in range(retries + 1):
        with serial_lock:
            ser.reset_input_buffer()
            # if verbose:
            #     print(f"\n[Try {attempt + 1}] Sending packet: " + ' '.join(f'{b:02X}' for b in packet))
            ser.write(packet)
            time.sleep(delay_after)
            # if wait_done:
            if True:
                reply = wait_for_reply(expected_reply_cmd)
                if reply is not None:
                    return reply
            else:
                return True
    if verbose:
        print(f"[Fail] No reply for CMD 0x{cmd:02X} after {retries + 1} tries.")
    return None

def wait_for_reply(expected_cmd, timeout=1):
    global ser
    start = time.time()
    buffer = bytearray()
    #print(f"Waiting for reply for CMD 0x{expected_cmd:02X}...")
    while time.time() - start < timeout:
        if ser.in_waiting:
            # print("Serianl in waiting: ", ser.in_waiting);
            buffer.extend(ser.read(ser.in_waiting))
        while len(buffer) >= 4:
            # print("Buffer length: ", len(buffer))
            # print("Buffer content: ", buffer.hex())
            if buffer[0] == 0xA5:
                cmd, size = buffer[1], buffer[2]
                total_length = 3 + size + 1
                if len(buffer) >= total_length:
                    crc = buffer[3 + size]
                    #print("CRC: ", crc, "Expected: ", (sum(buffer[:total_length - 1])&0xff) )
                    if cmd == expected_cmd and crc == sum(buffer[:total_length - 1])&0xff:
                        return buffer[3:3+size]
                    buffer = buffer[1:]
                else:
                    break
            else:
                buffer = buffer[1:]
        time.sleep(0.005)
    return None

# You can import send_command from this file into algopython_cmd.py and replace all uses of send_packet.


#--------------------------------------------------------------------------------------------------------------
#-----------------Constants and Mappings----------------------------------------------------------------------
SOUNDS_MAP = {
        1: "SIREN",
        2: "BELL",
        3: "BIRD",
        4: "BEAT",
        5: "DOG",
        6: "MONKEY",
        7: "ELEPHANT",
        8: "APPLAUSE",
        9: "VIOLINE",
        10: "GUITAR",
        11: "ROBOT_LIFT",
        12: "TRUCK",
        13: "SMASH",
        14: "CLOWN",
        15: "CHEERING"
    }


ALGOPYTHON_CMD_MOVE_REQ         =0x10
ALGOPYTHON_CMD_LIGHT_REQ        =0x11
ALGOPYTHON_CMD_PLAY_SOUND_REQ   =0x12
ALGOPYTHON_CMD_MOVE_STOP_REQ    =0x13
ALGOPYTHON_CMD_LIGHT_STOP_REQ   =0x14
ALGOPYTHON_CMD_SOUND_STOP_REQ   =0x15
ALGOPYTHON_CMD_LIGHT12_REQ      =0x16 # not implemented yet
ALGOPYTHON_CMD_WAIT_SENSOR_REQ  =0x17
ALGOPYTHON_CMD_GET_SENSOR_REQ   =0x18
ALGOPYTHON_CMD_GET_STATUS_REQ   =0x19

ALGOPYTHON_CMD_MOVE_REP         =0x80
ALGOPYTHON_CMD_LIGHT_REP        =0x81
ALGOPYTHON_CMD_PLAY_SOUND_REP   =0x82
ALGOPYTHON_CMD_MOVE_STOP_REP    =0x83
ALGOPYTHON_CMD_LIGHT_STOP_REP   =0x84
ALGOPYTHON_CMD_LIGHT12_REP      =0x86 # not implemented yet
ALGOPYTHON_CMD_WAIT_SENSOR_REP  =0x87
ALGOPYTHON_CMD_GET_SENSOR_REP   =0x88
ALGOPYTHON_CMD_GET_STATUS_REP   =0x89

FOREVER = math.inf
# --------------------------------------------------------------------------------------------------------------
#-----------------Move section----------------------------------------------------------------------------------
motor_map = {
    'A': 0b001,
    'B': 0b010,
    'C': 0b100,
    'AB': 0b011,
    'AC': 0b101,
    'BC': 0b110,
    'ABC': 0b111
}

ROTATIONS_TO_SECONDS_MAP = {
    'A': 0.63,
    'B': 0.63,
    'C': 0.63,
    'AB': 0.68,
    'ABC' : 0.68,
    'AC': 0.68,
    'BC': 0.68
}

def move(port:str, duration :float , power: int, direction: int, is_blocking = True):
    if port not in motor_map:
        raise ValueError("Invalid motor")
    if duration < 0 and duration > 10:
        raise ValueError("Duration must be between 0 and 10 seconds")
    if not (0 <= power <= 10):
        raise ValueError("Power must be 0-10")

    motor_port = motor_map[port.upper()];
    motor_power = int((power * 255) / 10);
    motor_direction = direction;
    motor_type = 0;

    if math.isinf(duration):
        print("x is positive infinity")
        motor_type = 1;
        motor_duration = 0;
        is_blocking = False;
    else:
        motor_duration = int(duration * 100);

    payload = bytearray([
        motor_port & 0xFF,
        motor_type & 0xFF,
        (motor_duration >> 24) & 0xFF,
        (motor_duration >> 16) & 0xFF,
        (motor_duration >> 8) & 0xFF,
        (motor_duration) & 0xFF,
        motor_power & 0xFF,
        motor_direction & 0xFF
    ])

    send_packet(ALGOPYTHON_CMD_MOVE_REQ, payload, wait_done=False)

    print("Wait for motor to finish...");

    if motor_port == 0b001:
        motor_prev_status = g_algopython_system_status.motor1;
        while is_blocking:
            if(motor_prev_status == 1) and (g_algopython_system_status.motor1 == 0): 
                print("MotorA completed movement")
                break;
            motor_prev_status = g_algopython_system_status.motor1;
    elif motor_port == 0b010:
        motor_prev_status = g_algopython_system_status.motor2;
        while is_blocking:
            if(motor_prev_status == 1) and (g_algopython_system_status.motor2 == 0):
                print("MotorB completed movement")
                break;
            motor_prev_status = g_algopython_system_status.motor2;
    elif motor_port == 0b100:
        motor_prev_status = g_algopython_system_status.motor3;
        while is_blocking:
            if(motor_prev_status == 1) and (g_algopython_system_status.motor3 == 0):
                print("MotorC completed movement")
                break;
            motor_prev_status = g_algopython_system_status.motor3;
    elif motor_port == 0b011:
        motorA_prev_status = g_algopython_system_status.motor1;
        motorB_prev_status = g_algopython_system_status.motor2;
        while is_blocking:
            if(motorA_prev_status == 1 and motorB_prev_status == 1) and (g_algopython_system_status.motor1 == 0 and g_algopython_system_status.motor2 == 0):
                print("Motors AB completed movement")
                break;
            motorA_prev_status = g_algopython_system_status.motor1;
            motorB_prev_status = g_algopython_system_status.motor2;
    elif motor_port == 0b101:   
        motorA_prev_status = g_algopython_system_status.motor1;
        motorC_prev_status = g_algopython_system_status.motor3;
        while is_blocking:
            if(motorA_prev_status == 1 and motorC_prev_status == 1) and (g_algopython_system_status.motor1 == 0 and g_algopython_system_status.motor3 == 0):
                print("Motors AC completed movement")
                break;
            motorA_prev_status = g_algopython_system_status.motor1;
            motorC_prev_status = g_algopython_system_status.motor3;
    elif motor_port == 0b110:
        motorB_prev_status = g_algopython_system_status.motor2;
        motorC_prev_status = g_algopython_system_status.motor3;
        while is_blocking:
            if(motorB_prev_status == 1 and motorC_prev_status == 1) and (g_algopython_system_status.motor2 == 0 and g_algopython_system_status.motor3 == 0):
                print("Motors BC completed movement")
                break;
            motorB_prev_status = g_algopython_system_status.motor2;
            motorC_prev_status = g_algopython_system_status.motor3;
# --------------------------------------------------------------------------------------------------------------

def rotations(port: str, rotations: float, power: float, direction: int):
    if isinstance(port, str):
        port = port.upper()
        if not all(m in ('A', 'B', 'C') for m in port):
            raise ValueError("Port string must contain only A, B, or C")
        if port not in ROTATIONS_TO_SECONDS_MAP:
            raise ValueError(f"No calibration factor for port: {port}")
        factor = ROTATIONS_TO_SECONDS_MAP[port]
        port_mask = 0
        for m in port:
            port_mask |= motor_map[m]
    elif isinstance(port, int):
        if not (0 <= port <= 0b111):
            raise ValueError("Port int must be between 0 and 7")
        port_mask = port
        if port == 0b001:
            factor = ROTATIONS_TO_SECONDS_MAP['A']
        elif port == 0b010:
            factor = ROTATIONS_TO_SECONDS_MAP['B']
        elif port == 0b100:
            factor = ROTATIONS_TO_SECONDS_MAP['C']
        else:
            raise ValueError("Calibration factor missing for this int port")
    else:
        raise TypeError("Port must be str or int")

    if not (0.1 <= rotations <= 100):
        raise ValueError("Rotations must be between 0.1 and 100")
    if not (0 <= power <= 255):
        raise ValueError("Power must be between 0 and 255")
    if direction not in (1, -1):
        raise ValueError("Direction must be 1 (CW) or -1 (CCW)")

    duration = rotations * factor

    move(
        port=port_mask,
        direction=direction,
        power=int(power),
        duration=duration
    )

def moveStop(stop_port: str):
    if stop_port not in motor_map:
        raise ValueError("Invalid motor")
    motor_stop_port = motor_map[stop_port.upper()];
    print(f"Stopping motor {stop_port}...")
    payload = bytes([
        motor_stop_port & 0xFF
        ])
    send_packet(ALGOPYTHON_CMD_MOVE_STOP_REQ, payload)

# --------------------------------------------------------------------------------------------------------------
#-----------------Light section---------------------------------------------------------------------------------
COLOR_MAP = {
    "red":     (255, 0, 0),
    "green":   (0, 255, 0),
    "blue":    (0, 0, 255),
    "yellow":  (255, 255, 0),
    "cyan":    (0, 255, 255),
    "magenta": (255, 0, 255),
    "white":   (255, 255, 255),
    "purple":  (128, 0, 128),
}

def light(port: int, duration: float , power: int, color: str | tuple[int, int, int], is_blocking = True):
    
    if port != 1 and port != 2:
        raise ValueError("Invalid LED")
    if not (0 <= power <= 10):
        raise ValueError("Power must be 0-10")

    if isinstance(color, str):
        color = color.lower()
        if color not in COLOR_MAP:
            raise ValueError(f"Unsupported color: {color}")
        r, g, b = COLOR_MAP[color]
    elif isinstance(color, (tuple, list)) and len(color) == 3:
        r, g, b = color
    else:
        raise ValueError("Color must be string or RGB tuple/list")

    led_port = port;
    led_power = int((power * 255) / 10);
    led_r = r;
    led_g = g;
    led_b = b;
    led_type = 0;

    if math.isinf(duration):
        print("x is positive infinity")
        led_type = 1;
        led_duration = 0;
        is_blocking = False;
    else:
        led_duration = int(duration * 100);

    payload = bytearray([
        led_port & 0xFF,
        led_type & 0xFF,
        (led_duration >> 24) & 0xFF,
        (led_duration >> 16) & 0xFF,
        (led_duration >> 8) & 0xFF,
        (led_duration) & 0xFF,
        led_power & 0xFF,
        led_r & 0xFF,
        led_g & 0xFF,
        led_b & 0xFF
    ])

    send_packet(ALGOPYTHON_CMD_LIGHT_REQ, payload, wait_done=False)

    print("Wait for led to finish..."); 
    if port == 1:
        led1_prev_status = g_algopython_system_status.led1;
        while is_blocking:
            if(led1_prev_status == 1) and (g_algopython_system_status.led1 == 0): 
                print("Led1 completed ")
                break;
            led1_prev_status = g_algopython_system_status.led1;
    elif port == 2:
        led2_prev_status = g_algopython_system_status.led2;
        while is_blocking:
            if(led2_prev_status == 1) and (g_algopython_system_status.led2 == 0): 
                print("Led2 completed ")
                break;
            led2_prev_status = g_algopython_system_status.led2;


def lightStop(stop_port: int):
    if stop_port not in (1, 2):
        raise ValueError("LED port must be 1 or 2")

    payload = bytes([
        stop_port & 0xFF
        ])
    send_packet(ALGOPYTHON_CMD_LIGHT_STOP_REQ, payload)

# --------------------------------------------------------------------------------------------------------------
#-----------------Play sound section----------------------------------------------------------------------------
def playSound(sound_id: int, volume: int, is_blocking= True):

    if not (0 <= volume <= 10):
        raise ValueError("Volume must be between 0 and 10")
    if sound_id not in SOUNDS_MAP:
        raise ValueError(f"Invalid sound ID: {sound_id}. Available sounds: {list(SOUNDS_MAP.keys())}")
    
    volume = int((volume / 10.0) * 255)

    payload = bytes([
        sound_id & 0xFF,         
        volume & 0xFF,     
    ])

    send_packet(ALGOPYTHON_CMD_PLAY_SOUND_REQ, payload,wait_done=False)
    print("Wait for sound to finish..."); 
    playSound_prev_status = g_algopython_system_status.sound;
    while is_blocking:
        if(playSound_prev_status == 1) and (g_algopython_system_status.sound == 0): 
            print("Sound completed ")
            break;
        playSound_prev_status = g_algopython_system_status.sound;

def soundStop(): 
    print("Stopping sound...")
    send_packet(ALGOPYTHON_CMD_SOUND_STOP_REQ, b"")

def listAvailableSounds():

    sounds = SOUNDS_MAP

    print("Available Sounds:")
    for sound_id, name in sounds.items():
        print(f"{sound_id}: {name}")
# --------------------------------------------------------------------------------------------------------------
#-----------------Sensor section--------------------------------------------------------------------------------
def get_sensor_value(sensor_port: int) -> int:

    if sensor_port not in (1, 2):
        raise ValueError("Port must be 1 or 2")

    payload = bytes([sensor_port])

    send_packet(ALGOPYTHON_CMD_GET_SENSOR_REQ, payload, wait_done=False)

def wait_sensor(sensor_port: int, min: int, max: int):

    if sensor_port not in (1, 2):
        raise ValueError("sensorPort mora biti 1 ili 2")

    print(f"Waiting for sensor {sensor_port} to detect value in range [{min}, {max}]")

    payload = bytes([
        sensor_port & 0xFF, 
        min & 0xFF, 
        max & 0xFF
        ])

    send_packet(ALGOPYTHON_CMD_WAIT_SENSOR_REQ, payload, wait_done=False)

    print("Wait for sensor to finish..."); 
    if sensor_port == 1:
        sensor1_prev_status = g_algopython_system_status.sensor1;
        while True:
            if(sensor1_prev_status == 1) and (g_algopython_system_status.sensor1 == 0): 
                print("Sensor 1 done ")
                break;
            sensor1_prev_status = g_algopython_system_status.sensor1;
    elif sensor_port == 2:
        sensor2_prev_status = g_algopython_system_status.sensor2;
        while True:
            if(sensor2_prev_status == 1) and (g_algopython_system_status.sensor2 == 0): 
                print("Sensor 2 done ")
                break;
            sensor2_prev_status = g_algopython_system_status.sensor2;
    

# --------------------------------------------------------------------------------------------------------------
#-----------------Status section--------------------------------------------------------------------------------

def wait(duration: float):
    duration = max(0.01, min(duration, 10.0))  
    print(f"Waiting for {duration:.2f} seconds...")
    time.sleep(duration)

