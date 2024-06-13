import serial
import serial.rs485
import crcmod.predefined
import struct
import serial.tools.list_ports
import threading
import time
import math
import gradio as gr
import can
from astarsu3 import a_star_search

class ddsm115:
    def __init__(self, device):
        self.ser_roda = serial.rs485.RS485(device, baudrate=115200)
        self.ser_roda.rs485_mode = serial.rs485.RS485Settings()
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')
        self.str_10bytes = ">BBBBBBBBBB"
        self.str_9bytes = ">BBBBBBBBB"
    
    def close(self):
        self.ser_roda.close()
        
    def set_drive_mode(self, id, mode):
        if mode == 2:
            print(f"Velocity Mode")
        else:
            mode = 2
            print(f"Velocity Mode")

        cmd_bytes = struct.pack(self.str_10bytes, id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode)
        self.ser_roda.write(cmd_bytes)
    
    def crc_attach(self, data_bytes):
        crc_int = self.crc8(data_bytes)
        data_bytesarray = bytearray(data_bytes)
        data_bytesarray.append(crc_int)
        full_cmd = bytes(data_bytesarray)
        
        return full_cmd
    
    def Int16ToBytesArray(self, data):
        byte1 = (data & 0xFF00) >> 8
        byte2 = (data & 0x00FF)
        return [byte1, byte2]
    
    def send_rpm(self, id, rpm):
        rpm = int(rpm)
        rpm_ints = self.Int16ToBytesArray(rpm)
        cmd_bytes = struct.pack(self.str_9bytes, id, 0x64, rpm_ints[0], rpm_ints[1], 0x00, 0x00, 0x00, 0x00, 0x00)
        cmd_bytes = self.crc_attach(cmd_bytes)	
        self.ser_roda.write(cmd_bytes)

class LPMSU2:
    def __init__(self, device):
        self.bus = can.interface.Bus(channel=device, bustype='socketcan', bitrate=1000000)
        self.heading_deg = 0

    def close(self):
        self.bus.shutdown()

    def combine_bytes(self, low_byte, high_byte):
        value = (low_byte & 0xFF) | ((high_byte & 0xFF) << 8)
        if value >= 32768:
            value -= 65536
        return value

    def map_value(self, x):
        if 0 <= x <= 107:
            return x * (90 / 107)
        elif 107 < x <= 180:
            return 90 + (x - 107) * (90 / 73)
        else:
            return x

    def receive_data(self):
        while(True):
            try:
                msg = self.bus.recv()

                if msg.arbitration_id == 0x381:
                    acc_z = self.combine_bytes(msg.data[0], msg.data[1])
                    mag_x = self.combine_bytes(msg.data[2], msg.data[3])
                    mag_y = self.combine_bytes(msg.data[4], msg.data[5])
                    mag_z = self.combine_bytes(msg.data[6], msg.data[7])

                    Mag_x = mag_x / 100.0
                    Mag_y = mag_y / 100.0

                    heading = math.degrees(math.atan2(Mag_y, Mag_x))
                    if heading < 0:
                        heading += 360
                    self.heading_deg = self.map_value(heading)

            except ValueError:
                print("LPMS Parsing Error")
                continue

class ULM1:
    def __init__ (self, device):
        self.ser_uwb = serial.Serial(device, baudrate=115200)
        self.head_raw = ['0x6d', '0x63']
        self.head_don = ['0x24', '0x4b']
        self.Ax0, self.Ay0 = 0, 0
        self.Ax1, self.Ay1 = 3.6, 2.7
        self.Ax2, self.Ay2 = 1.5, 6
        self.posisi_t = 0, 0
    
    def close(self):
        self.ser_uwb.close()
    
    def receive_data(self):
        while self.ser_uwb.isOpen():
            data = self.ser_uwb.read_until(size=70)
            data_hex = [hex(byte) for byte in data]
            if data_hex[:2] == self.head_don:
                try:
                    string_data = data.decode('utf-8')
                    range0 = float(string_data[5:9])
                    range1 = float(string_data[10:14])
                    range2 = float(string_data[15:19])

                    posisi_t = self.trilaterasi(self.Ax0, self.Ay0, range0, self.Ax1, self.Ay1, range1, self.Ax2, self.Ay2, range2)
                    self.posisi_t = posisi_t

                except ValueError:
                    print("UWB Parsing Error")
                    continue
            
    def trilaterasi(self, x1, y1, r1, x2, y2, r2, x3, y3, r3):
        A = 2 * x2 - 2 * x1
        B = 2 * y2 - 2 * y1
        C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
        D = 2 * x3 - 2 * x2
        E = 2 * y3 - 2 * y2
        F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
        x = (C * E - F * B) / (E * A - B * D)
        y = (C * D - A * F) / (B * D - A * E)
        
        return x, y
    
    def adj_grid(self, x_uwb, y_uwb):
        if 0 <= x_uwb <= 3.6 and 0 <= y_uwb <= 6:
            x = int(x_uwb // 0.6)
            y = int(y_uwb // 0.6)
            return x, y
        else:
            return None, None
    
    def thres_goal(self, x_adj, y_adj):
        x_thres = (x_adj + 0.5) * 0.6
        y_thres = (y_adj + 0.5) * 0.6

        return x_thres, y_thres

def headingcontrol(x_now, y_now, x_goal, y_goal):
    x_delta = x_goal - x_now
    y_delta = y_goal - y_now
    
    if x_delta == 1 and y_delta == 0:
        return 0
    elif x_delta == 1 and y_delta == 1:
        return 45
    elif x_delta == 0 and y_delta == 1:
        return 90
    elif x_delta == -1 and y_delta == 1:
        return 135
    elif x_delta == -1 and y_delta == 0:
        return 180
    elif x_delta == -1 and y_delta == -1:
        return 225
    elif x_delta == 0 and y_delta == -1:
        return 270
    elif x_delta == 1 and y_delta == -1:
        return 315
    else:
        return "Wrong Input"

class pidcontrol:
    def __init__(self, Kp = 0, Ki = 0, Kd = 0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        
    def update(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        
        return output

def ctrl_speed():
    global x_awal, y_awal, ket, path, t
    i = 0
    kecepatan = 8
    kecepatanlurus = 12

    start_time = time.time()

    for t in range(len(path)):
        ket = t
        x_tujuan = path[t+1][0]
        y_tujuan = path[t+1][1]
        
        target = headingcontrol(x_awal, y_awal, x_tujuan, y_tujuan)
        print(x_tujuan, y_tujuan)
        target_thres = uwb.thres_goal(x_tujuan, y_tujuan)
        x_thres = target_thres[0]
        y_thres = target_thres[1]

        while (i <= 2):
            if i == 0:
                while (True):
                    heading = imu.heading_deg
                    print(heading)
                    if heading is not None:
                        error = target - heading
                        print("Heading = {:.2f} Error = {:.2f} Target = {:.2f} Posisi = {},{}".format(heading, error, target, x_awal, y_awal))
                        if error > 0.3 and error < 180:
                            motor_kiri.send_rpm(2, kecepatan)
                            motor_kanan.send_rpm(1, (-1)*-kecepatan)
                        elif error > 0.3 and error >= 180:
                            motor_kiri.send_rpm(2, -kecepatan)
                            motor_kanan.send_rpm(1, (-1)*kecepatan)
                        elif error < -0.3 and error > -180:
                            motor_kiri.send_rpm(2, -kecepatan)
                            motor_kanan.send_rpm(1, (-1)*kecepatan)
                        elif error < -0.3 and error <= -180:
                            motor_kiri.send_rpm(2, kecepatan)
                            motor_kanan.send_rpm(1, (-1)*-kecepatan)
                        elif 0.3 >= error >= -0.3:
                            motor_kiri.send_rpm(2, 0)
                            motor_kanan.send_rpm(1, 0)
                            i = 1
                            break

            if i == 1:
                time.sleep(2)
                while uwb.ser_uwb.isOpen():
                    heading = imu.heading_deg
                    try:
                        posisi_t = uwb.posisi_t
                        if posisi_t:
                            x_pos, y_pos = posisi_t
                            if not (x_thres - 0.15 <= x_pos <= x_thres + 0.15 and y_thres - 0.15 <= y_pos <= y_thres + 0.15):
                                print(f"Posisi = {x_pos},{y_pos}, Threshold = {x_thres},{y_thres}, Error = {error}")
                                error = target - heading
                                pid_output = pid.update(error)
                                pid_output = max(min(pid_output, 17), -17)

                                if  5 >= error >= -5:
                                    motor_kiri.send_rpm(2, kecepatanlurus)
                                    motor_kanan.send_rpm(1, (-1)*kecepatanlurus)
                                elif error > 5:
                                    motor_kiri.send_rpm(2, pid_output)
                                    motor_kanan.send_rpm(1, (-1)*3)
                                elif error < -5:
                                    motor_kiri.send_rpm(2, 3)
                                    motor_kanan.send_rpm(1, (-1)*pid_output)
                                elif error > 10:
                                    motor_kiri.send_rpm(2, kecepatan)
                                    motor_kanan.send_rpm(1, (-1)*0)
                                elif error < -10:
                                    motor_kiri.send_rpm(2, 0)
                                    motor_kanan.send_rpm(1, (-1)*kecepatan)
                            else:
                                motor_kiri.send_rpm(2, 0)
                                motor_kanan.send_rpm(1, (-1)*0)
                                time.sleep(2)
                                i = 2
                                break
                    except ValueError as e:
                        print("Error:", e)
                        continue

            if i == 2:
                motor_kiri.send_rpm(2, 0)
                motor_kanan.send_rpm(1, 0)
    
                end_time = time.time()
                total = end_time - start_time
                print(f"Waktu Perjalanan = {total} detik")
                x_awal, y_awal = x_tujuan, y_tujuan
                i = 0
                break

def stater():
    global x_awal
    global y_awal
    ten_x = []
    ten_y = []
    ten_imu = []

    tr_imu.start()
    tr_uwb.start()
    time.sleep(1)
    
    while len(ten_x) < 20 and len(ten_y) < 20:
        ten_x.append(uwb.posisi_t[0])
        ten_y.append(uwb.posisi_t[1])
    avg_x = sum(ten_x)/20
    avg_y = sum(ten_y)/20
    x_awal, y_awal = uwb.adj_grid(avg_x, avg_y)

    while len(ten_imu) < 20:
        ten_imu.append(imu.heading_deg)
    avg_imu = sum(ten_imu)/20

    return "Heading = {:.1f}° || Koordinat = ({},{})".format(avg_imu, x_awal, y_awal)

def submitjarak(num1, num2):
    global path

    print(type(x_awal), type(num1), type(ROW))
    path = a_star_search(grid, [x_awal, y_awal], [num1, num2], ROW, COL)
    tr_kontrol.start()

    return f"Rute = {path}"

def information():
    ten_x = []
    ten_y = []
    ten_imu = []

    while ket >= 0:
        while len(ten_x) < 20 and len(ten_y) < 20:
            ten_x.append(uwb.posisi_t[0])
            ten_y.append(uwb.posisi_t[1])
        avg_x = sum(ten_x)/20
        avg_y = sum(ten_y)/20
        x_awal, y_awal = uwb.adj_grid(avg_x, avg_y)

        while len(ten_imu) < 20:
            ten_imu.append(imu.heading_deg)
        avg_imu = sum(ten_imu)/20

        yield "Heading = {:.1f}° || Koordinat = ({},{})".format(avg_imu, x_awal, y_awal)
        time.sleep(2)

def information2():
    i = 0

    while ket >= 0:
        yield "Tujuan yang akan dituju = ({},{})".format(path[i][0], path[i][1])
        time.sleep(2)

def interrupt():
    motor_kiri.send_rpm(2, 0)
    motor_kanan.send_rpm(1, 0)

    motor_kiri.close()
    motor_kanan.close()
    uwb.close()

def get_ports_by_locations(target_locs):
    ports = serial.tools.list_ports.comports()
    result = {}
    
    for port_info in sorted(ports, key=lambda x: x.device):
        location = port_info.location
        if location in target_locs:
            result[location] = port_info.device
    
    return result

if __name__ == "__main__":
    target_locations = ["1-2.1",
                        "1-2.2"]
    ports_dict = get_ports_by_locations(target_locations)
    port1, port2= (ports_dict.get(loc) for loc in target_locations)

    x_awal = 0
    y_awal = 0
    ket = 0
    t = 0
    path = []
    ROW = 10
    COL = 6
    
    motor_kiri = ddsm115(port2)   # ID 2
    motor_kanan = ddsm115(port2)  # ID 2
    imu = LPMSU2('can0')
    uwb = ULM1(port1)
    pid = pidcontrol(2.4)
    
    motor_kiri.set_drive_mode(2, 2)
    motor_kanan.set_drive_mode(1, 2)
    
    tr_imu = threading.Thread(target=imu.receive_data)
    tr_kontrol = threading.Thread(target=ctrl_speed)
    tr_uwb = threading.Thread(target=uwb.receive_data)

    grid = [
        [1, 0, 1, 1, 1, 1],
        [1, 1, 1, 0, 0, 1],
        [1, 1, 1, 0, 0, 1],
        [1, 1, 1, 1, 0, 1],
        [1, 0, 1, 0, 1, 1],
        [1, 1, 1, 1, 1, 1],
        [1, 0, 1, 1, 1, 1],
        [1, 1, 1, 0, 0, 1],
        [0, 1, 1, 1, 0, 1],
        [0, 1, 1, 1, 1, 1]
    ]

    with gr.Blocks() as demo:
        gr.Markdown("# Warehouse Autonomous Mobile Robot")

        with gr.Tabs():
            with gr.TabItem("Mulai"):
                with gr.Column():
                    with gr.Row():
                        start_button = gr.Button("Starter Robot", scale=1)
                        start_button.click(fn=stater, inputs=None, outputs=gr.Textbox(label="Posisi Awal"))
                    with gr.Column():
                        num1 = gr.Number(label="Koordinat X Tujuan")
                        num2 = gr.Number(label="Koordinat Y Tujuan")
                    with gr.Column():
                        button = gr.Button("Submit Tujuan")                        
                        button.click(fn=submitjarak, inputs=[num1,num2], outputs=gr.Textbox(label="Deskripsi"))

            with gr.TabItem("Informasi Real-Time"):
                with gr.Column():
                    button.click(fn=information2, inputs = None, outputs = gr.Textbox(label="Tujuan"))
                    button.click(fn=information, inputs = None, outputs = gr.Textbox(label="Posisi Terakhir"))
                    calculate_button = gr.Button("Force Stop")
                    calculate_button.click(fn=interrupt)

            with gr.TabItem("Peta"):
                gr.Image("/home/oliver/Self/Full Control/Map.png", width=700, height=530, scale=1)

    demo.launch(share=False, server_name="192.168.65.47", server_port=7867)

'''
skema 1 = [
        [1, 1, 1, 1, 0, 1],
        [1, 0, 0, 1, 1, 1],
        [1, 0, 0, 1, 1, 1],
        [1, 0, 1, 1, 0, 1],
        [1, 1, 1, 1, 0, 1],
        [1, 1, 1, 1, 0, 1],
        [1, 1, 1, 1, 1, 1],
        [1, 0, 0, 1, 1, 1],
        [1, 0, 1, 1, 1, 0],
        [1, 1, 1, 1, 1, 0]
    ]

skema 2 = [
        [1, 1, 1, 1, 0, 1],
        [1, 0, 0, 1, 1, 1],
        [1, 0, 0, 1, 1, 1],
        [1, 0, 1, 1, 1, 1],
        [1, 1, 0, 1, 0, 1],
        [1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1],
        [1, 0, 0, 1, 1, 1],
        [1, 0, 1, 1, 1, 0],
        [1, 1, 1, 1, 1, 0]
    ]
'''
