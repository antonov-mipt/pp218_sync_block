import FreeSimpleGUI as sg
import serial, threading, crc, queue, struct, time
from serial.tools import list_ports

def get_serial_ports():
    return [port.device for port in list_ports.comports()]

OUT_MODES = {0:'LOW',1:'HIGH',2:'PULSE LOW',3:'PULSE HIGH'}

class PORT:
    PORT_DATA_STRUCT = '>BBBBLlLl'
    def __init__(self, port_name, binary):
        tupl = struct.unpack(self.PORT_DATA_STRUCT, binary)
        self.name = port_name
        self.curr_state = tupl[0]
        self.rising_ctr = tupl[1]
        self.falling_ctr = tupl[2]
        self.err_ctr = tupl[3]
        self.falling = tupl[6] + (tupl[7]/1000000)
        self.rising = tupl[4] + (tupl[5]/1000000)

    def __str__(self):
        return f'[{self.name}]: [{OUT_MODES[self.curr_state]}]'\
                f' ERR:{self.err_ctr}' \
                f'\n\tFALLING [{self.falling_ctr:03d}]: {self.falling:.3f}'\
                f'\n\tRISING  [{self.rising_ctr:03d}]: {self.rising:.3f}'


class STATUS_PACKET:
    STATUS_PAYLOAD_STRUCT = '>BB Bx L 240s Ll'
    STATUS_PAYLOAD_SZ = struct.calcsize(STATUS_PAYLOAD_STRUCT)
    VER = lambda self, v:f'{(v&0xF0)>>4}.{v&0xF}'

    def __init__(self, b):
        self.recv_time = time.time()
        
        self.pps_valid = False
        self.mcu_freq_valid = False
        self.gps_packet_valid = False
        self.gps_timestamp_valid = False
        self.ports = list()

        # разбираем бинарные данные пакета
        tupl = struct.unpack(self.STATUS_PAYLOAD_STRUCT, b)
        self.fw_ver = self.VER(tupl[0])
        self.hw_ver = self.VER(tupl[1])
        flags = tupl[2]
        self.i2c_ok = (flags & (1<<0)) == 0
        self.gps_utc_time_valid = (flags & (1<<1)) == 0
        self.pps_approx_valid = (flags & (1<<3)) == 0
        self.pps_valid = (flags & (1<<4)) == 0
        self.gps_ts_valid = (flags & (1<<5)) == 0
        self.gps_packet_valid = (flags & (1<<6)) == 0 
        self.mcu_freq_valid = (flags & (1<<7)) == 0
        self.uptime = tupl[3]

        # тут 12 портов: PPS, 7 входящих и 4 исходящих, каждый по 20 байт
        port_bytes = [tupl[4][(20*i):(20*(i+1))] for i in range(12)]
        self.packet_ts = tupl[5] + (tupl[6]/1000000)

        # инициализируем классы портов
        pps_port = PORT('PPS', port_bytes[0])
        inpt_ports = [PORT(f'IN{i}', port_bytes[i]) for i in range(1,8)]
        outpt_port = [PORT(f'OUT{i-7}', port_bytes[i]) for i in range(8,12)]
        self.ports = [pps_port] + inpt_ports + outpt_port


    def __str__(self):
        ports = '\n'.join([str(p) for p in self.ports])
        timediff = self.packet_ts - self.recv_time

        return '\n'.join([
            f'Версия прошивки/платы: {self.fw_ver}/{self.hw_ver}',
            f'Аптайм системы {self.uptime}',
            f'Таймштамп самого пакета {self.packet_ts:.3f}',
            f'Разница с часами ПК {timediff:.3f}',
            f'',
            f'Таймштампы PPS валидны: {self.pps_valid}',
            f'Массив таймштампов PPS валиден: {self.pps_approx_valid}',
            f'Частота MCU валидна: {self.mcu_freq_valid }',
            f'GPS приемник найден: {self.i2c_ok}',
            f'Пакет GPS валиден: {self.gps_packet_valid}',
            f'Время UTC в пакете валидно: {self.gps_utc_time_valid}',
            f'Таймштамп пакета GPS валиден: {self.gps_ts_valid}',
            f'',
        ]) + '\n' + ports


if __name__ == "__main__":

    BAUDRATE = 115200
    HEADER = b'SB0'

    OUT_PORTS = ['P1','P2','P3','P4']
    OUT_MODES_INV = {'LOW':0,'HIGH':1,'PULSE LOW':2,'PULSE HIGH':3}

    serial_queue = queue.Queue()
    crc_calc = crc.Calculator(crc.Crc32.CRC32)

    ser = None  # Пока порт не выбран
    last_port_list = []
    port_update_timer = time.time()

    def serial_reader(q):
        global ser
        while True:
            time.sleep(1)
            try:
                if ser is None or not ser.is_open:
                    continue
                while True:
                    header = ser.read(4)
                    if header != HEADER + b'\x00': 
                        ser.read(1000)  # flush
                        continue
                    body = ser.read(STATUS_PACKET.STATUS_PAYLOAD_SZ + 4)
                    if len(body) != (STATUS_PACKET.STATUS_PAYLOAD_SZ + 4): continue
                    data, received_crc = body[:-4], body[-4:]
                    computed_crc = crc_calc.checksum(header+data).to_bytes(4, 'big')
                    if computed_crc == received_crc: 
                        q.put(STATUS_PACKET(data)) 
                    else: 
                        print("Packet CRC error")
            except Exception as e:
                print(f"Serial thread exception:\n\t{repr(e)}")

    def serial_send_cmd(mask, mode, duration):
        global ser
        if ser is None or not ser.is_open:
            print("Serial port not open")
            return
        payload = struct.pack('>BBH', mask, mode, duration)
        msg = HEADER +  b'\x80' + payload
        crc32 = crc_calc.checksum(msg).to_bytes(4, 'big')
        ser.write(msg + crc32)

    port_list = get_serial_ports()
    last_port_list = port_list.copy()

    lc = [  [ sg.Text("COM-порт"), sg.Combo(port_list, key='COMPORT', enable_events=True, readonly=True) ],
            [ sg.Text("Порты") ],
            [ sg.Checkbox(text='OUT'+str(i+1), key = OUT_PORTS[i]) for i in range(4) ],
            [ sg.Text("Режим"),  sg.Combo(list(OUT_MODES_INV.keys()), default_value='LOW', key = 'MODE')],
            [ sg.Text("Длительность"),  sg.Combo([1,10,100,1000], default_value=100, key = 'DURAT')],
            [ sg.Button("SEND", bind_return_key = True, key = 'SEND')]]

    rc = [  [ sg.Text("Текущее состояние", key='-OUTPUT-')]]

    layout = [[
        sg.Column(lc, size = (550, 750), vertical_alignment='top'),
        sg.Column(rc, vertical_alignment='top') ]]

    window = sg.Window("PP218 SyncroBlock test", layout)

    threading.Thread(target=serial_reader, args=(serial_queue,), daemon=True).start()

    while True:
        event, values = window.read(timeout=100)

        if event == sg.WIN_CLOSED:
            break

        if event == 'COMPORT':
            try:
                if ser and ser.is_open:
                    ser.close()
                ser = serial.Serial(values['COMPORT'], baudrate=BAUDRATE, timeout=1)
                print(f"Открыт порт {values['COMPORT']}")
            except Exception as e:
                print(f"Ошибка при открытии порта: {e}")
                ser = None

        if event == "SEND": 
            if ser is None or not ser.is_open:
                print("Порт не выбран или не открыт")
                continue
            mask = 0
            for port_name in OUT_PORTS: 
                idx = int(port_name[1])
                mask |= 1<<idx if values[port_name] else 0
            try:
                mode = OUT_MODES_INV[values['MODE']]
                duration = int(values['DURAT'])
                serial_send_cmd(mask, mode, duration)
            except Exception as e:
                print(f"Ошибка при отправке: {e}")

        while not serial_queue.empty():
            status = serial_queue.get()
            window['-OUTPUT-'].update(str(status) + '\n')

        # Обновление списка COM-портов раз в секунду
        if time.time() - port_update_timer >= 1:
            new_ports = get_serial_ports()
            if new_ports != last_port_list:
                window['COMPORT'].update(values=new_ports)
                last_port_list = new_ports
            port_update_timer = time.time()

    window.close()
    if ser and ser.is_open:
        ser.close()
