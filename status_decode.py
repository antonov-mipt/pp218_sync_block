import struct, time

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
    MCU_TIMER_FREQ = 16000000 #16 MHz
    CTR_OVERFLOW_VAL = 2**32
    STATUS_PAYLOAD_STRUCT = '>BB Bx L 240s Ll'
    STATUS_PAYLOAD_SZ = struct.calcsize(STATUS_PAYLOAD_STRUCT)
    VER = lambda self, v:f'{(v&0xF0)>>4}.{v&0xF}'

    def mcu_ts_validate(self, ts):
        if ts is None: return None
        elif ts&1 == 0: return None # самый младший бит содержит признак валидности
        elif ts > self.mcu_packet_ts: ts -= self.CTR_OVERFLOW_VAL
        return ts

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
        self.pps_valid = (flags & (1<<4)) == 0
        self.gps_ts_valid = (flags & (1<<5)) == 0
        self.gps_packet_valid = (flags & (1<<6)) == 0 
        self.mcu_freq_valid = (flags & (1<<7)) == 0
        self.uptime = tupl[3]

        # тут 12 портов: PPS, 7 входящих и 4 исходящих, каждый по 12 байт
        port_bytes = [tupl[4][(20*i):(20*(i+1))] for i in range(12)]
        self.packet_ts = tupl[5] + (tupl[6]/1000000)

        # инициализируем классы портов
        pps_port = PORT('PPS', port_bytes[0])
        inpt_ports = [PORT(f'IN{i}', port_bytes[i]) for i in range(1,8)]
        outpt_port = [PORT(f'OUT{i-7}', port_bytes[i]) for i in range(8,12)]
        self.ports = [pps_port] + inpt_ports + outpt_port


    def __str__(self):
        ports = '\n'.join([f'{str(self.ports[i])}' for i in range(12)])
        timediff = self.packet_ts - self.recv_time

        return '\n'.join([
            f'Версия прошивки/платы: {self.fw_ver}/{self.hw_ver}',
            f'Аптайм системы {self.uptime}',
            f'',
            f'Таймштампы PPS валидны: {self.pps_valid}',
            f'Аппроксимация частоты по PPS валидна: {self.mcu_freq_valid }',
            f'GPS приемник найден: {self.i2c_ok}',
            f'Время UTC в пакете валидно: {self.gps_utc_time_valid}',
            f'Пакет GPS валиден: {self.gps_packet_valid}',

            f'Таймштамп пакета GPS валиден: {self.gps_ts_valid}',
            f'',
            f'Таймштамп самого пакета {self.packet_ts:.3f}',
            f'Разница с часами ПК {timediff:.3f}',
            f'',
            f'Таймштампы портов:\n{ports}',
        ])

