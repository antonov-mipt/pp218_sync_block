import FreeSimpleGUI as sg
import serial, threading, crc, queue, struct
from status_decode import STATUS_PACKET

if __name__ == "__main__":

    SERIAL_PORT = 'COM40'
    BAUDRATE = 115200
    HEADER = b'SB0'

    OUT_PORTS = ['P0','P1','P2','P3']
    OUT_MODES = {'LOW':0,'HIGH':1,'PULSE LOW':2,'PULSE HIGH':3}

    serial_queue = queue.Queue()
    crc_calc = crc.Calculator(crc.Crc32.CRC32)
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)

    def serial_reader(ser, q):
        #try:
            while True:
                header = ser.read(4)
                if header != HEADER + b'\x00': 
                    try: ser.read(1000) # flush
                    except: break
                    continue
                body = ser.read(STATUS_PACKET.STATUS_PAYLOAD_SZ + 4) 
                if len(body) != (STATUS_PACKET.STATUS_PAYLOAD_SZ + 4): continue
                data, received_crc = body[:-4], body[-4:]
                computed_crc = crc_calc.checksum(header+data).to_bytes(4, 'big')
                if computed_crc == received_crc: q.put(STATUS_PACKET(data)) 
                else: print("Packet CRC error")
        #except Exception as e:
        #    print(f"Serial thread exception:\n\t{repr(e)}")

    def serial_send_cmd(mask, mode, duration):
        payload = struct.pack('>BBH', mask, mode, duration)
        msg = HEADER +  b'\x80' + payload
        crc32 = crc_calc.checksum(msg).to_bytes(4, 'big')
        ser.write(msg + crc32)

    lc = [  [ sg.Text("Порты") ],
            [ sg.Checkbox(text=str(i), key = OUT_PORTS[i]) for i in range(4) ],
            [ sg.Text("Режим"),  sg.Combo(list(OUT_MODES.keys()), key = 'MODE')],
            [ sg.Text("Длительность"),  sg.Combo([10,100,1000], key = 'DURAT')],
            [ sg.Button("SEND", bind_return_key = True, key = 'SEND')]]
    rc = [  [ sg.Text("Текущее состояние", key='-OUTPUT-')]]

    layout = [[
        sg.Column(lc, size = (550, 750), vertical_alignment='top'),
        sg.Column(rc, vertical_alignment='top') ]]

    window = sg.Window("PP218 SyncroBlock test", layout)
    threading.Thread(target=serial_reader, args=(ser, serial_queue), daemon=True).start()

    while True:
        event, values = window.read(timeout=100)  # неблокирующее чтение событий

        if event == sg.WIN_CLOSED:
            break

        if event == "SEND": 
            mask = 0
            for port_name in OUT_PORTS: 
                idx = int(port_name[1])
                mask |= 1<<idx if values[port_name] else 0
            mode = OUT_MODES[values['MODE']]
            duration = int(values['DURAT'])
            serial_send_cmd(mask, mode, duration)

        while not serial_queue.empty():
            status = serial_queue.get()
            window['-OUTPUT-'].update(str(status) + '\n')

    window.close()
    ser.close()
