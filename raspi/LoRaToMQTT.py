#!/usr/bin/env python3

# Empfangsskript für LoRa-ESP32-Sender der Wetterstation

from time import sleep
from lora.LoRa import *
from lora.board_config import BOARD
from cbor2 import loads
from ctypes import *
import logging
from datetime import datetime
import time

FORMAT = '%(asctime)s %(message)s'
logging.basicConfig(format=FORMAT, filename='LoRa_receiver.log', encoding='utf-8', level=logging.INFO)
LastReceivedTime = time.time()

# Lora-Paketheader, 16 Bytes
class LoraPacketHeader(Structure):
    _pack_ = 1
    _fields_ = [('Magic', c_ushort),
                ('Address', c_ushort),
                ('TotalTransmissionSize', c_ushort),
                ('PacketNumber', c_byte),
                ('NumPackets', c_byte),
                ('PacketPayloadSize', c_byte),
                ('Cmd', c_byte),
                ('Tag', c_uint, 32),
                ('PayloadCRC', c_ushort)]

BOARD.setup()


class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

    def on_rx_done(self):
        try:
            global LastReceivedTime
            LastReceivedTime=time.time()
            #print("\nRxDone")
            flags = self.get_irq_flags()
            self.clear_irq_flags(RxDone=1)
            payload = self.read_payload(nocheck=True)
            llen=len(payload)
            now = datetime.now()
 
            #print("now =", now)
            print(f'\n{now}: LoRa-Paket empfangen mit {llen:d} bytes')
            Header=LoraPacketHeader()
            HeaderSize = sizeof(Header)
            
            if flags['crc_error']==1:
                self.clear_irq_flags(PayloadCrcError=0)
                raise Exception("CRC-Fehler in Lora-Paket!")
            if flags['valid_header']==0:
                self.clear_irq_flags(ValidHeader=1)
                raise Exception("Ungültiger LoRa-Paket-Header!")
            if flags['rx_timeout']==1:
                self.clear_irq_flags(RxTimeout=0)
                raise Exception("Timeout in LoRa-Paket!")
            if llen<sizeof(Header):
                raise Exception("LoRa-Paket kleiner als LoRa-Header!")
            bs=bytearray(payload)           
            memmove(addressof(Header), bytes(bs), HeaderSize)
            
            #print('---LoRaHEADER---')
            #print(f'Magic: {Header.Magic:d}')
            #print(f'Address: {Header.Address:d}')
            #print(f'TotalTransmissionSize: {Header.TotalTransmissionSize:d}')
            #print(f'PacketNumber: {Header.PacketNumber:d}')
            #print(f'NumPackets: {Header.NumPackets:d}')
            #print(f'PacketPayloadSize: {Header.PacketPayloadSize:d}')
            #print(f'Cmd: {Header.Cmd:d}')
            #print(f'Tag: {Header.Tag:d}')
            
            # Die ersten 16 bytes des Paketes ist nur der
            # Protokollheader, den strippen wir ab
            bs=bs[HeaderSize:]
            
            if len(bs) != Header.PacketPayloadSize:
                raise Exception(f"Ungültiges LoRa-Paket empfangen. Größe: {len(bs):d}")
            #print(f'Header CRC: {Header.PayloadCRC:x}\n')
            #print(f'Calculated CRC: {crc16(bs):x}\n')
            if Header.PayloadCRC != crc16(bs):
                raise Exception(f"CRC-Payload-Fehler in Lora-Paket! Header:{Header.PayloadCRC:x} Calc:{crc16(bs):x}")
            gwhs=loads(bs)
            #with open('gwhs_temp.cbor', 'wb') as fp:
                #fp.write(bs)
            print(f'Paketnummer: {gwhs["PC"]:d}')
            print(f'Temperatur: {gwhs["TE"][0]["W"]:.2f}°C')
            print(f'Luftfeuchtigkeit: {gwhs["LF"][0]["W"]:.1f}%')
            self.set_mode(MODE.SLEEP)
            self.reset_ptr_rx()
            self.set_mode(MODE.RXCONT)
        except Exception as e:
            print(e)
            logging.exception(e)
            self.set_mode(MODE.SLEEP)
            self.reset_ptr_rx()
            self.set_mode(MODE.RXCONT)
            
    def on_tx_done(self):
        print("\nTxDone")
        print(self.get_irq_flags())

    def on_cad_done(self):
        print("\non_CadDone")
        print(self.get_irq_flags())

    def on_rx_timeout(self):
        print("\non_RxTimeout")
        print(self.get_irq_flags())

    def on_valid_header(self):
        print("\non_ValidHeader")
        print(self.get_irq_flags())

    def on_payload_crc_error(self):
        print("\non_PayloadCrcError")
        print(self.get_irq_flags())

    def on_fhss_change_channel(self):
        print("\non_FhssChangeChannel")
        print(self.get_irq_flags())

    def start(self):
        
        global LastReceivedTime
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        while True:
            try:
                # Receive-Timeout aufgetreten? Manchmal geht der LoRa-Receiver in einen
                # Zustand, wo er nichts mehr empfängt. Dann resetten!
                now=time.time()
                if now-LastReceivedTime > 15:
                    LastReceivedTime=now
                    print("\nTimeout beim Empfangen. Resette LoRa-Empfänger")
                    logging.error("Timeout beim Empfangen. Resette LoRa-Empfänger")
                    lora_init()
                    self.reset_ptr_rx()
                    self.set_mode(MODE.RXCONT)
                sleep(.5)
                rssi_value = self.get_rssi_value()
                status = self.get_modem_status()
                sys.stdout.flush()
                sys.stdout.write("\r%d %d %d" % (rssi_value, status['rx_ongoing'], status['modem_clear']))
            except Exception as e:
                print(e)
                logging.exception(e)
                sleep(2)
            
def crc16(data: bytes, poly=0x8408):
    '''
    CRC-16-CCITT Algorithm
    '''
    data = bytearray(data)
    crc = 0x0000
    for b in data:
        cur_byte = 0xFF & b
        for _ in range(0, 8):
            if (crc & 0x0001) ^ (cur_byte & 0x0001):
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
            cur_byte >>= 1
    
    return crc & 0xFFFF

lora = LoRaRcvCont(verbose=False)
def lora_init():
    global lora
    lora.reset()
    lora.set_mode(MODE.STDBY)
    lora.set_pa_config(pa_select=1)
    lora.set_rx_crc(True)
    lora.set_freq(434.54)
    lora.set_preamble(14)
    lora.set_bw(9)
    lora.set_sync_word(0x3d)
    #lora.set_coding_rate(CODING_RATE.CR4_6)
    #lora.set_pa_config(max_power=0, output_power=0)
    #lora.set_lna_gain(GAIN.G1)
    lora.set_implicit_header_mode(False)
    #lora.set_low_data_rate_optim(True)
    #lora.set_pa_ramp(PA_RAMP.RAMP_50_us)
    #lora.set_coding_rate(CODING_RATE.CR4_6)
    lora.set_agc_auto_on(True)
    #print(lora)
    assert(lora.get_agc_auto_on() == 1)

try:
    print("Starte LoRa-ESP32-Empänger...\n")
    lora_init()
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("")
    lora.set_mode(MODE.SLEEP)
    print(lora)
    BOARD.teardown()
