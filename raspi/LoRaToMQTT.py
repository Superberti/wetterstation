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
import threading
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import os

logging.basicConfig(format='%(asctime)s %(message)s', filename=os.path.expanduser("~")+'/LoRa_receiver.log', encoding='utf-8', level=logging.INFO)
#logging.basicConfig(format='%(asctime)s %(message)s', encoding='utf-8', level=logging.INFO)
LastReceivedTime = time.time()

PacketLostCounter={}
CurrentPacketCounter={}
FirstPacketCounter={}
TotalCount={}
PacketLossPer={}
# Lora-Paketheader, 16 Bytes
class LoraPacketHeader(Structure):
    _pack_ = 1
    _fields_ = [('Magic', c_ushort),
                ('Address', c_ushort),
                ('TotalTransmissionSize', c_ushort),
                ('PacketNumber', c_ubyte),
                ('NumPackets', c_ubyte),
                ('PacketPayloadSize', c_ubyte),
                ('Cmd', c_ubyte),
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
            global PacketLostCounter
            global CurrentPacketCounter
            global FirstPacketCounter
            global TotalCount
            global PacketLossPer
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
            LoraError=False
            channel_flags=self.get_hop_channel()
            if channel_flags['crc_on_payload']==0:
                LoraError=True
                logging.error("Paket ohne CRC empfangen. Phantom-Paket?")
            elif flags['crc_error']==1:
                LoraError=True
                self.clear_irq_flags(PayloadCrcError=0)
                logging.error("CRC-Fehler in Lora-Paket!")
            elif flags['valid_header']==0:
                LoraError=True
                self.clear_irq_flags(ValidHeader=1)
                logging.error("Ungültiger LoRa-Paket-Header!")
            elif flags['rx_timeout']==1:
                LoraError=True
                self.clear_irq_flags(RxTimeout=0)
                logging.error("Timeout in LoRa-Paket!")
            elif llen<sizeof(Header):
                LoraError=True
                logging.error("LoRa-Paket kleiner als LoRa-Header!")
                
            else:
                bs=bytearray(payload)           
                memmove(addressof(Header), bytes(bs), HeaderSize)      
                # Die ersten 16 bytes des Paketes ist nur der
                # Protokollheader, den strippen wir ab
                bs=bs[HeaderSize:]
                
                if len(bs) != Header.PacketPayloadSize:
                    LoraError=True
                    logging.error(f"Ungültiges LoRa-Paket empfangen. Größe: {len(bs):d} / {Header.PacketPayloadSize:d}")
                elif Header.PayloadCRC != crc16(bs):
                    LoraError=True
                    logging.error(f"CRC-Payload-Fehler in Lora-Paket! Header:{Header.PayloadCRC:x} Calc:{crc16(bs):x}")
                
            self.set_mode(MODE.SLEEP)
            self.reset_ptr_rx()
            self.set_mode(MODE.RXCONT)
            if LoraError == False:
                cbor_data=loads(bs)
                Ort=cbor_data['Ort']
                if Ort=="":
                    raise Exception("Ortsangabe im CBOR nicht gefunden!") 
                #print(f'Ort: {Ort}')
                
                cbor_data=cbor_data['DATA']
                #print (cbor_data)
                # Zähler initialisieren
                if not Ort in CurrentPacketCounter:
                    CurrentPacketCounter[Ort]=0
                    
                if not Ort in FirstPacketCounter:
                    FirstPacketCounter[Ort]=0
                    
                if not Ort in PacketLossPer:
                    PacketLossPer[Ort]=0
                    
                if not Ort in TotalCount:
                    TotalCount[Ort]=0
                    
                if not Ort in PacketLostCounter:
                    PacketLostCounter[Ort]=0
                    
                if CurrentPacketCounter[Ort]==0:
                    CurrentPacketCounter[Ort]=cbor_data["PC"] # Initialisierung!
                    FirstPacketCounter[Ort]=CurrentPacketCounter[Ort]
                    
                LastCounter=CurrentPacketCounter[Ort]
                CurrentPacketCounter[Ort]=cbor_data["PC"]
                if LastCounter<CurrentPacketCounter[Ort]-1:
                    lost_msg=f'LoRa-Paket verloren. Alt: {LastCounter} Neu: {CurrentPacketCounter[Ort]}'
                    print(lost_msg)
                    logging.error(lost_msg)
                    if TotalCount[Ort]>0:
                        logging.error(f'PC: {CurrentPacketCounter[Ort]}({TotalCount[Ort]-PacketLostCounter[Ort]}/{TotalCount[Ort]}) LOSS: {PacketLossPer[Ort]:.1f}%')
                    PacketLostCounter[Ort]+=1
                    LastCounter=CurrentPacketCounter[Ort]
                
                TotalCount[Ort]=CurrentPacketCounter[Ort]-FirstPacketCounter[Ort]+1
                temp=f'{cbor_data["TE"]["W"]:.2f}'
                hum=f'{cbor_data["LF"]["W"]:.1f}'
                volt=f'{cbor_data["V"]["W"]:.2f}'
                druck=f'{cbor_data["LD"]["W"]:.2f}'
                
                if TotalCount[Ort]>0:
                    PacketLossPer[Ort]=(PacketLostCounter[Ort]/TotalCount[Ort])*100.0
                    print(f'{Ort}: PC: {CurrentPacketCounter[Ort]}({TotalCount[Ort]-PacketLostCounter[Ort]}/{TotalCount[Ort]}) LOSS: {PacketLossPer[Ort]:.1f}% TE: {temp}°C LF: {hum}% VB: {volt} V')
                #print(f'Temperatur: {temp}°C')
                #print(f'Luftfeuchtigkeit: {hum}%')
                TopicTemp=f'/wetterstation/{Ort}/temperatur'
                TopicHum=f'/wetterstation/{Ort}/luftfeuchtigkeit'
                TopicVBatt=f'/wetterstation/{Ort}/vbatt'
                TopicDruck=f'/wetterstation/{Ort}/luftdruck'
                msgs = [(TopicTemp, temp),(TopicHum, hum, 0, False),(TopicVBatt,volt),(TopicDruck,druck)]
                pwd = {'username':"rutsch", 'password':"super_mqtt"}
                publish.multiple(msgs, auth=pwd, hostname="localhost")
            else:
                print(f'\n{now}: Lora-Paketfehler (s. Log)')
                
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
        global PacketLostCounter
        global LastReceivedTime
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        while True:
            try:
                # Receive-Timeout aufgetreten? Manchmal geht der LoRa-Receiver in einen
                # Zustand, wo er nichts mehr empfängt. Dann resetten!
                now=time.time()
                if now-LastReceivedTime > 300:
                    LastReceivedTime=now
                    print("\nTimeout beim Empfangen. Resette LoRa-Empfänger")
                    logging.error("Timeout beim Empfangen. Resette LoRa-Empfänger")
                    lora_init()
                    self.reset_ptr_rx()
                    self.set_mode(MODE.RXCONT)
                sleep(.5)
                #rssi_value = self.get_rssi_value()
                #status = self.get_modem_status()
                #sys.stdout.flush()
                #sys.stdout.write("\r%d %d %d" % (rssi_value, status['rx_ongoing'], status['modem_clear']))
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
    # Sendefrequenz: 434,54 MHz
    # Preambellänge: 14
    # Bandbreite 500 kHz
    # Sync-Byte: 0x37
    # Spreading-Factor: 8 = 256 Chips/symbol
    # Coding-Rate: 6 = 4/6 = 1,5-facher FEC-Overhead
    global lora
    lora.reset()
    lora.set_mode(MODE.STDBY)
    lora.set_pa_config(pa_select=1)
    lora.set_rx_crc(True)
    lora.set_freq(434.54)
    lora.set_preamble(14)
    lora.set_bw(9) # 8=250 kHz, 9=500 kHz
    lora.set_sync_word(0x37)
    lora.set_coding_rate(CODING_RATE.CR4_6)
    lora.set_spreading_factor(8)
    #lora.set_pa_config(max_power=0, output_power=0)
    #lora.set_lna_gain(GAIN.G1)
    lora.set_implicit_header_mode(False)
    #lora.set_low_data_rate_optim(True)
    #lora.set_pa_ramp(PA_RAMP.RAMP_50_us)
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
