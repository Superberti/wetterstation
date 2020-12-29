#ifndef WETTERSTATION_H_INCLUDED
#define WETTERSTATION_H_INCLUDED

// Prototypen:
uint8_t ComputeChecksum(uint8_t* bytes, int len);
uint8_t Crc8b(uint8_t aData);
void disp_buf(uint8_t *buf, int len);
esp_err_t i2c_master_init(void);
esp_err_t ReadSHT35(double * aTemp, double * aHum, uint8_t*rCRC_Err);


#endif // WETTERSTATION_H_INCLUDED
