/**
  ******************************************************************************
  * @file    src/main.c
  * @author  Oliver Rutsch
  * @brief   Testprogramm Raspi mit LoRa (SX1278)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lora.h"
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <atomic>
#include <sys/time.h>
#include "../shared/lora/lorastructs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/*
static const char *TEMP_TAG = "TE";
static const char *HUM_TAG = "LF";
static const char *PRESS_TAG = "LD";
static const char *ILLU_TAG = "BS";
static const char *COOL_TAG = "LU";
static const char *POS_TAG = "Ort";
static const char *VAL_TAG = "W";

static const char *ORT_SCHUPPEN_SCHATTEN = "A";
static const char *ORT_SCHUPPEN_SONNE = "B";
static const char *ORT_SCHUPPEN_INNEN = "C";
*/
std::atomic<bool> quit(false);    // signal flag

long long GetTime_us()
{
	timeval tv;
	gettimeofday(&tv,NULL);
	return ((long long) tv.tv_sec)*1000000LL+tv.tv_usec;
}

double GetTime_s()
{
  return GetTime_us()/1000000.0;
}

void got_signal(int)
{
  quit.store(true);
  fprintf(stderr,"got abort signal. Waiting for termination...\n");
}

void install_sighandler()
{
  struct sigaction sa;
  memset( &sa, 0, sizeof(sa) );
  sa.sa_handler = got_signal;
  sigfillset(&sa.sa_mask);
  sigaction(SIGINT,&sa,NULL);
}

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
		fprintf (stdout,"Usage: %s send|rec\n",argv[0]);
		exit(1);
	}
  if (strcmp("send", argv[1]) && strcmp("rec", argv[1]))
  {
		fprintf (stdout,"Invalid argument: %s\n",argv[1]);
		exit(1);
	}
  bool SendMode=!strcmp("send", argv[1]);
  SX1278_LoRa Transmitter;
  ReturnStatus stat=Transmitter.Init();

  install_sighandler();

  if (stat!=RT_OK)
  {
    fprintf(stderr,"Lora init failed!\n");
    exit(1);
  }
  fprintf(stdout,"Lora init success!\n");
  char buf[80]={};
  const int LoraBufSize=256;
  uint8_t lora_buf[LoraBufSize];
  int i=0;
  if (!SendMode)
  {
    if (Transmitter.lora_receive()!=RT_OK)
    {
      fprintf(stderr,"Could not activate receive mode\n");
      exit(1);
    }
    fprintf(stdout,"Start listening to LORA...\n");
  }
  uint16_t BytesRead;
  const double Timeout=20.0;
  double StartTime=GetTime_s();
  bool IsReceiving;
  int RecCounter=0;
  while (true)
  {
    if (SendMode)
    {
      sprintf(buf,"%d - Hallo LORA!!!",i);
      fprintf(stdout,"%s\n",buf);
      Transmitter.lora_send_packet((uint8_t *)buf, strlen(buf));
      sleep(5);
    }
    else
    {
      if (Transmitter.IsReceiving(IsReceiving)!=RT_OK)
      {
        fprintf(stderr,"IsReceiving() failed.\n");
        IsReceiving=false;
      }

      if ((GetTime_s()-StartTime)>Timeout || !IsReceiving)
      {
        if (IsReceiving)
          fprintf(stderr,"Resetting receiver due to read timeout...\n");
        else
          fprintf(stderr,"Resetting receiver due to IsReceiving=false...\n");
        Transmitter.Close();
        Transmitter.Init();
        install_sighandler();
        Transmitter.delay(100);
        Transmitter.lora_receive();
        StartTime=GetTime_s();
      }

      //Transmitter.lora_receive();
      if(Transmitter.lora_received(true))
      {
        StartTime=GetTime_s();
        ReturnStatus stat=Transmitter.lora_receive_packet(lora_buf, LoraBufSize, BytesRead);
        if (stat==RT_LORA_CRC_ERR)
        {
          fprintf(stderr,"Discarding packet: CRC-error.\n");
        }
        else if (stat==RT_HEADER_WITHOUT_CRC)
        {
          fprintf(stderr,"Discarding packet: Header without CRC received.\n");
        }
        else if (stat==RT_OK)
        {
          if (BytesRead>0)
          {
            // Paket empfangen

            //lora_buf[LoraBufSize-1]=0;
            //if (BytesRead<LoraBufSize-1)
              //lora_buf[BytesRead]=0;
            fprintf(stdout,"Lora Paket empfangen. Lfd. Nummer: %d Groesse: %d: \n",RecCounter,BytesRead);
			LoraPacketHeader ph;
			memcpy(&ph,lora_buf,sizeof(ph));
			fprintf(stdout,"Lora Tag (Lfd. Nummer remote: %d \n",ph.Tag);

			RecCounter++;
          }
          else
            fprintf(stderr,"got zero length packet...\n");
        }
        else
          fprintf(stderr,"Read failed with status: %d\n",stat);
      }
      else
        Transmitter.delay(100);
    }
    if( quit.load() )
      break;    // exit normally after SIGINT
    i++;
  }
  return 0;
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
