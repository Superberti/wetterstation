/*

  u8x8_d_ssd1362.c
  
  https://github.com/olikraus/u8g2/issues/2051
  
  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2022, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  


  SSD1362: 
    256 x 64 (ssd1322: 480 x 128)
    16 gray scale
  
  
*/
#include "u8x8.h"




static const uint8_t u8x8_d_ssd1362_powersave0_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_C(0x0af),		                /* ssd1362: display on */
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};

static const uint8_t u8x8_d_ssd1362_powersave1_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_C(0x0ae),		                /* ssd1362: display off */
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};



/* interpret b as a monochrome bit pattern, write value 15 for high bit and value 0 for a low bit */
/* topbit (msb) is sent last */
/* example: b = 0x083 will send 0xff, 0x00, 0x00, 0xf0 */

/* 4 Jan 2017: I think this procedure not required any more. Delete? */
/*
static uint8_t u8x8_write_byte_to_16gr_device(u8x8_t *u8x8, uint8_t b)
{
  static uint8_t buf[4];
  static uint8_t map[4] = { 0, 0x00f, 0x0f0, 0x0ff };
  buf [3] = map[b & 3];
  b>>=2;
  buf [2] = map[b & 3];
  b>>=2;
  buf [1] = map[b & 3];
  b>>=2;
  buf [0] = map[b & 3];
  return u8x8_cad_SendData(u8x8, 4, buf);
}
*/


/*
  input:
    one tile (8 Bytes)
  output:
    Tile for SSD1325 (32 Bytes)
*/

static uint8_t u8x8_ssd1362_to32_dest_buf[32];

#ifdef no_used

static uint8_t *u8x8_ssd1362_8to32(U8X8_UNUSED u8x8_t *u8x8, uint8_t *ptr)
{
  uint8_t v;
  uint8_t a,b;
  uint8_t i, j;
  uint8_t *dest;
  
  for( j = 0; j < 4; j++ )
  {
    dest = u8x8_ssd1362_to32_dest_buf;
    dest += j;
    a =*ptr;
    ptr++;
    b = *ptr;
    ptr++;
    for( i = 0; i < 8; i++ )
    {
      v = 0;
      if ( a&1 ) v |= 0xf0;
      if ( b&1 ) v |= 0x0f;
      *dest = v;
      dest+=4;
      a >>= 1;
      b >>= 1;
    }
  }
  
  return u8x8_ssd1362_to32_dest_buf;
}


uint8_t u8x8_d_ssd1362_common(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint8_t x; 
  uint8_t y, c;
  uint8_t *ptr;
  switch(msg)
  {
    /* U8X8_MSG_DISPLAY_SETUP_MEMORY is handled by the calling function */
    /*
    case U8X8_MSG_DISPLAY_SETUP_MEMORY:
      break;
    case U8X8_MSG_DISPLAY_INIT:
      u8x8_d_helper_display_init(u8x8);
      u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1362_256x64_init_seq);
      break;
    */
    case U8X8_MSG_DISPLAY_SET_POWER_SAVE:
      if ( arg_int == 0 )
	u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1362_powersave0_seq);
      else
	u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1362_powersave1_seq);
      break;
#ifdef U8X8_WITH_SET_CONTRAST
    case U8X8_MSG_DISPLAY_SET_CONTRAST:
      u8x8_cad_StartTransfer(u8x8);
      u8x8_cad_SendCmd(u8x8, 0x0C1 );
      u8x8_cad_SendArg(u8x8, arg_int );	/* ssd1362 has range from 0 to 255 */
      u8x8_cad_EndTransfer(u8x8);
      break;
#endif
    case U8X8_MSG_DISPLAY_DRAW_TILE:
      u8x8_cad_StartTransfer(u8x8);
      x = ((u8x8_tile_t *)arg_ptr)->x_pos;    
      x *= 2;		// only every 4th col can be addressed
      x += u8x8->x_offset;		
    
      y = (((u8x8_tile_t *)arg_ptr)->y_pos);
      y *= 8;
    
      
      u8x8_cad_SendCmd(u8x8, 0x075 );	/* set row address, moved out of the loop (issue 302) */
      u8x8_cad_SendArg(u8x8, y);
      u8x8_cad_SendArg(u8x8, y+7);
      
      do
      {
	c = ((u8x8_tile_t *)arg_ptr)->cnt;
	ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;

	do
	{
	  u8x8_cad_SendCmd(u8x8, 0x015 );	/* set column address */
	  u8x8_cad_SendArg(u8x8, x );	/* start */
	  u8x8_cad_SendArg(u8x8, x+1 );	/* end */

	  u8x8_cad_SendData(u8x8, 32, u8x8_ssd1362_8to32(u8x8, ptr));
	  
	  ptr += 8;
	  x += 2;
	  c--;
	} while( c > 0 );
	
	//x += 2;
	arg_int--;
      } while( arg_int > 0 );
      
      u8x8_cad_EndTransfer(u8x8);
      break;
    default:
      return 0;
  }
  return 1;
}

#endif

static uint8_t *u8x8_ssd1362_4to32(U8X8_UNUSED u8x8_t *u8x8, uint8_t *ptr)
{
  uint8_t v;
  uint8_t a;
  uint8_t i, j;
  uint8_t *dest;
  
  for( j = 0; j < 4; j++ )
  {
    dest = u8x8_ssd1362_to32_dest_buf;
    dest += j;
    a =*ptr;
    ptr++;
    for( i = 0; i < 8; i++ )
    {
      v = 0;
      if ( a&1 ) v = 0xff;
      *dest = v;
      dest+=4;
      a >>= 1;
    }
  }
  
  return u8x8_ssd1362_to32_dest_buf;
}


uint8_t u8x8_d_ssd1362_common2(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint8_t x; 
  uint8_t y, c;
  uint8_t *ptr;
  switch(msg)
  {
    /* U8X8_MSG_DISPLAY_SETUP_MEMORY is handled by the calling function */
    /*
    case U8X8_MSG_DISPLAY_SETUP_MEMORY:
      break;
    case U8X8_MSG_DISPLAY_INIT:
      u8x8_d_helper_display_init(u8x8);
      u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1322_256x64_init_seq);
      break;
    */
    case U8X8_MSG_DISPLAY_SET_POWER_SAVE:
      if ( arg_int == 0 )
	u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1362_powersave0_seq);
      else
	u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1362_powersave1_seq);
      break;
#ifdef U8X8_WITH_SET_CONTRAST
    case U8X8_MSG_DISPLAY_SET_CONTRAST:
      u8x8_cad_StartTransfer(u8x8);
      u8x8_cad_SendCmd(u8x8, 0x0C1 );
      u8x8_cad_SendArg(u8x8, arg_int );	/* ssd1322 has range from 0 to 255 */
      u8x8_cad_EndTransfer(u8x8);
      break;
#endif
    case U8X8_MSG_DISPLAY_DRAW_TILE:
      u8x8_cad_StartTransfer(u8x8);
      x = ((u8x8_tile_t *)arg_ptr)->x_pos;    
      x *= 2;		// only every 4th col can be addressed
      x *= 2;		// only every second pixel is used in the 128x64 NHD OLED 
    
      x += u8x8->x_offset;
    
      y = (((u8x8_tile_t *)arg_ptr)->y_pos);
      y *= 8;
          
      u8x8_cad_SendCmd(u8x8, 0x075 );	/* set row address, moved out of the loop (issue 302) */
      u8x8_cad_SendArg(u8x8, y);
      u8x8_cad_SendArg(u8x8, y+7);
      
      do
      {
	c = ((u8x8_tile_t *)arg_ptr)->cnt;
	ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;

	do
	{
	  u8x8_cad_SendCmd(u8x8, 0x015 );	/* set column address */
	  u8x8_cad_SendArg(u8x8, x );	/* start */
	  u8x8_cad_SendArg(u8x8, x+1 );	/* end */
	  u8x8_cad_SendData(u8x8, 32, u8x8_ssd1362_4to32(u8x8, ptr));	  
	  ptr += 4;
	  x += 2;
	  
	  u8x8_cad_SendCmd(u8x8, 0x015 );	/* set column address */
	  u8x8_cad_SendArg(u8x8, x );	/* start */
	  u8x8_cad_SendArg(u8x8, x+1 );	/* end */
	  u8x8_cad_SendData(u8x8, 32, u8x8_ssd1362_4to32(u8x8, ptr));	  
	  ptr += 4;
	  x += 2;
	  
	  c--;
	} while( c > 0 );
	
	//x += 2;
	arg_int--;
      } while( arg_int > 0 );
      
      u8x8_cad_EndTransfer(u8x8);
      break;
    default:
      return 0;
  }
  return 1;
}

/*=========================================================*/

static const uint8_t u8x8_d_ssd1362_256x64_flip0_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_CAA(0x0a0, 0x006, 0x011),		/* remap */
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};

static const uint8_t u8x8_d_ssd1362_256x64_flip1_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_CAA(0x0a0, 0x014, 0x011),		/* remap */
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};

static const u8x8_display_info_t u8x8_ssd1362_256x64_display_info =
{
  /* chip_enable_level = */ 0,
  /* chip_disable_level = */ 1,
  
  /* post_chip_enable_wait_ns = */ 20,
  /* pre_chip_disable_wait_ns = */ 10,
  /* reset_pulse_width_ms = */ 100, 	/* ssd1362: 2 us */
  /* post_reset_wait_ms = */ 100, /* far east OLEDs need much longer setup time */
  /* sda_setup_time_ns = */ 50,		/* ssd1362: 15ns, but cycle time is 100ns, so use 100/2 */
  /* sck_pulse_width_ns = */ 50,	/* ssd1362: 20ns, but cycle time is 100ns, so use 100/2, AVR: below 70: 8 MHz, >= 70 --> 4MHz clock */
  /* sck_clock_hz = */ 10000000UL,	/* since Arduino 1.6.0, the SPI bus speed in Hz. Should be  1000000000/sck_pulse_width_ns, increased to 8MHz (issue 215), 10 MHz (issue 301) */
  /* spi_mode = */ 0,		/* active high, rising edge */
  /* i2c_bus_clock_100kHz = */ 4,
  /* data_setup_time_ns = */ 10,
  /* write_pulse_width_ns = */ 150,	/* ssd1362: cycle time is 300ns, so use 300/2 = 150 */
  /* tile_width = */ 32,		/* 256 pixel, so we require 32 bytes for this */
  /* tile_hight = */ 8,
  /* default_x_offset = */ 0x01c,	/* this is the byte offset (there are two pixel per byte with 4 bit per pixel) */
  /* flipmode_x_offset = */ 0x01c,
  /* pixel_width = */ 256,
  /* pixel_height = */ 64
};


#ifdef OBSOLETE
static const uint8_t u8x8_d_ssd1362_256x64_init_seq_OBSOLETE[] = {
    
  U8X8_DLY(1),
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_DLY(1),
  
  U8X8_CA(0xfd, 0x12),            	/* unlock */
  U8X8_C(0xae),		                /* display off */
  U8X8_CA(0xb3, 0x91),			/* set display clock divide ratio/oscillator frequency (set clock as 80 frames/sec)  */  
  U8X8_CA(0xca, 0x3f),			/* multiplex ratio 1/64 Duty (0x0F~0x3F) */  
  U8X8_CA(0xa2, 0x00),			/* display offset, shift mapping ram counter */  
  U8X8_CA(0xa1, 0x00),			/* display start line */  
  //U8X8_CAA(0xa0, 0x14, 0x11),	/* Set Re-Map / Dual COM Line Mode */  
  U8X8_CAA(0xa0, 0x06, 0x011),	/* Set Re-Map / Dual COM Line Mode */  
  U8X8_CA(0xab, 0x01),			/* Enable Internal VDD Regulator */  
  U8X8_CAA(0xb4, 0xa0, 0x005|0x0fd),	/* Display Enhancement A */  
  U8X8_CA(0xc1, 0x9f),			/* contrast */  
  U8X8_CA(0xc7, 0x0f),			/* Set Scale Factor of Segment Output Current Control */  
  U8X8_C(0xb9),		                /* linear grayscale */
  U8X8_CA(0xb1, 0xe2),			/* Phase 1 (Reset) & Phase 2 (Pre-Charge) Period Adjustment */  
  U8X8_CAA(0xd1, 0x082|0x020, 0x020),	/* Display Enhancement B */  
  U8X8_CA(0xbb, 0x1f),			/* precharge  voltage */  
  U8X8_CA(0xb6, 0x08),			/* precharge  period */  
  U8X8_CA(0xbe, 0x07),			/* vcomh */  
  U8X8_C(0xa6),		                /* normal display */
  U8X8_C(0xa9),		                /* exit partial display */


  U8X8_DLY(1),					/* delay 2ms */

  
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};
#endif

/* https://github.com/olikraus/u8g2/issues/2051 */
static const uint8_t u8x8_d_ssd1362_256x64_init_seq[] = {
    
  U8X8_DLY(1),
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_DLY(1),
  
  U8X8_CA(0xfd, 0x12),            	/* unlock */
  U8X8_C(0xae),		                /* display off */
  //U8X8_C(0XFD), //Set Command Lock
  //U8X8_C(0X12), //(12H=Unlock,16H=Lock)

  U8X8_C(0XAE), //Display OFF(Sleep Mode)

  U8X8_C(0X15), //Set column Address
  U8X8_C(0X00), //Start column Address
  U8X8_C(0X7F), //End column Address

  U8X8_C(0X75), //Set Row Address
  U8X8_C(0X00), //Start Row Address
  U8X8_C(0X3F), //End Row Address

  U8X8_C(0X81), //Set contrast
  U8X8_C(0x2f),

	
/*
Re- map setting in Graphic Display Data RAM
(GDDRAM)

A[0] = 0b, Disable Column Address Re-map (RESET)
A[0] = 1b, Enable Column Address Re-map			***

A[1] = 0b, Disable Nibble Re-map (RESET)
A[1] = 1b, Enable Nibble Re-map				***

A[2] = 0b, Enable Horizontal Address Increment (RESET)   ***
A[2] = 1b, Enable Vertical Address Increment

A[4] = 0b, Disable COM Re-map (RESET)
A[4] = 1b, Enable COM Re-map

A[6] = 0b, Disable SEG Split Odd Even			***
A[6] = 1b, Enable SEG Split Odd Even (RESET)

A[7] = 0b, Disable SEG left/right remap (RESET)
A[7] = 1b, Enable SEG left/right remap

	?00?0011	--> 0x03 0x83 0x13 0x93
*/
  U8X8_CA(0XA0, 0xc3), //Set Remap c3 = 11000011

  U8X8_C(0XA1), //Set Display Start Line
  U8X8_C(0X00),

  U8X8_C(0XA2), //Set Display Offset
  U8X8_C(0X00),

  U8X8_C(0XA4), //Normal Display

  U8X8_C(0XA8), //Set Multiplex Ratio
  U8X8_C(0X3F),

  U8X8_C(0XAB), //Set VDD regulator
  U8X8_C(0X01), //Regulator Enable

  U8X8_C(0XAD), //External /Internal IREF Selection
  U8X8_C(0X8E),

  U8X8_C(0XB1), //Set Phase Length
  U8X8_C(0X22),

  U8X8_C(0XB3), //Display clock Divider
  U8X8_C(0XA0),

  U8X8_C(0XB6), //Set Second precharge Period
  U8X8_C(0X04),

  U8X8_C(0XB9), //Set Linear LUT

  U8X8_C(0XBc), //Set pre-charge voltage level
  U8X8_C(0X10), //0.5*Vcc

  U8X8_C(0XBD), //Pre-charge voltage capacitor Selection
  U8X8_C(0X01),

  U8X8_C(0XBE), //Set cOM deselect voltage level
  U8X8_C(0X07), //0.82*Vcc

  U8X8_DLY(1),					/* delay 1ms */

  
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};


uint8_t u8x8_d_ssd1362_ws_256x64(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_DISPLAY_SETUP_MEMORY:
      u8x8_d_helper_display_setup_memory(u8x8, &u8x8_ssd1362_256x64_display_info);
      break;
    case U8X8_MSG_DISPLAY_INIT:
      u8x8_d_helper_display_init(u8x8);
      u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1362_256x64_init_seq);
      break;
    case U8X8_MSG_DISPLAY_SET_FLIP_MODE:
      if ( arg_int == 0 )
      {
	u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1362_256x64_flip0_seq);
	u8x8->x_offset = u8x8->display_info->default_x_offset;
      }
      else
      {
	u8x8_cad_SendSequence(u8x8, u8x8_d_ssd1362_256x64_flip1_seq);
	u8x8->x_offset = u8x8->display_info->flipmode_x_offset;
      }
      break;
    
    default:
      return u8x8_d_ssd1362_common2(u8x8, msg, arg_int, arg_ptr);
  }
  return 1;
}

