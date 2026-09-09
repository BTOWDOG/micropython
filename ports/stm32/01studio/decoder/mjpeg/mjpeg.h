#ifndef __MJPEG_H
#define __MJPEG_H 

#include <cdjpeg.h> 
#include <setjmp.h>
#include "stdio.h" 

struct my_error_mgr {
  struct jpeg_error_mgr pub;	
  jmp_buf setjmp_buffer;		//for return to caller 
}; 
typedef struct my_error_mgr * my_error_ptr;
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned long  int uint32_t;

uint8_t mjpegdec_init(uint16_t offx,uint16_t offy);
void mjpegdec_free(void);
uint8_t mjpegdec_decode(uint8_t* buf,uint32_t bsize);

#endif

