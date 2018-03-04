
#ifndef  __CH375DRIV_H__
#define  __CH375DRIV_H__

#include "sys.h"
#include "delay.h"

/* 默认使用并口模式、使用INT脚，并口操作U盘速度远高于串口速度。 */

//#define CH375_PORT_MODE   1            //串口查询接收，使用INT脚
#define CH375_PORT_MODE   2            //并口，使用INT脚
//#define CH375_PORT_MODE   3            //并口，不使用INT脚


void  CH375_Init( void );

#if ( CH375_PORT_MODE==3 )
extern uint8_t xReadCH375Cmd( void );
#endif

#endif

