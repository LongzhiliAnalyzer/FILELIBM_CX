
#ifndef  __CH375DRIV_H__
#define  __CH375DRIV_H__

#include "sys.h"
#include "delay.h"

/* Ĭ��ʹ�ò���ģʽ��ʹ��INT�ţ����ڲ���U���ٶ�Զ���ڴ����ٶȡ� */

//#define CH375_PORT_MODE   1            //���ڲ�ѯ���գ�ʹ��INT��
#define CH375_PORT_MODE   2            //���ڣ�ʹ��INT��
//#define CH375_PORT_MODE   3            //���ڣ���ʹ��INT��


void  CH375_Init( void );

#if ( CH375_PORT_MODE==3 )
extern uint8_t xReadCH375Cmd( void );
#endif

#endif
