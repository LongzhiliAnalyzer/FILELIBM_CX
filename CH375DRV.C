
/******************************************************************************
* 注意：CH375的串口模式，必须使用三根线(TX、RX、INT），不可少。但是并口模式可以INT脚不接，使用xReadCH375Cmd查询中断。
*      当使用INT脚的时候，可以使用中断，也可以使用查询，但是此代码都是基于INT脚的查询的(PC9)。推荐使用查询INT脚的方式。
*      如果想使用INT脚的中断方式，需要定义 NO_DEFAULT_CH375_INT 以开启外部中断方式。
*      对比CH376，CH376的三种接口下，串口、并口、SPI接口都是可以省略INT脚不接的，通过其他的方式查询中断。
* 注意：CH375如果5V供电，可能与STM32存在电平不匹配导致数据不稳定，最好3.3V供电，如果必须5V，建议多共地线，可以开漏输出+5V上拉。
********************************************************************************
*/

#include "CH375DRV.H"

#define RCC_APB2Periph_GPIO_CH375_Data    RCC_APB2Periph_GPIOC
#define RCC_APB2Periph_GPIO_CH375_CTL     RCC_APB2Periph_GPIOC

#if (  CH375_PORT_MODE==1 || CH375_PORT_MODE==2 )
#define CH375_INT_WIRE   PCin(9)   
#endif


#include "CH375HFM.H"

/* 串口模式 */
#if ( CH375_PORT_MODE==1 )

//CH375写命令函数
void xWriteCH375Cmd( UINT8 mCmd )	/* 外部定义的被CH375程序库调用的子程序,向CH375写命令,最小周期为4uS,否则之前之后各延时2uS */
{
	USART_SendData(USART2, (uint16_t)mCmd|0x0100);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	delay_us(2);	
}
//CH375写数据函数
void xWriteCH375Data( UINT8 mData )	/* 外部定义的被CH375程序库调用的子程序,向CH375写数据,最小周期为1.5uS,否则之后延时1.5uS */
{
	USART_SendData(USART2, (uint16_t)mData );
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	delay_us(1);	
}
//CH375读数据函数
UINT8 xReadCH375Data( void )
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==RESET);
	return( (UINT8)USART_ReceiveData(USART2) );	
}

//串口PD5==tx   PD6==rx 复用
//PC9 == INT   //波特率默认配置为9600
void CH375_Init( void )
{
    /* 使能 CH375 引脚时钟-------------------------------------*/
//    SystemInit();
	GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CH375_Data | 
                           RCC_APB2Periph_GPIO_CH375_CTL , ENABLE);
    
    /* CH375 控制引脚配置--------------------------------------*/ 
    
    
    /* 中断输入脚 INT# PB9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    
    /* A0 , CS# , RD#, WR#  PB0-LED测试*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11
                                 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
}

#endif


/* 并口模式 */
#if ( CH375_PORT_MODE==2 || CH375_PORT_MODE==3 )

//DATAPORT: E0~E7
//WR: E8
//CS: E9
//A0: E10
//RD: E11
//INT: C9

#define CH375_WR                      PCout(10)
#define CH375_CS                      PCout(8)
#define CH375_A0                      PCout(12)
#define CH375_RD                      PCout(11)
#define CH375_DATA_DIR_IN()           GPIOC->CRL = 0x44444444
#define CH375_DATA_DIR_OUT()          GPIOC->CRL = 0x33333333
#define CH375_DATA_DAT_OUT( mCmd )    GPIOC->ODR = (GPIOC->ODR & 0xFF00) | (uint8_t)mCmd
#define CH375_DATA_DAT_IN( )          (uint8_t)GPIOC->IDR

//CH375写命令函数
void xWriteCH375Cmd( UINT8 mCmd )
{
	CH375_DATA_DAT_OUT( mCmd );  /* 向CH376的并口输出数据 */
	CH375_DATA_DIR_OUT( );  /* 设置并口方向为输出 */
	CH375_A0 = 1;
	CH375_CS = 0;
	CH375_WR = 0;  /* 输出有效写控制信号, 写CH376芯片的命令端口 */
	CH375_WR = 0;  /* 该操作无意义,仅作延时,CH376要求读写脉冲宽度大于40nS */
	CH375_WR = 1;  /* 输出无效的控制信号, 完成操作CH376芯片 */
	CH375_CS = 1;
	CH375_A0 = 0;
	CH375_DATA_DIR_IN( );  /* 禁止数据输出 */
	delay_us(2);
}
//CH375写数据函数
void xWriteCH375Data( UINT8 mData )	
{
	CH375_DATA_DAT_OUT( mData );  /* 向CH376的并口输出数据 */
	CH375_DATA_DIR_OUT( );  /* 设置并口方向为输出 */
	CH375_A0 = 0;
	CH375_CS = 0;
	CH375_WR = 0;  /* 输出有效写控制信号, 写CH376芯片的数据端口 */
	CH375_WR = 0;  /* 该操作无意义,仅作延时,CH376要求读写脉冲宽度大于40nS */
	CH375_WR = 1;  /* 输出无效的控制信号, 完成操作CH376芯片 */
	CH375_CS = 1;
	CH375_DATA_DIR_IN( );  /* 禁止数据输出 */
	delay_us( 1 );  /* 确保读写周期大于0.6uS */	
}
//CH375读数据函数
UINT8 xReadCH375Data( void )
{
	UINT8	mData;
	delay_us( 1 );  /* 确保读写周期大于0.6uS */
	CH375_DATA_DIR_IN( );  /* 设置并口方向为输入 */
	CH375_A0 = 0;
	CH375_CS = 0;
	CH375_RD = 0;  /* 输出有效读控制信号, 读CH376芯片的数据端口 */
	CH375_RD = 0;  /* 该操作无意义,仅作延时 */
	mData = CH375_DATA_DAT_IN( );  /* 从CH376的并口输入数据 */
	CH375_RD = 1;
	CH375_CS = 1;  /* 输出无效的控制信号, 完成操作CH376芯片 */
	return( mData );	
	
}

#if ( CH375_PORT_MODE==3 )
//CH375读状态函数
UINT8 xReadCH375Cmd( void )
{
	UINT8	mData;
	delay_us( 1 );  /* 确保读写周期大于0.6uS */
	CH375_DATA_DIR_IN( );  /* 设置并口方向为输入 */
	CH375_A0 = 1;
	CH375_CS = 0;
	CH375_RD = 0;  /* 输出有效读控制信号, 读CH376芯片的数据端口 */
	CH375_RD = 0;  /* 该操作无意义,仅作延时 */
	mData = CH375_DATA_DAT_IN( );  /* 从CH376的并口输入数据 */
	CH375_RD = 1;
	CH375_CS = 1;  /* 输出无效的控制信号, 完成操作CH376芯片 */
	CH375_A0 = 0;
	return( mData );	
}
#endif

//注意：CH375如果5V供电，可能与STM32存在电平不匹配，最好3.3V供电，如果必须5V，建议多共地线，可以开漏输出 + 5V上拉。
void CH375_Init( void )
{
    /* 使能 CH375 引脚时钟-------------------------------------*/
//    SystemInit();
	GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CH375_Data | 
                           RCC_APB2Periph_GPIO_CH375_CTL , ENABLE);
    
    /* CH375 控制引脚配置--------------------------------------*/ 
    
    
    /* 中断输入脚 INT# PB9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    
    /* A0 , CS# , RD#, WR#  PB0-LED测试*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11
                                 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);	

#ifdef CH375_INT_WIRE	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//INT
    GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
	
	CH375_CS = 1;
	CH375_WR = 1;
	CH375_RD = 1;
	CH375_A0 = 0;
	CH375_DATA_DIR_IN( );                                   /* 设置并口输入 */
}



#endif



