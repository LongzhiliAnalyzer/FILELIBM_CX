
/******************************************************************************
* ע�⣺CH375�Ĵ���ģʽ������ʹ��������(TX��RX��INT���������١����ǲ���ģʽ����INT�Ų��ӣ�ʹ��xReadCH375Cmd��ѯ�жϡ�
*      ��ʹ��INT�ŵ�ʱ�򣬿���ʹ���жϣ�Ҳ����ʹ�ò�ѯ�����Ǵ˴��붼�ǻ���INT�ŵĲ�ѯ��(PC9)���Ƽ�ʹ�ò�ѯINT�ŵķ�ʽ��
*      �����ʹ��INT�ŵ��жϷ�ʽ����Ҫ���� NO_DEFAULT_CH375_INT �Կ����ⲿ�жϷ�ʽ��
*      �Ա�CH376��CH376�����ֽӿ��£����ڡ����ڡ�SPI�ӿڶ��ǿ���ʡ��INT�Ų��ӵģ�ͨ�������ķ�ʽ��ѯ�жϡ�
* ע�⣺CH375���5V���磬������STM32���ڵ�ƽ��ƥ�䵼�����ݲ��ȶ������3.3V���磬�������5V������๲���ߣ����Կ�©���+5V������
********************************************************************************
*/

#include "CH375DRV.H"

#define RCC_APB2Periph_GPIO_CH375_Data    RCC_APB2Periph_GPIOC
#define RCC_APB2Periph_GPIO_CH375_CTL     RCC_APB2Periph_GPIOC

#if (  CH375_PORT_MODE==1 || CH375_PORT_MODE==2 )
#define CH375_INT_WIRE   PCin(9)   
#endif


#include "CH375HFM.H"

/* ����ģʽ */
#if ( CH375_PORT_MODE==1 )

//CH375д�����
void xWriteCH375Cmd( UINT8 mCmd )	/* �ⲿ����ı�CH375�������õ��ӳ���,��CH375д����,��С����Ϊ4uS,����֮ǰ֮�����ʱ2uS */
{
	USART_SendData(USART2, (uint16_t)mCmd|0x0100);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	delay_us(2);	
}
//CH375д���ݺ���
void xWriteCH375Data( UINT8 mData )	/* �ⲿ����ı�CH375�������õ��ӳ���,��CH375д����,��С����Ϊ1.5uS,����֮����ʱ1.5uS */
{
	USART_SendData(USART2, (uint16_t)mData );
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	delay_us(1);	
}
//CH375�����ݺ���
UINT8 xReadCH375Data( void )
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==RESET);
	return( (UINT8)USART_ReceiveData(USART2) );	
}

//����PD5==tx   PD6==rx ����
//PC9 == INT   //������Ĭ������Ϊ9600
void CH375_Init( void )
{
    /* ʹ�� CH375 ����ʱ��-------------------------------------*/
//    SystemInit();
	GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CH375_Data | 
                           RCC_APB2Periph_GPIO_CH375_CTL , ENABLE);
    
    /* CH375 ������������--------------------------------------*/ 
    
    
    /* �ж������ INT# PB9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    
    /* A0 , CS# , RD#, WR#  PB0-LED����*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11
                                 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
}

#endif


/* ����ģʽ */
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

//CH375д�����
void xWriteCH375Cmd( UINT8 mCmd )
{
	CH375_DATA_DAT_OUT( mCmd );  /* ��CH376�Ĳ���������� */
	CH375_DATA_DIR_OUT( );  /* ���ò��ڷ���Ϊ��� */
	CH375_A0 = 1;
	CH375_CS = 0;
	CH375_WR = 0;  /* �����Чд�����ź�, дCH376оƬ������˿� */
	CH375_WR = 0;  /* �ò���������,������ʱ,CH376Ҫ���д������ȴ���40nS */
	CH375_WR = 1;  /* �����Ч�Ŀ����ź�, ��ɲ���CH376оƬ */
	CH375_CS = 1;
	CH375_A0 = 0;
	CH375_DATA_DIR_IN( );  /* ��ֹ������� */
	delay_us(2);
}
//CH375д���ݺ���
void xWriteCH375Data( UINT8 mData )	
{
	CH375_DATA_DAT_OUT( mData );  /* ��CH376�Ĳ���������� */
	CH375_DATA_DIR_OUT( );  /* ���ò��ڷ���Ϊ��� */
	CH375_A0 = 0;
	CH375_CS = 0;
	CH375_WR = 0;  /* �����Чд�����ź�, дCH376оƬ�����ݶ˿� */
	CH375_WR = 0;  /* �ò���������,������ʱ,CH376Ҫ���д������ȴ���40nS */
	CH375_WR = 1;  /* �����Ч�Ŀ����ź�, ��ɲ���CH376оƬ */
	CH375_CS = 1;
	CH375_DATA_DIR_IN( );  /* ��ֹ������� */
	delay_us( 1 );  /* ȷ����д���ڴ���0.6uS */	
}
//CH375�����ݺ���
UINT8 xReadCH375Data( void )
{
	UINT8	mData;
	delay_us( 1 );  /* ȷ����д���ڴ���0.6uS */
	CH375_DATA_DIR_IN( );  /* ���ò��ڷ���Ϊ���� */
	CH375_A0 = 0;
	CH375_CS = 0;
	CH375_RD = 0;  /* �����Ч�������ź�, ��CH376оƬ�����ݶ˿� */
	CH375_RD = 0;  /* �ò���������,������ʱ */
	mData = CH375_DATA_DAT_IN( );  /* ��CH376�Ĳ����������� */
	CH375_RD = 1;
	CH375_CS = 1;  /* �����Ч�Ŀ����ź�, ��ɲ���CH376оƬ */
	return( mData );	
	
}

#if ( CH375_PORT_MODE==3 )
//CH375��״̬����
UINT8 xReadCH375Cmd( void )
{
	UINT8	mData;
	delay_us( 1 );  /* ȷ����д���ڴ���0.6uS */
	CH375_DATA_DIR_IN( );  /* ���ò��ڷ���Ϊ���� */
	CH375_A0 = 1;
	CH375_CS = 0;
	CH375_RD = 0;  /* �����Ч�������ź�, ��CH376оƬ�����ݶ˿� */
	CH375_RD = 0;  /* �ò���������,������ʱ */
	mData = CH375_DATA_DAT_IN( );  /* ��CH376�Ĳ����������� */
	CH375_RD = 1;
	CH375_CS = 1;  /* �����Ч�Ŀ����ź�, ��ɲ���CH376оƬ */
	CH375_A0 = 0;
	return( mData );	
}
#endif

//ע�⣺CH375���5V���磬������STM32���ڵ�ƽ��ƥ�䣬���3.3V���磬�������5V������๲���ߣ����Կ�©��� + 5V������
void CH375_Init( void )
{
    /* ʹ�� CH375 ����ʱ��-------------------------------------*/
//    SystemInit();
	GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CH375_Data | 
                           RCC_APB2Periph_GPIO_CH375_CTL , ENABLE);
    
    /* CH375 ������������--------------------------------------*/ 
    
    
    /* �ж������ INT# PB9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    
    /* A0 , CS# , RD#, WR#  PB0-LED����*/
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
	CH375_DATA_DIR_IN( );                                   /* ���ò������� */
}



#endif


