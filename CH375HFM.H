/* 2004.06.05, 2004.09.20, 2004.10.22, 2004.11.20, 2004.12.12, 2004.12.28, 2005.01.04, 2005.01.12, 2005.01.26, 2005.03.01, 2005.07.29, 2005.12.28, 2006.08.02, 2007.08.16, 2011.08.02
****************************************
**  Copyright  (C)  W.ch  1999-2007   **
**  Web:  http://www.winchiphead.com  **
****************************************
**  USB Host File Interface for CH375 **
**  TC2.0@PC, KEIL_C_3.11@ARM         **
****************************************
*/
/* CH375 主机文件系统接口 */
/* 支持: FAT12/FAT16/FAT32 */
/* ARM单片机, Cortex-M3内核指令, 小端数据格式, 查询方式或者中断方式 */

#ifndef __CH375HF_H__
#define __CH375HF_H__

#define CH375_LIB_VER		0x39

#ifdef __cplusplus
extern "C" {
#endif

/* FILE: CH375INC.H */

/* ********************************************************************************************************************* */
/* 硬件特性 */

#define	CH375_MAX_DATA_LEN	0x40			/* 最大数据包的长度, 缓冲区的长度 */

/* ********************************************************************************************************************* */
/* 命令代码 */

#define	CMD_GET_IC_VER		0x01			/* 获取芯片及固件版本 */
/* 输出: 版本号( 位7为1, 位6为0, 位5~位0为版本号 ) */
/*           CH375B返回版本号的值为0B7H即版本号为37H */

#define	CMD_ENTER_SLEEP		0x03			/* 进入睡眠状态 */

#define	CMD_SET_USB_SPEED	0x04			/* 设置USB总线速度, 在每次CMD_SET_USB_MODE设置USB工作模式时会自动恢复到12Mbps全速 */
/* 输入: 总线速度代码 */
/*           00H=12Mbps全速FullSpeed（默认值）, 01H=1.5Mbps（仅修改频率）, 02H=1.5Mbps低速LowSpeed */
#define	CMD_SET_SYS_FREQ	CMD_SET_USB_SPEED

#define	CMD_RESET_ALL		0x05			/* 执行硬件复位 */

#define	CMD_CHECK_EXIST		0x06			/* 测试工作状态 */
/* 输入: 任意数据 */
/* 输出: 输入数据的按位取反 */

#define	CMD_GET_TOGGLE		0x0A			/* 获取OUT事务的同步状态 */
/* 输入: 数据1AH */
/* 输出: 同步状态 */
/*           位4为1则OUT事务同步, 否则OUT事务不同步 */

#define	CMD_CHK_SUSPEND		0x0B			/* 设备方式: 设置检查USB总线挂起状态的方式 */
/* 输入: 数据10H, 检查方式 */
/*                    00H=不检查USB挂起, 04H=以50mS为间隔检查USB挂起, 05H=以10mS为间隔检查USB挂起 */

#define	CMD_DELAY_100US		0x0F			/* 并口方式: 延时100uS */
/* 输出: 延时期间输出0, 延时结束输出非0 */

#define	CMD_SET_USB_ID		0x12			/* 设备方式: 设置USB厂商VID和产品PID */
/* 输入: 厂商ID低字节, 厂商ID高字节, 产品ID低字节, 产品ID高字节 */

#define	CMD_SET_USB_ADDR	0x13			/* 设置USB地址 */
/* 输入: 地址值 */

#define	CMD_SET_USB_MODE	0x15			/* 设置USB工作模式 */
/* 输入: 模式代码 */
/*       00H=未启用的设备方式, 01H=已启用的设备方式并且使用外部固件模式, 02H=已启用的设备方式并且使用内置固件模式 */
/*       04H=未启用的主机方式, 05H=已启用的主机方式, 06H=已启用的主机方式并且自动产生SOF包, 07H=已启用的主机方式并且复位USB总线 */
/* 输出: 操作状态( CMD_RET_SUCCESS或CMD_RET_ABORT, 其它值说明操作未完成 ) */

#define	CMD_SET_ENDP2		0x18			/* 设备方式: 设置USB端点0的接收器 */
/* 输入: 工作方式 */
/*           位7为1则位6为同步触发位, 否则同步触发位不变 */
/*           位3~位0为事务响应方式:  0000-就绪ACK, 1110-正忙NAK, 1111-错误STALL */

#define	CMD_SET_ENDP3		0x19			/* 设备方式: 设置USB端点0的发送器 */
/* 输入: 工作方式 */
/*           位7为1则位6为同步触发位, 否则同步触发位不变 */
/*           位3~位0为事务响应方式:  0000~1000-就绪ACK, 1110-正忙NAK, 1111-错误STALL */

#define	CMD_SET_ENDP4		0x1A			/* 设备方式: 设置USB端点1的接收器 */
/* 输入: 工作方式 */
/*           位7为1则位6为同步触发位, 否则同步触发位不变 */
/*           位3~位0为事务响应方式:  0000-就绪ACK, 1110-正忙NAK, 1111-错误STALL */

#define	CMD_SET_ENDP5		0x1B			/* 设备方式: 设置USB端点1的发送器 */
/* 输入: 工作方式 */
/*           位7为1则位6为同步触发位, 否则同步触发位不变 */
/*           位3~位0为事务响应方式:  0000~1000-就绪ACK, 1110-正忙NAK, 1111-错误STALL */

#define	CMD_SET_ENDP6		0x1C			/* 设置USB端点2/主机端点的接收器 */
/* 输入: 工作方式 */
/*           位7为1则位6为同步触发位, 否则同步触发位不变 */
/*           位3~位0为事务响应方式:  0000-就绪ACK, 1101-就绪但不返回ACK, 1110-正忙NAK, 1111-错误STALL */

#define	CMD_SET_ENDP7		0x1D			/* 设置USB端点2/主机端点的发送器 */
/* 输入: 工作方式 */
/*           位7为1则位6为同步触发位, 否则同步触发位不变 */
/*           位3~位0为事务响应方式:  0000-就绪ACK, 1101-就绪但无须应答, 1110-正忙NAK, 1111-错误STALL */

#define	CMD_GET_STATUS		0x22			/* 获取中断状态并取消中断请求 */
/* 输出: 中断状态 */

#define	CMD_UNLOCK_USB		0x23			/* 设备方式: 释放当前USB缓冲区 */

#define	CMD_RD_USB_DATA0	0x27			/* 从当前USB中断的端点缓冲区读取数据块 */
/* 输出: 长度, 数据流 */

#define	CMD_RD_USB_DATA		0x28			/* 从当前USB中断的端点缓冲区读取数据块, 并释放缓冲区, 相当于 CMD_RD_USB_DATA0 + CMD_UNLOCK_USB */
/* 输出: 长度, 数据流 */

#define	CMD_WR_USB_DATA3	0x29			/* 设备方式: 向USB端点0的发送缓冲区写入数据块 */
/* 输入: 长度, 数据流 */

#define	CMD_WR_USB_DATA5	0x2A			/* 设备方式: 向USB端点1的发送缓冲区写入数据块 */
/* 输入: 长度, 数据流 */

#define	CMD_WR_USB_DATA7	0x2B			/* 向USB端点2或者主机端点的发送缓冲区写入数据块 */
/* 输入: 长度, 数据流 */

/* ************************************************************************** */
/* 以下命令用于USB主机方式, 只有CH375支持 */

#define	CMD_SET_BAUDRATE	0x02			/* 主机方式 & 串口方式: 设置串口通讯波特率 */
/* 输入: 波特率分频系数, 波特率分频常数 */
/* 输出: 操作状态( CMD_RET_SUCCESS或CMD_RET_ABORT, 其它值说明操作未完成 ) */

#define	CMD_SET_RETRY		0x0B			/* 主机方式: 设置USB事务操作的重试次数 */
/* 输入: 数据25H, 重试次数 */
/*                    位7为0则收到NAK时不重试, 位7为1位6为0则收到NAK时无限重试, 位7为1位6为1则收到NAK时最多重试2秒, 位5~位0为超时后的重试次数 */

#define	CMD_SET_DISK_LUN	0x0B			/* 主机方式: 设置USB存储器的当前逻辑单元号 */
/* 输入: 数据34H, 新的当前逻辑单元号(00H-0FH) */

#define	CMD_SET_PKT_P_SEC	0x0B			/* 主机方式: 设置USB存储器的每扇区数据包总数 */
/* 输入: 数据39H, 新的每扇区数据包总数(08H,10H,20H,40H) */

#define	CMD_TEST_CONNECT	0x16			/* 主机方式: 检查USB设备连接状态 */
/* 输出: 状态( USB_INT_CONNECT或USB_INT_DISCONNECT或USB_INT_USB_READY, 其它值说明操作未完成 ) */

#define	CMD_ABORT_NAK		0x17			/* 主机方式: 放弃当前NAK的重试 */

#define	CMD_CLR_STALL		0x41			/* 主机方式: 控制传输-清除端点错误 */
/* 输入: 端点号 */
/* 输出中断 */

#define	CMD_SET_ADDRESS		0x45			/* 主机方式: 控制传输-设置USB地址 */
/* 输入: 地址值 */
/* 输出中断 */

#define	CMD_GET_DESCR		0x46			/* 主机方式: 控制传输-获取描述符 */
/* 输入: 描述符类型 */
/* 输出中断 */

#define	CMD_SET_CONFIG		0x49			/* 主机方式: 控制传输-设置USB配置 */
/* 输入: 配置值 */
/* 输出中断 */

#define	CMD_AUTO_SETUP		0x4D			/* 主机方式: 自动配置USB设备 */
/* 输出中断 */

#define	CMD_ISSUE_TKN_X		0x4E			/* 主机方式: 发出同步令牌, 执行事务, 该命令可代替 CMD_SET_ENDP6/CMD_SET_ENDP7 + CMD_ISSUE_TOKEN */
/* 输入: 同步标志, 事务属性 */
/*           同步标志的位7为主机端点IN的同步触发位, 位6为主机端点OUT的同步触发位, 位5~位0必须为0 */
/*           事务属性的低4位是令牌, 高4位是端点号 */
/* 输出中断 */

#define	CMD_ISSUE_TOKEN		0x4F			/* 主机方式: 发出令牌, 执行事务 */
/* 输入: 事务属性 */
/*           低4位是令牌, 高4位是端点号 */
/* 输出中断 */

#define	CMD_DISK_BOC_CMD	0x50			/* 主机方式: 对USB存储器执行BulkOnly传输协议的命令 */
/* 输出中断 */

#define	CMD_DISK_INIT		0x51			/* 主机方式: 初始化USB存储器 */
/* 输出中断 */

#define	CMD_DISK_RESET		0x52			/* 主机方式: 复位USB存储器 */
/* 输出中断 */

#define	CMD_DISK_SIZE		0x53			/* 主机方式: 获取USB存储器的容量 */
/* 输出中断 */

#define	CMD_DISK_READ		0x54			/* 主机方式: 从USB存储器读数据块(以扇区为单位) */
/* 输入: LBA扇区地址(总长度32位, 低字节在前), 扇区数(01H~FFH) */
/* 输出中断 */

#define	CMD_DISK_RD_GO		0x55			/* 主机方式: 继续执行USB存储器的读操作 */
/* 输出中断 */

#define	CMD_DISK_WRITE		0x56			/* 主机方式: 向USB存储器写数据块(以扇区为单位) */
/* 输入: LBA扇区地址(总长度32位, 低字节在前), 扇区数(01H~FFH) */
/* 输出中断 */

#define	CMD_DISK_WR_GO		0x57			/* 主机方式: 继续执行USB存储器的写操作 */
/* 输出中断 */

#define	CMD_DISK_INQUIRY	0x58			/* 主机方式: 查询USB存储器特性 */
/* 输出中断 */

#define	CMD_DISK_READY		0x59			/* 主机方式: 检查USB存储器就绪 */
/* 输出中断 */

#define	CMD_DISK_R_SENSE	0x5A			/* 主机方式: 检查USB存储器错误 */
/* 输出中断 */

#define	CMD_DISK_MAX_LUN	0x5D			/* 主机方式: 获取USB存储器最大逻辑单元号 */
/* 输出中断 */

/* ********************************************************************************************************************* */
/* 操作状态 */

#define	CMD_RET_SUCCESS		0x51			/* 命令操作成功 */
#define	CMD_RET_ABORT		0x5F			/* 命令操作失败 */

/* ********************************************************************************************************************* */
/* USB中断状态 */

#ifndef	USB_INT_EP0_SETUP

/* 以下状态代码为特殊事件中断, 如果通过CMD_CHK_SUSPEND启用USB总线挂起检查, 那么必须处理USB总线挂起和睡眠唤醒的中断状态 */
#define	USB_INT_USB_SUSPEND	0x05			/* USB总线挂起事件 */
#define	USB_INT_WAKE_UP		0x06			/* 从睡眠中被唤醒事件 */

/* 以下状态代码0XH用于USB设备方式 */
/*   内置固件模式下只需要处理: USB_INT_EP1_OUT, USB_INT_EP1_IN, USB_INT_EP2_OUT, USB_INT_EP2_IN */
/*   位7-位4为0000 */
/*   位3-位2指示当前事务, 00=OUT, 10=IN, 11=SETUP */
/*   位1-位0指示当前端点, 00=端点0, 01=端点1, 10=端点2, 11=USB总线复位 */
#define	USB_INT_EP0_SETUP	0x0C			/* USB端点0的SETUP */
#define	USB_INT_EP0_OUT		0x00			/* USB端点0的OUT */
#define	USB_INT_EP0_IN		0x08			/* USB端点0的IN */
#define	USB_INT_EP1_OUT		0x01			/* USB端点1的OUT */
#define	USB_INT_EP1_IN		0x09			/* USB端点1的IN */
#define	USB_INT_EP2_OUT		0x02			/* USB端点2的OUT */
#define	USB_INT_EP2_IN		0x0A			/* USB端点2的IN */
/* USB_INT_BUS_RESET	0x0000XX11B */		/* USB总线复位 */
#define	USB_INT_BUS_RESET1	0x03			/* USB总线复位 */
#define	USB_INT_BUS_RESET2	0x07			/* USB总线复位 */
#define	USB_INT_BUS_RESET3	0x0B			/* USB总线复位 */
#define	USB_INT_BUS_RESET4	0x0F			/* USB总线复位 */

#endif

/* 以下状态代码2XH-3XH用于USB主机方式的通讯失败代码, 仅CH375支持 */
/*   位7-位6为00 */
/*   位5为1 */
/*   位4指示当前接收的数据包是否同步 */
/*   位3-位0指示导致通讯失败时USB设备的应答: 0010=ACK, 1010=NAK, 1110=STALL, 0011=DATA0, 1011=DATA1, XX00=超时 */
/* USB_INT_RET_ACK	0x001X0010B */			/* 错误:对于IN事务返回ACK */
/* USB_INT_RET_NAK	0x001X1010B */			/* 错误:返回NAK */
/* USB_INT_RET_STALL	0x001X1110B */		/* 错误:返回STALL */
/* USB_INT_RET_DATA0	0x001X0011B */		/* 错误:对于OUT/SETUP事务返回DATA0 */
/* USB_INT_RET_DATA1	0x001X1011B */		/* 错误:对于OUT/SETUP事务返回DATA1 */
/* USB_INT_RET_TOUT	0x001XXX00B */			/* 错误:返回超时 */
/* USB_INT_RET_TOGX	0x0010X011B */			/* 错误:对于IN事务返回数据不同步 */
/* USB_INT_RET_PID	0x001XXXXXB */			/* 错误:未定义 */

/* 以下状态代码1XH用于USB主机方式的操作状态代码, 仅CH375支持 */
#ifndef	USB_INT_SUCCESS
#define	USB_INT_SUCCESS		0x14			/* USB事务或者传输操作成功 */
#define	USB_INT_CONNECT		0x15			/* 检测到USB设备连接事件 */
#define	USB_INT_DISCONNECT	0x16			/* 检测到USB设备断开事件 */
#define	USB_INT_BUF_OVER	0x17			/* USB控制传输的数据太多, 缓冲区溢出 */
#define	USB_INT_USB_READY	0x18			/* USB设备已经被初始化（已分配USB地址） */
#define	USB_INT_DISK_READ	0x1D			/* USB存储器读数据块, 请求数据读出 */
#define	USB_INT_DISK_WRITE	0x1E			/* USB存储器写数据块, 请求数据写入 */
#define	USB_INT_DISK_ERR	0x1F			/* USB存储器操作失败 */
#endif

/* ********************************************************************************************************************* */
/* 常用USB定义 */

/* USB的包标识PID, 主机方式可能用到 */
#ifndef	DEF_USB_PID_SETUP
#define	DEF_USB_PID_NULL	0x00			/* 保留PID, 未定义 */
#define	DEF_USB_PID_SOF		0x05
#define	DEF_USB_PID_SETUP	0x0D
#define	DEF_USB_PID_IN		0x09
#define	DEF_USB_PID_OUT		0x01
#define	DEF_USB_PID_ACK		0x02
#define	DEF_USB_PID_NAK		0x0A
#define	DEF_USB_PID_STALL	0x0E
#define	DEF_USB_PID_DATA0	0x03
#define	DEF_USB_PID_DATA1	0x0B
#define	DEF_USB_PID_PRE		0x0C
#endif

/* USB请求类型, 外置固件模式可能用到 */
#ifndef	DEF_USB_REQ_TYPE
#define	DEF_USB_REQ_READ	0x80			/* 控制读操作 */
#define	DEF_USB_REQ_WRITE	0x00			/* 控制写操作 */
#define	DEF_USB_REQ_TYPE	0x60			/* 控制请求类型 */
#define	DEF_USB_REQ_STAND	0x00			/* 标准请求 */
#define	DEF_USB_REQ_CLASS	0x20			/* 设备类请求 */
#define	DEF_USB_REQ_VENDOR	0x40			/* 厂商请求 */
#define	DEF_USB_REQ_RESERVE	0x60			/* 保留请求 */
#endif

/* USB标准设备请求, RequestType的位6位5=00(Standard), 外置固件模式可能用到 */
#ifndef	DEF_USB_GET_DESCR
#define	DEF_USB_CLR_FEATURE	0x01
#define	DEF_USB_SET_FEATURE	0x03
#define	DEF_USB_GET_STATUS	0x00
#define	DEF_USB_SET_ADDRESS	0x05
#define	DEF_USB_GET_DESCR	0x06
#define	DEF_USB_SET_DESCR	0x07
#define	DEF_USB_GET_CONFIG	0x08
#define	DEF_USB_SET_CONFIG	0x09
#define	DEF_USB_GET_INTERF	0x0A
#define	DEF_USB_SET_INTERF	0x0B
#define	DEF_USB_SYNC_FRAME	0x0C
#endif

/* ********************************************************************************************************************* */

/* FILE: CH375HF.H */

typedef unsigned char                BOOL1;

#ifndef UINT8
typedef unsigned char                UINT8;
#endif
#ifndef UINT16
typedef unsigned short               UINT16;
#endif
#ifndef UINT32
typedef unsigned long                UINT32;
#endif
#ifndef PUINT8
typedef unsigned char               *PUINT8;
#endif
#ifndef PUINT16
typedef unsigned short              *PUINT16;
#endif
#ifndef PUINT32
typedef unsigned long               *PUINT32;
#endif
#ifndef UINT8V
typedef unsigned char volatile       UINT8V;
#endif

/* 错误码 */
#define ERR_SUCCESS				0x00	/* 操作成功 */
#define ERR_CH375_ERROR			0x81	/* CH375硬件错误,可能需要复位CH375 */
#define ERR_DISK_DISCON			0x82	/* 磁盘尚未连接,可能磁盘已经断开 */
#define ERR_STATUS_ERR			0x83	/* 磁盘状态错误,可能正在连接或者断开磁盘 */
#define ERR_MBR_ERROR			0x91	/* 磁盘的主引导记录无效,可能磁盘尚未分区或者尚未格式化 */
#define ERR_TYPE_ERROR			0x92	/* 磁盘分区类型不支持,只支持FAT12/FAT16/BigDOS/FAT32,需要由磁盘管理工具重新分区 */
#define ERR_BPB_ERROR			0xA1	/* 磁盘尚未格式化,或者参数错误,需要由WINDOWS采用默认参数重新格式化 */
#define ERR_TOO_LARGE			0xA2	/* 磁盘非正常格式化并且容量大于4GB,或者容量大于250GB,需要由WINDOWS采用默认参数重新格式化 */
#define ERR_FAT_ERROR			0xA3	/* 磁盘的文件系统不支持,只支持FAT12/FAT16/FAT32,需要由WINDOWS采用默认参数重新格式化 */
#define ERR_DISK_FULL			0xB1	/* 磁盘文件太满,剩余空间太少或者已经没有,需要磁盘整理 */
#define ERR_FDT_OVER			0xB2	/* 目录内文件太多,没有空闲的目录项,FAT12/FAT16根目录下的文件数应该少于500个,需要磁盘整理 */
#define ERR_MISS_DIR			0xB3	/* 指定路径的某个子目录没有找到,可能是目录名称错误 */
#define ERR_FILE_CLOSE			0xB4	/* 文件已经关闭,如果需要使用,应该重新打开文件 */
#define ERR_OPEN_DIR			0x41	/* 指定路径的目录被打开 */
#define ERR_MISS_FILE			0x42	/* 指定路径的文件没有找到,可能是文件名称错误 */
#define ERR_FOUND_NAME			0x43	/* 搜索到与通配符相匹配的文件名,文件名及其完整路径在命令缓冲区中,如果需要使用,应该打开该文件 */
/* 代码2XH-3XH用于USB主机方式的通讯失败代码,由CH375返回 */
/* 代码1XH用于USB主机方式的操作状态代码,由CH375返回 */
#define	ERR_USB_CONNECT			0x15	/* 检测到USB设备连接事件,磁盘已经连接 */
#define	ERR_USB_DISCON			0x16	/* 检测到USB设备断开事件,磁盘已经断开 */
#define	ERR_USB_DISK_ERR		0x1F	/* USB存储器操作失败,在初始化时可能是USB存储器不支持,在读写操作中可能是磁盘损坏或者已经断开 */

/* 磁盘及文件状态 */
#define DISK_UNKNOWN			0x00	/* 尚未初始化,未知状态 */
#define DISK_DISCONNECT			0x01	/* 磁盘没有连接或者已经断开 */
#define DISK_CONNECT			0x02	/* 磁盘已经连接,但是尚未初始化或者无法识别该磁盘 */
#define DISK_MOUNTED			0x03	/* 磁盘已经初始化成功,但是尚未分析文件系统或者文件系统不支持 */
#define DISK_READY				0x10	/* 已经分析磁盘的文件系统并且能够支持 */
#define DISK_OPEN_ROOT			0x12	/* 已经打开根目录,扇区模式,只能以扇区为单位读写目录的内容,使用后必须关闭,注意FAT12/FAT16根目录是固定长度 */
#define DISK_OPEN_DIR			0x13	/* 已经打开子目录,扇区模式,只能以扇区为单位读写目录的内容 */
#define DISK_OPEN_FILE			0x14	/* 已经打开文件,扇区模式,可以以扇区为单位进行数据读写 */
#define DISK_OPEN_FILE_B		0x15	/* 已经打开文件,字节模式,可以以字节为单位进行数据读写 */

/* FAT类型标志 */
#define DISK_FS_UNKNOWN			0		/* 未知的文件系统 */
#define DISK_FAT12				1		/* FAT12文件系统 */
#define DISK_FAT16				2		/* FAT16文件系统 */
#define DISK_FAT32				3		/* FAT32文件系统 */

/* 文件属性 */
#define ATTR_READ_ONLY			0x01	/* 文件为只读属性 */
#define ATTR_HIDDEN				0x02	/* 文件为隐含属性 */
#define ATTR_SYSTEM				0x04	/* 文件为系统属性 */
#define ATTR_VOLUME_ID			0x08	/* 卷标 */
#define ATTR_DIRECTORY			0x10	/* 子目录 */
#define ATTR_ARCHIVE			0x20	/* 文件为存档属性 */
#define ATTR_LONG_NAME			( ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_VOLUME_ID )
/* 文件属性 UINT8 */
/* bit0 bit1 bit2 bit3 bit4 bit5 bit6 bit7 */
/*  只   隐   系   卷   目   存   未定义   */
/*  读   藏   统   标   录   档            */
/* 文件时间 UINT16 */
/* Time = (Hour<<11) + (Minute<<5) + (Second>>1) */
#define MAKE_FILE_TIME( h, m, s )	( (h<<11) + (m<<5) + (s>>1) )	/* 生成指定时分秒的文件时间数据 */
/* 文件日期 UINT16 */
/* Date = ((Year-1980)<<9) + (Month<<5) + Day */
#define MAKE_FILE_DATE( y, m, d )	( ((y-1980)<<9) + (m<<5) + d )	/* 生成指定年月日的文件日期数据 */

/* 文件名 */
#define PATH_WILDCARD_CHAR		0x2A	/* 路径名的通配符 '*' */
#define PATH_SEPAR_CHAR1		0x5C	/* 路径名的分隔符 '\' */
#define PATH_SEPAR_CHAR2		0x2F	/* 路径名的分隔符 '/' */
#ifndef MAX_PATH_LEN
#define MAX_PATH_LEN			30		/* 最大路径长度,含所有斜杠分隔符和小数点间隔符以及路径结束符00H */
#endif
#ifndef MAX_BYTE_IO
#define MAX_BYTE_IO		( MAX_PATH_LEN - 1 )	/* 以字节为单位单次读写文件时的最大长度,超过该长度可以分多次读写 */
#endif

/* 外部命令参数 */
typedef union _CMD_PARAM {
	struct {
		UINT8	mBuffer[ MAX_PATH_LEN ];
	} Other;
	struct {
		UINT32	mReserved;
		UINT32	mTotalSector;			/* 返回: 当前逻辑盘的总扇区数 */
		UINT32	mFreeSector;			/* 返回: 当前逻辑盘的剩余扇区数 */
		UINT8	mDiskFat;				/* 返回: 当前逻辑盘的FAT类型 */
	} Query;							/* CMD_DiskQuery, 查询磁盘信息 */
	struct {
		UINT8	mPathName[ MAX_PATH_LEN ];	/* 输入参数: 路径: [盘符,冒号,斜杠,目录名或者文件名及扩展名...,结束符00H], 其中盘符和冒号可以省略, 例如"C:\DIR1.EXT\DIR2\FILENAME.EXT",00H */
	} Open;								/* CMD_FileOpen, 打开文件 */
	struct {
		UINT8	mPathName[ MAX_PATH_LEN ];	/* 输入参数: 路径: [盘符,冒号,斜杠,目录名或者文件名及扩展名(含通配符*)...,枚举序号], 其中盘符和冒号可以省略, 例如"C:\DIR1.EXT\DIR2\FILE*",00H */
	} Enumer;							/* CMD_FileEnumer, 枚举文件,返回文件名 */
	struct {
		UINT8	mUpdateLen;				/* 输入参数: 是否允许更新长度: 0禁止,1允许 */
	} Close;							/* CMD_FileClose, 关闭当前文件 */
	struct {
		UINT8	mPathName[ MAX_PATH_LEN ];	/* 输入参数: 路径: [盘符,冒号,斜杠,目录名或者文件名及扩展名...,结束符00H], 其中盘符和冒号可以省略, 例如"C:\DIR1.EXT\DIR2\FILENAME.EXT",00H */
	} Create;							/* CMD_FileCreate, 新建文件并打开,如果文件已经存在则先删除后再新建 */
	struct {
		UINT8	mPathName[ MAX_PATH_LEN ];	/* 输入参数: 路径: [盘符,冒号,斜杠,目录名或者文件名及扩展名...,结束符00H], 其中盘符和冒号可以省略, 例如"C:\DIR1.EXT\DIR2\FILENAME.EXT",00H */
	} Erase;							/* CMD_FileErase, 删除文件并关闭 */
	struct {
		UINT32	mFileSize;				/* 输入参数: 新的文件长度,为0FFFFFFFFH则不修改, 返回: 原长度 */
		UINT16	mFileDate;				/* 输入参数: 新的文件日期,为0FFFFH则不修改, 返回: 原日期 */
		UINT16	mFileTime;				/* 输入参数: 新的文件时间,为0FFFFH则不修改, 返回: 原时间 */
		UINT8	mFileAttr;				/* 输入参数: 新的文件属性,为0FFH则不修改, 返回: 原属性 */
	} Modify;							/* CMD_FileQuery, 查询当前文件的信息; CMD_FileModify, 查询或者修改当前文件的信息 */
	struct {
		UINT32	mSectorOffset;			/* 输入参数: 扇区偏移,0则移动到文件头,0FFFFFFFFH则移动到文件尾, 返回: 当前文件指针对应的绝对线性扇区号, 0FFFFFFFFH则已到文件尾 */
	} Locate;							/* CMD_FileLocate, 移动当前文件指针 */
	struct {
		UINT8	mSectorCount;			/* 输入参数: 读取扇区数, 返回: 实际读取扇区数 */
	} Read;								/* CMD_FileRead, 从当前文件读取数据 */
	struct {
		UINT8	mSectorCount;			/* 输入参数: 写入扇区数, 返回: 实际写入扇区数 */
	} Write;							/* CMD_FileWrite, 向当前文件写入数据 */
	struct {
		UINT8	mSectorCount;			/* 输入参数: 读取扇区数, 返回: 实际读取扇区数 */
		UINT8	mReserved[7];
		PUINT8	mDataBuffer;			/* 输入参数: 缓冲区起始地址, 返回: 缓冲区当前地址 */
	} ReadX;							/* CMD_FileReadX, 从当前文件读取数据到指定缓冲区 */
	struct {
		UINT8	mSectorCount;			/* 输入参数: 写入扇区数, 返回: 实际写入扇区数 */
		UINT8	mReserved[7];
		PUINT8	mDataBuffer;			/* 输入参数: 缓冲区起始地址, 返回: 缓冲区当前地址 */
	} WriteX;							/* CMD_FileWriteX, 向当前文件写入指定缓冲区的数据 */
	struct {
		UINT32	mDiskSizeSec;			/* 返回: 整个物理磁盘的总扇区数 */
	} DiskSize;							/* CMD_DiskSize, 查询磁盘容量 */
	struct {
		UINT32	mByteOffset;			/* 输入参数: 以字节为单位的偏移量, 以字节为单位的文件指针, 返回: 当前文件指针对应的绝对线性扇区号, 0FFFFFFFFH则已到文件尾 */
	} ByteLocate;						/* CMD_ByteLocate, 以字节为单位移动当前文件指针 */
	struct {
		UINT8	mByteCount;				/* 输入参数: 准备读取的字节数,不得大于MAX_BYTE_IO, 返回: 实际读出的字节数 */
		UINT8	mByteBuffer[ MAX_BYTE_IO ];	/* 返回: 读出的数据块 */
	} ByteRead;							/* CMD_ByteRead, 以字节为单位从当前文件读取数据块 */
	struct {
		UINT8	mByteCount;				/* 输入参数: 准备写入的字节数,不得大于MAX_BYTE_IO, 返回: 实际写入的字节数 */
		UINT8	mByteBuffer[ MAX_BYTE_IO ];	/* 输入参数: 准备写入的数据块 */
	} ByteWrite;						/* CMD_ByteWrite, 以字节为单位向当前文件写入数据块 */
	struct {
		UINT8	mSaveVariable;			/* 输入参数: 为0则恢复单个U盘的变量,为0x80则恢复多个U盘的变量,其它值则备份/保存变量 */
		UINT8	mReserved[3];
		PUINT8	mBuffer;				/* 输入参数: 指向子程序库的变量的备份缓冲区,长度不小于80个字节 */
	} SaveVariable;						/* CMD_SaveVariable, 备份/保存/恢复子程序库的变量 */
	union {
		struct {
			UINT32	mCBW_Sig;
			UINT32	mCBW_Tag;
			UINT8	mCBW_DataLen;		/* 输入: 数据传输长度,有效值是0到255 */
			UINT8	mCBW_DataLen1;
			UINT8	mCBW_DataLen2;
			UINT8	mCBW_DataLen3;
			UINT8	mCBW_Flag;			/* 输入: 传输方向等标志 */
			UINT8	mCBW_LUN;
			UINT8	mCBW_CB_Len;		/* 输入: 命令块的长度,有效值是1到16 */
			UINT8	mCBW_CB_Buf[6];		/* 输入: 命令块,该缓冲区最多为16个字节 */
		} mCBW;							/* BulkOnly协议的命令块, 输入CBW结构 */
		struct {
			UINT32	mCSW_Sig;
			UINT32	mCSW_Tag;
			UINT32	mCSW_Residue;		/* 返回: 剩余数据长度 */
			UINT8	mCSW_Status;		/* 返回: 命令执行结果状态 */
			UINT8	mReserved;
		} mCSW;							/* BulkOnly协议的命令状态块, 输出CSW结构 */
	} BOC;								/* CMD_BulkOnlyCmd, 执行基于BulkOnly协议的命令, 如果有数据传输那么数据在pDISK_BASE_BUF中 */
} CMD_PARAM;

typedef CMD_PARAM CMD_PARAM_I;
typedef CMD_PARAM *P_CMD_PARAM;

/* FILE: CH375HF?.C */

#define EN_DISK_WRITE			1
#define EN_DISK_FAT12			1
#define EN_DISK_FAT32			1
#define EN_BYTE_ACCESS			1
#define EN_SAVE_VARIABLE		1
#define EXT_BLK_INTERFACE		1
#define EN_SEC_SIZE_AUTO		1

#define LIB_CFG_DISK_IO			1		/* 默认情况下,磁盘读写的数据的复制方式 */
#ifndef LIB_CFG_FILE_IO
#define LIB_CFG_FILE_IO			1		/* 默认情况下,文件读写的数据的复制方式为"内部复制" */
#endif
#ifndef LIB_CFG_UPD_SIZE
#define LIB_CFG_UPD_SIZE		0		/* 默认情况下,在添加数据后文件长度的更新方式为"不更新" */
#endif
#ifndef LIB_CFG_NO_DLY
#define LIB_CFG_NO_DLY			0		/* 默认情况下,在写操作结束后的延时方式为"写后延时" */
#endif
#ifndef LIB_CFG_INT_EN
#define LIB_CFG_INT_EN			0		/* 默认情况下,CH375的INT#引脚连接方式为"查询方式" */
#endif
#ifndef DISK_BASE_BUF_LEN
#define DISK_BASE_BUF_LEN		512		/* 默认的磁盘数据缓冲区大小为512字节,建议选择为2048甚至4096以支持某些大扇区的U盘,为0则禁止在.H文件中定义缓冲区并由应用程序在pDISK_BASE_BUF中指定 */
#endif

#define LIB_CFG_VALUE		( ( LIB_CFG_INT_EN << 7 ) | ( LIB_CFG_NO_DLY << 5 ) | ( LIB_CFG_UPD_SIZE << 4 ) | ( LIB_CFG_FILE_IO << 2 ) | LIB_CFG_DISK_IO )	/* CH375程序库配置值 */

/* 子程序库中提供的变量 */
extern UINT8V	CH375IntStatus;			/* CH375操作的中断状态 */
extern UINT8V	CH375DiskStatus;		/* 磁盘及文件状态 */
extern UINT8	CH375LibConfig;			/* CH375程序库配置,下行说明 */
/* 位7: CH375的INT#引脚连接方式: 0查询方式,1中断方式 */
/* 位6: 该位为1且CH375Version2为1则芯片为CH375B */
/* 位5: 在写操作结束后是否延时: 0写后延时,1不延时 */
/* 位4: 在添加数据后是否自动更新文件长度: 0不更新,1自动更新 */
/* 位3位2: 针对文件读写的多扇区数据的复制方式: 00外部子程序, 01,10,11内部复制 */
/* 位1位0: 针对磁盘读写的单扇区数据的复制方式: 总是内部复制 */

/* 如果CH375的INT#引脚连接到单片机的中断输入引脚并且准备使用中断方式,那么LIB_CFG_INT_EN定义为1,否则定义为0由单片机查询INT#引脚 */
/* 在CH375子程序读写文件数据时,CH375的程序库提供两种优化速度的方式,在LIB_CFG_FILE_IO中定义:
   方式0:"外部子程序", 只适用于文件读写时的数据复制(只用于CH375FileReadX和CH375FileWriteX两个子程序),
                       是指由子程序xWriteToExtBuf和xReadFromExtBuf进行数据复制,这两个子程序是在应用程序中定义的,由CH375的程序库调用,
   方式1:"内部复制", 程序内置的常规数据复制方式
   对于文件数据读写,也就是应用程序调用CH375FileReadX和CH375FileWriteX子程序时:
     在方式0下,由应用程序在xWriteToExtBuf和xReadFromExtBuf子程序中自行管理缓冲区,设定缓冲区初值等,
     在方式1下,应用程序每次调用CH375FileReadX和CH375FileWriteX时,CH375的程序库都会从指定缓冲区的起始地址开始读写数据,
     例如: 某文件长度为1K(占用2个扇区), 如果调用CH375FileReadX时读1K(指定mCmdParam.Read.mSectorCount为2), 那么1K数据全读到指定缓冲区中,
           如果缓冲区较小只有0.5K, 那么分两次读取, 第一次调用CH375FileReadX时读0.5K, 处理完这0.5K数据后再调用CH375FileReadX读下一个0.5K并处理
*/
extern UINT8	CH375vDiskFat;			/* 逻辑盘的FAT标志:1=FAT12,2=FAT16,3=FAT32 */
extern UINT8	CH375vSecPerClus;		/* 逻辑盘的每簇扇区数 */
extern UINT32	CH375vStartCluster;		/* 当前文件或者目录的起始簇号 */
extern UINT32	CH375vFileSize;			/* 当前文件的长度 */
extern UINT32	CH375vCurrentOffset;	/* 当前文件指针,当前读写位置的字节偏移 */

/* FAT数据区中文件目录信息 */
typedef struct _FAT_DIR_INFO {
	UINT8	DIR_Name[11];				/* 00H,文件名,共11字节,不足处填空格 */
	UINT8	DIR_Attr;					/* 0BH,文件属性,参考前面的说明 */
	UINT8	DIR_NTRes;					/* 0CH */
	UINT8	DIR_CrtTimeTenth;			/* 0DH,文件创建的时间,以0.1秒单位计数 */
	UINT16	DIR_CrtTime;				/* 0EH,文件创建的时间 */
	UINT16	DIR_CrtDate;				/* 10H,文件创建的日期 */
	UINT16	DIR_LstAccDate;				/* 12H,最近一次存取操作的日期 */
	UINT16	DIR_FstClusHI;				/* 14H */
	UINT16	DIR_WrtTime;				/* 16H,文件修改时间,参考前面的宏MAKE_FILE_TIME */
	UINT16	DIR_WrtDate;				/* 18H,文件修改日期,参考前面的宏MAKE_FILE_DATA */
	UINT16	DIR_FstClusLO;				/* 1AH */
	UINT32	DIR_FileSize;				/* 1CH,文件长度 */
} FAT_DIR_INFO;							/* 20H */
typedef FAT_DIR_INFO *P_FAT_DIR_INFO;

extern BOOL1	CH375Version2;			/* 芯片版本:0-CH375,1-CH375A/B */
extern UINT32	CH375vDataStart;		/* 逻辑盘的数据区域的起始LBA */
extern UINT32	CH375vFdtLba;			/* 当前FDT所在的LBA地址 */
extern UINT16	CH375vFdtOffset;		/* 当前FDT在扇区内的偏移地址 */
extern UINT32	CH375vDiskRoot;			/* 对于FAT16盘为根目录占用扇区数,对于FAT32盘为根目录起始簇号 */
#ifdef EN_SEC_SIZE_AUTO
extern UINT16	CH375vSectorSize;		/* 磁盘的扇区大小 */
#else
#define	CH375vSectorSize	512			/* 磁盘的扇区大小 */
#endif
extern PUINT8	pDISK_BASE_BUF;			/* 指向外部RAM的磁盘数据缓冲区,缓冲区长度不小于CH375vSectorSize,由应用程序初始化 */

extern UINT8	CH375ReadBlock( void );		/* 从磁盘读取多个扇区的数据到外部接口交换区 */
#ifdef EN_DISK_WRITE
extern UINT8	CH375WriteBlock( void );	/* 将外部接口交换区的多个扇区的数据块写入磁盘 */
#endif

/* 子程序库中提供的子程序 */
/* 下述子程序中, 文件操作子程序CH375File*和磁盘查询子程序CH375DiskQuery都可能会用到磁盘数据缓冲区pDISK_BASE_BUF,
   并且有可能在pDISK_BASE_BUF中保存了磁盘信息, 所以必须保证pDISK_BASE_BUF不被用于其它用途,
   如果RAM较少, 要将pDISK_BASE_BUF临时用于其它用途, 那么在临时用完后必须调用CH375DirtyBuffer清除磁盘缓冲区 */
extern UINT8	CH375GetVer( void );		/* 获取当前子程序库的版本号 */
extern void		CH375Reset( void );			/* 复位CH375 */
extern UINT8	CH375Init( void );			/* 初始化CH375 */
extern UINT8	CH375DiskConnect( void );	/* 检查磁盘是否连接 */
extern UINT8	CH375DiskReady( void );		/* 查询磁盘是否准备好 */
extern void		CH375DirtyBuffer( void );	/* 清除磁盘缓冲区 */
extern UINT8	CH375FileOpen( void );		/* 打开文件或者枚举文件 */
extern UINT8	CH375FileClose( void );		/* 关闭当前文件 */
#ifdef EN_DISK_WRITE
extern UINT8	CH375FileErase( void );		/* 删除文件并关闭 */
extern UINT8	CH375FileCreate( void );	/* 新建文件并打开,如果文件已经存在则先删除后再新建 */
#endif
extern UINT8	CH375FileModify( void );	/* 查询或者修改当前文件的信息 */
extern UINT8	CH375FileLocate( void );	/* 移动当前文件指针 */
extern UINT8	CH375FileReadX( void );		/* 从当前文件读取数据到指定缓冲区 */
#ifdef EN_DISK_WRITE
extern UINT8	CH375FileWriteX( void );	/* 向当前文件写入指定缓冲区的数据 */
#endif
#ifdef EN_BYTE_ACCESS
extern UINT8	CH375ByteLocate( void );	/* 以字节为单位移动当前文件指针 */
extern UINT8	CH375ByteRead( void );		/* 以字节为单位从当前位置读取数据块 */
#ifdef EN_DISK_WRITE
extern UINT8	CH375ByteWrite( void );		/* 以字节为单位向当前位置写入数据块 */
#endif
#endif
extern UINT8	CH375DiskSize( void );		/* 查询磁盘容量 */
extern UINT8	CH375DiskQuery( void );		/* 查询磁盘信息 */
#ifdef EN_SAVE_VARIABLE
extern void		CH375SaveVariable( void );	/* 备份/保存/恢复子程序库的变量,用于子程序库在多个CH375芯片之间进行切换 */
#endif
extern UINT8	CH375BulkOnlyCmd( void );	/* 执行基于BulkOnly协议的命令 */
extern UINT8	CH375sDiskReady( void );	/* 查询磁盘是否准备好,支持CH375S */

/* 该头文件可以为CH375子程序库分配必要的I/O及内存资源,并产生必要的与硬件有关的目标代码,
   如果该文件是被工程项目的多个源程序包含作为头文件,那么应该只允许一个头文件分配资源和产生代码,
   除此之外的头文件应该被事先定义CH375HF_NO_CODE,从而禁止该头文件产生重复的目标代码,例如:
#define		CH375HF_NO_CODE		1
#include	CH375HF?.H
*/
#ifdef CH375HF_NO_CODE

extern void xWriteCH375Cmd( UINT8 mCmd );	/* 外部定义的被CH375程序库调用的子程序,向CH375写命令,最小周期为4uS,否则之前之后各延时2uS */
extern void xWriteCH375Data( UINT8 mData );	/* 外部定义的被CH375程序库调用的子程序,向CH375写数据,最小周期为1.5uS,否则之后延时1.5uS */
extern UINT8 xReadCH375Data( void );		/* 外部定义的被CH375程序库调用的子程序,从CH375读数据,最小周期为1.5uS,否则之前延时1.5uS */
extern CMD_PARAM_I mCmdParam;				/* 命令参数 */
#if DISK_BASE_BUF_LEN
extern UINT8  DISK_BASE_BUF[ DISK_BASE_BUF_LEN ];	/* 外部RAM的磁盘数据缓冲区,缓冲区长度为一个扇区的长度 */
#endif
#ifdef FILE_DATA_BUF_LEN
extern UINT8  FILE_DATA_BUF[ FILE_DATA_BUF_LEN ];	/* 外部RAM的文件数据缓冲区,缓冲区长度不小于一次读写的数据长度 */
extern UINT8	CH375FileRead( void );		/* 从当前文件读取数据 */
#ifdef EN_DISK_WRITE
extern UINT8	CH375FileWrite( void );		/* 向当前文件写入数据 */
#endif
#endif
#ifndef NO_DEFAULT_CH375_F_ENUM
extern UINT8	CH375FileEnumer( void );	/* 枚举文件 */
#endif
#ifndef NO_DEFAULT_CH375_F_QUERY
extern UINT8	CH375FileQuery( void );		/* 查询当前文件的信息 */
#endif
extern void xQueryInterrupt( void );		/* 外部定义的被CH375程序库调用的子程序,查询CH375中断并更新中断状态 */
extern void xDelay100uS( void );			/* 外部定义的被CH375程序库调用的子程序,延时100uS */
#ifdef EN_DISK_WRITE
extern void xDelayAfterWrite( void );		/* 外部定义的被CH375程序库调用的子程序,写操作后延时 */
#endif
extern void xFileNameEnumer( void );		/* 外部定义的被CH375程序库调用的子程序,文件名枚举回调子程序 */
extern UINT8 CH375LibInit( void );			/* 初始化CH375程序库和CH375芯片,操作成功返回0 */

#else

void xWriteCH375Cmd( UINT8 mCmd );		/* 外部定义的被CH375程序库调用的子程序,向CH375写命令,最小周期为4uS,否则之前之后各延时2uS */
void xWriteCH375Data( UINT8 mData );	/* 外部定义的被CH375程序库调用的子程序,向CH375写数据,最小周期为1.5uS,否则之后延时1.5uS */
UINT8 xReadCH375Data( void );			/* 外部定义的被CH375程序库调用的子程序,从CH375读数据,最小周期为1.5uS,否则之前延时1.5uS */

CMD_PARAM_I mCmdParam;					/* 命令参数 */

//#pragma SAVE
//#pragma PACK(4)
#if DISK_BASE_BUF_LEN
#ifdef DISK_BASE_BUF_ADDR
__align(8)	UINT8  DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] _at_ DISK_BASE_BUF_ADDR;	/* 外部RAM的磁盘数据缓冲区,缓冲区长度为一个扇区的长度,起始地址必须为8字节边界地址 */
#else
__align(8)	UINT8  DISK_BASE_BUF[ DISK_BASE_BUF_LEN ];	/* 外部RAM的磁盘数据缓冲区,缓冲区长度为一个扇区的长度,起始地址必须为8字节边界地址 */
#endif
#endif
//#pragma RESTORE
#ifdef FILE_DATA_BUF_LEN
//#pragma SAVE
//#pragma PACK(4)
#ifdef FILE_DATA_BUF_ADDR
__align(8)	UINT8  FILE_DATA_BUF[ FILE_DATA_BUF_LEN ] _at_ FILE_DATA_BUF_ADDR;	/* 外部RAM的文件数据缓冲区,缓冲区长度不小于一次读写的数据长度,起始地址建议为8字节边界地址 */
#else
__align(8)	UINT8  FILE_DATA_BUF[ FILE_DATA_BUF_LEN ];	/* 外部RAM的文件数据缓冲区,缓冲区长度不小于一次读写的数据长度,起始地址建议为8字节边界地址 */
#endif
//#pragma RESTORE
UINT8	CH375FileRead( void )		/* 从当前文件读取数据 */
{
	mCmdParam.ReadX.mDataBuffer = &FILE_DATA_BUF[0];  /* 指向文件数据缓冲区 */
	return( CH375FileReadX( ) );
}
#ifdef EN_DISK_WRITE
UINT8	CH375FileWrite( void )		/* 向当前文件写入数据 */
{
	mCmdParam.WriteX.mDataBuffer = &FILE_DATA_BUF[0];  /* 指向文件数据缓冲区 */
	return( CH375FileWriteX( ) );
}
#endif
#endif

/* 以下程序可以根据需要修改 */

#ifndef NO_DEFAULT_CH375_F_ENUM			/* 在应用程序中定义NO_DEFAULT_CH375_F_ENUM可以禁止默认的枚举文件程序,然后用自行编写的程序代替它 */
UINT8	CH375FileEnumer( void )			/* 枚举文件 */
{
	UINT8	status;
	status = CH375FileOpen( );
	if ( status == ERR_FOUND_NAME ) status = ERR_SUCCESS;  /* 操作成功 */
	return( status );
}
#endif

#ifndef NO_DEFAULT_CH375_F_QUERY		/* 在应用程序中定义NO_DEFAULT_CH375_F_QUERY可以禁止默认的查询当前文件的信息程序,然后用自行编写的程序代替它 */
UINT8	CH375FileQuery( void )			/* 查询当前文件的信息 */
{
	PUINT8	buf;
	UINT8	count;
	buf = (PUINT8)( & mCmdParam.Modify.mFileSize );
	for ( count = sizeof( mCmdParam.Modify ); count != 0; count -- ) {
		*buf = 0xFF;  /* 输入参数全部无效,仅查询不修改 */
		buf ++;
	}
	return( CH375FileModify( ) );
}
#endif

#ifndef NO_DEFAULT_CH375_INT			/* 在应用程序中定义NO_DEFAULT_CH375_INT可以禁止默认的中断处理程序,然后用自行编写的程序代替它 */
#if LIB_CFG_INT_EN == 0					/* CH375的INT#引脚连接方式为"查询方式" */
void xQueryInterrupt( void )			/* 查询CH375中断并更新中断状态 */
{
#ifdef CH375_INT_WIRE  /* 查询中断引脚 */
	while ( CH375_INT_WIRE );  /* 如果CH375的中断引脚输出高电平则等待 */
#else  /* 对于CH375B芯片还可以查询命令端口的位7 */
	while ( xReadCH375Cmd( ) & 0x80 );  /* 查询CH375B的命令端口的位7为1说明中断引脚输出高电平则等待 */
#endif
	xWriteCH375Cmd( CMD_GET_STATUS );  /* 获取当前中断状态,发出命令后至少延时2uS */
	CH375IntStatus = xReadCH375Data( );  /* 获取中断状态 */
	if ( CH375IntStatus == USB_INT_DISCONNECT ) CH375DiskStatus = DISK_DISCONNECT;  /* 检测到USB设备断开事件 */
	else if ( CH375IntStatus == USB_INT_CONNECT ) CH375DiskStatus = DISK_CONNECT;  /* 检测到USB设备连接事件 */
}
#else									/* LIB_CFG_INT_EN != 0, CH375的INT#引脚连接方式为"中断方式" */
void xQueryInterrupt( void )			/* 查询中断状态,等待硬件中断 */
{
	while ( CH375IntStatus == 0 );		/* 子程序库调用该子程序之前CH375IntStatus=0,硬件中断后,由中断服务程序置为非0的实际中断状态后返回 */
}
void CH375Interrupt( void )	__irq	/* CH375中断服务程序,由CH375的INT#的低电平或者下降沿触发单片机中断 */
{
	xWriteCH375Cmd( CMD_GET_STATUS );  /* 获取中断状态并取消中断请求 */
	CH375IntStatus = xReadCH375Data( );  /* 获取中断状态 */
	if ( CH375IntStatus == USB_INT_DISCONNECT ) CH375DiskStatus = DISK_DISCONNECT;  /* 检测到USB设备断开事件 */
	else if ( CH375IntStatus == USB_INT_CONNECT ) CH375DiskStatus = DISK_CONNECT;  /* 检测到USB设备连接事件 */
#ifdef CLEAR_INT_MARK
	CLEAR_INT_MARK( );  /* 某些单片机需要由软件清除中断标志 */
#endif
}
#endif
#endif

#ifndef NO_DEFAULT_DELAY_100US			/* 在应用程序中定义NO_DEFAULT_DELAY_100US可以禁止默认的延时100uS子程序,然后用自行编写的程序代替它 */
void xDelay100uS( void )				/* 延时100uS */
{
	delay_us(100);
}
#endif

#ifdef EN_DISK_WRITE
#ifndef NO_DEFAULT_DELAY_WRITE			/* 在应用程序中定义NO_DEFAULT_DELAY_WRITE可以禁止默认的写操作后延时程序,然后用自行编写的程序代替它 */
void xDelayAfterWrite( void )			/* 写操作后延时 */
{
	delay_us(200);  /* 延时200uS左右 */
}
#endif
#endif

#ifndef NO_DEFAULT_FILE_ENUMER			/* 在应用程序中定义NO_DEFAULT_FILE_ENUMER可以禁止默认的文件名枚举回调程序,然后用自行编写的程序代替它 */
void xFileNameEnumer( void )			/* 文件名枚举回调子程序 */
{
/* 如果指定枚举序号CH375vFileSize为0xFFFFFFFF后调用FileOpen，那么每搜索到一个文件FileOpen都会调用本回调程序，
   回调程序xFileNameEnumer返回后，FileOpen递减CH375vFileSize并继续枚举直到搜索不到文件或者目录。建议做法是，
   在调用FileOpen之前定义一个全局变量为0，当FileOpen回调本程序后，本程序由CH375vFdtOffset得到结构FAT_DIR_INFO，
   分析结构中的DIR_Attr以及DIR_Name判断是否为所需文件名或者目录名，记录相关信息，并将全局变量计数增量，
   当FileOpen返回后，判断返回值如果是ERR_MISS_FILE或ERR_FOUND_NAME都视为操作成功，全局变量为搜索到的有效文件数。
   如果在本回调程序xFileNameEnumer中将CH375vFileSize置为1，那么可以通知FileOpen提前结束搜索。以下是回调程序例子 */
#if 0
#ifdef FILE_DATA_BUF_LEN
	UINT8			i;
	UINT16			FileCount;
	P_FAT_DIR_INFO	pFileDir;
	PUINT8			NameBuf;
	pFileDir = (P_FAT_DIR_INFO)( pDISK_BASE_BUF + CH375vFdtOffset );  /* 当前FDT的起始地址 */
	FileCount = (UINT16)( 0xFFFFFFFF - CH375vFileSize );  /* 当前文件名的枚举序号,CH375vFileSize初值是0xFFFFFFFF,找到文件名后递减 */
	if ( FileCount < FILE_DATA_BUF_LEN / 12 ) {  /* 检查缓冲区是否足够存放,假定每个文件名需占用12个字节存放 */
		NameBuf = & FILE_DATA_BUF[ FileCount * 12 ];  /* 计算保存当前文件名的缓冲区地址 */
		for ( i = 0; i < 11; i ++ ) NameBuf[ i ] = pFileDir -> DIR_Name[ i ];  /* 复制文件名,长度为11个字符,未处理空格 */
		if ( pFileDir -> DIR_Attr & ATTR_DIRECTORY ) NameBuf[ i ] = 1;  /* 判断是目录名 */
		NameBuf[ i ] = 0;  /* 文件名结束符 */
	}
#endif
#endif
}
#endif

#ifdef EXT_BLK_INTERFACE
#if LIB_CFG_FILE_IO == 0				/* 文件读写的数据的复制方式为"外部子程序" */
#ifdef LIB_CFG_FILE_IO_DEFAULT			/* 如果应用程序中定义该值则使用默认"外部子程序",否则应该自行编写程序代替 */
unsigned char *current_buffer;	/* 保存文件数据读写时的缓冲区的当前指针,由应用程序在调用CH375FileReadX和CH375FileWriteX子程序前设置初值 */
void xWriteToExtBuf( UINT8 mLength )	/* 该子程序由CH375的子程序库调用,用于从CH375读取文件数据到外部缓冲区,被CH375FileReadX调用 */
{
/*	if ( (UINT32)current_buffer + mLength >= (UINT32)&FILE_DATA_BUF + sizeof( FILE_DATA_BUF ) ) return;*/  /* 防止缓冲区溢出 */
	if ( mLength ) {
		do {  /* 根据长度读取数据,实际上长度总是CH375_MAX_DATA_LEN,也就是64 */
			*current_buffer = xReadCH375Data( );  /* 读出数据并保存,可以用这种方式将文件数据保存到单片机的各种串行存储器中 */
			current_buffer ++;
		} while ( -- mLength );
	}  /* 复制上述数据的总时间不得超过2mS */
	else {  /* 重试,恢复缓冲区起址,如果将文件数据读写的缓冲区的当前指针放在mCmdParam.ReadX.mDataBuffer中则会被自动恢复,无需下面的两行程序 */
		current_buffer += (UINT32)mCmdParam.ReadX.mDataBuffer;  /* mDataBuffer中为负值 */
		mCmdParam.ReadX.mDataBuffer = 0;  /* 为了支持重试,在调用CH375FileReadX之前也应该清0 */
	}
}
#ifdef EN_DISK_WRITE
void xReadFromExtBuf( UINT8 mLength )	/* 该子程序由CH375的子程序库调用,用于从外部缓冲区读取文件数据到CH375,被CH375FileWriteX调用 */
{
	if ( mLength ) {
		do {  /* 根据长度写入数据,实际上长度总是CH375_MAX_DATA_LEN,也就是64 */
			xWriteCH375Data( *current_buffer );  /* 将数据写入,可以用这种方式从单片机的各种串行存储器中取出文件数据 */
			current_buffer ++;
		} while ( -- mLength );
	}  /* 复制上述数据的总时间不得超过2mS */
	else {  /* 重试,恢复缓冲区起址,如果将文件数据读写的缓冲区的当前指针放在mCmdParam.WriteX.mDataBuffer中则会被自动恢复,无需下面的两行程序 */
		current_buffer += (UINT32)mCmdParam.WriteX.mDataBuffer;  /* mDataBuffer中为负值 */
		mCmdParam.WriteX.mDataBuffer = 0;  /* 为了支持重试,在调用CH375FileWriteX之前也应该清0 */
	}
}
#endif
#endif
#else									/* LIB_CFG_FILE_IO != 0,文件读写的数据的复制方式不是"外部子程序" */
#ifdef FILE_DATA_BUF_LEN
void xWriteToExtBuf( UINT8 mLength )	/* 不会调用该子程序 */
{
	mLength --;  /* 该操作无意义,只是避免出现警告信息 */
}
#ifdef EN_DISK_WRITE
void xReadFromExtBuf( UINT8 mLength )	/* 不会调用该子程序 */
{
	mLength --;  /* 该操作无意义,只是避免出现警告信息 */
}
#endif
#endif
#endif
#endif

UINT8	CH375LibInit( void )  /* 初始化CH375程序库和CH375芯片,操作成功返回0 */
{
	CH375LibConfig = LIB_CFG_VALUE;  /* CH375程序库配置值 */
	if ( CH375GetVer( ) < CH375_LIB_VER ) return( 0xFF );  /* 获取当前子程序库的版本号,版本太低则返回错误 */
#if DISK_BASE_BUF_LEN
	pDISK_BASE_BUF = &DISK_BASE_BUF[0];  /* 指向外部RAM的磁盘数据缓冲区 */
#endif
	return( CH375Init( ) );  /* 初始化CH375 */
}

#endif

#ifdef __cplusplus
}
#endif

#endif
