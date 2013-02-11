#ifndef _CA_H_
#define _CA_H_

#define UART_REG_BASE  0xd4026000
#define PWM2_REG_BASE 0xd401a400
#define GPIO_REG_BASE  0xd4019100

#define LCR_DLAB	(1 << 7)	/* Divisor Latch Access Bit */
#define LCR_SB		(1 << 6)	/* Set Break */
#define LCR_STKYP	(1 << 5)	/* Sticky Parity */
#define LCR_EPS		(1 << 4)	/* Even Parity Select */
#define LCR_PEN		(1 << 3)	/* Parity Enable */
#define LCR_STB		(1 << 2)	/* Stop Bit */
#define LCR_WLS1	(1 << 1)	/* Word Length Select */
#define LCR_WLS0	(1 << 0)	/* Word Length Select */

#define IER_UUE		(1 << 6)	/* UART Unit Enable */
#define IER_RAVIE		(1 << 0)	/* UART Unit Enable */

#define LSR_DR		(1 << 0)	/* Data Ready */
#define LSR_TEMT	(1 << 6)	/* Transmitter Empty */

#define __REG_PXA168(x) (*((volatile u32 *)((x & 0xffffff) | 0xfe000000)))

#define PWM2_APBC __REG_PXA168(0xd4015010)
/* PWM2 in ASPEN */
#define PWM2_CR		__REG_PXA168(PWM2_REG_BASE)
#define PWM2_DCR	__REG_PXA168(PWM2_REG_BASE + 0x4)
#define PWM2_PCR	__REG_PXA168(PWM2_REG_BASE + 0x8)

/* UART3 regs */
#define SerialDATA  __REG_PXA168(UART_REG_BASE)
#define SerialFIFO  __REG_PXA168(UART_REG_BASE + 0x8)
#define SerialLCR   __REG_PXA168(UART_REG_BASE + 0xc)
#define SerialMCR   __REG_PXA168(UART_REG_BASE + 0x10)
#define SerialLSR  __REG_PXA168(UART_REG_BASE + 0x14)
#define SerialIER   __REG_PXA168(UART_REG_BASE + 0x4)
#define SerialMSR   __REG_PXA168(UART_REG_BASE + 0x18)

#define UART3TRX __REG_PXA168(0xd401e188)

/* GPIO regs */
#define GPIO_RST_PDR __REG_PXA168(GPIO_REG_BASE + 0xc)
#define GPIO_RST_PSR __REG_PXA168(GPIO_REG_BASE + 0x18)


#define MINOR_NUMBER 10

#define DIRECTION_TX 0
#define DIRECTION_RX 1

/* command */
#define CMD_CA_RESET 1
#define CMD_CA_TIMEOUT 2

#define MAX_ATR_LEN 33
#define MAX_BUFFER_SIZE (2 * 1024)

#define STATUS_START_WAIT 0
#define STATUS_END_WAIT 1

#define TIMEOUT_THRESHOLD 90000000

#define CA_BUFFER_SIZE 1024

#define GET_WRITE_NUM(x) (x>>16)
#define GET_READ_NUM(x) (x & 0xffff)

#define TA_bit_Mask							0x10
#define TB_bit_Mask							0x20
#define TC_bit_Mask							0x40
#define TD_bit_Mask							0x80
#define ATR_Max_Interface_Bytes		30
#define ATR_Max_Historical_Bytes	15
#define ATR_Max_TA_Bytes			15
#define ATR_Max_TB_Bytes			15
#define ATR_Max_TC_Bytes			15
#define ATR_Max_TD_Bytes			15

typedef struct _ATRParser {
	u8 TS;						/* Initial Character */
	u8 T0;						/* Format character */
	u8 TA[ATR_Max_TA_Bytes];			/* Interface Bytes: TA */
	u8 TB[ATR_Max_TB_Bytes];			/* Interface Bytes: TB */
	u8 TC[ATR_Max_TC_Bytes];			/* Interface Bytes: TC */
	u8 TD[ATR_Max_TD_Bytes];			/* Interface Bytes: TD */
	u8 HistoricalBytes[ATR_Max_Historical_Bytes];	/* minimum = 0 , maximum = 15 */
	u8 TCK;						/* Check character */

	u8 LengthOfAtr;					/* Maximum = 33 */
	u8 LengthOfInterfaceBytes;			/* Maximum = 30 */
	u8 LengthOfHistoricalBytes;			/* Maximum = 15 */
	u8 LengthOfTA;					/* Maximum = 15 */
	u8 LengthOfTB;					/* Maximum = 15 */
	u8 LengthOfTC;					/* Maximum = 15 */
	u8 LengthOfTD;					/* Maximum = 15 */
	u8 LengthOfProtocol;				/* Maximum = 15 */
	u8 TCKPresent;
	u8 ProtocolType[ATR_Max_TD_Bytes];
} ATRParser;

int smd_init();
int uart_write(u8 *buf, u16 length, u16 num);
int uart_read(int num);





#endif
