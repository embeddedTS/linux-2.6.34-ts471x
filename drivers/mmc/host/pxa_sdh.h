/* register definitions of PXA SD Host Controller*/
#define SD_SYS_ADDR_LOW		0x0000		/* DMA System Address Low */
#define SD_SYS_ADDR_HIGH	0x0002		/* DMA System Address High */
#define SD_BLOCK_SIZE		0x0004 		/* Block Size*/
#define SD_BLOCK_COUNT		0x0006 		/* Block Count */
#define SD_ARG_LOW	 	0x0008		/* Command Argument Low */
#define SD_ARG_HIGH	 	0x000a		/* Command Argument High */
#define SD_TRANS_MODE		0x000c		/* Transfer Mode */
#define SD_COMMAND	 	0x000e		/* Command */
#define SD_RESP_0		0x0010		/* Command Response 0 */
#define SD_RESP_1		0x0012		/* Command Response 1 */
#define SD_RESP_2		0x0014		/* Command Response 2 */
#define SD_RESP_3		0x0016		/* Command Response 3 */
#define SD_RESP_4		0x0018		/* Command Response 4 */
#define SD_RESP_5		0x001a		/* Command Response 5 */
#define SD_RESP_6		0x001c		/* Command Response 6 */
#define SD_RESP_7		0x001e		/* Command Response 7 */
#define SD_BUF_DPORT_0		0x0020		/* Buffer Data Port 0 */
#define SD_BUF_DPORT_1		0x0022		/* Buffer Data Port 1 */
#define SD_PRESENT_STAT_1	0x0024		/* Present State 1 */
#define SD_PRESENT_STAT_2	0x0026		/* Present State 2 */
#define SD_HOST_CTRL		0x0028		/* Host Control */
#define SD_BLOCK_GAP_CTRL	0x002a		/* Block Gap Control */
#define SD_CLOCK_CNTL		0x002c		/* Clock Control */
#define SD_TO_CTRL_SW_RST	0x002e		/* Timeout Control/SW Reset */
#define SD_NOR_I_STAT		0x0030		/* Normal Interrupt Status */
#define SD_ERR_I_STAT		0x0032		/* Error Interrupt Status */
#define SD_NOR_I_STAT_EN	0x0034		/* Normal Interrupt Status Enable */
#define SD_ERR_I_STAT_EN	0x0036		/* Error Interrupt Status Enable */
#define SD_NOR_INT_EN		0x0038		/* Normal Interrupt Generation Enable */
#define SD_ERR_INT_EN		0x003a		/* Error Interrupt Generation Enable */
#define SD_ACMD12_ERR_STAT	0x003c		/* Auto CMD12 Error Status */
#define SD_CAP_1		0x0040		/* Capabilities 1 */
#define SD_CAP_3		0x0044		/* Capabilities 3 */
#define SD_CAP_4		0x0046		/* Capabilities 4 */
#define SD_MAX_CUR_1		0x0048		/* Maximum Current 1 */
#define SD_MAX_CUR_2		0x004a		/* Maximum Current 2 */
#define SD_MAX_CUR_3		0x004c		/* Maximum Current 3 */
#define SD_MAX_CUR_4		0x004e		/* Maximum Current 4 */
#define SD_FE_ACMD12_ERR	0x0050		/* Force Event for Auto CMD12 Error */
#define SD_FE_ERR_STAT		0x0052		/* Force Event for Error Status */
#define SD_ADMA_ERR_STAT	0x0054		/* ADMA Error Status */
#define SD_ADMA_SADDR_1		0x0058		/* ADMA System Address[15:0] */
#define SD_ADMA_SADDR_2		0x005a		/* ADMA System Address[31:16] */
#define SD_ADMA_SADDR_3		0x005c		/* ADMA System Address[47:32] */
#define SD_ADMA_SADDR_4		0x005e		/* ADMA System Address[64:48] */
#define SD_FIFO_PARAM		0x00e0		/* FIRO Parameters */
#define SD_SPI_MODE		0x00e4		/* SPI Mode */
#define SD_CLK_BURST_SET	0x00e6 		/* Clock and Burst Size Setup */
#define SD_CE_ATA_1		0x00e8		/* CE-ATA 1 */
#define SD_CE_ATA_2		0x00ea		/* CE-ATA 2 */
#define SD_PAD_IO_SETUP		0x00ec		/* Pad I/O Setup */
#define SD_SLOT_INT_STAT	0x00fc		/* Slot Interrupt Status*/
#define SD_HOST_CTRL_VER	0x00fe		/* Host Controller Version */

/* SD_BLOCK_SIZE */
#define HOST_DMA_BDRY_OFFSET	12
#define HOST_DMA_BDRY_MASK	((u16)0x7)
#define BLOCK_SIZE_OFFSET	0
#define BLOCK_SIZE_MASK		((u16)0x0fff)
#define BLOCK_SIZE_MAX		((u16)0x0800)

/* SD_TRANS_MODE */
#define MULTI_BLK_SEL		((u16)1 << 5)
#define TO_HOST_DIR		((u16)1 << 4)
#define AUTO_CMD12_EN		((u16)1 << 2)
#define BLK_CNT_EN		((u16)1 << 1)
#define DMA_EN			((u16)1 << 0)

/* SD_COMMAND */
#define CMD_IDX_OFFSET		8
#define CMD_IDX_MASK		((u16)0x3f)
#define CMD_TYPE_OFFSET		6
#define CMD_TYPE_MASK		((u16)0x3)
#define CMD_TYPE_NORMAL		((u16)0x0)
#define CMD_TYPE_RESUME		((u16)0x1)
#define CMD_TYPE_SUSPEND	((u16)0x2)
#define CMD_TYPE_ABORT		((u16)0x3)
#define DATA_PRESENT		((u16)1 << 5)
#define CMD_IDX_CHK_EN		((u16)1 << 4)
#define CMD_CRC_CHK_EN		((u16)1 << 3)
#define RESP_TYPE_OFFSET	0
#define RESP_TYPE_MASK		((u16)0x3)
/* RES_TYPE */
#define CMD_RESP_NONE		((u16)0x0)
#define CMD_RESP_136BIT		((u16)0x1)
#define CMD_RESP_48BIT		((u16)0x2)
#define CMD_RESP_48BITB 	((u16)0x3)

/* SD_PRESENT_STAT_1 */
#define CMD_INHBT_DAT		((u16)1 << 1)
#define CMD_INHBT_CMD		((u16)1 << 0)

/* SD_PRESENT_STAT_2 */
#define CARD_STABLE		((u16)1 << 1)
#define CARD_DETECTED		((u16)1 << 2)
#define CARD_PROT		((u16)1 << 3)
#define DATA_LINE_LEVEL_MASK	((u16)0xf << 4)
#define CMD_LINE_LEVEL_MASK	((u16)1 << 8)

/* SD_HOST_CTRL */
#define SD_BUS_VLT_OFFSET	9
#define SD_BUS_VLT_MASK		((u16)0x7)
#define SD_BUS_VLT_18V		((u16)0x5)
#define SD_BUS_VLT_30V		((u16)0x6)
#define SD_BUS_VLT_33V		((u16)0x7)
#define SD_BUS_POWER		((u16)1 << 8)
#define DMA_SEL_OFFSET		3
#define DMA_SEL_MASK		((u16)0x3)
#define DMA_SEL_SDMA		((u16)0)
#define DMA_SEL_ADMA1		((u16)1)
#define DMA_SEL_ADMA2_32	((u16)2)
#define DMA_SEL_ADMA2_64	((u16)3)
#define HI_SPEED_EN		((u16)1 << 2)
#define DATA_WIDTH_4BIT		((u16)1 << 1)

/* SD_BLOCK_GAP_CTRL */
#define INT_BLK_GAP		((u16)1 << 3)
#define RD_WT_CNTL		((u16)1 << 2)
#define CONT_REQ		((u16)1 << 1)
#define STOP_AT_BLK_GAP_REQ	((u16)1 << 0)

/* SD_CLOCK_CNTL */
#define SD_FREQ_SEL_OFFSET	8
#define SD_FREQ_SEL_MASK	((u16)0xff)
#define EXT_CLK_EN		((u16)1 << 2)
#define INT_CLK_STABLE		((u16)1 << 1)
#define INT_CLK_EN		((u16)1 << 0)

/* SD_TO_CTRL_SW_RST */
#define SW_RST_DAT		((u16)1 << 10)
#define SW_RST_CMD		((u16)1 << 9)
#define SW_RST_ALL		((u16)1 << 8)
#define DAT_TO_VAL_OFFSET	0
#define DAT_TO_MASK		((u16)0xf)

/* SD_NOR_I_STAT,  SD_NOR_I_STAT_EN, SD_NOR_INT_EN */
#define ERR_INT			((u16)1 << 15) /* Error Interrupt*/
#define CARD_INT		((u16)1 << 8) /* Card Interrupt */
#define CARD_REM		((u16)1 << 7) /* Card Removal Interrupt */
#define CARD_INS		((u16)1 << 6) /* Card Insertion Interrupt */
#define RX_RDY			((u16)1 << 5) /* Buffer Read Ready */
#define TX_RDY			((u16)1 << 4) /* Buffer Write Ready */
#define DMA_INT			((u16)1 << 3) /* DMA Interrupt */
#define BLK_GAP_EVNT		((u16)1 << 2) /* Block Gap Event */
#define XFER_COMP		((u16)1 << 1) /* Transfer Complete */
#define CMD_COMP		((u16)1 << 0) /* Command Complete */
#define SD_NOR_I_STAT_RVD_MASK 	((u16)0x7e00) /* Mask for SD_NOR_I_STAT Reserved Bits[14 :9] */
#define SD_NOR_INT_EN_RVD_MASK	((u16)0xfe00) /* Mask for SD_NOR_INT_EN/SD_NOR_I_STAT_EN Reserved Bits[15 :9] */

/* SD_ERR_I_STAT,  SD_ERR_I_STAT_EN, SD_ERR_INT_EN */
#define CRC_STATUS_ERR		((u16)1 << 15) /* CRC Status Error  Returned from Card in Write Transaction*/
#define CPL_TO_ERR		((u16)1 << 14) /* Command Completion Signal Timeout Error, for CE-ATA mode only*/
#define AXI_RESP_ERR		((u16)1 << 13) /* AXI Bus Response Error */
#define SPI_ERR			((u16)1 << 12) /* SPI Mode Error*/
#define ADMA_ERR		((u16)1 << 9) /* AMDA Error */
#define AUTO_CMD12_ERR		((u16)1 << 8) /* Auto CMD12 Error*/
#define CUR_LIMIT_ERR		((u16)1 << 7) /* Current Limit Error*/
#define RD_DATA_END_ERR		((u16)1 << 6) /* Read Data End Bit Error*/
#define RD_DATA_CRC_ERR		((u16)1 << 5) /* Read Data CRC Error*/
#define DATA_TO_ERR		((u16)1 << 4) /* Data Timeout Error*/
#define CMD_IDX_ERR		((u16)1 << 3) /* Command Index Error*/
#define CMD_END_BIT_ERR		((u16)1 << 2) /* Command End Bit Error*/
#define CMD_CRC_ERR		((u16)1 << 1) /* Command CRC Error*/
#define CMD_TO_ERR		((u16)1 << 0) /* Command Timeout Error*/
#define SD_ERR_INT_EN_RVD_MASK		((u16)0x0c00) /* Mask for SD_ERR_INT_EN/SD_ERR_I_STAT_EN Reserved Bits[11 :10] */
#define SD_ERR_INT_DATA_ERR_MASK	(DATA_TO_ERR | RD_DATA_CRC_ERR | RD_DATA_END_ERR) /*DATA Line Error*/
#define SD_ERR_INT_CMD_ERR_MASK		(CMD_TO_ERR | CMD_CRC_ERR | CMD_END_BIT_ERR | CMD_IDX_ERR) /* CMD Line Error*/

/* SD_FIFO_PARAM */
#define DIS_PAD_SD_CLK_GATE	((u16)1 << 10) /* Turn on/off Dynamic SD Clock Gating */

/* SD_CLK_BURST_SET */
#define SDCLK_DELAY_OFFSET	10
#define SDCLK_DELAY_MASK	((u16)0xf)
#define SDCLK_DELAY_MAX	((u16)0xf)
#define SDCLK_SEL_OFFSET	8
#define SDCLK_SEL_MASK		((u16)0x3)
#define SDCLK_SEL_INIT_VAL	((u16)0x3)
#define DMA_BURST_SIZE		((u16)0)

/* SD_SLOT_INT_STAT */
#define SLOT_INT1 		((u16)1<<1)
#define SLOT_INT0 		((u16)1<<0)
#define SlOT_INT_MASK		(SLOT_INT0 | SLOT_INT1)

/* SD_CE_ATA_2 */
#define DATA_WIDTH_8BIT		((u16)1 << 8)
#define MMC_CARD		((u16)1 << 12)

/* Bus width setting*/
#define SDH_BUS_WIDTH_4  4 /* default */
#define SDH_BUS_WIDTH_1  1
#define SDH_BUS_WIDTH_8  8

