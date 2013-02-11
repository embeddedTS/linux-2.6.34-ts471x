/*
 *pxa168 ethernet platform device data definition file.
 */
#ifndef __LINUX_PXA168_ETH_H
#define __LINUX_PXA168_ETH_H

struct pxa168_eth_platform_data {
	u32		port_number;
	u16		force_phy_addr;	/* force override if phy_addr == 0 */
	u16		phy_addr;

	/* If speed is 0, then speed and duplex are autonegotiated. */
	u32		speed;		/* 0, SPEED_10, SPEED_100, SPEED_1000 */
	u32		duplex;		/* DUPLEX_HALF or DUPLEX_FULL */

	/* non-zero values of the following fields override defaults */
	u32		tx_queue_size;
	u32		rx_queue_size;
	u8		mac_addr[6];	/* mac address if non-zero*/
 	
	int (*init)(void);
 
};

#endif /* __LINUX_PXA168_ETH_H */
