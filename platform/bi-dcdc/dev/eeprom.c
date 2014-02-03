#include "eeprom.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_clkpwr.h"
#include "lpc_types.h"

#define I2CDEV_S_ADDR	(0xA0 >> 1)

void eeprom_init() {
	PINSEL_CFG_Type PinCfg;

	/*
	 * Configure I2C pins
	 */
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.Funcnum = 1;
	PinCfg.Pinnum = 27;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 28;
	PINSEL_ConfigPin(&PinCfg);

	/*
	 * Set up clock and power for I2C0 module
	 */
	I2C_Init(LPC_I2C0, 100000);
	I2C_Cmd(LPC_I2C0, I2C_MASTER_MODE, 1);
}

void eeprom_write(eeprom_addr_t addr, unsigned char *buf, int size) {
	I2C_M_SETUP_Type transferMCfg;
	uint8_t eeprom_addr;
	eeprom_addr = (uint8_t) addr;

	transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
	transferMCfg.tx_data = &eeprom_addr;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	//LPC_GPIO1->FIOPIN ^= (1 << 27);
	I2C_MasterTransferData(LPC_I2C0, &transferMCfg, I2C_TRANSFER_POLLING);

	transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
	transferMCfg.tx_data = buf;
	transferMCfg.tx_length = size;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	//LPC_GPIO1->FIOPIN ^= (1 << 27);
	I2C_MasterTransferData(LPC_I2C0, &transferMCfg, I2C_TRANSFER_POLLING);
}

void eeprom_read(eeprom_addr_t addr, unsigned char *buf, int size){
	I2C_M_SETUP_Type transferMCfg;
	uint8_t eeprom_addr;
	eeprom_addr = (uint8_t) addr;
	printf("b4 write eeprom read addr\n");
	transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
	transferMCfg.tx_data = &eeprom_addr;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 3;
	//LPC_GPIO1->FIOPIN ^= (1 << 27);
	I2C_MasterTransferData(LPC_I2C0, &transferMCfg, I2C_TRANSFER_POLLING);
	printf("after write eeprom addr\n");
	transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
	transferMCfg.tx_data = NULL;
	transferMCfg.tx_length = 0;
	transferMCfg.rx_data = buf;
	transferMCfg.rx_length = size;
	transferMCfg.retransmissions_max = 3;
	//LPC_GPIO1->FIOPIN ^= (1 << 27);
	I2C_MasterTransferData(LPC_I2C0, &transferMCfg, I2C_TRANSFER_POLLING);
	printf("after read\n");
}
