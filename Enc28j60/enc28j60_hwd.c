/*
 * enc28j60_hwd.c
 *
 *  Created on: Feb 8, 2025
 *      Author:  NK Kalambay
 */

#include <stdint.h>
#include "enc28j60_hwd.h"

static void enc28j60_SoftReset(enc28j60Drv * dev);
static void enc28j60_writeReg(enc28j60Drv * dev, uint8_t u8Reg, uint8_t u8Value);
//static uint8_t enc28j60_readEtherReg(enc28j60Drv * dev, uint8_t u8Reg);
static uint8_t enc28j60_readMacMIIReg(enc28j60Drv * dev, uint8_t u8Reg);
static void enc28j60_BitFieldSet(enc28j60Drv * dev, uint8_t u8Reg, uint8_t u8data);
static void enc28j60_BitFieldClear(enc28j60Drv * dev, uint8_t u8Reg, uint8_t u8data);
static uint16_t enc28j60_readPhyReg(enc28j60Drv * dev, uint8_t addr);

static uint16_t enc28j60_getPhyPartNumber(enc28j60Drv * dev);
//static uint8_t enc28j60_getPhyRevNumber(enc28j60Drv * dev);
static uint32_t enc28j60_getPhyIdentifier(enc28j60Drv * dev);
static bool enc28j60_getPhylinkHasBeenDown(enc28j60Drv * dev);
static bool enc28j60_getPhyjabberStatusBit(enc28j60Drv * dev);
static bool enc28j60_getPhyPolarityStatus(enc28j60Drv * dev);
static bool enc28j60_getPhyDuplexStatus(enc28j60Drv * dev);
static bool enc28j60_getPhyLinkStatus(enc28j60Drv * dev);
static bool enc28j60_getPhyCollisionStatus(enc28j60Drv * dev);
static bool enc28j60_getPhyIsRxStatus(enc28j60Drv * dev);
static bool enc28j60_getPhyIsTxStatus(enc28j60Drv * dev);

static encj28j60_bank convertRegValToBank(uint8_t value);
static uint8_t convertBankToBits(encj28j60_bank bBank);

void enc28j60_initDr(enc28j60Drv * dev, spiChipSel cs, spiChipDSl dCS, slaveRead rd,
		slaveWrite wr, intHandler hdle, delayMs delay)
{
	dev->spi.fncPtrCS 		= cs;
	dev->spi.fncPtrChipDS 	= dCS;
	dev->spi.fncPtrRead 	= rd;
	dev->spi.fncPtrWrite 	= wr;
	dev->fncPtrcallBack 	= hdle;
	dev->fncPtrDelayFunc 	= delay;
}

void enc28j60_strtDr(enc28j60Drv * dev)
{
	enc28j60_sftRst(dev);

	//Just for information, but no need for anything further.
	(void) enc28j60_getPhyRevNumber(dev);
	(void) enc28j60_getPhyPartNumber(dev);
	(void) enc28j60_getPhyIdentifier(dev);

#if 0
	enc28j60_writeReg(dev, dev->bank0.EDMACSH, 0);

	//No Writing of transmit start and end address

	//Receive buffer = 6K meaning from 0x800 to 0x1FFF
	//Start
	//enc28j60_reg_value startAddr = {.u16Val = 0x0800};
	enc28j60_writeReg(dev, dev->bank0.ERXSTH, (uint8_t) dev->rxBufStartAddr.u8ValHi);
	enc28j60_writeReg(dev, dev->bank0.ERXSTL, (uint8_t) dev->rxBufStartAddr.u8ValLo);

	//End
	//enc28j60_reg_value endAddr = {.u16Val = 0x1FFF};
	enc28j60_writeReg(dev, dev->bank0.ERXNDH, (uint8_t) (dev->rxBufEndAddr.u8ValHi));
	enc28j60_writeReg(dev, dev->bank0.ERXNDL, (uint8_t) (dev->rxBufEndAddr.u8ValLo));

	//Are we going to do increment?
	enc28j60_BitFieldSet(dev, dev->bank0.commonRegs.ECON2, (1 << 7));

	//Are we going to auto decrement?
	enc28j60_BitFieldSet(dev, dev->bank0.commonRegs.ECON2, (1 << 6));

	//Write to RXEN
	enc28j60_BitFieldSet(dev, dev->bank0.commonRegs.ECON1, (1 << 2));

#endif
}

bool enc28j60_intPnd(enc28j60Drv * dev)
{
	return dev->bInterruptFlag;
}

void enc28j60_intSet(enc28j60Drv * dev)
{
	dev->bInterruptFlag = true;
}

void enc28j60_intCls(enc28j60Drv * dev)
{
	dev->bInterruptFlag = false;

	//Clear the bits as well
}

uint8_t enc28j60_getEtherInterrupt(enc28j60Drv * dev)
{
	switch(dev->bnBank)
	{
	default:
		return 0;
	case bank_0:
		return enc28j60_readEtherReg(dev, dev->bank0.commonRegs.EIR);
		break;
	case bank_1:
		return enc28j60_readEtherReg(dev, dev->bank1.commonRegs.EIR);
		break;
	case bank_2:
		return enc28j60_readEtherReg(dev, dev->bank2.commonRegs.EIR);
		break;
	case bank_3:
		return enc28j60_readEtherReg(dev, dev->bank3.commonRegs.EIR);
		break;
	}
}

uint16_t enc28j60_getPhyInterrupt(enc28j60Drv * dev)
{
	return enc28j60_readPhyReg(dev, dev->phyReg.PHIR);
}

uint8_t enc28j60_getwakeUpInterrupt(enc28j60Drv * dev)
{
	return enc28j60_readEtherReg(dev, dev->bank1.EWOLIR);
}

void enc28j60_sftRst(enc28j60Drv * dev)
{
	enc28j60_SoftReset(dev);
}

void enc28j60_etherTransmit(enc28j60Drv * dev, uint8_t * u8PtrData, const uint16_t * length)
{

}

void enc28j60_etherReceive(enc28j60Drv * dev, uint8_t * u8PtrData, const uint16_t * length)
{

}

static void enc28j60_bankChange(enc28j60Drv * dev, encj28j60_bank bBank)
{
	uint8_t u8Command;
	uint8_t u8data = 0x03; //clear the last two bits, meaning we are going to bank 0

	switch(dev->bnBank)
	{
		default:
			return;
		case bank_0:
			u8Command =	(dev->opcode.u8BitFieldClear) | (0x1F & dev->bank0.commonRegs.ECON1);
			break;
		case bank_1:
			u8Command =	(dev->opcode.u8BitFieldClear) | (0x1F & dev->bank1.commonRegs.ECON1);
			break;
		case bank_2:
			u8Command =	(dev->opcode.u8BitFieldClear) | (0x1F & dev->bank2.commonRegs.ECON1);
			break;
		case bank_3:
			u8Command =	(dev->opcode.u8BitFieldClear) | (0x1F & dev->bank3.commonRegs.ECON1);
			break;
	}

	dev->spi.fncPtrCS();
	dev->spi.fncPtrWrite(&u8Command, 1);
	dev->spi.fncPtrWrite(&u8data, 1);
	dev->spi.fncPtrChipDS();

	uint8_t u8CmdNextBank;
	u8CmdNextBank = (dev->opcode.u8BitFieldSet) | (0x1F & dev->bank0.commonRegs.ECON1);

	uint8_t u8BitsToSets = convertBankToBits(bBank);
	//Zero is the only value with all clear bits, so there is no need for any processing.
	if(u8BitsToSets != 0 && u8BitsToSets <= 3u)
	{
		dev->spi.fncPtrCS();
		dev->spi.fncPtrWrite(&u8CmdNextBank, 1);
		dev->spi.fncPtrWrite(&u8BitsToSets, 1);
		dev->spi.fncPtrChipDS();
		dev->bnBank = bBank;
	}

}

static encj28j60_bank convertRegValToBank(uint8_t value)
{
	encj28j60_bank bReturnValue = undefBank;
	switch((value >> 06))
	{
		default:
			break;
		case 0:
			bReturnValue = bank_0;
			break;
		case 1:
			bReturnValue = bank_1;
			break;
		case 2:
			bReturnValue = bank_2;
			break;
		case 3:
			bReturnValue = bank_3;
			break;
	}
	return bReturnValue;
}

static uint8_t convertBankToBits(encj28j60_bank bBank)
{
	uint8_t u8TempValue = (uint8_t) bBank;
	u8TempValue >>= 6u;
	return u8TempValue;
}
uint8_t enc28j60_readEtherReg(enc28j60Drv * dev, uint8_t u8Reg)
//static uint8_t enc28j60_readEtherReg(enc28j60Drv * dev, uint8_t u8Reg)
{
	encj28j60_bank bBank = convertRegValToBank(u8Reg);
	if(bBank != dev->bnBank) enc28j60_bankChange(dev, bBank);
	uint8_t u8Command = (dev->opcode.u8readControlRegister) | (0x1F & u8Reg);
	uint8_t u8ReturnValue;
	dev->spi.fncPtrCS();
	dev->spi.fncPtrWrite(&u8Command, 1);
	(void) dev->spi.fncPtrRead(&u8ReturnValue);
	dev->spi.fncPtrChipDS();
	return u8ReturnValue;
}

static uint8_t enc28j60_readMacMIIReg(enc28j60Drv * dev, uint8_t u8Reg)
{
	encj28j60_bank bBank = convertRegValToBank(u8Reg);
	if(bBank != dev->bnBank) enc28j60_bankChange(dev, bBank);
	uint8_t u8Command = (dev->opcode.u8readControlRegister) | (0x1F & u8Reg);
	uint8_t u8ReturnValue;
	dev->spi.fncPtrCS();
	dev->spi.fncPtrWrite(&u8Command, 1);
	(void) dev->spi.fncPtrRead(&u8ReturnValue); //At this point it is dummy data
	(void) dev->spi.fncPtrRead(&u8ReturnValue); //Good Data
	dev->spi.fncPtrChipDS();
	return u8ReturnValue;
}
static void enc28j60_writeReg(enc28j60Drv * dev, uint8_t u8Reg, uint8_t u8Value)
{
	encj28j60_bank bBank = convertRegValToBank(u8Reg);
	if(bBank != dev->bnBank) enc28j60_bankChange(dev, bBank);
	uint8_t u8Command = (dev->opcode.u8writeControlRegister) | (0x1F & u8Reg);
	dev->spi.fncPtrCS();
	dev->spi.fncPtrWrite(&u8Command, 1);
	dev->spi.fncPtrWrite(&u8Value, 1);
	dev->spi.fncPtrChipDS();
}

static void enc28j60_BitFieldSet(enc28j60Drv * dev, uint8_t u8Reg, uint8_t u8data)
{
	encj28j60_bank bBank = convertRegValToBank(u8Reg);
	if(bBank != dev->bnBank) enc28j60_bankChange(dev, bBank);
	uint8_t u8Command = (dev->opcode.u8BitFieldSet) | (0x1F & u8Reg);
	dev->spi.fncPtrCS();
	dev->spi.fncPtrWrite(&u8Command, 1);
	dev->spi.fncPtrWrite(&u8data, 1);
	dev->spi.fncPtrChipDS();
}

static void enc28j60_BitFieldClear(enc28j60Drv * dev, uint8_t u8Reg, uint8_t u8data)
{
	encj28j60_bank bBank = convertRegValToBank(u8Reg);
	if(bBank != dev->bnBank) enc28j60_bankChange(dev, bBank);
	uint8_t u8Command = (dev->opcode.u8BitFieldClear) | (0x1F & u8Reg);
	dev->spi.fncPtrCS();
	dev->spi.fncPtrWrite(&u8Command, 1);
	dev->spi.fncPtrWrite(&u8data, 1);
	dev->spi.fncPtrChipDS();
}

static void enc28j60_SoftReset(enc28j60Drv * dev)
{
	uint8_t u8Command = (dev->opcode.u8SoftReset) | (0x1F);
	dev->spi.fncPtrCS();
	dev->spi.fncPtrWrite(&u8Command, 1);
	dev->spi.fncPtrChipDS();
}

static uint16_t enc28j60_readPhyReg(enc28j60Drv * dev, uint8_t addr)
{
	enc28j60_writeReg(dev, dev->bank2.MIREGADR, addr);
	uint8_t u8ReturnValuefrSet = enc28j60_readMacMIIReg(dev, dev->bank2.MICMD);

	u8ReturnValuefrSet |= (1 << 0);
	enc28j60_writeReg(dev, dev->bank2.MICMD, u8ReturnValuefrSet);

	dev->fncPtrDelayFunc(1);

	while(enc28j60_readMacMIIReg(dev, dev->bank3.MISTAT) & (0x01)) continue;

	uint8_t u8ReturnValueFrClr = enc28j60_readMacMIIReg(dev, dev->bank2.MICMD);
	u8ReturnValueFrClr &= ~(1 << 0);
	enc28j60_writeReg(dev, dev->bank2.MICMD, u8ReturnValueFrClr);

	uint16_t u16Value;
	u16Value = enc28j60_readMacMIIReg(dev, dev->bank2.MIRDH) << 8;
	u16Value |= enc28j60_readMacMIIReg(dev, dev->bank2.MIRDL);

	return u16Value;
}

static void enc28j60_writePhyReg(enc28j60Drv * dev, uint8_t addr, uint16_t data)
{
	enc28j60_writeReg(dev, dev->bank2.MIREGADR, addr);
	enc28j60_writeReg(dev, dev->bank2.MIRDL, (uint8_t)(data >> 8));
	enc28j60_writeReg(dev, dev->bank2.MIRDH, (uint8_t)(data & 0xFF));
	dev->fncPtrDelayFunc(1);
	while(enc28j60_readMacMIIReg(dev, dev->bank3.MISTAT) & (1 << 0)) continue;
}

static uint16_t enc28j60_getPhyPartNumber(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHID2);
	return ((u16returnValue & 0x03F0) >> 4);
}

uint8_t enc28j60_getPhyRevNumber(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHID2);
	return ((uint8_t) u16returnValue & 0x000F);
}

static uint32_t enc28j60_getPhyIdentifier(enc28j60Drv * dev)
{
	uint16_t u32returnValue;
	u32returnValue = (uint32_t) enc28j60_readPhyReg(dev, dev->phyReg.PHID1);

	u32returnValue >>= 0x03;

	u32returnValue |= ((uint32_t) enc28j60_readPhyReg(dev, dev->phyReg.PHID2) << 0x09);

	return u32returnValue;
}

static bool enc28j60_getPhylinkHasBeenDown(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHSTAT1);
	return (u16returnValue & (1 << 2));
}

static bool enc28j60_getPhyjabberStatusBit(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHSTAT1);
	return (u16returnValue & (1 << 1));
}

static bool enc28j60_getPhyPolarityStatus(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHSTAT2);
	return (u16returnValue & (1 << 4));
}

static bool enc28j60_getPhyDuplexStatus(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHSTAT2);
	return (u16returnValue & (1 << 9));
}

static bool enc28j60_getPhyLinkStatus(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHSTAT2);
	return (u16returnValue & (1 << 10));
}

static bool enc28j60_getPhyCollisionStatus(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHSTAT2);
	return (u16returnValue & (1 << 11));
}

static bool enc28j60_getPhyIsRxStatus(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHSTAT2);
	return (u16returnValue & (1 << 12));
}

static bool enc28j60_getPhyIsTxStatus(enc28j60Drv * dev)
{
	uint16_t u16returnValue;
	u16returnValue = enc28j60_readPhyReg(dev, dev->phyReg.PHSTAT2);
	return (u16returnValue & (1 << 13));
}

enc28j60Drv dev =
{
	.bank0 = 
	{
		.ERDPTL				= BANK_0 | 0x00,
		.ERDPTH				= BANK_0 | 0x01,
		.EWRPTL				= BANK_0 | 0x02,
		.EWRPTH				= BANK_0 | 0x03,
		.ETXSTL				= BANK_0 | 0x04,
		.ETXSTH				= BANK_0 | 0x05,
		.ETXNDL				= BANK_0 | 0x06,
		.ETXNDH				= BANK_0 | 0x07,
		.ERXSTL				= BANK_0 | 0x08,
		.ERXSTH				= BANK_0 | 0x09,
		.ERXNDL				= BANK_0 | 0x0A,
		.ERXNDH				= BANK_0 | 0x0B,
		.ERXRDPTL			= BANK_0 | 0x0C,
		.ERXRDPTH			= BANK_0 | 0x0D,
		.ERXWRPTL			= BANK_0 | 0x0E,
		.ERXWRPTH			= BANK_0 | 0x0F,
		.EDMASTL			= BANK_0 | 0x10,
		.EDMASTH			= BANK_0 | 0x11,
		.EDMANDL			= BANK_0 | 0x12,
		.EDMANDH			= BANK_0 | 0x13,
		.EDMADSTL			= BANK_0 | 0x14,
		.EDMADSTH			= BANK_0 | 0x15,
		.EDMACSL			= BANK_0 | 0x16,
		.EDMACSH			= BANK_0 | 0x17,
		.undefined1			= BANK_0 | 0x18,
		.undefined2			= BANK_0 | 0x19,
		.Reversed1			= BANK_0 | 0x1A,
		.commonRegs.EIE 	= BANK_0 | 0x1B,
		.commonRegs.EIR 	= BANK_0 | 0x1C,
		.commonRegs.ESTAT	= BANK_0 | 0x1D,
		.commonRegs.ECON2 	= BANK_0 | 0x1E,
		.commonRegs.ECON1	= BANK_0 | 0x1F
	},

	.bank1 = 
	{
		.EHT0				= BANK_1 | 0x00,
		.EHT1				= BANK_1 | 0x01,
		.EHT2				= BANK_1 | 0x02,
		.EHT3				= BANK_1 | 0x03,
		.EHT4				= BANK_1 | 0x04,
		.EHT5				= BANK_1 | 0x05,
		.EHT6				= BANK_1 | 0x06,
		.EHT7				= BANK_1 | 0x07,
		.EPMM0				= BANK_1 | 0x08,
		.EPMM1				= BANK_1 | 0x09,
		.EPMM2				= BANK_1 | 0x0A,
		.EPMM3				= BANK_1 | 0x0B,
		.EPMM4				= BANK_1 | 0x0C,
		.EPMM5				= BANK_1 | 0x0D,
		.EPMM6				= BANK_1 | 0x0E,
		.EPMM7				= BANK_1 | 0x0F,
		.EPMCSL				= BANK_1 | 0x10,
		.EPMCSH				= BANK_1 | 0x11,
		.undefined1			= BANK_1 | 0x12,
		.undefined2			= BANK_1 | 0x13,
		.EPMOL				= BANK_1 | 0x14,
		.EPMOH				= BANK_1 | 0x15,
		.EWOLIE				= BANK_1 | 0x16,
		.EWOLIR				= BANK_1 | 0x17,
		.ERXFCON			= BANK_1 | 0x18,
		.EPKTCNT			= BANK_1 | 0x19,
		.Reserved1			= BANK_1 | 0x1A,
		.commonRegs.EIE 	= BANK_1 | 0x1B,
		.commonRegs.EIR 	= BANK_1 | 0x1C,
		.commonRegs.ESTAT	= BANK_1 | 0x1D,
		.commonRegs.ECON2 	= BANK_1 | 0x1E,
		.commonRegs.ECON1	= BANK_1 | 0x1F
	},

	.bank2 = 
	{
		.MACON1				= BANK_2 | 0x00,
		.MACON2				= BANK_2 | 0x01,
		.MACON3				= BANK_2 | 0x02,
		.MACON4				= BANK_2 | 0x03,
		.MABBIPG			= BANK_2 | 0x04,
		.undefined1			= BANK_2 | 0x05,
		.MAIPGL				= BANK_2 | 0x06,
		.MAIPGH				= BANK_2 | 0x07,
		.MACLCON1			= BANK_2 | 0x08,
		.MACLCON2			= BANK_2 | 0x09,
		.MAMXFLL			= BANK_2 | 0x0A,
		.MAMXFLH			= BANK_2 | 0x0B,
		.Reserved1			= BANK_2 | 0x0C,
		.MAPHSUP			= BANK_2 | 0x0D,
		.Reserved2			= BANK_2 | 0x0E,
		.undefined2			= BANK_2 | 0x0F,
		.Reserved3			= BANK_2 | 0x10,
		.MICON				= BANK_2 | 0x11,
		.MICMD				= BANK_2 | 0x12,
		.undefined3			= BANK_2 | 0x13,
		.MIREGADR			= BANK_2 | 0x14,
		.Reserved4			= BANK_2 | 0x15,
		.MIWRL				= BANK_2 | 0x16,
		.MIWRH				= BANK_2 | 0x17,
		.MIRDL				= BANK_2 | 0x18,
		.MIRDH				= BANK_2 | 0x19,
		.Reserved5			= BANK_2 | 0x1A,
		.commonRegs.EIE 	= BANK_2 | 0x1B,
		.commonRegs.EIR 	= BANK_2 | 0x1C,
		.commonRegs.ESTAT	= BANK_2 | 0x1D,
		.commonRegs.ECON2 	= BANK_2 | 0x1E,
		.commonRegs.ECON1	= BANK_2 | 0x1F
	},

	.bank3 = 
	{
		.MAADR0				= BANK_3 | 0x00,
		.MAADR1				= BANK_3 | 0x01,
		.MAADR3				= BANK_3 | 0x02,
		.MAADR2				= BANK_3 | 0x03,
		.MAADR5				= BANK_3 | 0x04,
		.MAADR4				= BANK_3 | 0x05,
		.EBSTSD				= BANK_3 | 0x06,
		.EBSTCON			= BANK_3 | 0x07,
		.EBSTCSL			= BANK_3 | 0x08,
		.EBSTCSH			= BANK_3 | 0x09,
		.MISTAT				= BANK_3 | 0x0A,
		.undefined[0]		= BANK_3 | 0x0B,
		.undefined[1]		= BANK_3 | 0x0C,
		.undefined[2]		= BANK_3 | 0x0D,
		.undefined[3]		= BANK_3 | 0x0E,
		.undefined[4]		= BANK_3 | 0x0F,
		.undefined[5]		= BANK_3 | 0x10,
		.undefined[6]		= BANK_3 | 0x11,
		.EREVID				= BANK_3 | 0x12,
		.undefined8			= BANK_3 | 0x13,
		.undefined9			= BANK_3 | 0x14,
		.ECOCON				= BANK_3 | 0x15,
		.Reserved1			= BANK_3 | 0x16,
		.EFLOCON			= BANK_3 | 0x17,
		.EPAUSL				= BANK_3 | 0x18,
		.EPAUSH				= BANK_3 | 0x19,
		.Reserved2			= BANK_3 | 0x1A,
		.commonRegs.EIE 	= BANK_3 | 0x1B,
		.commonRegs.EIR 	= BANK_3 | 0x1C,
		.commonRegs.ESTAT	= BANK_3 | 0x1D,
		.commonRegs.ECON2 	= BANK_3 | 0x1E,
		.commonRegs.ECON1	= BANK_3 | 0x1F
	},

	.phyReg = 
	{
		.PHCON1 	= 0x0000,
		.PHSTAT1 	= 0x0001,
		.PHID1 		= 0x0002,
		.PHID2 		= 0x0003,
		.PHCON2 	= 0x0010,
		.PHSTAT2 	= 0x0000,
		.PHIE		= 0x0012,
		.PHIR		= 0x0013,
		.PHLCON		= 0x0014
	},

	.opcode =
	{
		.u8readControlRegister 	= (0b000 << 0x05),
		.u8readBufferMemory 	= (0b001 << 0x05) | (0b11010),
		.u8writeControlRegister = (0b010 << 0x05),
		.u8WriteBufferMemory 	= (0b011 << 0x05) | (0b11010),
		.u8BitFieldSet 			= (0b100 << 0x05),
		.u8BitFieldClear 		= (0b101 << 0x05),
		.u8SoftReset 			= (0b111 << 0x05) | (0b11111)
	},

	.bnBank = bank_0,
	.rxBufStartAddr.u16Val 	= 0x0800,
	.rxBufEndAddr.u16Val 	= 0x1FFF,
	.rxLockAddr.u16Val		= 0x0000,
	.txBufStartAddr			= 0x0000,
	.txBufEndAddr			= 0x07FF
};

