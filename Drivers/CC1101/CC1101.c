#include "CC1101.h"
#include "CC1101_port.h"
#include "main.h"
#include "CC1101_macro.h"

uint8_t txAddr = 0;
/* Read single register
 * addr - register address
 * return value - value of register
 */
uint8_t __CC1101_ReadReg(uint8_t addr);

/* Write to single register
 * addr - register address
 * data - value to write into register
 */
void __CC1101_WriteReg(uint8_t addr, uint8_t data);

/* Burst read from registers
 * addr - base register address
 * lth - length data to be received
 * data - pointer to data
 */
void __CC1101_BurstReadReg(uint8_t addr, uint8_t lth, uint8_t* data);

/* Write read to registers
 * addr - base register address
 * lth - length data to be transmitted
 * data - pointer to data
 */
void __CC1101_BurstWriteReg(uint8_t addr, uint8_t lth, uint8_t* data);

/* Write command to the device
 * cmd - command code
 */
void __CC1101_WriteCMD(uint8_t cmd);

/* Read status registers
 * addr - address of register
 * return value - value of register
 */
uint8_t __CC1101_ReadStatusRegs(uint8_t addr);

/*
 * Initial SPI settings and CS prepare should be implemented here
 */
uint8_t CC1101_GPIO_Prepare(){
	___CC1101_USER_CS_High();
	return CC1101_OK;
}

/* Set GDO0 mode */
uint8_t CC1101_Set_GDO0(uint8_t GDO0_mode)
{
	__CC1101_WriteReg(IOCFG0, GDO0_mode);
	return CC1101_OK;
}

/* Set GDO2 mode */
uint8_t CC1101_Set_GDO2(uint8_t GDO2_mode)
{
	__CC1101_WriteReg(IOCFG2, GDO2_mode);
	return CC1101_OK;
}

/*
 * Check RF`s ID, write initial config, defined in CC1101_CC1101.h
 */
uint8_t CC1101_Init(){

	__CC1101_WriteCMD(CC1101_SRES);
	HAL_Delay(100);
	uint8_t ID = __CC1101_ReadReg(AGCTEST);
	if(ID != CC1101_DEVICE_ID)
		return CC1101_ERROR;

	__CC1101_BurstWriteReg(0x00, 47, CC1101_DefaultSettings);

	return CC1101_OK;
}

/*
 * Transmitting of byte flow with 1 <= length <= 62 in pooling mode
 * data - pointer to data array
 * size - quantity of bytes to be sent
 */
uint8_t CC1101_TransmitPacketPooling(uint8_t* data, uint8_t size){
	__CC1101_WriteCMD(CC1101_SIDLE);
	/*
	if((__CC1101_ReadReg(PKTCTRL1) & 0b11) != 0b00) // if var lenght mode, size +1??? WTF, but it works
		size++;
		*/
	/* Length select */
	uint8_t pktformat = __CC1101_ReadReg(PKTCTRL0) & 0b11;
	if(pktformat == CC1101_DYNAM_PKTLN)
		__CC1101_WriteReg(CC1101_TXFIFO, size);
	if(pktformat == CC1101_FIXED_PKTLN)
		__CC1101_WriteReg(PKTLEN, size);

	/* Address select */
	if((__CC1101_ReadReg(PKTCTRL1) & 0b11) != 0b00)
		__CC1101_WriteReg(CC1101_TXFIFO, txAddr);


	/* Transmitting */
	__CC1101_BurstWriteReg(CC1101_TXFIFO, size, data);

	__CC1101_WriteCMD(CC1101_STX);
	while((__CC1101_ReadStatusRegs(CC1101_TXBYTES) & 0b01111111) != 0);
	__CC1101_WriteCMD(CC1101_SFTX);
	return CC1101_OK;
}

/*
 * Transmitting of byte flow with 1 <= length <= 62 in interrupt mode
 * data - pointer to data array
 * size - quantity of bytes to be sent
 */
uint8_t CC1101_TransmitPacketInt(uint8_t* data, uint8_t size){
	__CC1101_WriteCMD(CC1101_SIDLE);
/*
	if((__CC1101_ReadReg(PKTCTRL1) & 0b11) != 0b00)		// if var lenght mode, size +1??? WTF, but it works
		size++;
*/
	/* Length select */
	uint8_t pktformat = __CC1101_ReadReg(PKTCTRL0) & 0b11;
	if(pktformat == CC1101_DYNAM_PKTLN)
		__CC1101_WriteReg(CC1101_TXFIFO, size);
	if(pktformat == CC1101_FIXED_PKTLN)
		__CC1101_WriteReg(PKTLEN, size);

	/* Address select */
	if((__CC1101_ReadReg(PKTCTRL1) & 0b11) != 0b00)
		__CC1101_WriteReg(CC1101_TXFIFO, txAddr);

	CC1101_Set_GDO2(CC1101_GDO_SYNCW_SENT);
	/* Transmitting */
	__CC1101_BurstWriteReg(CC1101_TXFIFO, size, data);

	__CC1101_WriteCMD(CC1101_STX);
	while(HAL_GPIO_ReadPin(GDO2_GPIO_Port, GDO2_Pin));
	CC1101_Set_GDO2(CC1101_GDO_HI_Z_STATE);
	__CC1101_WriteCMD(CC1101_SFTX);
	return CC1101_OK;
}

/*
 * Go to RX mode thought idle. In this mode tranceiver can receive packets
 */
uint8_t CC1101_GoToRX()
{
	__CC1101_WriteCMD(CC1101_SIDLE);
	__CC1101_WriteCMD(CC1101_SRX);        //start receive
	return CC1101_OK;
}

/*
 * Enter wakeup-on-radio mode with automatic packet detection
 */
uint8_t CC1101_GoToWOR()
{
	__CC1101_WriteCMD(CC1101_SIDLE);
	__CC1101_WriteCMD(CC1101_SWORRST);
	__CC1101_WriteCMD(CC1101_SWOR);
	return CC1101_OK;
}

/*
 * Check RX FIFO state
 * return value - number of bytes in RX FIFO or CRC flag
 */
uint8_t CC1101_IsDataAvailable()
{
	if (__CC1101_ReadReg(PKTCTRL0) & 0b00000100) // Check CRC bit is set
		{return __CC1101_ReadStatusRegs(CC1101_LQI) & 0b10000000;}
	else
		{return __CC1101_ReadStatusRegs(CC1101_RXBYTES) & 0x7F;}
}

/*
 * Read packet from RX FIFO
 * data - pointer to data array
 * RSSI - pointer to instant RSSI
 * LQI - pointer to instant LQI
 * return value - number of received bytes in RX FIFO
 */
uint8_t CC1101_ReadPacket(uint8_t* data, uint8_t* RSSI, uint8_t* LQI)
{


	uint8_t size;
	uint8_t status[2];

	if(__CC1101_ReadStatusRegs(CC1101_RXBYTES) & 0x7F)
	{
		size=__CC1101_ReadReg(CC1101_RXFIFO);
		__CC1101_BurstReadReg(CC1101_RXFIFO, size, data);
		__CC1101_BurstReadReg(CC1101_RXFIFO, 2, status);
		__CC1101_WriteCMD(CC1101_SFRX);
		__CC1101_WriteCMD(CC1101_SRX);
		*RSSI = status[0];
		*LQI = status[1];
		return size;
	}
	else
	{
		__CC1101_WriteCMD(CC1101_SFRX);
		__CC1101_WriteCMD(CC1101_SRX);
 		return 0;
	}
}

/*
 * Read status register
 * return value - status register
 */
uint8_t CC1101_ReadStatus()
{
	___CC1101_USER_CS_Low();
	uint8_t status_value = ___CC1101_USER_SPI_TxRx(0x3D);
	___CC1101_USER_CS_High();
	return status_value;
}
/*
 * Go to Iddle CMD
 */
uint8_t CC1101_ToIddle()
{
	__CC1101_WriteCMD(CC1101_SIDLE);
	return CC1101_OK;
}
/*
 * Go to sleep CMD
 */
uint8_t CC1101_ToSleep()
{
	__CC1101_WriteCMD(CC1101_SIDLE);
	__CC1101_WriteCMD(CC1101_SPWD);
	return CC1101_OK;
}
/* set settings functions */


/* Set ISM band . Need for correct PATABLE
 * band - type of band, see below
 */
void CC1101_SetISMband(uint8_t band){
	/*
	 * Deprecated by CC1101_SetBaseFreq(float), although the second still doesn't configure the PATABLES registers, so it is still needed.
	 */
    uint8_t freq2, freq1, freq0;
   const uint8_t* patable;

    switch (band)
    {
        case CC1101_MHZ315:
                    freq2=0x0C;
                    freq1=0x1D;
                    freq0=0x89;
                    patable = patable_power_315;
                    break;
        case CC1101_MHZ434:                                                          //433.92MHz
                    freq2=0x10;
                    freq1=0xB0;
                    freq0=0x71;
                    patable = patable_power_433;
                    break;
        case CC1101_MHZ868:                                                          //868.3MHz
                    freq2=0x21;
                    freq1=0x65;
                    freq0=0x6A;
                    patable = patable_power_868;
                    break;
        case CC1101_MHZ915:
                    freq2=0x23;
                    freq1=0x31;
                    freq0=0x3B;
                    patable = patable_power_915;
                    break;
        default:                                                          //868.3MHz
					freq2=0x21;
					freq1=0x65;
					freq0=0x6A;
					patable = patable_power_868;
					break;
    }
    __CC1101_WriteReg(FREQ2,freq2);
    __CC1101_WriteReg(FREQ1,freq1);
    __CC1101_WriteReg(FREQ0,freq0);
    __CC1101_BurstWriteReg(CC1101_PATABLE, 8,patable);

}

/* Set Base Frequency
 * mhz - freq in MHz
 */
void CC1101_SetBaseFreq(float mhz)
{
	uint8_t freq2 = 0;
	uint8_t freq1 = 0;
	uint8_t freq0 = 0;


	for (uint8_t i = 0; i==0;){
	if (mhz >= 26){
	mhz-=26;
	freq2+=1;
	}
	else if (mhz >= 0.1015625){
	mhz-=0.1015625;
	freq1+=1;
	}
	else if (mhz >= 0.00039675){
	mhz-=0.00039675;
	freq0+=1;
	}
	else{i=1;}
	}
	if (freq0 > 255){freq1+=1;freq0-=256;}

	__CC1101_WriteReg(FREQ2, freq2);
	__CC1101_WriteReg(FREQ1, freq1);
	__CC1101_WriteReg(FREQ0, freq0);

}

/* Set synq word
 * synqword - 16-bit synq word
 */
void CC1101_SetSynqWord(uint16_t synqword)
{
	__CC1101_WriteReg(SYNC1, synqword>>8);
	__CC1101_WriteReg(SYNC0, (uint8_t)synqword);
}

/* Set channel
 * channel - number of channel
 */
void CC1101_SetChannel(uint8_t channel){
	__CC1101_WriteReg(CHANNR, channel);
}

/* Set modulation
 * modulation - refer to CC1101.h file to choose correct value
 */
void CC1101_SetModulation(uint8_t modulation){
	uint8_t data = __CC1101_ReadReg(MDMCFG2);
	data &= 0b10001111;
	data |= modulation<<4;
	__CC1101_WriteReg(MDMCFG2, data);
}

/* Set attenuation in RX mode
 * attenuation - refer to CC1101.h file to choose correct value
 */
void CC1101_SetAttenuator(uint8_t attenuation){
	uint8_t data = __CC1101_ReadReg(FIFOTHR);
	data &= 0b11001111;
	data |= attenuation<<4;
	__CC1101_WriteReg(FIFOTHR, data);
}

/* Set transmit power
 * txPower - refer to CC1101.h file to choose correct value
 */
void CC1101_SetTXPower(uint8_t txPower){
	
	//__CC1101_BurstWriteReg(CC1101_PATABLE, 1, &txPower);
	uint8_t data = __CC1101_ReadReg(FREND0);
	data &= 0b11111100;
	data |= txPower;
}

/* Set addressation mode
 * addressationMode - refer to CC1101.h file to choose correct value
 * devAddr - device address(8 bit)
 * txAddr - address mark in packet(8 bit)
 */
void CC1101_SetAddressation(uint8_t addressationMode, uint8_t devAddr, uint8_t NewTxAddress){

	txAddr = NewTxAddress;

	uint8_t data = __CC1101_ReadReg(PKTCTRL1);
	data &= 0b11111100;
	data |= addressationMode;
	__CC1101_WriteReg(PKTCTRL1, data);


	__CC1101_WriteReg(ADDR, devAddr);
}

/* Set packet length mode
 * Lmode - refer to CC1101.h file to choose correct value
 */
void CC1101_SetPacketLengthMode(uint8_t Lmode){
	if(Lmode == CC1101_FIXED_PKTLN)
	{
		uint8_t data = 0b11111100 & __CC1101_ReadReg(PKTCTRL0);
		data |= 0b00;
		__CC1101_WriteReg(PKTCTRL0, data);

	}
	if(Lmode == CC1101_DYNAM_PKTLN)
	{
		__CC1101_WriteReg(PKTLEN, 0xFF);
		uint8_t data = 0b11111100 & __CC1101_ReadReg(PKTCTRL0);
		data |= 0b01;
		__CC1101_WriteReg(PKTCTRL0, data);
	}

}

/* Set Autoflush mode
 * Fmode - refer to CC1101.h file to choose correct value
 */
void CC1101_SetAutoFlushRX(uint8_t Fmode)
{
	uint8_t data = __CC1101_ReadReg(PKTCTRL1);
	data &= 0b11110111;
	data |= Fmode<<3;
	__CC1101_WriteReg(PKTCTRL1, data);
}
/* Set calibration mode
 * Cal_mode - refer to CC1101.h file to choose correct value
 */
void CC1101_SetCalibr(uint8_t Cal_mode)
{
	uint8_t data = __CC1101_ReadReg(MCSM0);
	data &= 0b11001111;
	data |= Cal_mode<<4;
	__CC1101_WriteReg(MCSM0, data);
}

/* Set additional mark status
 * Addmode - refer to CC1101.h file to choose correct value
 */
void CC1101_SetAddStatus(uint8_t Addmode)
{
	uint8_t data = __CC1101_ReadReg(PKTCTRL1);
	data &= 0b11111011;
	data |= Addmode<<2;
	__CC1101_WriteReg(PKTCTRL1, Addmode);
}

/* Set data rate
 * datarate - refer to CC1101.h file to choose correct value
 */
void CC1101_SetDataRate(uint16_t datarate){
	__CC1101_WriteReg(MDMCFG3, (uint8_t)datarate);
	uint8_t data = __CC1101_ReadReg(MDMCFG4) & 0b11110000;
	data |= (datarate>>8) & 0b1111;
	__CC1101_WriteReg(MDMCFG4, data);

}

/* Set preambule minimal size
 * preamb - refer to CC1101.h file to choose correct value
 */
void CC1101_SetPreambuleMinSize(uint8_t preamb)
{
	uint8_t data = __CC1101_ReadReg(MDMCFG1) & 0b10001111;
	data |= preamb << 4;
	__CC1101_WriteReg(MDMCFG1, data);
}

/* Set CRC mode
 * CRCmode - refer to CC1101.h file to choose correct value
 */
void CC1101_SetCRCmode(uint8_t CRCmode)
{
	uint8_t data = __CC1101_ReadReg(PKTCTRL0) & 0b11111011;
	data |= CRCmode << 2;
	__CC1101_WriteReg(PKTCTRL0, data);
}

/* Set data whitening
 * whitening - refer to CC1101.h file to choose correct value
 */
void CC1101_SetWhitening(uint8_t whitening)
{
	uint8_t data = __CC1101_ReadReg(PKTCTRL0) & 0b10111111;
	data |= whitening << 6;
	__CC1101_WriteReg(PKTCTRL0, data);
}

/* Set forward error correction
 * mode - refer to CC1101.h file to choose correct value
 * only available in fixed-size packet mode
 */
void CC1101_SetFEC(uint8_t FECmode)
{
	uint8_t data = __CC1101_ReadReg(MDMCFG1) & 0b01111111;
	data |= FECmode << 7;
	__CC1101_WriteReg(MDMCFG1, data);
}

/* Set preambule quality indicator treshold
 * mode - refer to CC1101.h file to choose correct value
 */
void CC1101_SetPQI(uint8_t PQI){
	uint8_t data = __CC1101_ReadReg(PKTCTRL1) & 0b00011111;
	data |= PQI << 5;
	__CC1101_WriteReg(PKTCTRL1, data);
}

/* Set deviation(1/2 of TX_bw)
 * d - deviation in KHz
 */
void CC1101_setDeviation(float d){
	float f = 1.586914;
	float v = 0.19836425;
	int c = 0;

	if (d > 380.859375) {d = 380.859375;}
	if (d < 1.586914) {d = 1.586914;}

	for (int i = 0; i<255; i++){
		f+=v;
		if (c==7){v*=2;c=-1;i+=8;}
		if (f>=d){c=i;i=255;}
		c++;
	}
	__CC1101_WriteReg(DEVIATN,c);
}

/* Set carrier sense threshold */
void CC1101_SetCS_Thr(uint8_t thr)
{
	uint8_t data = __CC1101_ReadReg(PKTCTRL0) & 0b11001111;
	data |= thr << 4;
	__CC1101_WriteReg(PKTCTRL0, data);
}

/* non-user functions */
uint8_t __CC1101_ReadReg(uint8_t addr)
{
	uint8_t data = 0;
	addr &= 0b00111111;
	___CC1101_USER_CS_Low();
	___CC1101_USER_SPI_TxRx((1<<7) | addr);
	data = ___CC1101_USER_SPI_TxRx(0x00);
	___CC1101_USER_CS_High();
	return data;
}
void __CC1101_WriteReg(uint8_t addr, uint8_t data)
{
	addr &= 0b00111111;
	___CC1101_USER_CS_Low();
	___CC1101_USER_SPI_TxRx(addr);
	___CC1101_USER_SPI_TxRx(data);
	___CC1101_USER_CS_High();
}
void __CC1101_BurstReadReg(uint8_t addr, uint8_t lth, uint8_t* data)
{
	//addr &= 0b00111111;
	___CC1101_USER_CS_Low();
	___CC1101_USER_SPI_TxRx( (1<<7) | (1<<6) | addr); //Read and burst bits

	for(uint8_t n = 0; n < lth; n++)
	{
		data[n] = ___CC1101_USER_SPI_TxRx(0x00);
	}

	___CC1101_USER_CS_High();
}
void __CC1101_BurstWriteReg(uint8_t addr, uint8_t lth, uint8_t* data)
{
	addr &= 0b00111111;
	___CC1101_USER_CS_Low();
	___CC1101_USER_SPI_TxRx( (1<<6) | addr);		//Write and burst bits

	for(uint8_t n = 0; n < lth; n++)
	{
		___CC1101_USER_SPI_TxRx(data[n]);
	}

	___CC1101_USER_CS_High();
}
void __CC1101_WriteCMD(uint8_t cmd)
{
	___CC1101_USER_CS_Low();
	___CC1101_USER_SPI_TxRx(cmd);
	___CC1101_USER_CS_High();

}
uint8_t __CC1101_ReadStatusRegs(uint8_t addr)
{
  uint8_t value,temp;
  temp = addr | (1<<7) | (1<<6);
  ___CC1101_USER_CS_Low();
  ___CC1101_USER_SPI_TxRx(temp);
  value= ___CC1101_USER_SPI_TxRx(0);
  ___CC1101_USER_CS_High();
  return value;
}
