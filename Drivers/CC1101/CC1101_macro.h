/*
 * CC1101_macro.h
 *
 *  Created on: Jan 11, 2022
 *      Author: Sergey
 *
 *  Modified on: 15.5.22
 *      Author: Marat
 */

#ifndef CC1101_CC1101_MACRO_H_
#define CC1101_CC1101_MACRO_H_

#include <stdint.h>

uint8_t CC1101_DefaultSettings[47] =
//433 MHz
		/* Default config:
		 * Base Freq: 		433.0
		 * Channel: 		0
		 * Modulation: 		GFSK
		 * TxPower: 		0 dBm
		 * Data rate: 		20 kBaud
		 * CRC: 			enabled
		 * Channel Spacing: 199.5 kHz
		 * RX filter BW: 	325 kHz
		 * Deviation: 		47.6 kHz
		 * Whitening:		Disabled
		 * Manchester: 		Disabled
		 * Mode: 			Variable length
		 * SynqWord:		D3 91
		 * Address check:	No address check
		 * Preabule:		30/32 synq word detected
		 * Attenuator: 		0 dBm
		 * Address:			0x00
		 */

/*
{
	    0x0F,  // IOCFG2              GDO2 Output Pin Configuration
	    0x2E,  // IOCFG1              GDO1 Output Pin Configuration
	    0x2E,  // IOCFG0              GDO0 Output Pin Configuration
	    0x07,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
	    0xD3,  // SYNC1               Sync Word, High Byte
	    0x91,  // SYNC0               Sync Word, Low Byte
	    0xFF,  // PKTLEN              Packet Length
	    0x04,  // PKTCTRL1            Packet Automation Control
	    0x05,  // PKTCTRL0            Packet Automation Control
	    0x00,  // ADDR                Device Address
	    0x00,  // CHANNR              Channel Number
	    0x08,  // FSCTRL1             Frequency Synthesizer Control
	    0x00,  // FSCTRL0             Frequency Synthesizer Control
	    0x10,  // FREQ2               Frequency Control Word, High Byte
	    0xA7,  // FREQ1               Frequency Control Word, Middle Byte
	    0x62,  // FREQ0               Frequency Control Word, Low Byte
	    0x59,  // MDMCFG4             Modem Configuration
	    0x93,  // MDMCFG3             Modem Configuration
	    0x13,  // MDMCFG2             Modem Configuration
	    0x22,  // MDMCFG1             Modem Configuration
	    0xF8,  // MDMCFG0             Modem Configuration
	    0x47,  // DEVIATN             Modem Deviation Setting
	    0x01,  // MCSM2               Main Radio Control State Machine Configuration
	    0x30,  // MCSM1               Main Radio Control State Machine Configuration
	    0x18,  // MCSM0               Main Radio Control State Machine Configuration
	    0x1D,  // FOCCFG              Frequency Offset Compensation Configuration
	    0x1C,  // BSCFG               Bit Synchronization Configuration
	    0xC7,  // AGCCTRL2            AGC Control
	    0x00,  // AGCCTRL1            AGC Control
	    0xB2,  // AGCCTRL0            AGC Control
	    0x28,  // WOREVT1             High Byte Event0 Timeout
	    0xA0,  // WOREVT0             Low Byte Event0 Timeout
	    0x78,  // WORCTRL             Wake On Radio Control
	    0xB6,  // FREND1              Front End RX Configuration
	    0x10,  // FREND0              Front End TX Configuration
	    0xEA,  // FSCAL3              Frequency Synthesizer Calibration
	    0x2A,  // FSCAL2              Frequency Synthesizer Calibration
	    0x00,  // FSCAL1              Frequency Synthesizer Calibration
	    0x1F,  // FSCAL0              Frequency Synthesizer Calibration
	    0x41,  // RCCTRL1             RC Oscillator Configuration
	    0x00,  // RCCTRL0             RC Oscillator Configuration
	    0x59,  // FSTEST              Frequency Synthesizer Calibration Control
	    0x7F,  // PTEST               Production Test
	    0x3F,  // AGCTEST             AGC Test
	    0x88,  // TEST2               Various Test Settings
	    0x31,  // TEST1               Various Test Settings
	    0x09  // TEST0               Various Test Settings


};
*/
//868  MHz

		/* Default config:
		 * Base Freq: 		868.29
		 * Channel: 		0
		 * Modulation: 		GFSK
		 * TxPower: 		0 dBm
		 * Data rate: 		20 kBaud
		 * CRC: 			enabled
		 * Channel Spacing: 199.5 kHz
		 * RX filter BW: 	101.562500 kHz
		 * Deviation: 		19.042969 kHz
		 * Whitening:		Disabled
		 * Manchester: 		Disabled
		 * Mode: 			Variable length
		 * SynqWord:		57 43
		 * Address check:	No address check
		 * Preabule:		30/32 synq word detected
		 * Attenuator: 		0 dBm
		 * Address:			0x00
		 */

{
        0x07,  // IOCFG2        GDO2 Output Pin Configuration
        0x2E,  // IOCFG1        GDO1 Output Pin Configuration
        0x80,  // IOCFG0        GDO0_Pin Output Pin Configuration
        0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
        0x57,  // SYNC1         Sync Word, High Byte
        0x43,  // SYNC0         Sync Word, Low Byte
        0x3E,  // PKTLEN        Packet Length
        0xD8,  // PKTCTRL1      Packet Automation Control
        0x45,  // PKTCTRL0      Packet Automation Control
        0xFF,  // ADDR          Device Address
        0x00,  // CHANNR        Channel Number
        0x08,  // FSCTRL1       Frequency Synthesizer Control
        0x00,  // FSCTRL0       Frequency Synthesizer Control
        0x21,  // FREQ2         Frequency Control Word, High Byte
        0x65,  // FREQ1         Frequency Control Word, Middle Byte
        0x6A,  // FREQ0         Frequency Control Word, Low Byte
        0xF5,  // MDMCFG4       Modem Configuration
        0x83,  // MDMCFG3       Modem Configuration
        0x13,  // MDMCFG2       Modem Configuration
        0xC0,  // MDMCFG1       Modem Configuration
        0xF8,  // MDMCFG0       Modem Configuration
        0x15,  // DEVIATN       Modem Deviation Setting
        0x07,  // MCSM2         Main Radio Control State Machine Configuration
        0x00,  // MCSM1         Main Radio Control State Machine Configuration
        0x18,  // MCSM0         Main Radio Control State Machine Configuration
        0x16,  // FOCCFG        Frequency Offset Compensation Configuration
        0x6C,  // BSCFG         Bit Synchronization Configuration
        0x03,  // AGCCTRL2      AGC Control
        0x40,  // AGCCTRL1      AGC Control
        0x91,  // AGCCTRL0      AGC Control
        0x02,  // WOREVT1       High Byte Event0 Timeout
        0x26,  // WOREVT0       Low Byte Event0 Timeout
        0x09,  // WORCTRL       Wake On Radio Control
        0x56,  // FREND1        Front End RX Configuration
        0x17,  // FREND0        Front End TX Configuration
        0xA9,  // FSCAL3        Frequency Synthesizer Calibration
        0x0A,  // FSCAL2        Frequency Synthesizer Calibration
        0x00,  // FSCAL1        Frequency Synthesizer Calibration
        0x11,  // FSCAL0        Frequency Synthesizer Calibration
        0x41,  // RCCTRL1       RC Oscillator Configuration
        0x00,  // RCCTRL0       RC Oscillator Configuration
        0x59,  // FSTEST        Frequency Synthesizer Calibration Control,
        0x7F,  // PTEST         Production Test
        0x3F,  // AGCTEST       AGC Test
        0x81,  // TEST2         Various Test Settings
        0x3F,  // TEST1         Various Test Settings
        0x0B   // TEST0         Various Test Settings
};

//----------------------[PATABLES]---------------------------------------------
/*PATABLES Registers presets for various frequencies. This values are the (suposed) optimal values for -30, -20, -15,
-10, 0, 5, 7, 10 dBm for each carrier frequency.
*/
const uint8_t patable_power_315[]  = {0x17,0x1D,0x26,0x69,0x51,0x86,0xCC,0xC3};
const uint8_t patable_power_433[]  = {0x6C,0x1C,0x06,0x3A,0x51,0x85,0xC8,0xC0};
const uint8_t patable_power_868[]  = {0x03,0x17,0x1D,0x26,0x50,0x86,0xCD,0xC0};
const uint8_t patable_power_915[]  = {0x0B,0x1B,0x6D,0x67,0x50,0x85,0xC9,0xC1};



#define CC1101_DEVICE_ID  0x3F

#define IOCFG2         0x00		//GDO2 Output Pin Configuration
#define IOCFG1         0x01		//GDO1 Output Pin Configuration
#define IOCFG0         0x02		//GDO0 Output Pin Configuration
#define FIFOTHR        0x03		//RX FIFO and TX FIFO Thresholds
#define SYNC1          0x04		//Sync Word, High Byte
#define SYNC0          0x05		//Sync Word, Low Byte
#define PKTLEN         0x06		//Packet Length
#define PKTCTRL1       0x07		//Packet Automation Control
#define PKTCTRL0       0x08		//Packet Automation Control
#define ADDR           0x09		//Device Address
#define CHANNR         0x0A		//Channel Number
#define FSCTRL1        0x0B		//Frequency Synthesizer Control
#define FSCTRL0        0x0C		//Frequency Synthesizer Control
#define FREQ2          0x0D		//Frequency Control Word, High Byte
#define FREQ1          0x0E		//Frequency Control Word, Middle Byte
#define FREQ0          0x0F		//Frequency Control Word, Low Byte
#define MDMCFG4        0x10		//Modem Configuration
#define MDMCFG3        0x11		//Modem Configuration
#define MDMCFG2        0x12		//Modem Configuration
#define MDMCFG1        0x13		//Modem Configuration
#define MDMCFG0        0x14		//Modem Configuration
#define DEVIATN        0x15		//Modem Deviation Setting
#define MCSM2          0x16		//Main Radio Control State Machine Configuration
#define MCSM1          0x17		//Main Radio Control State Machine Configuration
#define MCSM0          0x18		//Main Radio Control State Machine Configuration
#define FOCCFG         0x19		//Frequency Offset Compensation Configuration
#define BSCFG          0x1A		//Bit Synchronization Configuration
#define AGCCTRL2       0x1B		//AGC Control
#define AGCCTRL1       0x1C		//AGC Control
#define AGCCTRL0       0x1D		//AGC Control
#define WOREVT1        0x1E		//High Byte Event0 Timeout
#define WOREVT0        0x1F		//Low Byte Event0 Timeout
#define WORCTRL        0x20		//Wake On Radio Control
#define FREND1         0x21		//Front End RX Configuration
#define FREND0         0x22		//Front End TX Configuration
#define FSCAL3         0x23		//Frequency Synthesizer Calibration
#define FSCAL2         0x24		//Frequency Synthesizer Calibration
#define FSCAL1         0x25		//Frequency Synthesizer Calibration
#define FSCAL0         0x26		//Frequency Synthesizer Calibration
#define RCCTRL1        0x27		//RC Oscillator Configuration
#define RCCTRL0        0x28		//RC Oscillator Configuration
#define FSTEST         0x29		//Frequency Synthesizer Calibration Control
#define PTEST          0x2A		//Production Test
#define AGCTEST        0x2B		//AGC Test
#define TEST2          0x2C		//Various Test Settings
#define TEST1          0x2D		//Various Test Settings
#define TEST0          0x2E		//Various Test Settings

#define CC1101_SRES         0x30        // Reset chip.
#define CC1101_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
#define CC1101_SXOFF        0x32        // Turn off crystal oscillator.
#define CC1101_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
#define CC1101_SRX          0x34        // Enable RX.
#define CC1101_STX          0x35        // In IDLE state: Enable TX.
#define CC1101_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
#define CC1101_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST      0x3C        // Reset real time clock.
#define CC1101_SNOP         0x3D        // No operation. May be used to pad strobe commands to two

#define CC1101_PARTNUM      0x30
#define CC1101_VERSION      0x31
#define CC1101_FREQEST      0x32
#define CC1101_LQI          0x33
#define CC1101_RSSI         0x34
#define CC1101_MARCSTATE    0x35
#define CC1101_WORTIME1     0x36
#define CC1101_WORTIME0     0x37
#define CC1101_PKTSTATUS    0x38
#define CC1101_VCO_VC_DAC   0x39
#define CC1101_TXBYTES      0x3A
#define CC1101_RXBYTES      0x3B

#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F
#define CC1101_RXFIFO       0x3F


/* Bit masks */
#define CC1101_PKTCTRL1_ADDR 0b00000011



#endif /* CC1101_CC1101_MACRO_H_ */
