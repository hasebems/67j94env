/*
 * File:   i2cdevice.c
 * Author: M.Hasebe
 *   for PIC18F67J94 / 87J94
 * Created on 2016/10/08
 */


#include	"system.h"
//#include	"config.h"
//  Set H/W
#define     USE_I2C_LPS25H					0   //  Pressure Sensor
#define     USE_I2C_MPR121                  0   //  Touch Sensor
#define		USE_I2C_CY8CMBR3110             0   //  Touch Sensor(Cypress)
#define     USE_I2C_ADXL345                 0   //  Accelerator Sensor
#define     USE_I2C_PCA9685                 0   //  LED Driver
#define     USE_I2C_ADS1015                 0   //  ADC
#define     USE_I2C_ACM1602N1               0   //  LCD Module
#define     USE_I2C_AQM0802A                1   //  LCD Module
#define		USE_USART_RX_AS_MIDI			0	//	Usart Rx

//-------------------------------------------------------------------------
//			Constants
//-------------------------------------------------------------------------
//static const unsigned char GPIO_EXPANDER_ADDRESS = 0x3e;
static const unsigned char LPS25H_ADDRESS = 0x5d;
static const unsigned char LPS331AP_ADDRESS = 0x5d;
static const unsigned char MPR121_ADDRESS = 0x5a;
static const unsigned char LED_BLINKM_ADDRESS = 0x09;
static const unsigned char ADS1015_ADDRESS = 0x48;
//static const unsigned char LED_ADA88_ADDRESS = 0x70;
static const unsigned char ADXL345_ADDRESS = 0x1d;
static const unsigned char ADXL345_ADDRESS2 = 0x53;
static const unsigned char CAP_SENSE_ADDRESS = 0x37;
static const unsigned char PCA9685_ADDRESS = 0x40;
static const unsigned char ACM1602N1_ADDRESS = 0x50;
static const unsigned char AQM0802A_ADDRESS = 0x3e;

// I2C Bus Control Definition
#define I2C_WRITE_CMD 0
#define I2C_READ_CMD 1

#define	I2C_SSPSTAT		SSP1STAT
#define	I2C_SSPADD		SSP1ADD
#define	I2C_SSPCON1		SSP1CON1
#define	I2C_SSPCON1BITS	SSP1CON1bits
#define	I2C_SSPCON2		SSP1CON2
#define	I2C_SSPCON2BITS	SSP1CON2bits

bool i2cErr;

//-------------------------------------------------------------------------
//			I2C Basic Functions
//-------------------------------------------------------------------------
void initI2c( void )	//	16MHz
{
    I2C_SSPSTAT = 0b00000000;      // I2C 400kHz
# if ( _XTAL_FREQ == 16000000 )
    I2C_SSPADD = 0x09;             // I2Cbus Baud rate,  16MHz/((I2C_SSPADD + 1)*4) = 400kHz -> 0x09
#elif ( _XTAL_FREQ == 48000000 )
	I2C_SSPADD = 0x1d;             // I2Cbus Baud rate,  48MHz/((I2C_SSPADD + 1)*4) = 400kHz -> 0x1d, 100kHz -> 0x77
#elif ( _XTAL_FREQ == 64000000 )
	I2C_SSPADD = 0x27;             // I2Cbus Baud rate,  64MHz/((I2C_SSPADD + 1)*4) = 400kHz -> 0x27
#else
	コンパイルエラー
#endif
    I2C_SSPCON1 = 0b00001000;      // I2C enable, Master Mode
	I2C_SSPCON2 = 0x00;
	i2cErr = false;
	I2C_SSPCON1BITS.SSPEN = 1;		//	I2C enable
}
//-------------------------------------------------------------------------
void quitI2c( void )
{
    I2C_SSPCON1BITS.SSPEN = 0;      // I2C disable
}
//-------------------------------------------------------------------------
// I2C通信がビジー状態を脱するまで待つ
int i2c_checkWrite( bool target ){
	volatile int cnt=0;
    while ( I2C_SSPCON2BITS.ACKSTAT || ( I2C_SSPSTAT & 0x05 ) ){	//	Buffer Full Check
		if ( cnt++ > 1000 ){
			i2cErr = i2cErr||target;
			return (int)(I2C_SSPCON2 & 0x5f);
		}
	}
	return 0;
}
//-------------------------------------------------------------------------
int i2c_checkRead( bool target ){
	volatile int cnt=0;
    while ( I2C_SSPSTAT & 0x01 ){	//	Buffer Full Check
		if ( cnt++ > 1000 ){
			i2cErr = i2cErr||target;
			return (int)(I2C_SSPCON2 & 0x5f);
		}
	}
	return 0;
}
//-------------------------------------------------------------------------
int i2c_check( bool target ){
	volatile int cnt=0;
    while ( ( I2C_SSPCON2 & 0x1F ) || ( I2C_SSPSTAT & 0x05 ) ){	//	Buffer Full Check
		if ( cnt++ > 1000 ){
			i2cErr = i2cErr||target;
			return (int)(I2C_SSPCON2 & 0x5f);
		}
	}
	return 0;
}
//-------------------------------------------------------------------------
int i2c_wait( bool target ){
	volatile int cnt=0;
    while ( ( I2C_SSPCON2 & 0x5F ) || ( I2C_SSPSTAT & 0x04 ) ){
		if ( cnt++ > 1000 ){
			i2cErr = i2cErr||target;
			return (int)(I2C_SSPCON2 & 0x5f);
		}
	}
	return 0;
}
//-------------------------------------------------------------------------
void i2c_err(void)
{
	I2C_SSPCON1BITS.WCOL=0;
	I2C_SSPCON2BITS.PEN = 1;
	while(I2C_SSPCON2BITS.PEN);
}

//-------------------------------------------------------------------------
//			I2c Device Access Functions
//-------------------------------------------------------------------------
int i2c_checkAdrs( unsigned char adrs )
{
	int err = i2c_check(true);
    if (err){i2c_err(); return err;}

	//	Start I2C
	I2C_SSPCON2BITS.SEN = 1;       //  Start Condition Enabled bit
	while(I2C_SSPCON2BITS.SEN);

	SSP1BUF = (adrs<<1) | I2C_WRITE_CMD;
	err = i2c_checkWrite(false);
	if (err){i2c_err(); return -1;}

	I2C_SSPCON2BITS.PEN = 1;       // Stop Condition Enable bit
	while(I2C_SSPCON2BITS.PEN);

	return 0;
}
//-------------------------------------------------------------------------
int writeI2c( unsigned char adrs, unsigned char data )
{
	int	err = 0;

	err = i2c_check(false);
	if (err){i2c_err(); return 1;}

	I2C_SSPCON2BITS.SEN = 1;       //  Start Condition Enabled bit
	while(I2C_SSPCON2BITS.SEN);

	SSP1BUF = (adrs<<1) | I2C_WRITE_CMD;
	err = i2c_checkWrite(false);
    if (err){i2c_err(); return 1;}

	SSP1BUF = data;
	err = i2c_checkWrite(false);
    if (err){i2c_err(); return 1;}

	I2C_SSPCON2BITS.PEN = 1;       // Stop Condition Enable bit
	while(I2C_SSPCON2BITS.PEN);

	return 0;
}
//-------------------------------------------------------------------------
int writeI2cWithCmd( unsigned char adrs, unsigned char cmd, unsigned char data )
{
	int	err = 0;

	err = i2c_check(false);
	if (err){i2c_err(); return 1;}

	//	Start I2C
	I2C_SSPCON2BITS.SEN = 1;       //  Start Condition Enabled bit
	while(I2C_SSPCON2BITS.SEN);

	SSP1BUF = (adrs<<1) | I2C_WRITE_CMD;
	err = i2c_checkWrite(false);
	if (err){i2c_err(); return 1;}

	SSP1BUF = cmd;
	err = i2c_checkWrite(false);
	if (err){i2c_err(); return 1;}

	SSP1BUF = data;
	err = i2c_checkWrite(false);
	if (err){i2c_err(); return 1;}

	I2C_SSPCON2BITS.PEN = 1;       // Stop Condition Enable bit
	while(I2C_SSPCON2BITS.PEN);

	return 0;
}
//-------------------------------------------------------------------------
int writeI2cWithCmdAndMultiData( unsigned char adrs, unsigned char cmd, unsigned char* data, int length )
{
	int i=0, err=0;

	err = i2c_check(false);
    if (err){i2c_err(); return 1;}

	//	Start I2C
	I2C_SSPCON2BITS.SEN = 1;       //  Start Condition Enabled bit
	while(I2C_SSPCON2BITS.SEN);

	SSP1BUF = (adrs<<1) | I2C_WRITE_CMD;
	err = i2c_checkWrite(false);
    if (err){i2c_err(); return 1;}

	SSP1BUF = cmd;
	err = i2c_checkWrite(false);
    if (err){i2c_err(); return 1;}

	while (i<length){
		SSP1BUF = *(data+i);
		err = i2c_checkWrite(false);
		if (err){i2c_err(); return 1;}
		i++;
	}

	I2C_SSPCON2BITS.PEN = 1;       // Stop Condition Enable bit
	while(I2C_SSPCON2BITS.PEN);

	return 0;
}
//-------------------------------------------------------------------------
int readI2cWithCmd( unsigned char adrs, unsigned char cmd, unsigned char* data, int length )
{
	int i=0, err=0;

	err = i2c_check(true);
    if (err){i2c_err(); return err;}

	//	Start I2C
	I2C_SSPCON2BITS.SEN = 1;       //  Start Condition Enabled bit
	while(I2C_SSPCON2BITS.SEN);

	SSP1BUF = (adrs<<1) | I2C_WRITE_CMD;
	err = i2c_checkWrite(false);
	if (err){i2c_err(); return 34;}	//	errが32以降である理由: 冒頭の i2c_check では 0x20 が立たないため

	SSP1BUF = cmd;
	err = i2c_checkWrite(false);
	if (err){i2c_err(); return 35;}

    I2C_SSPCON2BITS.RSEN = 1;      //  Start Condition Enabled bit
	while(I2C_SSPCON2BITS.RSEN);

	SSP1BUF = (adrs<<1) | I2C_READ_CMD;
	err = i2c_checkWrite(false);
	if (err){i2c_err(); return 36;}

	while (i<length){

		I2C_SSPCON2BITS.RCEN = 1;
		//while(SSPCON2bits.RCEN==1);
		err = i2c_wait(false);
		if (err){i2c_err(); return 38;}

		*(data+i) = SSP1BUF;
		err = i2c_check(false);
		if (err){i2c_err(); return 39;}

		if ( length > i+1 )	I2C_SSPCON2BITS.ACKDT = 0;     // ACK
		else				I2C_SSPCON2BITS.ACKDT = 1;     // NO_ACK

		I2C_SSPCON2BITS.ACKEN = 1;
		//while(SSPCON2bits.ACKEN==1);
		err = i2c_check(false);
		if (err){i2c_err(); return err+i;}

		i++;
	}

	I2C_SSPCON2BITS.PEN = 1;       // Stop Condition Enable bit
	while(I2C_SSPCON2BITS.PEN);
	PIR1bits.SSP1IF = 0;

	return 0;
}

//-------------------------------------------------------------------------
//			LPS25H (Pressure Sencer : I2c Device)
//-------------------------------------------------------------------------
#if USE_I2C_LPS25H
//	for Pressure Sencer
#define		PRES_SNCR_RESOLUTION		0x10
#define		PRES_SNCR_PWRON				0x20
#define		PRES_SNCR_START				0x21
#define		PRES_SNCR_ONE_SHOT			0x01
#define		PRES_SNCR_RCV_DT_FLG		0x27
#define		PRES_SNCR_RCV_TMPR			0x01
#define		PRES_SNCR_RCV_PRES			0x02
#define		PRES_SNCR_DT_L				0x28
#define		PRES_SNCR_DT_M				0x29
#define		PRES_SNCR_DT_H				0x2a
//-------------------------------------------------------------------------
void LPS25H_init( void )
{
#if 1
	writeI2cWithCmd( LPS25H_ADDRESS, PRES_SNCR_PWRON, 0xc0 );	//	Power On
#else
	//	LPS331AP
	writeI2cWithCmd( LPS331AP_ADDRESS, PRES_SNCR_RESOLUTION, 0x6A );	//	Resolution
	writeI2cWithCmd( LPS331AP_ADDRESS, PRES_SNCR_PWRON, 0xf0 );	//	Power On
#endif
}
//-------------------------------------------------------------------------
int LPS25H_getPressure( int* retPrs )
{
	unsigned char	dt[3];
	float	tmpPrs = 0;
	int		err;

	err = readI2cWithCmd( LPS25H_ADDRESS, PRES_SNCR_DT_L|0x80, dt, 3 );

	if ( !err ){
		tmpPrs = (float)(((unsigned long)dt[2]<<16)|((unsigned long)dt[1]<<8)|dt[0]);
		tmpPrs = tmpPrs*10/4096;
		*retPrs = (int)tmpPrs;
	}
	else *retPrs = 0;

	return err;
}
#endif

//-------------------------------------------------------------------------
//			MPR121 (Touch Sencer : I2c Device)
//-------------------------------------------------------------------------
#if USE_I2C_MPR121
//	for Touch Sencer
#define		TCH_SNCR_TOUCH_STATUS1		0x00
#define		TCH_SNCR_TOUCH_STATUS2		0x01
#define 	TCH_SNCR_ELE_CFG			0x5e
#define 	TCH_SNCR_MHD_R				0x2b
#define 	TCH_SNCR_MHD_F				0x2f
#define 	TCH_SNCR_ELE0_T				0x41
#define 	TCH_SNCR_FIL_CFG1			0x5c
#define 	TCH_SNCR_FIL_CFG2			0x5d
#define 	TCH_SNCR_MHDPROXR			0x36
#define 	TCH_SNCR_EPROXTTH			0x59

// Threshold defaults
#define		E_THR_T      0x02	// Electrode touch threshold
#define		E_THR_R      0x01	// Electrode release threshold
#define		PROX_THR_T   0x02	// Prox touch threshold
#define		PROX_THR_R   0x02	// Prox release threshold
//-------------------------------------------------------------------------
void MPR121_init( void )
{
	int	i, j;

	//	Init Parameter
	// Put the MPR into setup mode
	writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_ELE_CFG, 0x00 );

    // Electrode filters for when data is > baseline
    const unsigned char gtBaseline[] = {
		0x01,  //MHD_R
		0x01,  //NHD_R
		0x00,  //NCL_R
		0x00   //FDL_R
	};
	for ( i=0; i<4; i++ )
		writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_MHD_R+i, gtBaseline[i] );


	// Electrode filters for when data is < baseline
	const unsigned char ltBaseline[] = {
        0x01,   //MHD_F
        0x01,   //NHD_F
        0xFF,   //NCL_F
        0x02    //FDL_F
	};
	for ( i=0; i<4; i++ )
		writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_MHD_F+i, ltBaseline[i] );

    // Electrode touch and release thresholds
    const unsigned char electrodeThresholds[] = {
        E_THR_T, // Touch Threshhold
        E_THR_R  // Release Threshold
	};

    for( j=0; j<12; j++ ){
		for ( i=0; i<2; i++ ){
			writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_ELE0_T+(j*2)+i, electrodeThresholds[i] );
    	}
	}

    // Proximity Settings
    const unsigned char proximitySettings[] = {
        0xff,   //MHD_Prox_R
        0xff,   //NHD_Prox_R
        0x00,   //NCL_Prox_R
        0x00,   //FDL_Prox_R
        0x01,   //MHD_Prox_F
        0x01,   //NHD_Prox_F
        0xff,   //NCL_Prox_F
        0xff,   //FDL_Prox_F
        0x00,   //NHD_Prox_T
        0x00,   //NCL_Prox_T
        0x00    //NFD_Prox_T
	};
    for ( i=0; i<11; i++ )
		writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_MHDPROXR+i, proximitySettings[i] );

    const unsigned char proxThresh[] = {
        PROX_THR_T, // Touch Threshold
        PROX_THR_R  // Release Threshold
	};
    for ( i=0; i<2; i++ )
		writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_EPROXTTH+i, proxThresh[i] );

	writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_FIL_CFG1, 0x10 );
	writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_FIL_CFG2, 0x24 );

    // Set the electrode config to transition to active mode
	writeI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_ELE_CFG, 0x86 );
}
//-------------------------------------------------------------------------
int MPR121_getTchSwData( unsigned char* retSw )
{
	unsigned char buf[2];
	int	err;

	err = readI2cWithCmd( MPR121_ADDRESS, TCH_SNCR_TOUCH_STATUS1, buf, 2 );

	if ( !err )	*retSw = buf[0];//	return (buf[1]<<8) | buf[0];
	else *retSw = 0;

	return err;
}
#endif

//-------------------------------------------------------------------------
//			ADXL345 (Acceleration Sencer : I2c Device)
//-------------------------------------------------------------------------
#if USE_I2C_ADXL345
//	for Acceleration Sencer
#define	ACCEL_SNCR_RATE				0x2c
#define ACCEL_SNCR_PWR_CTRL			0x2d
#define ACCEL_SNCR_DATA_FORMAT		0x31
//-------------------------------------------------------------------------
void ADXL345_init( unsigned char chipnum )
{
	unsigned char i2cadrs = ADXL345_ADDRESS;
	if ( chipnum == 1 ){ i2cadrs = ADXL345_ADDRESS2; }

	//	Start Access
	writeI2cWithCmd(i2cadrs,ACCEL_SNCR_PWR_CTRL,0x08);			//	Start Measurement
	writeI2cWithCmd(i2cadrs,ACCEL_SNCR_DATA_FORMAT,0x04);		//	Left Justified, 2g
#if 0	//	if Shaker
	writeI2cWithCmd(i2cadrs,ACCEL_SNCR_RATE,0x0b);				//	200Hz (5msec)
	writeI2cWithCmd(i2cadrs,ACCEL_SNCR_PWR_CTRL,0x08);			//	Start Measurement
	writeI2cWithCmd(i2cadrs,ACCEL_SNCR_DATA_FORMAT,0x05);		//	Left Justified, 4g
#endif
}
//-------------------------------------------------------------------------
int ADXL345_getAccel( unsigned char chipnum, signed short* value )
{
	unsigned short tmp;
	unsigned char reg[2];
	unsigned char i2cadrs = ADXL345_ADDRESS;
	int err;

	if ( chipnum == 1 ){ i2cadrs = ADXL345_ADDRESS2; }

	err = readI2cWithCmd(i2cadrs,0x32,reg,2);
	if (!err){
		tmp = reg[0];
		tmp |= (unsigned short)reg[1] << 8;
		*value = (signed short)tmp;
	}
	else return err;

	err = readI2cWithCmd(i2cadrs,0x34,reg,2);
	if (!err){
		tmp = reg[0];
		tmp |= (unsigned short)reg[1] << 8;
		*(value+1) = (signed short)tmp;
	}
	else return err;

	err = readI2cWithCmd(i2cadrs,0x36,reg,2);
	if (!err){
		tmp = reg[0];
		tmp |= (unsigned short)reg[1] << 8;
		*(value+2) = (signed short)tmp;
	}

	return err;
}
#endif

//-------------------------------------------------------------------------
//			ADS1015 (ADC Sencer : I2c Device)
//-------------------------------------------------------------------------
#if USE_I2C_ADS1015    //	for ADC Sencer
//-------------------------------------------------------------------------
void ADS1015_setNext( int adNum )
{
	unsigned char buf[2];

	buf[0] = 0xc1 + (adNum << 4);	//	start | single convert | AD number | +-6.144V
	buf[1] = 0xe3;					//	3300SPS | rest are default

    writeI2cWithCmdAndMultiData(ADS1015_ADDRESS,0x01,buf,2);
}
//-------------------------------------------------------------------------
void ADS1015_init( void )
{	//	Init Parameter
	ADS1015_setNext(0);
}
//-------------------------------------------------------------------------
int ADS1015_getVolume( signed short* value )
{
	unsigned char buf[2];
	signed short adValue;

	//	Check if converted
	int err = readI2cWithCmd(ADS1015_ADDRESS,0x01,buf,2);
	if ( err ){ return err;}
	if (!( buf[0] & 0x80 )){ return -1;}

	//	Read from ADC
    err =readI2cWithCmd(ADS1015_ADDRESS,0x00,buf,2);

	//	Make it 0-2047 range
	adValue = (signed short)(((unsigned short)buf[0] << 8) | (unsigned short)buf[1]);
	if ( adValue < 0 ){ adValue = 0; }
	adValue /= 16;
	*value = adValue;

	return err;
}
#endif

//-------------------------------------------------------------------------
//			Cap Sense CY8CMBR3110 (Touch Sencer : I2c Device)
//-------------------------------------------------------------------------
#if USE_I2C_CY8CMBR3110
#define		CONFIG_DATA_OFFSET		0
#define		CONFIG_DATA_SZ			128

#define		SENSOR_EN				0x00	//	Register Address
#define		SENSITIVITY0			0x08	//	Register Address
#define		SENSITIVITY1			0x09	//	Register Address
#define		SENSITIVITY2			0x0a	//	Register Address
#define		I2C_ADDR				0x51	//	Register Address
#define		CONFIG_CRC				0x7e	//	Register Address

#define		CTRL_CMD				0x86	//	Register Address
#define		POWER_ON_AND_FINISHED	0x00
#define		SAVE_CHECK_CRC			0x02
#define		DEVICE_RESET			0xff

#define		CTRL_CMD_ERR			0x89	//	Register Address

#define		FAMILY_ID_ADRS			0x8f	//	Register Address
#define		FAMILY_ID				0x9a
#define		DEVICE_ID_ADRS			0x90	//	Register Address
#define		DEVICE_ID_LOW			0x02
#define		DEVICE_ID_HIGH			0x0a

#define		TOTAL_WORKING_SNS		0x97	//	Register Address
#define		SNS_VDD_SHORT			0x9a	//	Register Address
#define		SNS_GND_SHORT			0x9c	//	Register Address
#define		BUTTON_STAT				0xaa	//	Register Address
//-------------------------------------------------------------------------
void MBR3110_init( void )
{
	writeI2cWithCmd(CAP_SENSE_ADDRESS,CTRL_CMD,POWER_ON_AND_FINISHED);
}
//-------------------------------------------------------------------------
int MBR3110_readData( unsigned char cmd, unsigned char* data, int length )
{
	int err;
	volatile int cnt = 0;

	while(1) {
		err = readI2cWithCmd(CAP_SENSE_ADDRESS,cmd,data,length);
		if ( err == 0 ) break;
		if ( ++cnt > 500 ){	//	if more than 500msec, give up and throw err
			return err;
		}
		__delay_ms(1);
	}
	return 0;
}
//-------------------------------------------------------------------------
int MBR3110_selfTest( unsigned char* result )
{
	int err;

	err = MBR3110_readData(TOTAL_WORKING_SNS,result,1);
	if ( err ){ return err; }

	return 0;
}
//-------------------------------------------------------------------------
void MBR3110_changeSensitivity( unsigned char data )		//	data : 0-3
{
	unsigned char regData2 = data & 0x03;
	regData2 |= (regData2 << 2);
	unsigned char regData4 = regData2 | (regData2<<4);

	if ( writeI2cWithCmd(CAP_SENSE_ADDRESS,SENSITIVITY0,regData4) ){
	if ( writeI2cWithCmd(CAP_SENSE_ADDRESS,SENSITIVITY1,regData4) ){
	if ( writeI2cWithCmd(CAP_SENSE_ADDRESS,SENSITIVITY2,regData2) ){
		return;
	}}}

}
//-------------------------------------------------------------------------
int MBR3110_checkWriteConfig( unsigned char checksumL, unsigned char checksumH )
{
	unsigned char data[2];
	int err;

	err = MBR3110_readData(CONFIG_CRC,data,2);
	if ( err ){ return err; }

	//	err=-1 means it's present config
	if (( data[0] == checksumL ) && ( data[1] == checksumH )){ return -1; }

	return 0;
}
//-------------------------------------------------------------------------
int MBR3110_readTouchSw( unsigned char* touchSw )
{
	int err;

	err = MBR3110_readData(BUTTON_STAT,touchSw,2);
	if ( err ){ return err; }

	return 0;
}
//-------------------------------------------------------------------------
int MBR3110_writeConfig( unsigned char* capSenseConfigData )
{
	unsigned char	data[CONFIG_DATA_SZ];
	int				err;

	//*** Step 1 ***
	//	Check Power On


	//	Check I2C Address
	err = MBR3110_readData(I2C_ADDR,data,1);
	if ( err != 0 ){ return err; }
	if ( data[0] != CAP_SENSE_ADDRESS ){ return -1; }

	//*** Step 2 ***
	err = MBR3110_readData(DEVICE_ID_ADRS,data,2);
	if ( err != 0 ){ return err; }
	if (( data[0] != DEVICE_ID_LOW ) || ( data[1] != DEVICE_ID_HIGH )){ return -2; }

	err = MBR3110_readData(FAMILY_ID_ADRS,data,1);
	if ( err != 0 ){ return err; }
	if ( data[0] != FAMILY_ID ){ return -3; }

	//*** Step 3 ***
	//	send Config Data
	err = writeI2cWithCmdAndMultiData(CAP_SENSE_ADDRESS,CONFIG_DATA_OFFSET,capSenseConfigData,CONFIG_DATA_SZ);
	if ( err != 0 ){ return err;}

	//	Write to flash
	err = writeI2cWithCmd(CAP_SENSE_ADDRESS,CTRL_CMD,SAVE_CHECK_CRC);
	if ( err != 0 ){ return err;}

	//	220msec Wait
	for ( int tm=0; tm<30; tm++ ){
		__delay_ms(10);
	}

	//	Check to finish writing
	err = readI2cWithCmd(CAP_SENSE_ADDRESS,CTRL_CMD_ERR,data,1);
	if ( data[0] != 0x00 ){ return -4;}

	//	Reset
	err = writeI2cWithCmd(CAP_SENSE_ADDRESS,CTRL_CMD,DEVICE_RESET);
	if ( err != 0 ){ return err;}

	//	100msec Wait
	for ( int tm=0; tm<10; tm++ ){
		__delay_ms(10);
	}

	//*** Step 4 ***
	//	Get Config Data
	err =  readI2cWithCmd(CAP_SENSE_ADDRESS,CONFIG_DATA_OFFSET,data,CONFIG_DATA_SZ);
	if ( err != 0 ){ return err; }

	//	Compare both Data
	for ( int i=0; i<CONFIG_DATA_SZ; i++ ){
		if ( capSenseConfigData[i] != data[i] ){ return CONFIG_DATA_SZ; }
	}

	return 0;
}
#endif

//-------------------------------------------------------------------------
//			PCA9685 (LED Driver : I2c Device)
//-------------------------------------------------------------------------
#if USE_I2C_PCA9685    //	for LED Driver
//-------------------------------------------------------------------------
void PCA9685_init( int chipNumber )
{
	//	Init Parameter
	writeI2cWithCmd( PCA9685_ADDRESS+chipNumber, 0x00, 0x00 );	//
	writeI2cWithCmd( PCA9685_ADDRESS+chipNumber, 0x01, 0x12 );	//	Invert, OE=high-impedance
}
//-------------------------------------------------------------------------
//		rNum, gNum, bNum : 0 - 4094  bigger, brighter
//-------------------------------------------------------------------------
int PCA9685_setFullColorLED( int chipNumber, int ledNum, unsigned short* color  )
{
    int err = 0;
    ledNum &= 0x03;

	for ( int i=0; i<3; i++ ){
		//	figure out PWM counter
		unsigned short colorCnt = *(color+i);
		colorCnt = 4095 - colorCnt;
		if ( colorCnt <= 0 ){ colorCnt = 1;}
		//	Set PWM On Timing
		err = writeI2cWithCmd( PCA9685_ADDRESS+chipNumber, (unsigned char)(0x06 + i*4 + ledNum*16),
				(unsigned char)(colorCnt & 0x00ff) );	//
        if ( err != 0 ){ return err; }
		err = writeI2cWithCmd( PCA9685_ADDRESS+chipNumber, (unsigned char)(0x07 + i*4 + ledNum*16),
				(unsigned char)((colorCnt & 0xff00)>>8) );	//
        if ( err != 0 ){ return err; }

		//	Set PWM Off Timing
		err = writeI2cWithCmd( PCA9685_ADDRESS+chipNumber, (unsigned char)(0x08 + i*4 + ledNum*16), 0 );	//
        if ( err != 0 ){ return err; }
        err = writeI2cWithCmd( PCA9685_ADDRESS+chipNumber, (unsigned char)(0x09 + i*4 + ledNum*16), 0 );	//
        if ( err != 0 ){ return err; }
	}
    return err;
}
#endif

//-------------------------------------------------------------------------
//			AC1602N1 (LCD Module : I2c Device)
//-------------------------------------------------------------------------
#if USE_I2C_ACM1602N1    //	for LCD Module
//-------------------------------------------------------------------------
void ACM1602N1_init( void )
{	//	Init Parameter
	writeI2cWithCmd( ACM1602N1_ADDRESS, 0x00, 0x01 );
    __delay_ms(5);
    writeI2cWithCmd( ACM1602N1_ADDRESS, 0x00, 0x38 );
    __delay_ms(5);
	writeI2cWithCmd( ACM1602N1_ADDRESS, 0x00, 0x0c );
    __delay_ms(5);
	writeI2cWithCmd( ACM1602N1_ADDRESS, 0x00, 0x06 );
}
//-------------------------------------------------------------------------
//
//-------------------------------------------------------------------------
int ACM1602N1_setString( int locate, unsigned char* str, int strNum  )
{
    int err = 0;

    if ( locate >= 32 ) return;
    else if ( locate >= 16 ){ locate = 0x30 + locate;}
    err = writeI2cWithCmd( ACM1602N1_ADDRESS, 0x00, 0x80+locate );
    if ( err != 0 ){ return err; }

    for ( int i=0; i<strNum; i++ ){
        err = writeI2cWithCmd( ACM1602N1_ADDRESS, 0x80, *(str+i) );
        if ( err != 0 ){ return err; }
    }
    return err;
}
#endif

//-------------------------------------------------------------------------
//			AQM0802A (LCD Module : I2c Device)
//				Caution!! 3.3 v only
//-------------------------------------------------------------------------
#if USE_I2C_AQM0802A    //	for LCD Module
//-------------------------------------------------------------------------
void AQM0802A_init( void )
{	//	Init Parameter
    for ( int i=0; i<4; i++ ){ __delay_ms(10);}
	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x38 );
    __delay_us(30);
	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x39 );
    __delay_us(30);
    writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x14 );
    __delay_us(30);
	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x70 );
    __delay_us(30);
	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x56 );
    __delay_us(30);
	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x6c );
	for ( int i=0; i<20; i++ ){ __delay_ms(10);}
	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x38 );
    __delay_us(30);
	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x0c );
	__delay_us(30);
	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x01 );
	__delay_us(30);
}
//-------------------------------------------------------------------------
//				write to LCD
//-------------------------------------------------------------------------
int AQM0802A_setAllString( unsigned char* str1, unsigned char* str2 )
{
    int err = 0;

	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0x80 );
    __delay_us(30);

	for ( int i=0; i<8; i++ ){
		err = writeI2cWithCmd( AQM0802A_ADDRESS, 0xc0, str1[i] );
        if ( err != 0 ){ return err; }
	}
	__delay_us(30);

	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, 0xc0 );
    __delay_us(30);

	for ( int i=0; i<8; i++ ){
		err = writeI2cWithCmd( AQM0802A_ADDRESS, 0xc0, str2[i] );
        if ( err != 0 ){ return err; }
	}

    return err;
}
//-------------------------------------------------------------------------
int AQM0802A_setString( int locate, unsigned char* str, int strNum, bool upper  )
{
    int err = 0;
	char startPoint;

    if ( locate >= 8 ) return;
	if ( locate + strNum > 8 ){ strNum = 8 - locate; }

	if ( upper == true ){ startPoint = (char)locate | 0x80; }
	else { startPoint = (char)locate | 0xc0; }

	writeI2cWithCmd( AQM0802A_ADDRESS, 0x80, startPoint );
    __delay_us(15);

	for ( int i=0; i<strNum; i++ ){
		err = writeI2cWithCmd( AQM0802A_ADDRESS, 0xc0, str[i] );
        if ( err != 0 ){ return err; }
	}

    return err;
}
//-------------------------------------------------------------------------
int AQM0802A_setStringUpper( int locate, unsigned char* str, int strNum  )
{
    return AQM0802A_setString(locate,str,strNum,true);
}
//-------------------------------------------------------------------------
int AQM0802A_setStringLower( int locate, unsigned char* str, int strNum  )
{
	return AQM0802A_setString(locate,str,strNum,false);
}
#endif
