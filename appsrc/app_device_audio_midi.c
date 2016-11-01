/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license),
please contact mla_licensing@microchip.com
*******************************************************************************/

/** INCLUDES *******************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "system.h"

#include "usb.h"
#include "usb_device_midi.h"


/*----------------------------------------------------------------------------*/
//
//      Macros
//
/*----------------------------------------------------------------------------*/
#define	MIDI_BUF_MAX		8
#define	MIDI_BUF_MAX_MASK	0x07;
#define	PORT_MAX_NUM		3

/** VARIABLES ******************************************************/
/* Some processors have a limited range of RAM addresses where the USB module
 * is able to access.  The following section is for those devices.  This section
 * assigns the buffers that need to be used by the USB module into those
 * specific areas.
 */
#if defined(FIXED_ADDRESS_MEMORY)
    #if defined(COMPILER_MPLAB_C18)
        #pragma udata DEVICE_AUDIO_MIDI_RX_DATA_BUFFER=DEVCE_AUDIO_MIDI_RX_DATA_BUFFER_ADDRESS
            static uint8_t ReceivedDataBuffer[64];
        #pragma udata DEVICE_AUDIO_MIDI_EVENT_DATA_BUFFER=DEVCE_AUDIO_MIDI_EVENT_DATA_BUFFER_ADDRESS
            static USB_AUDIO_MIDI_EVENT_PACKET midiData;
        #pragma udata
    #elif defined(__XC8)
        static uint8_t ReceivedDataBuffer[64] @ DEVCE_AUDIO_MIDI_RX_DATA_BUFFER_ADDRESS;
        static USB_AUDIO_MIDI_EVENT_PACKET midiData @ DEVCE_AUDIO_MIDI_EVENT_DATA_BUFFER_ADDRESS;
    #endif
#else
    static uint8_t ReceivedDataBuffer[64];
    static USB_AUDIO_MIDI_EVENT_PACKET midiData;
#endif

static USB_HANDLE USBTxHandle;
static USB_HANDLE USBRxHandle;

static bool sentNoteOff;

static USB_VOLATILE uint8_t msCounter;


static uint8_t		midiOutEvent[MIDI_BUF_MAX][3];
static int			midiOutEventReadPointer;
static int			midiOutEventWritePointer;
static uint8_t		oldMod;

static uint8_t		photoState[PORT_MAX_NUM];
static uint8_t		oldState[PORT_MAX_NUM];

static int ddebug;

/*----------------------------------------------------------------------------*/
//      Interrupt
/*----------------------------------------------------------------------------*/
void APP_Int1Msec( void )
{
#if 0
	photoState[0] = PORTA;
	photoState[1] = PORTB;
	photoState[2] = PORTD;
	TRISA = 0x00;
	TRISB = 0x00;
	TRISD = 0x00;

	if ( photoState[0] & 0x08 ) LED_On(LED_D4);
	else LED_Off(LED_D4);

	PORTA = 0xff;
	PORTB = 0xff;
	PORTD = 0xff;
	__delay_us(10);
	TRISA = 0xff;
	TRISB = 0xff;
	TRISD = 0xff;
#endif
}

/*********************************************************************
* Function: void APP_DeviceAudioMIDIInitialize(void);
*
* Overview: Initializes the demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceAudioMIDIInitialize()
{
    USBTxHandle = NULL;
    USBRxHandle = NULL;

    sentNoteOff = true;
    msCounter = 0;

    //enable the HID endpoint
    USBEnableEndpoint(USB_DEVICE_AUDIO_MIDI_ENDPOINT,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBRxHandle = USBRxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&ReceivedDataBuffer,64);

	for ( int i=0; i<MIDI_BUF_MAX; i++ ){
		midiOutEvent[i][0] = 0;
		midiOutEvent[i][1] = 0;
		midiOutEvent[i][2] = 0;
	}
	midiOutEventReadPointer = 0;
	midiOutEventWritePointer = 0;
	oldMod = 0;
	ddebug = 0;

	for ( int j=0; j<PORT_MAX_NUM; j++ ){
		photoState[j] = 0;
		oldState[j] = 0;
	}
}

/*********************************************************************
* Function: void APP_DeviceAudioMIDIInitialize(void);
*
* Overview: Initializes the demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceAudioMIDISOFHandler()
{
    if(msCounter != 0)
    {
        msCounter--;
    }
}

/*----------------------------------------------------------------------------*/
//      Set MIDI Buffer
/*----------------------------------------------------------------------------*/
void setMidiBuffer( uint8_t status, uint8_t dt1, uint8_t dt2 )
{
    //  for USB MIDI
    midiOutEvent[midiOutEventWritePointer][0] = status;
    midiOutEvent[midiOutEventWritePointer][1] = dt1;
    midiOutEvent[midiOutEventWritePointer][2] = dt2;
    midiOutEventWritePointer++;
    midiOutEventWritePointer &= MIDI_BUF_MAX_MASK;
}
/*----------------------------------------------------------------------------*/
//      MIDI Out
/*----------------------------------------------------------------------------*/
void midiOut( void )
{
	if ( USBHandleBusy(USBTxHandle) ){ return;}

	if ( midiOutEventReadPointer != midiOutEventWritePointer ){
		uint8_t	statusByte = midiOutEvent[midiOutEventReadPointer][0];

		midiData.Val = 0;   //must set all unused values to 0 so go ahead
                            //  and set them all to 0

		midiData.CableNumber = 0;
		midiData.CodeIndexNumber = statusByte >> 4;

        midiData.DATA_0 = statusByte;									// Status Byte
        midiData.DATA_1 = midiOutEvent[midiOutEventReadPointer][1];		// Data Byte 1
        midiData.DATA_2 = midiOutEvent[midiOutEventReadPointer][2];		// Data Byte 2
        USBTxHandle = USBTxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&midiData,4);

		midiOutEventReadPointer++;
		midiOutEventReadPointer &= MIDI_BUF_MAX_MASK;
	}
}

/*----------------------------------------------------------------------------*/
//		Application Job
/*----------------------------------------------------------------------------*/
void mainLoopJob( void )
{
	int i;

#if 0
//	if ( counter10msec%100 < 20 ){
//		LED_On(LED_D4);
//	}
//	else {
//		LED_Off(LED_D4);
//	}

	//	Check Photo Sensor State & Output MIDI
	if ( event10msec == true ){
		for ( int j=0; j<PORT_MAX_NUM; j++ ){
			uint8_t	pstt = photoState[j];
			if ( pstt != oldState[j] ){
				uint8_t	bitPtn = 0x01;
				for ( i=0; i<8; i++ ){
					if (( pstt & bitPtn ) && !( oldState[j] & bitPtn )){
						setMidiBuffer(0x90,0x3c+j*8+i,0x7f);
					}
					else if (!( pstt & bitPtn ) && ( oldState[j] & bitPtn )){
						setMidiBuffer(0x90,0x3c+j*8+i,0x00);
					}
					bitPtn << 1;
				}
				oldState[j] = pstt;
			}
		}
	}


#else
	unsigned char str[] = "<67J94> ";
	AQM0802A_setStringLower(0,str,8);

	if ( event100msec == true ){
		unsigned char s[8] = {0};
		uint16_t dt = measureTouchSenser(0);
		utoa(s,dt,10) ;         // カウント値を文字列に変換する
		AQM0802A_setStringUpper(0,s,8) ;         // 表示する

		uint8_t dt8 = (uint8_t)(dt>>3) & 0x7f;
		if ( dt8 != oldMod ){
			setMidiBuffer( 0xb0, 0x01, dt8 );
			oldMod = dt8;
		}
	}
#endif
}

/*********************************************************************
* Function: void APP_DeviceAudioMIDITasks(void);
*
* Overview: Keeps the Custom HID demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_DeviceAudioMIDIInitialize() and APP_DeviceAudioMIDIStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceAudioMIDITasks()
{
    /* If the device is not configured yet, or the device is suspended, then
     * we don't need to run the demo since we can't send any data.
     */
    if( (USBGetDeviceState() < CONFIGURED_STATE) ||
        (USBIsDeviceSuspended() == true))
    {
        return;
    }

    if(!USBHandleBusy(USBRxHandle))
    {
        //We have received a MIDI packet from the host, process it and then
        //  prepare to receive the next packet

        //INSERT MIDI PROCESSING CODE HERE

        //Get ready for next packet (this will overwrite the old data)
        USBRxHandle = USBRxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&ReceivedDataBuffer,64);
    }

	mainLoopJob();

	midiOut();

}
