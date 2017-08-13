// USB IR REMOCON KIT
//	ver 2.1	2013．03．26 DAIKIN 33kHz対応
//	ver 2	2013．03．08 エアコン対応
//	ver 1	初版


/********************************************************************
 FileName:		main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the 鼎ompany・ for its PICｮ Microcontroller is intended and
 supplied to you, the Company痴 customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN 鄭S IS・CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
********************************************************************/

#ifndef IRREMOCONMAIN_C
#define IRREMOCONMAIN_C

/** INCLUDES *******************************************************/
#include <adc.h>
#include <timers.h>
#include <pwm.h>
#include <string.h>
#include "./USB/usb.h"
#include "HardwareProfile.h"
#include "./USB/usb_function_hid.h"
#include "eeprom.h"

/** CONFIGURATION **************************************************/
#pragma config CPUDIV = NOCLKDIV
#pragma config USBDIV = OFF
#pragma config FOSC   = HS
#pragma config PLLEN  = ON
#pragma config FCMEN  = OFF
#pragma config IESO   = OFF
#pragma config PWRTEN = OFF
#pragma config BOREN  = OFF
#pragma config BORV   = 30
#pragma config WDTEN  = OFF
#pragma config WDTPS  = 32768
#pragma config MCLRE  = OFF
#pragma config HFOFST = OFF
#pragma config STVREN = ON
#pragma config LVP    = OFF
#pragma config XINST  = OFF
#pragma config BBSIZ  = OFF
#pragma config CP0    = OFF
#pragma config CP1    = OFF
#pragma config CPB    = OFF
#pragma config WRT0   = OFF
#pragma config WRT1   = OFF
#pragma config WRTB   = OFF
#pragma config WRTC   = OFF
#pragma config EBTR0  = OFF
#pragma config EBTR1  = OFF
#pragma config EBTRB  = OFF


/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
int RemoconReceiveData(void);
int RemoconOutData(void);

/** DECLARATIONS ***************************************************/
#define MODE_MOUSE		0
#define MODE_KEYBOARD	1
#define	MODE_VOLUME		2
//#define	MODE_JOYSTICK	2
#define EEPROM_DATA_NUM			20	/* EEPROM格納データ数 */
#define EEPROM_DATA_SIZE		10	/* EEPROM格納１データのサイズ */
#define EEPROM_DATA_TOTAL_SIZE		EEPROM_DATA_NUM*EEPROM_DATA_SIZE	/* EEPROM格納データのトータルサイズ */
#define	EEPROM_DATA_MODE		0	//	0:モード
#define	EEPROM_DATA_VALUE		1	//	1:値
#define	EEPROM_DATA_MODIFIER	2	//	2:Modifier（キーボード用）
#define EEPROM_DATA_READERCODE	3	/* 赤外線リーダコード格納位置 */
#define EEPROM_DATA_DATACODE	4	/* 赤外線データコード格納位置 */

#define MASTER_MOUSE_SPEED		50	//	Mouseの移動速度の調整値
#define MASTER_WHEEL_SPEED		1000

#define TX_BUFFER_SIZE 64			/* USB送信バッファサイズ */
#define RX_BUFFER_SIZE 64			/* USB受信バッファサイズ */

#define MOVE_OFF	0
#define MOVE_ON		1

#define NUM_OF_PINS		12

#define PIN1	PORTCbits.RC5
#define	PIN2	PORTCbits.RC4
#define	PIN3	PORTCbits.RC3
#define PIN4	PORTCbits.RC6
#define	PIN5	PORTCbits.RC7
#define	PIN6	PORTBbits.RB7
#define	PIN7	PORTBbits.RB6
#define	PIN8	PORTBbits.RB5
#define	PIN9	PORTBbits.RB4
#define	PIN10	PORTCbits.RC2
#define PIN11	PORTCbits.RC1
#define	PIN12	PORTCbits.RC0

/* Timer0 0.1ms */
/* 100us / ( 0.021us x 4 ) = 1200  48MHz = 1/48us = 0.021us */
/* 1200 / 8 = 150    プリスケーラ=8*/
/* 0x10000 - 0x96 = 0xFF88    16bitカウンタ*/
#define WRITE_TIMER0_COUNT        0xFF6A        //Timer0の時間 

//周期38kHz=26us
// 38kHz = ( 78 + 1 ) * 4 * 0.0208us(48MHz) * 4(プリスケーラ) = 26.2912us
#define PWM_PERIOD	0x4E 	/* Period 78 = 0x4E */
#define	PWM_OFF		0
#define PWM_50		0x9E	/* PWM Duty 50% = (Period + 1) * 4 * 0.5 */
#define PWM_100		0x13C	/* PWM Duty 100% = (Period + 1) * 4 * 1.0 */

//周期33kHz=30us
// 33kHz = ( 89 + 1 ) * 4 * 0.0208us(48MHz) * 4(プリスケーラ) = 29.952us
#define PWM_PERIOD_33k	0x59 	/* Period 89 = 0x59 */
#define	PWM_OFF_33k		0
#define PWM_50_33k		0xB4	/* PWM Duty 50% = (Period + 1) * 4 * 0.5 */
#define PWM_100_33k		0x168	/* PWM Duty 100% = (Period + 1) * 4 * 1.0 */



// 赤外線リモコン用
#define ON		1
#define OFF		0
#define R_ON	0
#define R_OFF	1
#define READSTATUS_READERCODE_WAIT		0x00	/* 受信状態　リーダコード待ち */
#define READSTATUS_DATA_WAIT			0x01	/* 受信状態　データ待ち */
#define READSTATUS_READ_END				0x02	/* 受信状態　データ終了 */
#define READSTATUS_NEXT_DATA_WAIT		0x04	/* 受信状態　次データ待ち */
#define READSTATUS_2ND_READERCODE_WAIT	0x08	/* 受信状態　2ndリーダコード待ち */
#define READSTATUS_2ND_DATA_WAIT		0x10	/* 受信状態　2ndデータ待ち */
#define READBUFFER_READER_SIZE			1		/* 受信バッファサイズ　リーダコード(1byte) */
#define READBUFFER_DATA_SIZE			6		/* 受信バッファサイズ　データ(6byte) */
#define READBUFFER_SIZE					READBUFFER_READER_SIZE+READBUFFER_DATA_SIZE	/* 受信バッファサイズ　リーダコード(1byte) + データ(6byte) */
#define READBUFFER_FORMAT_SIZE_EX		1		/* 拡張 受信バッファサイズ　フォーマットコード(1byte) */
#define READBUFFER_DATA_SIZE_EX			2		/* 拡張 受信バッファサイズ　データサイズ（単位bit）(1byte) */
#define READBUFFER_DATA_AREA_SIZE_EX	32		/* 拡張 受信バッファサイズ　データエリアサイズ(32byte) */
#define READBUFFER_SIZE_EX				READBUFFER_FORMAT_SIZE_EX+READBUFFER_DATA_SIZE_EX+READBUFFER_DATA_AREA_SIZE_EX	/* 拡張 受信バッファサイズ　フォーマットコード(1byte) + データサイズ(1byte) + データエリア(32byte) */
#define OUTSTATUS_OUTPUT_WAIT			0x00	/* 出力状態　出力待ち */
#define OUTSTATUS_READER_CODE			0x01	/* 出力状態　リーダコード */
#define OUTSTATUS_DATA_CODE				0x02	/* 出力状態　データコード */
#define OUTSTATUS_END					0x04	/* 出力状態　終了 */
#define OUTSTATUS_2ND_READER_CODE		0x08	/* 出力状態　2ndリーダコード */
#define OUTSTATUS_2ND_DATA_CODE			0x10	/* 出力状態　2ndデータコード */
#define OUTBUFFER_READER_SIZE			1		/* 出力バッファサイズ　リーダコード(1byte) */
#define OUTBUFFER_DATA_SIZE				6		/* 出力バッファサイズ　データ(6byte) */
#define OUTBUFFER_SIZE					OUTBUFFER_READER_SIZE+OUTBUFFER_DATA_SIZE	/* 出力バッファサイズ　リーダコード(1byte) + データ(6byte) */
#define OUTBUFFER_FORMAT_SIZE_EX		1		/* 拡張 出力バッファサイズ　フォーマットコード(1byte) */
#define OUTBUFFER_DATA_SIZE_EX			2		/* 拡張 出力バッファサイズ　データサイズ（単位bit）(1byte) */
#define OUTBUFFER_DATA_AREA_SIZE_EX		32		/* 拡張 出力バッファサイズ　データエリアサイズ(32byte) */
#define OUTBUFFER_SIZE_EX				OUTBUFFER_FORMAT_SIZE_EX+OUTBUFFER_DATA_SIZE_EX+OUTBUFFER_DATA_AREA_SIZE_EX	/* 拡張 出力バッファサイズ　リーダコード(1byte) + データ(32byte) */
#define FORMATCODE_UNKNOWN				0		/* 赤外線通信フォーマット 不明 */
#define FORMATCODE_KADEN				1		/* 赤外線通信フォーマット 家電協 */
#define FORMATCODE_NEC					2		/* 赤外線通信フォーマット NEC */
#define FORMATCODE_SONY					3		/* 赤外線通信フォーマット SONY */
#define FORMATCODE_MITSU				4		/* 赤外線通信フォーマット MITSUBISHI */
#define FORMATCODE_DAIKIN				5		/* 赤外線通信フォーマット DAIKIN */
#define FORMATCODE_DAIKIN2				6		/* 赤外線通信フォーマット DAIKIN2 */
#define RECEIVE_WAIT_MODE_NONE			0		/* PC側赤外線コード受信待ちモード　NONE */
#define RECEIVE_WAIT_MODE_WAIT			1		/* PC側赤外線コード受信待ちモード　WAIT */

#define READTIMING						100		/* 読み込み周期[usec] */
#define READCODE_END_CNT				200 	/* この回数OFFが続いたら終了 1000 = 100[ms] / 0.1[ms] */
#define SENDCODE_END_CNT				200 	/* 終了コードこの回数OFFを続ける 1000 = 100[ms] / 0.1[ms] */
#define NEXT_DATA_WAIT					500		/* 次のデータの読み込みをこの時間OFFを継続したら開始する 2000 = 200[ms] / 0.1[ms] */
#define DATA_CODE_INTERVAL_MIN_CNT		40 		/* データが２つにわかれているコードのデータ間最小間隔 40 = 4[ms] / 0.1[ms] */
#define DATA_CODE_INTERVAL_MAX_CNT		500 	/* データが２つにわかれているコードのデータ間最大間隔 500 = 50[ms] / 0.1[ms] */
#define DATA_CODE_INTERVAL_SEND_CNT		260 	/* データが２つにわかれているコードの送信データ間隔 300 = 30[ms] / 0.1[ms] */
#define DATA_MAX_BITS					0xFF	/* データコードの最大ビット長 */

/* 家電協 */
#define READERCODE_ON_KADEN			32		/* リーダコード ON時間 3.2ms / 0.1ms */
#define READERCODE_OFF_KADEN		16		/* リーダコード OFF時間 1.6ms / 0.1ms */
#define READERCODE_ON_T_KADEN		2		/* リーダコード ON時間 2T */
#define READERCODE_ON_MIN_KADEN		24		/* リーダコード ON時間最小 24 = MIN40 / 12T * 8T */
#define READERCODE_OFF_MIN_KADEN	12		/* リーダコード OFF時間最小 12 = MIN40 / 12T * 4T */
//#define READERCODE_OFF_KADEN		1600	/* リーダコード OFF時間 [usec] */
#define READERCODE_MIN_KADEN		40		/* リーダコード長 ((3.2 + 1.6) - 0.8) / 0.1 [msec] */
#define READERCODE_MAX_KADEN		56		/* リーダコード長 ((3.2 + 1.6) + 0.8) / 0.1 [msec] */
#define DATACODE_DATA1_KADEN		8		/* データコード0/1判断基準 OFFの長さがこれ以上なら1 */
#define DATACODE_MIN_KADEN			4		/* データコード長最小 data0 ((0.4 + 0.4) - 0.4) / 0.1 [msec] */
#define DATACODE_MAX_KADEN			20		/* データコード長最大 data1 ((0.4 + 1.2) + 0.4) / 0.1 [msec] */
#define DATA0_ON_KADEN				4		/* データ０ ON時間 0.4ms / 0.1ms */
#define DATA0_OFF_KADEN				4		/* データ０ OFF時間 0.4ms / 0.1ms */
#define DATA1_ON_KADEN				4		/* データ１ ON時間 0.4ms / 0.1ms */
#define DATA1_OFF_KADEN				12		/* データ１ OFF時間 1.2ms / 0.1ms */
/* NEC */
#define READERCODE_ON_NEC			90		/* リーダコード ON時間 9.0ms / 0.1ms */
#define READERCODE_OFF_NEC			45		/* リーダコード OFF時間 4.5ms / 0.1ms */
#define READERCODE_ON_T_NEC			2		/* リーダコード ON時間 2T */
#define READERCODE_ON_MIN_NEC		80		/* リーダコード ON時間最小 80 = MIN125 / 24T * 16T */
#define READERCODE_OFF_MIN_NEC		40		/* リーダコード OFF時間最小 40 = MIN125 / 24T * 8T */
//#define READERCODE_OFF_NEC			4500	/* リーダコード OFF時間 [usec] */
#define READERCODE_MIN_NEC			90		/* リーダコード長 ((9.0 + 4.5) - 4.5) / 0.1 [msec] */
#define READERCODE_MAX_NEC			180		/* リーダコード長 ((9.0 + 4.5) + 4.5) / 0.1 [msec] */
#define DATACODE_DATA1_NEC			12		/* データコード0/1判断基準 OFFの長さがこれ以上なら1 */
#define DATACODE_MIN_NEC			5		/* データコード長最小 data0 ((0.56 + 0.56) - 0.56) / 0.1 [msec] */
#define DATACODE_MAX_NEC			28		/* データコード長最大 data1 ((0.56 + 1.68) + 0.56) / 0.1 [msec] */
#define DATA0_ON_NEC				5		/* データ０ ON時間 0.56ms / 0.1ms */
#define DATA0_OFF_NEC				5		/* データ０ OFF時間 0.56ms / 0.1ms */
#define DATA1_ON_NEC				5		/* データ１ ON時間 0.56ms / 0.1ms */
#define DATA1_OFF_NEC				16		/* データ１ OFF時間 1.68ms / 0.1ms */
/* SONY */
#define READERCODE_ON_SONY			24		/* リーダコード ON時間 2.4ms / 0.1ms */
#define READERCODE_OFF_SONY			6		/* リーダコード OFF時間 0.6ms / 0.1ms */
#define READERCODE_ON_T_SONY		4		/* リーダコード ON時間 4T */
#define READERCODE_ON_MIN_SONY		16		/* リーダコード ON時間最小 16 = MIN24 / 5T * 4T */
#define READERCODE_OFF_MIN_SONY		4		/* リーダコード OFF時間最小 4 = MIN24 / 5T * 1T */
//#define READERCODE_OFF_SONY			600		/* リーダコード OFF時間 [usec] */
#define READERCODE_MIN_SONY			24		/* リーダコード長 ((2.4 + 0.6) - 0.6) / 0.1 [msec] */
#define READERCODE_MAX_SONY			36		/* リーダコード長 ((2.4 + 0.6) + 0.6) / 0.1 [msec] */
#define DATACODE_DATA1_SONY			9		/* データコード0/1判断基準 ONの長さがこれ以上なら1 */
#define DATACODE_MIN_SONY			6		/* データコード長最小 data0 ((0.6 + 0.6) - 0.6) / 0.1 [msec] */
#define DATACODE_MAX_SONY			24		/* データコード長最大 data1 ((1.2 + 0.6) + 0.6) / 0.1 [msec] */
#define DATA0_ON_SONY				6		/* データ０ ON時間 0.6ms / 0.1ms */
#define DATA0_OFF_SONY				6		/* データ０ OFF時間 0.6ms / 0.1ms */
#define DATA1_ON_SONY				12		/* データ１ ON時間 1.2ms / 0.1ms */
#define DATA1_OFF_SONY				6		/* データ１ OFF時間 0.6ms / 0.1ms */
/* MITSUBISHI リーダコードなし、いきなりデータ */
#define DATACODE_DATA1_M			14		/* データコード0/1判断基準 OFFの長さがこれ以上なら1 */
#define DATACODE_MIN_M				8		/* データコード長最小 data0 ((0.4 + 0.8) - 0.4) / 0.1 [msec] */
#define DATACODE_MAX_M				28		/* データコード長最大 data1 ((0.4 + 2.0) + 0.4) / 0.1 [msec] */
#define DATA0_ON_M					4		/* データ０ ON時間 0.4ms / 0.1ms */
#define DATA0_OFF_M					8		/* データ０ OFF時間 0.8ms / 0.1ms */
#define DATA1_ON_M					4		/* データ１ ON時間 0.4ms / 0.1ms */
#define DATA1_OFF_M					20		/* データ１ OFF時間 2.0ms / 0.1ms */
/* DAIKIN 1 */
#if 1
#define READERCODE_ON_DAI			44		/* リーダコード ON時間 4.4ms / 0.1ms */
#define READERCODE_OFF_DAI			22		/* リーダコード OFF時間 2.2ms / 0.1ms */
#define READERCODE_ON_T_DAI			2		/* リーダコード ON時間 2T */
#define READERCODE_ON_MIN_DAI		29		/* リーダコード ON時間最小 29 = MIN44 / 15T * 10T */
#define READERCODE_OFF_MIN_DAI		14		/* リーダコード OFF時間最小 14 = MIN44 / 15T * 5T */
//#define READERCODE_OFF_DAI		1600	/* リーダコード OFF時間 [usec] */
#define READERCODE_MIN_DAI			44		/* リーダコード長 ((4.4 + 2.2) - 2.2) / 0.1 [msec] */
#define READERCODE_MAX_DAI			88		/* リーダコード長 ((4.4 + 2.2) + 2.2) / 0.1 [msec] */
#define DATACODE_DATA1_DAI			7		/* データコード0/1判断基準 OFFの長さがこれ以上なら1 */
#define DATACODE_MIN_DAI			4		/* データコード長最小 data0 ((0.4 + 0.4) - 0.4) / 0.1 [msec] */
#define DATACODE_MAX_DAI			23		/* データコード長最大 data1 ((0.4 + 1.3) + 0.4) / 0.1 [msec] */
#define DATA0_ON_DAI				4		/* データ０ ON時間 0.4ms / 0.1ms */
#define DATA0_OFF_DAI				4		/* データ０ OFF時間 0.5ms / 0.1ms */
#define DATA1_ON_DAI				4		/* データ１ ON時間 0.4ms / 0.1ms */
#define DATA1_OFF_DAI				13		/* データ１ OFF時間 1.3ms / 0.1ms */
#endif
#if 0
#define READERCODE_ON_DAI			50		/* リーダコード ON時間 5.0ms / 0.1ms */
#define READERCODE_OFF_DAI			25		/* リーダコード OFF時間 2.5ms / 0.1ms */
#define READERCODE_ON_T_DAI			2		/* リーダコード ON時間 2T */
#define READERCODE_ON_MIN_DAI		33		/* リーダコード ON時間最小 33 = MIN50 / 15T * 10T */
#define READERCODE_OFF_MIN_DAI		16		/* リーダコード OFF時間最小 16 = MIN50 / 15T * 5T */
//#define READERCODE_OFF_DAI		1600	/* リーダコード OFF時間 [usec] */
#define READERCODE_MIN_DAI			50		/* リーダコード長 ((5.0 + 2.5) - 2.5) / 0.1 [msec] */
#define READERCODE_MAX_DAI			100		/* リーダコード長 ((5.0 + 2.5) + 2.5) / 0.1 [msec] */
#define DATACODE_DATA1_DAI			7		/* データコード0/1判断基準 OFFの長さがこれ以上なら1 */
#define DATACODE_MIN_DAI			5		/* データコード長最小 data0 ((0.5 + 0.5) - 0.5) / 0.1 [msec] */
#define DATACODE_MAX_DAI			25		/* データコード長最大 data1 ((0.5 + 1.5) + 0.5) / 0.1 [msec] */
#define DATA0_ON_DAI				5		/* データ０ ON時間 0.5ms / 0.1ms */
#define DATA0_OFF_DAI				5		/* データ０ OFF時間 0.5ms / 0.1ms */
#define DATA1_ON_DAI				5		/* データ１ ON時間 0.5ms / 0.1ms */
#define DATA1_OFF_DAI				15		/* データ１ OFF時間 1.5ms / 0.1ms */
#endif
/* DAIKIN 2 */
#define READERCODE_ON_DAI2			30		/* リーダコード ON時間 3.0ms / 0.1ms */
#define READERCODE_OFF_DAI2			90		/* リーダコード OFF時間 9.0ms / 0.1ms */
#define READERCODE_OFF_T_DAI2		3		/* リーダコード OFF時間 3T */
#define READERCODE_ON_MIN_DAI2		22		/* リーダコード ON時間最小 22 = MIN90 / 12T * 3T */
#define READERCODE_OFF_MIN_DAI2		67		/* リーダコード OFF時間最小 67 = MIN90 / 12T * 9T */
//#define READERCODE_OFF_DAI		1600	/* リーダコード OFF時間 [usec] */
#define READERCODE_MIN_DAI2			90		/* リーダコード長 ((3.0 + 9.0) - 3.0) / 0.1 [msec] */
#define READERCODE_MAX_DAI2			150		/* リーダコード長 ((3.0 + 9.0) + 3.0) / 0.1 [msec] */
// データ部はDAIKIN1と同じ
//#define DATACODE_DATA1_DAI2			7		/* データコード0/1判断基準 OFFの長さがこれ以上なら1 */
//#define DATACODE_MIN_DAI2			5		/* データコード長最小 data0 ((0.5 + 0.5) - 0.5) / 0.1 [msec] */
//#define DATACODE_MAX_DAI2			25		/* データコード長最大 data1 ((0.5 + 1.5) + 0.5) / 0.1 [msec] */
//#define DATA0_ON_DAI2				5		/* データ０ ON時間 0.5ms / 0.1ms */
//#define DATA0_OFF_DAI2				5		/* データ０ OFF時間 0.5ms / 0.1ms */
//#define DATA1_ON_DAI2				5		/* データ１ ON時間 0.5ms / 0.1ms */
//#define DATA1_OFF_DAI2				15		/* データ１ OFF時間 1.5ms / 0.1ms */




#define CODE_TYPE_STANDARD	0	// コードタイプ　標準
#define CODE_TYPE_EXTENSION	1	// コードタイプ　拡張

/** VARIABLES ******************************************************/
#pragma udata
char	c_version[]="2.1.0";
BYTE mouse_buffer[4] = {0};
BYTE volume_buffer[1] = {0};
BYTE keyboard_buffer[8]; 
USB_HANDLE lastTransmission;
USB_HANDLE lastTransmission2;
USB_HANDLE lastINTransmissionKeyboard;
USB_HANDLE lastOUTTransmissionKeyboard;
USB_HANDLE USBOutHandle = 0;
USB_HANDLE USBInHandle = 0;

unsigned char result_button_press_set = 0;
unsigned char result_button_press_set2 = 0;

// 赤外線データの0バイト目を格納する変数 20データ分20バイト
unsigned char eeprom_check_data[EEPROM_DATA_NUM] = {0};
// 赤外線データ１データ分　10バイト
unsigned char eeprom_1data[EEPROM_DATA_SIZE];

// 赤外線リモコンデータ受信用変数
unsigned char uc_read_buff[READBUFFER_SIZE_EX] = {0};			/* 読み込み中のデータを保存 */
unsigned char uc_fix_read_buff[READBUFFER_SIZE_EX] = {0};		/* 読み込み完了データを保存 */
unsigned char send_read_data_flag = 0;							/* 読み込み完了データをPCに送信するときにフラグを立てる */
unsigned char process_read_data_flag = 0;							/* 読み込み完了データを処理するするときにフラグを立てる */
unsigned char uc_process_read_data[READBUFFER_SIZE_EX];			/* 読み込みデータを処理するするときに保存 */
unsigned int ui_data_pos = 0;
unsigned char uc_read_status = READSTATUS_READERCODE_WAIT;
unsigned int ui_off_count = 0;
unsigned int ui_on_count = 0;
unsigned char uc_tick_base = 0;
unsigned char uc_format_type = FORMATCODE_UNKNOWN;
unsigned char uc_receive_wait_mode = RECEIVE_WAIT_MODE_NONE;/* PC側が赤外線データ受信待ちのとき1 */
unsigned int ui_next_data_wait_cnt = 0;						/* 次のデータまでのOFFの継続時間カウンタ */

unsigned char uc_now_signal = 0;
unsigned char uc_pre_signal = 0;

// 赤外線リモコンデータ送信用変数
unsigned int ui_out_pos = 0;
unsigned char uc_out_status = OUTSTATUS_OUTPUT_WAIT;
unsigned char uc_out_buff[OUTBUFFER_SIZE_EX];
unsigned int ui_out_on_time = 0;
unsigned int ui_out_off_time = 0;
unsigned char uc_out_data_len = 0;
unsigned char uc_out_data0_on = 0;
unsigned char uc_out_data0_off = 0;
unsigned char uc_out_data1_on = 0;
unsigned char uc_out_data1_off = 0;
unsigned char uc_send_bit_size = 0;
unsigned char uc_send_bit_size_2nd_data = 0;

unsigned char uc_first_data_bit = 0;

unsigned char uc_read_code_type = CODE_TYPE_STANDARD;
unsigned char uc_out_code_type = CODE_TYPE_STANDARD;

unsigned int ui_PWM50_Set_Val = PWM_50;

// デバッグ
unsigned char debug_ary[4] = {0};


// usbでの送信に使うバッファはここで宣言
#pragma udata usbram2

BYTE volume_input[1];
BYTE mouse_input[4];
BYTE hid_report[8];
unsigned char volume_input_out_flag = 0;
unsigned char mouse_input_out_flag = 0;
unsigned char hid_report_out_flag = 0;
unsigned char ReceivedDataBuffer[RX_BUFFER_SIZE];
unsigned char ToSendDataBuffer[TX_BUFFER_SIZE];

#pragma udata

/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
        #if defined(USB_INTERRUPT)
	        USBDeviceTasks();
        #endif

	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.	
		
		if(INTCONbits.TMR0IF == 1)
		{
			INTCONbits.TMR0IF = 0;
			WriteTimer0(WRITE_TIMER0_COUNT);

			// データ出力中の時は受信しない
			if( uc_out_status == OUTSTATUS_OUTPUT_WAIT )
			{
				/* 赤外線リモコンのデータ受信 */
				RemoconReceiveData();
			}	
			
			/* 赤外線リモコンのデータ出力 */
			RemoconOutData();
		}

	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif

#pragma code

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void main(void)
{   
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

//	タイマ0の設定
	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8);
	WriteTimer0(WRITE_TIMER0_COUNT);
//	CCPモジュールのPWM設定
	OpenPWM1(PWM_PERIOD);
//	タイマ2の設定 プリスケーラ1:4
	OpenTimer2(TIMER_INT_OFF & T2_PS_1_4 & T2_POST_1_1 );
	SetDCPWM1(PWM_OFF);
//	SetDCPWM1(PWM_50);

//	割り込み開始
	RCONbits.IPEN = 1;	//割り込み優先付あり
	INTCON2bits.TMR0IP = 0;	//タイマ0を低位レベル割り込みに設定
	INTCONbits.TMR0IE = 1;	//タイマ0割り込み許可
	INTCONbits.GIEL = 1;	//低位レベル割り込みの許可
//	INTCONbits.GIEH = 1;	//高位レベル割り込みの許可
	INTCONbits.GIE = 1;
	
    while(1)
    {
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // frequently (such as once about every 100 microseconds) at any
        				  // time that a SETUP packet might reasonably be expected to
        				  // be sent by the host to your device.  In most cases, the
        				  // USBDeviceTasks() function does not take very long to
        				  // execute (~50 instruction cycles) before it returns.
        #endif
    				  

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    ADCON1 |= 0x0F;                 // Default all pins to digital

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    UserInit();

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
}//end InitializeSystem



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
	int fi,fj;
	char mode_changedp = 1;

	TRISB = 0x00;	//RB5を入力 b7:RB7 b6:RB6 b5:RB5 b4:RB4
	TRISC = 0x80;	//RCを出力  b7:RC7 b6:RC6 b5:RC5 b4:RC4 b3:RC3 b2:RC2 b1:RC1 bit0:RC0

	LATC = 0xff;

	ANSEL = 0x00;
	ANSELH = 0x00;	//全てデジタル

	INTCON2bits.RABPU = 0;	//内蔵プルアップを使えるようにする
	WPUBbits.WPUB4 = 1;
	WPUBbits.WPUB5 = 1;
	WPUBbits.WPUB6 = 1;
	WPUBbits.WPUB7 = 1;

    //initialize the variable holding the handle for the last
	
    for(fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
    {
	    uc_out_buff[fi] = 0;
	}
	for(fi = 0; fi < READBUFFER_SIZE_EX; fi++)
	{
		uc_process_read_data[fi] = 0;
	}
	for(fi = 0; fi < EEPROM_DATA_SIZE; fi++)
	{
		eeprom_1data[fi] = 0;
	}

	
    // transmission
    lastTransmission = 0;
    lastTransmission2 = 0;
    lastINTransmissionKeyboard = 0;
    lastOUTTransmissionKeyboard = 0;
    USBOutHandle = 0;
    USBInHandle = 0;

	volume_input[0] = 0;

    mouse_input[0] = 
    mouse_input[1] = 
    mouse_input[2] = 0;

	hid_report_in[0] = 
	hid_report_in[1] = 
	hid_report_in[2] = 
	hid_report_in[3] = 
	hid_report_in[4] = 
	hid_report_in[5] = 
	hid_report_in[6] = 
	hid_report_in[7] = 0;

	mouse_buffer[0] =
	mouse_buffer[1] =
	mouse_buffer[2] =
	mouse_buffer[3] = 0;

	keyboard_buffer[0] = 
	keyboard_buffer[1] = 
	keyboard_buffer[2] = 
	keyboard_buffer[3] = 
	keyboard_buffer[4] = 
	keyboard_buffer[5] = 
	keyboard_buffer[6] = 
	keyboard_buffer[7] = 0;

	volume_buffer[0] = 0;

	// EEPROMの赤外線データの0バイト目を格納する
	for( fi = 0, fj = EEPROM_DATA_DATACODE; fi < EEPROM_DATA_NUM; fi++, fj+=EEPROM_DATA_SIZE )
	{
		eeprom_check_data[fi] = ReadEEPROM( fi * EEPROM_DATA_SIZE + EEPROM_DATA_DATACODE);
	}
}//end UserInit


/********************************************************************
 * Function:        void ProcessIO(void)
 *	
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{
	int fi,fj;
	int pressed_keys;
	char result_button_press;
	char tmp;
	char result;
	char code_match_flag = 0;
	unsigned char uc_1data = 0;


    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

	/* 赤外線受信データをコピーする */
	/* 一旦フラグをクリア */
	process_read_data_flag = 0;
	/* 赤外線コード読み込み済み */
	if( uc_fix_read_buff[0] != FORMATCODE_UNKNOWN )
	{
		/* PC側赤外線コード受信待ち */
		if( uc_receive_wait_mode == RECEIVE_WAIT_MODE_WAIT )
		{
			send_read_data_flag = 1;
			for( fi = 0; fi < READBUFFER_SIZE_EX; fi++ )
			{
				uc_process_read_data[fi] = uc_fix_read_buff[fi];
				uc_fix_read_buff[fi] = 0;
			}
		}
		else
		{
			send_read_data_flag = 0;
			process_read_data_flag = 1;
			for( fi = 0; fi < READBUFFER_SIZE; fi++ )
			{
				uc_process_read_data[fi] = uc_fix_read_buff[fi];
			}
			for( fi = 0; fi < READBUFFER_SIZE_EX; fi++ )
			{
				uc_fix_read_buff[fi] = 0;
			}
		}
	}
	else
	{	/* PC側赤外線コード受信待ちでないときはバッファをクリア */
		if( uc_receive_wait_mode != RECEIVE_WAIT_MODE_WAIT )
		{
			send_read_data_flag = 0;
			for( fi = 0; fi < (READBUFFER_SIZE_EX - READBUFFER_DATA_AREA_SIZE_EX); fi++ )
			{
				uc_process_read_data[fi] = 0;
			}
		}
	}	

	code_match_flag = 0;
	pressed_keys = 2;
	/* 赤外線コード受信？　リーダコードが０以外で受信したと見なす */
	if(process_read_data_flag == 1)
	{
		/* データ数分ループ */
		for(fi = 0; fi < EEPROM_DATA_NUM; fi++ )
		{
			code_match_flag = 0;
			/* 0バイトのみ比較して一致していたら1バイト目以降をEEPROMから読み出し比較する */
			if(eeprom_check_data[fi] == uc_process_read_data[1])
			{
				code_match_flag++;	//一致
				eeprom_1data[EEPROM_DATA_DATACODE] = eeprom_check_data[fi];

				uc_1data = ReadEEPROM(fi * EEPROM_DATA_SIZE + EEPROM_DATA_READERCODE);
				/* リーダコード比較 */
				if( uc_1data == uc_process_read_data[0] )
				{	/*リーダコード一致*/
					code_match_flag++;	//一致
					eeprom_1data[EEPROM_DATA_READERCODE] = uc_1data;

					/* 0バイト目が一致していたので1バイト以降を読み出し比較する */
					for( fj = 1; fj < READBUFFER_DATA_SIZE; fj++ )
					{
						uc_1data = ReadEEPROM((unsigned char)fi * (unsigned char)EEPROM_DATA_SIZE + (unsigned char)EEPROM_DATA_DATACODE + (unsigned char)fj );
						if(uc_1data == uc_process_read_data[1+fj])
						{
							code_match_flag++;	//一致
							eeprom_1data[EEPROM_DATA_DATACODE+fj] = uc_1data;
						}
					}

					// 赤外線コード全て一致
					if(code_match_flag == (READBUFFER_READER_SIZE + READBUFFER_DATA_SIZE))
					{
						eeprom_1data[EEPROM_DATA_MODE] = ReadEEPROM(fi * EEPROM_DATA_SIZE + EEPROM_DATA_MODE);
						eeprom_1data[EEPROM_DATA_VALUE] = ReadEEPROM(fi * EEPROM_DATA_SIZE + EEPROM_DATA_VALUE);
						eeprom_1data[EEPROM_DATA_MODIFIER] = ReadEEPROM(fi * EEPROM_DATA_SIZE + EEPROM_DATA_MODIFIER);
						/* 受信した赤外線コードが一致したので設定されているコードを送信する */

						switch(eeprom_1data[EEPROM_DATA_MODE])
						{
							case MODE_MOUSE:
								if(eeprom_1data[EEPROM_DATA_VALUE] == 0){  // 左クリック
									mouse_buffer[0] |= 1;
								}	
								else if(eeprom_1data[EEPROM_DATA_VALUE] == 1){  // 右クリック
									mouse_buffer[0] |= 0x02;
								}	
								else if(eeprom_1data[EEPROM_DATA_VALUE] == 2){  // ホイールクリック
									mouse_buffer[0] |= 0x04;
								}
								mouse_input_out_flag = 5;
								break;
							case MODE_KEYBOARD:
								if(pressed_keys != 8)
								{
									keyboard_buffer[0] |= eeprom_1data[EEPROM_DATA_MODIFIER];
									keyboard_buffer[pressed_keys] = eeprom_1data[EEPROM_DATA_VALUE];
									pressed_keys++;
								}
								hid_report_out_flag = 5;
								break;
							case MODE_VOLUME:
							    switch(eeprom_1data[EEPROM_DATA_VALUE])
							    {
								    case 0: volume_buffer[0] |= 0x01; break;
								    case 1: volume_buffer[0] |= 0x02; break;
								    case 2: volume_buffer[0] |= 0x04; break;
								}
								volume_input_out_flag = 5;
								break;
							default:
								break;
						}
						/* 受信した赤外線コードが一致したので設定されているコードを送信する ここまで */
					}	
				}
			}	
		}	
	}	

//---------------------------------------------------------------------
//	USBデータ送信部
    if(!HIDTxHandleBusy(lastTransmission))
    {
        //copy over the data to the HID buffer
        //マウスデータの送信
        mouse_input[0] = mouse_buffer[0];
        mouse_input[1] = mouse_buffer[1];
        mouse_input[2] = mouse_buffer[2];
        mouse_input[3] = mouse_buffer[3];

		mouse_buffer[0] =0;
		mouse_buffer[1] =0;
		mouse_buffer[2] =0;
		mouse_buffer[3] =0;

		if( mouse_input_out_flag > 0 )
		{
        	//Send the 8 byte packet over USB to the host.
        	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)&mouse_input, sizeof(mouse_input));
        	mouse_input_out_flag--;
 		}      	
    }
    if(!HIDTxHandleBusy(lastINTransmissionKeyboard))
    {	       	//Load the HID buffer
    	hid_report_in[0] = keyboard_buffer[0];
    	hid_report_in[1] = keyboard_buffer[1];
		hid_report_in[2] = keyboard_buffer[2];
	    hid_report_in[3] = keyboard_buffer[3];
    	hid_report_in[4] = keyboard_buffer[4];
    	hid_report_in[5] = keyboard_buffer[5];
    	hid_report_in[6] = keyboard_buffer[6];
    	hid_report_in[7] = keyboard_buffer[7];

		keyboard_buffer[0] =
		keyboard_buffer[2] =
		keyboard_buffer[3] =
		keyboard_buffer[4] =
		keyboard_buffer[5] =
		keyboard_buffer[6] =
		keyboard_buffer[7] = 0;

		if( hid_report_out_flag > 0 )
		{
    		//Send the 8 byte packet over USB to the host.
			lastINTransmissionKeyboard = HIDTxPacket(HID_EP3, (BYTE*)hid_report_in, 0x08);
			hid_report_out_flag--;
		}	
	}
   if(!HIDTxHandleBusy(lastTransmission2))
    {
        volume_input[0] = volume_buffer[0];
		volume_buffer[0] = 0;

		if( volume_input_out_flag > 0 )
		{
        	lastTransmission2 = HIDTxPacket(HID_EP2, (BYTE*)&volume_input, sizeof(volume_input));
        	volume_input_out_flag--;
 		}      	
    }
//---------------------------------------------------------------------
//	USBデータ通信部
    if(!HIDRxHandleBusy(USBOutHandle))				//Check if data was received from the host.
    {
	    // 送信バッファクリア
	    for( fi = 0; fi < TX_BUFFER_SIZE; fi++)
	    {
		    ToSendDataBuffer[fi] = 0;
		}
		
        switch(ReceivedDataBuffer[0])
        {
            case 0x56: // V=0x56 Get Firmware version
                ToSendDataBuffer[0] = 0x56;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
				tmp = strlen(c_version);
				if( 0 < tmp && tmp <= (64-2) )
				{
					for( fi = 0; fi < tmp; fi++ )
					{
						ToSendDataBuffer[fi+1] = c_version[fi];
					}
					// 最後にNULL文字を設定
					ToSendDataBuffer[fi+1] = 0x00;
				}
				else
				{
					//バージョン文字列異常
					ToSendDataBuffer[1] = 0x00;
				}				
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x80:  // EEPROMにデータを設定 [0x80,先頭アドレス,サイズ,data1, ... ,data61]
            	/* パラメータチェック 先頭アドレス(0-199)+データサイズ(1-61)<=200 サイズ=1-61 */
            	if((ReceivedDataBuffer[1]+ReceivedDataBuffer[2]) <= EEPROM_DATA_TOTAL_SIZE 
            		&& 1 <= ReceivedDataBuffer[2] && ReceivedDataBuffer[2] <= 61)
            	{
	            	for( fi = 0; fi < ReceivedDataBuffer[2]; fi++)
	            	{
		            	WriteEEPROM(ReceivedDataBuffer[1]+fi,ReceivedDataBuffer[3+fi]);
		            }
		            // OKアンサ
		            ToSendDataBuffer[1] = 0x00;
	           	}
	           	else
	           	{
		           	// NGアンサ
		           	ToSendDataBuffer[1] = 0xFF;
		        }
                ToSendDataBuffer[0] = 0x80;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x81:  // EEPROMにデータを設定 [0x81,データNo,サイズ,data1, ... ,data10]
            	/* パラメータチェック データNO=1-20 サイズ=10 */
            	if( 1 <= ReceivedDataBuffer[1] && ReceivedDataBuffer[1] <= EEPROM_DATA_NUM && ReceivedDataBuffer[2] == EEPROM_DATA_SIZE)
            	{
	            	for( fi = 0; fi < ReceivedDataBuffer[2]; fi++)
	            	{
		            	WriteEEPROM((ReceivedDataBuffer[1]-1)*EEPROM_DATA_SIZE+fi,ReceivedDataBuffer[3+fi]);
		            }
		            // OKアンサ
		            ToSendDataBuffer[1] = 0x00;
		            
		        	// 0バイト目を読み込んでいるバッファを更新
					eeprom_check_data[ReceivedDataBuffer[1]-1] = ReceivedDataBuffer[7];
	           	}
	           	else
	           	{
		           	// NGアンサ
		           	ToSendDataBuffer[1] = 0xFF;
		        }
                ToSendDataBuffer[0] = 0x81;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x82:  // EEPROMからデータ読み出し [0x82,先頭アドレス,サイズ] -> アンサ[0x82,サイズ,data1, ... , data61]
				ToSendDataBuffer[0] = 0x82;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
            	/* パラメータチェック 先頭アドレス(0-199)+データサイズ(1-61)<=200 サイズ=1-61 */
            	if((ReceivedDataBuffer[1]+ReceivedDataBuffer[2]) <= EEPROM_DATA_TOTAL_SIZE 
            		&& 1 <= ReceivedDataBuffer[2] && ReceivedDataBuffer[2] <= 61)
            	{
	            	ToSendDataBuffer[1] = ReceivedDataBuffer[2];
	            	for( fi = 0; fi < ReceivedDataBuffer[2]; fi++)
	            	{
		            	ToSendDataBuffer[2+fi] = ReadEEPROM(ReceivedDataBuffer[1]+fi);
		            }
	            }
	           	else
	           	{
		           	// NGアンサ
		           	ToSendDataBuffer[1] = 0xFF;
		        }
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x83:  // EEPROMからデータ読み出し [0x83,データNo,サイズ] -> アンサ[0x83,サイズ,data1, ... , data10]
				ToSendDataBuffer[0] = 0x83;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
            	/* パラメータチェック データNO=1-20 サイズ=10 */
            	if( 1 <= ReceivedDataBuffer[1] && ReceivedDataBuffer[1] <= EEPROM_DATA_NUM && ReceivedDataBuffer[2] == EEPROM_DATA_SIZE)
            	{
	            	ToSendDataBuffer[1] = ReceivedDataBuffer[2];
	            	for( fi = 0; fi < ReceivedDataBuffer[2]; fi++)
	            	{
		            	ToSendDataBuffer[2+fi] = ReadEEPROM((ReceivedDataBuffer[1]-1)*EEPROM_DATA_SIZE+fi);
		            }
	           	}
	           	else
	           	{
		           	// NGアンサ
		           	ToSendDataBuffer[1] = 0xFF;
		        }
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
#if 1	//DEBUG
            case 0x40:
                ToSendDataBuffer[0] = 0x40;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
				ToSendDataBuffer[1] = (unsigned char)(ui_off_count & 0xff);
				ToSendDataBuffer[2] = (unsigned char)(ui_on_count & 0xff);
				ToSendDataBuffer[3] = debug_ary[0];
				ToSendDataBuffer[4] = debug_ary[1];
				ToSendDataBuffer[5] = debug_ary[2];
				ToSendDataBuffer[6] = debug_ary[3];

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x41:
                ToSendDataBuffer[0] = 0x41;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
#endif	//DEBUG
            case 0x50:	/* 赤外線受信データを送信 */
                ToSendDataBuffer[0] = 0x50;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

				if(send_read_data_flag == 1)
				{
					for(fi = 0; fi < READBUFFER_SIZE; fi++)
					{
						ToSendDataBuffer[fi+1] = uc_process_read_data[fi];
					}
				}
				else
				{
					for(fi = 0; fi < READBUFFER_SIZE; fi++)
					{
						ToSendDataBuffer[fi+1] = 0x00;
					}
				}

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x51:	/* 赤外線受信待ちモード設定 */
	            uc_read_code_type = CODE_TYPE_STANDARD;
            	if( RECEIVE_WAIT_MODE_WAIT == ReceivedDataBuffer[1] )
            	{	/* 受信待ちモード設定 */
	            	uc_receive_wait_mode = RECEIVE_WAIT_MODE_WAIT;
	            }
	            else
	            {	/* 受信待ちモード解除 */
	            	uc_receive_wait_mode = RECEIVE_WAIT_MODE_NONE;
	         	}
                ToSendDataBuffer[0] = 0x51;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x52:	/* 赤外線受信データを送信 拡張版 */
                ToSendDataBuffer[0] = 0x52;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

				if(send_read_data_flag == 1)
				{
					for(fi = 0; fi < READBUFFER_SIZE_EX; fi++)
					{
						ToSendDataBuffer[fi+1] = uc_process_read_data[fi];
					}
				}
				else
				{
					for(fi = 0; fi < READBUFFER_SIZE_EX; fi++)
					{
						ToSendDataBuffer[fi+1] = 0x00;
					}
				}

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x53:	/* 赤外線受信待ちモード設定 拡張版 */
            	if( RECEIVE_WAIT_MODE_WAIT == ReceivedDataBuffer[1] )
            	{	/* 受信待ちモード設定 */
	            	uc_read_code_type = CODE_TYPE_EXTENSION;
	            	uc_receive_wait_mode = RECEIVE_WAIT_MODE_WAIT;
	            }
	            else
	            {	/* 受信待ちモード解除 */
	            	uc_receive_wait_mode = RECEIVE_WAIT_MODE_NONE;
	            	uc_read_code_type = CODE_TYPE_STANDARD;
	         	}
                ToSendDataBuffer[0] = 0x53;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
                
            case 0x60:	/* 赤外線出力データを受信する */
                ToSendDataBuffer[0] = 0x60;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.
				
				uc_out_code_type = CODE_TYPE_STANDARD;
				for(fi = 0; fi < OUTBUFFER_SIZE; fi++){
					uc_out_buff[fi] = ReceivedDataBuffer[fi+1];
				}
                break;
            case 0x61:	/* 赤外線出力データを受信する 拡張版 */
                ToSendDataBuffer[0] = 0x61;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.
				
				uc_out_code_type = CODE_TYPE_EXTENSION;
				for(fi = 0; fi < OUTBUFFER_SIZE_EX; fi++){
					uc_out_buff[fi] = ReceivedDataBuffer[fi+1];
				}
                break;
        }
         //Re-arm the OUT endpoint for the next packet
        USBOutHandle = HIDRxPacket(HID_EP4,(BYTE*)&ReceivedDataBuffer,64);
    }
}




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {       
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(HID_EP2,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(HID_EP3,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(HID_EP4,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function should only be called when:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
            Nop();
            break;
        default:
            break;
    }      
    return TRUE; 
}

/*******************************************************************
 * Function:        int RemoconOutData(
 *                        void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          int - A processing result is returned. 0 is normal.
 *
 * Side Effects:    None
 *
 * Overview:        この関数は、赤外線リモコンのデータを出力します。    
 *                  出力バッファ[0]のフォーマットタイプ従い、[1]以降のデータを出力します。 
 *                  0 is returned at the time of normal processing.
 *
 * Note:            None
 *******************************************************************/
int RemoconOutData(void)
{
	int ret_code = 0;
	int fi = 0;
	unsigned int byte_pos = 0;
	unsigned int bit_pos = 0;
	unsigned char send_bit = 0;
	unsigned char format_type;
	unsigned char out_buffer_offset;
	
	/* 出力状態でない */
	if( 0 >= ui_out_on_time && 0 >= ui_out_off_time )
	{
		/* 出力待ち */
		if( uc_out_status == OUTSTATUS_OUTPUT_WAIT )
		{

			/* 初期化 */
			ui_out_pos = 0;	/* 出力位置 */
			
			// フォーマットタイプセット
			if(uc_out_code_type == CODE_TYPE_EXTENSION)
			{	// 拡張型は 0 バイト目
				format_type = uc_out_buff[0];
			}
			else
			{	// 標準型は 0バイト目の下位4bit
				format_type = (uc_out_buff[0] & 0x0F);
			}
			
			/* 出力バッファに出力データがある？ */
			if( format_type != FORMATCODE_UNKNOWN )
			{
				/* 次の状態へ */
				uc_out_status = OUTSTATUS_DATA_CODE;
				/* データサイズ設定(bit) */
				if(uc_out_code_type == CODE_TYPE_EXTENSION)
				{	// 拡張型は 1バイト目と2バイト目
					uc_send_bit_size = uc_out_buff[1];
					uc_send_bit_size_2nd_data = uc_out_buff[2];
				}
				else
				{	// 標準型は 0バイト目の上位4bit	
					/* uc_out_buff[0]の上位4ビットに出力データ長/4の値が入っている */
					/* (uc_out_buff[0] >> 4) * 4 = uc_out_buff[0] >> 2 */
					uc_send_bit_size = uc_out_buff[0];
					uc_send_bit_size = ((uc_send_bit_size&0xF0)>>2);
					uc_send_bit_size_2nd_data = 0;
				}
				
				// PWM Duty Set
				ClosePWM1();
				if(format_type == FORMATCODE_DAIKIN && uc_send_bit_size == 0x38)
				{	// 33kHz
					ui_PWM50_Set_Val = PWM_50_33k;
					OpenPWM1(PWM_PERIOD_33k);
				}
				else
				{	// 38kHz
					ui_PWM50_Set_Val = PWM_50;
					OpenPWM1(PWM_PERIOD);
				}
					
				
				/* ONとOFF時間をセット*/
				/* 家電協フォーマット */
				if( format_type == FORMATCODE_KADEN )
				{
					ui_out_on_time = READERCODE_ON_KADEN;
					ui_out_off_time = READERCODE_OFF_KADEN;
					
					uc_out_data0_on = DATA0_ON_KADEN;
					uc_out_data0_off = DATA0_OFF_KADEN;
					uc_out_data1_on = DATA1_ON_KADEN;
					uc_out_data1_off = DATA1_OFF_KADEN;
				}
				/* NECフォーマット */
				else if( format_type == FORMATCODE_NEC )
				{
					ui_out_on_time = READERCODE_ON_NEC;
					ui_out_off_time = READERCODE_OFF_NEC;
					
					uc_out_data0_on = DATA0_ON_NEC;
					uc_out_data0_off = DATA0_OFF_NEC;
					uc_out_data1_on = DATA1_ON_NEC;
					uc_out_data1_off = DATA1_OFF_NEC;
				}
				/* SONYフォーマット */
				else if( format_type == FORMATCODE_SONY )
				{
					ui_out_on_time = READERCODE_ON_SONY;
					ui_out_off_time = READERCODE_OFF_SONY;
					
					uc_out_data0_on = DATA0_ON_SONY;
					uc_out_data0_off = DATA0_OFF_SONY;
					uc_out_data1_on = DATA1_ON_SONY;
					uc_out_data1_off = DATA1_OFF_SONY;
				}
				/* MITSUBISHIフォーマット */
				else if( format_type == FORMATCODE_MITSU )
				{
					/* リーダコードなし */
					ui_out_on_time = 0;
					ui_out_off_time = 0;
					
					uc_out_data0_on = DATA0_ON_M;
					uc_out_data0_off = DATA0_OFF_M;
					uc_out_data1_on = DATA1_ON_M;
					uc_out_data1_off = DATA1_OFF_M;
				}
				/* DAIKINフォーマット */
				else if( format_type == FORMATCODE_DAIKIN )
				{
					ui_out_on_time = READERCODE_ON_DAI;
					ui_out_off_time = READERCODE_OFF_DAI;
					
					uc_out_data0_on = DATA0_ON_DAI;
					uc_out_data0_off = DATA0_OFF_DAI;
					uc_out_data1_on = DATA1_ON_DAI;
					uc_out_data1_off = DATA1_OFF_DAI;
				}
				/* DAIKIN2フォーマット */
				else if( format_type == FORMATCODE_DAIKIN2 )
				{
					ui_out_on_time = READERCODE_ON_DAI2;
					ui_out_off_time = READERCODE_OFF_DAI2;
					
					uc_out_data0_on = DATA0_ON_DAI;
					uc_out_data0_off = DATA0_OFF_DAI;
					uc_out_data1_on = DATA1_ON_DAI;
					uc_out_data1_off = DATA1_OFF_DAI;
				}
				/* フォーマット不明 */
				else
				{
					/* 出力バッファクリア */
					for( fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
					{
						uc_out_buff[fi] = 0;
					}
					uc_out_status = OUTSTATUS_OUTPUT_WAIT;
					uc_send_bit_size = 0;
					uc_send_bit_size_2nd_data = 0;
					format_type = 0;
				}
			}
		}
		else if( uc_out_status == OUTSTATUS_DATA_CODE )
		{
			// フォーマットタイプセット
			if(uc_out_code_type == CODE_TYPE_EXTENSION)
			{	// 拡張型は 0 バイト目
				format_type = uc_out_buff[0];
				out_buffer_offset = OUTBUFFER_SIZE_EX - OUTBUFFER_DATA_AREA_SIZE_EX;
			}
			else
			{	// 標準型は 0バイト目の下位4bit
				format_type = (uc_out_buff[0] & 0x0F);
				out_buffer_offset = OUTBUFFER_SIZE - OUTBUFFER_DATA_SIZE;
			}

			byte_pos = ui_out_pos / 8;
			bit_pos = ui_out_pos % 8;
			
			if ( ( ui_out_pos == (unsigned int)uc_send_bit_size && uc_send_bit_size_2nd_data == 0 ) || ui_out_pos > DATA_MAX_BITS )
			{	/* 終了コード */
				send_bit = 0xF0;
			}
			else if(ui_out_pos == (unsigned int)uc_send_bit_size && uc_send_bit_size_2nd_data > 0)
			{	/* 2ndデータあり */
				send_bit = 3;
			}	
			else if( 0 <= (byte_pos+out_buffer_offset) && (byte_pos+out_buffer_offset) < OUTBUFFER_SIZE_EX )
			{
				send_bit = (uc_out_buff[byte_pos+out_buffer_offset] >> bit_pos ) & 0x01;
			}
			else
			{	/* エラー */
				send_bit = 0xFF;
			}	

			if( send_bit == 0 )
			{	/* DATA0出力 */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = uc_out_data0_off;
			}
			else if ( send_bit == 1 )
			{	/* DATA1出力 */
				ui_out_on_time = uc_out_data1_on;
				ui_out_off_time = uc_out_data1_off;
			}
			else if ( send_bit == 0xF0 )
			{	/* 終了コード出力 */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = SENDCODE_END_CNT;
			}
			else if ( send_bit == 3 )
			{	/* 次のデータまでの間隔を空ける */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = DATA_CODE_INTERVAL_SEND_CNT;
				
				// リーダコード送信状態へ
				uc_out_status = OUTSTATUS_2ND_READER_CODE;
			}
			else if ( send_bit == 0xFF )
			{	/* エラー */
				ui_out_on_time = 0;
				ui_out_off_time = 0;
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
			}
			
			/* SONYフォーマットはトレーラーコードなし */
			if( format_type == FORMATCODE_SONY )
			{
				//最終データ送信時のOFFを長くする
				if ( ui_out_pos == (unsigned int)(uc_send_bit_size - 1) )
				{	/* 終了コード */
					ui_out_off_time = SENDCODE_END_CNT;
				}	
			}
				
			if( send_bit >= 0xF0 )
			{	/* 次の送信データなし */
				/* 出力バッファクリア */
				for( fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
				{
					uc_out_buff[fi] = 0;
				}
				/* 出力待ち */
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
			}
			else if(send_bit == 0 || send_bit == 1)
			{
				/* 出力データ位置をインクリメント */
				ui_out_pos++;
			}
		}
		else if( uc_out_status == OUTSTATUS_2ND_READER_CODE )
		{	// 2nd Reader Code出力

			uc_out_status = OUTSTATUS_2ND_DATA_CODE;
			
			// フォーマットタイプセット
			if(uc_out_code_type == CODE_TYPE_EXTENSION)
			{	// 拡張型は 0 バイト目
				format_type = uc_out_buff[0];
				out_buffer_offset = OUTBUFFER_SIZE_EX - OUTBUFFER_DATA_AREA_SIZE_EX;
			}
			else
			{	// 標準型は 0バイト目の下位4bit
				format_type = (uc_out_buff[0] & 0x0F);
				out_buffer_offset = OUTBUFFER_SIZE - OUTBUFFER_DATA_SIZE;
			}
			
			/* 家電協フォーマット */
			if( format_type == FORMATCODE_KADEN )
			{
				ui_out_on_time = READERCODE_ON_KADEN;
				ui_out_off_time = READERCODE_OFF_KADEN;
			}
			/* NECフォーマット */
			else if( format_type == FORMATCODE_NEC )
			{
				ui_out_on_time = READERCODE_ON_NEC;
				ui_out_off_time = READERCODE_OFF_NEC;
			}
#if 0
			/* SONYフォーマット */
			else if( format_type == FORMATCODE_SONY )
			{
				ui_out_on_time = READERCODE_ON_SONY;
				ui_out_off_time = READERCODE_OFF_SONY;
			}
			/* MITSUBISHIフォーマット */
			else if( format_type == FORMATCODE_MITSU )
			{
				/* リーダコードなし */
				ui_out_on_time = 0;
				ui_out_off_time = 0;
			}
#endif
			/* DAIKINフォーマット */
			else if( format_type == FORMATCODE_DAIKIN )
			{
				ui_out_on_time = READERCODE_ON_DAI;
				ui_out_off_time = READERCODE_OFF_DAI;
			}
			/* DAIKIN2フォーマット */
			else if( format_type == FORMATCODE_DAIKIN2 )
			{
				ui_out_on_time = READERCODE_ON_DAI2;
				ui_out_off_time = READERCODE_OFF_DAI2;
			}
			/* フォーマット不明 */
			else
			{
				/* 出力バッファクリア */
				for( fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
				{
					uc_out_buff[fi] = 0;
				}
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
				uc_send_bit_size = 0;
				uc_send_bit_size_2nd_data = 0;
				format_type = 0;
			}
		}
		else if( uc_out_status == OUTSTATUS_2ND_DATA_CODE )
		{	// 2nd Data Code出力

			// フォーマットタイプセット
			if(uc_out_code_type == CODE_TYPE_EXTENSION)
			{	// 拡張型は 0 バイト目
				format_type = uc_out_buff[0];
				out_buffer_offset = OUTBUFFER_SIZE_EX - OUTBUFFER_DATA_AREA_SIZE_EX;
			}
			else
			{	// 標準型は 0バイト目の下位4bit
				format_type = (uc_out_buff[0] & 0x0F);
				out_buffer_offset = OUTBUFFER_SIZE - OUTBUFFER_DATA_SIZE;
			}
				
			byte_pos = ui_out_pos / 8;
			bit_pos = ui_out_pos % 8;
			
			if ( ui_out_pos == (unsigned int)(uc_send_bit_size + uc_send_bit_size_2nd_data) || ui_out_pos > DATA_MAX_BITS )
			{	/* 終了コード */
				send_bit = 0xF0;
			}
			else if( 0 <= (byte_pos+out_buffer_offset) && (byte_pos+out_buffer_offset) < OUTBUFFER_SIZE_EX )
			{
				send_bit = (uc_out_buff[byte_pos+out_buffer_offset] >> bit_pos ) & 0x01;
			}
			else
			{	/* エラー */
				send_bit = 0xFF;
			}	

			if( send_bit == 0 )
			{	/* DATA0出力 */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = uc_out_data0_off;
			}
			else if ( send_bit == 1 )
			{	/* DATA1出力 */
				ui_out_on_time = uc_out_data1_on;
				ui_out_off_time = uc_out_data1_off;
			}
			else if ( send_bit == 0xF0 )
			{	/* 終了コード出力 */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = SENDCODE_END_CNT;
			}
			else if ( send_bit == 0xFF )
			{	/* エラー */
				ui_out_on_time = 0;
				ui_out_off_time = 0;
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
			}

			if( send_bit >= 0xF0 )
			{	/* 次の送信データなし */
				/* 出力バッファクリア */
				for( fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
				{
					uc_out_buff[fi] = 0;
				}
				/* 出力待ち */
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
			}
			/* 出力データ位置をインクリメント */
			ui_out_pos++;
		}				
	}

	/* 出力ON時 */
	if( 0 < ui_out_on_time )
	{
		SetDCPWM1(ui_PWM50_Set_Val);
		//SetDCPWM1(PWM_50);
		ui_out_on_time--;
	}
	/* 出力OFF時 */
	else if( 0 < ui_out_off_time )
	{
		SetDCPWM1(PWM_OFF);
		ui_out_off_time--;
	}
	else
	{
		SetDCPWM1(PWM_OFF);
	}

    return ret_code; 
}

/*******************************************************************
 * Function:        int RemoconReceiveData(
 *                        void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          int - A processing result is returned. 0 is normal.
 *
 * Side Effects:    None
 *
 * Overview:        This function receives the data of infrared    
 *                  remote control. The received data is stored 
 *                  in a receiving buffer. 
 *                  0 is returned at the time of normal processing.
 *
 * Note:            None
 *******************************************************************/
int RemoconReceiveData(void)
{
	
	int ret_code = 0;
	int ret = 0;
	int fi = 0;
	unsigned char set_size_data = 0;
	unsigned char set_2nd_size_data = 0;
	unsigned char set_bit_data = 0;
	unsigned int byte_pos = 0;
	unsigned int bit_pos = 0;
	unsigned char bit_mask_data = 1;
	unsigned char first_bit = 0;
	unsigned char second_bit = 0;
	unsigned char samae_bit_count = 0;

	// 製品基板PIN5  試作基板PIN8
//	if( !PIN8 == ON ){
	if( !PIN5 == ON ){
		uc_now_signal = ON;
	}
	else
	{
		uc_now_signal = OFF;
	}

	/* ON/OFFカウント */
	if( uc_now_signal == ON )
	{
		ui_on_count++;
	}
	else
	{
		ui_off_count++;
	}
	
	/* 前回OFF　＆＆　今回ON ＆＆　カウント中 のとき */
	if( (uc_pre_signal == OFF && uc_now_signal == ON && ui_on_count > 0 && ui_off_count > 0)
		|| ( ui_on_count > 0 && ui_off_count > READCODE_END_CNT))
	{
		/* リータコード待ち？ */
		if ( uc_read_status == READSTATUS_READERCODE_WAIT ){

			set_bit_data = 0xFF;

			/* リーダコード受信 */
			/* 判定 */
			/* 家電協判定 */
			if( uc_format_type == FORMATCODE_UNKNOWN )
			{
				if(READERCODE_MIN_KADEN <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_KADEN)
				{
					if( (ui_off_count*(READERCODE_ON_T_KADEN-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_KADEN+1)))
					{
						uc_format_type = FORMATCODE_KADEN;
					}
				}
			}
			/* NEC判定 */
			if( uc_format_type == FORMATCODE_UNKNOWN )
			{
				if(READERCODE_MIN_NEC <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_NEC)
				{
					if( (ui_off_count*(READERCODE_ON_T_NEC-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_NEC+1)))
					{
						uc_format_type = FORMATCODE_NEC;
					}
				}	
			}
			/* SONY判定 */
			if( uc_format_type == FORMATCODE_UNKNOWN )
			{
				if(READERCODE_MIN_SONY <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_SONY)
				{
					if( (ui_off_count*(READERCODE_ON_T_SONY-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_SONY+1)))
					{
						uc_format_type = FORMATCODE_SONY;
					}
				}
			}
			/* DAIKIN判定 */
			if( uc_format_type == FORMATCODE_UNKNOWN )
			{
				if(READERCODE_MIN_DAI <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_DAI)
				{
					if( (ui_off_count*(READERCODE_ON_T_DAI-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_DAI+1)))
					{
						uc_format_type = FORMATCODE_DAIKIN;
					}
				}	
			}
			/* DAIKIN2判定 */
			if( uc_format_type == FORMATCODE_UNKNOWN )
			{
				if(READERCODE_MIN_DAI2 <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_DAI2)
				{
					if( (ui_on_count*(READERCODE_OFF_T_DAI2-1)) < ui_off_count && ui_off_count < (ui_on_count*(READERCODE_OFF_T_DAI2+1)))
					{
						uc_format_type = FORMATCODE_DAIKIN2;
					}
				}
			}
			/* リーダコード無しのため最後に判定すること */
			/* MITSUBISHI判定 リーダコードなしのため、いきなりデータ判定 */
			if( uc_format_type == FORMATCODE_UNKNOWN )
			{
				if(DATACODE_MIN_M <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_M)
				{
					if( 2 < ui_on_count && ui_on_count < 6)
					{
						uc_format_type = FORMATCODE_MITSU;

						if( ui_off_count >= DATACODE_DATA1_M )
						{	/* DATA 1 */
							set_bit_data = ON;
						}
						else
						{	/*DATA 0*/
							set_bit_data = OFF;
						}
					}
				}
			}
		
			/* カウンタリセット */
			ui_on_count = 1;	/* 今回ONなので0ではなく1をセット */
			ui_off_count = 0;

			if( uc_format_type != FORMATCODE_UNKNOWN )
			{	
				for( fi = 0 ; fi < READBUFFER_SIZE_EX ; fi++ ){
					uc_read_buff[fi] = 0;
				}

				/* リーダコードセット */
				uc_read_buff[0] = uc_format_type;
	
				/* 受信状態インクリメント */
				uc_read_status = READSTATUS_DATA_WAIT;
			}
			//リーダコードなしで、いきなりデータの場合
			if( set_bit_data == ON || set_bit_data == OFF )
			{
				if( set_bit_data == ON )
				{
					uc_read_buff[1] = 0x01;
				}
				else if ( set_bit_data == OFF )
				{
					uc_read_buff[1] = 0x00;
				}		
				ui_data_pos = 1;
			}	
		}
		/* データ受信待ち？ */
		else if ( uc_read_status == READSTATUS_DATA_WAIT ){
			
			/* データコード受信 */
			/*  */
			set_bit_data = 0xFF;
		
			/* 判定 */
			/* 家電協判定 */
			if( uc_format_type == FORMATCODE_KADEN )
			{
				if(DATACODE_MIN_KADEN <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_KADEN)
				{
					if( ui_off_count >= DATACODE_DATA1_KADEN )
					{	/* DATA 1 */
						set_bit_data = ON;
					}
					else
					{	/*DATA 0*/
						set_bit_data = OFF;
					}
				}
			}
			/* NEC判定 */
			else if( uc_format_type == FORMATCODE_NEC )
			{
				if(DATACODE_MIN_NEC <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_NEC)
				{
					if( ui_off_count >= DATACODE_DATA1_NEC )
					{	/* DATA 1 */
						set_bit_data = ON;
					}
					else
					{	/*DATA 0*/
						set_bit_data = OFF;
					}
				}
			}
			/* SONY判定 */
			else if( uc_format_type == FORMATCODE_SONY )
			{
				// SONYはトレーラーコードなしのため、終了コード時にもデータ判定する
				if((DATACODE_MIN_SONY <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_SONY)
					|| ( ui_on_count > 0 && ui_off_count > READCODE_END_CNT))
				{
					if( ui_on_count >= DATACODE_DATA1_SONY )
					{	/* DATA 1 */
						set_bit_data = ON;
					}
					else
					{	/*DATA 0*/
						set_bit_data = OFF;
					}
				}
			}
			/* DAIKIN or DAIKIN2 判定 */
			else if( uc_format_type == FORMATCODE_DAIKIN || uc_format_type == FORMATCODE_DAIKIN2 )
			{
				if(DATACODE_MIN_DAI <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_DAI)
				{
					if( ui_off_count >= DATACODE_DATA1_DAI )
					{	/* DATA 1 */
						set_bit_data = ON;
					}
					else
					{	/*DATA 0*/
						set_bit_data = OFF;
					}
				}
			}
			/* MITSUBISHI判定 */
			else if( uc_format_type == FORMATCODE_MITSU )
			{
				if(DATACODE_MIN_M <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_M)
				{
					if( ui_off_count >= DATACODE_DATA1_M )
					{	/* DATA 1 */
						set_bit_data = ON;
					}
					else
					{	/*DATA 0*/
						set_bit_data = OFF;
					}
				}
			}

			if( set_bit_data == ON || set_bit_data == OFF )
			{
				byte_pos = ui_data_pos / 8;
				bit_pos = ui_data_pos % 8;
				
				if(uc_read_code_type == CODE_TYPE_EXTENSION)
				{	// 拡張の時
					if( 0 <= byte_pos && byte_pos < READBUFFER_DATA_AREA_SIZE_EX )
					{
						if( set_bit_data == ON )
						{
							uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] = uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] | (bit_mask_data << bit_pos);
						}
						else if ( set_bit_data == OFF )
						{
							uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] = uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] & ~(bit_mask_data << bit_pos);
						}		
					}
				}
				else
				{	// 標準のときは
					if( 0 <= byte_pos && byte_pos < READBUFFER_DATA_SIZE )
					{
						if( set_bit_data == ON )
						{
							uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] = uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] | (bit_mask_data << bit_pos);
						}
						else if ( set_bit_data == OFF )
						{
							uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] = uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] & ~(bit_mask_data << bit_pos);
						}		
					}
				}
				
				ui_data_pos++;
			}

			if(uc_read_code_type == CODE_TYPE_EXTENSION)
			{	// 拡張
				// データ部が２つに分かれている場合
				if((uc_format_type == FORMATCODE_KADEN || uc_format_type == FORMATCODE_NEC || uc_format_type == FORMATCODE_DAIKIN || uc_format_type == FORMATCODE_DAIKIN2) 
					&& DATA_CODE_INTERVAL_MIN_CNT <= ui_off_count && ui_off_count <= DATA_CODE_INTERVAL_MAX_CNT )
				{
					/* 2ndデータのリーダコード待ちへ */
					uc_read_status = READSTATUS_2ND_READERCODE_WAIT;
					/* 1stデータのデータ長をセット */
					uc_first_data_bit = (unsigned char)(ui_data_pos & 0xFF);
				}
				if(ui_off_count > DATA_CODE_INTERVAL_MAX_CNT || ui_data_pos == DATA_MAX_BITS )
				{
					/* 受信状態インクリメント */
					uc_read_status = READSTATUS_READ_END;
					/* 1stデータのデータ長をセット */
					uc_first_data_bit = (unsigned char)(ui_data_pos & 0xFF);
				}
			}
			else
			{	// 標準
				if(ui_off_count > READCODE_END_CNT || ui_data_pos == DATA_MAX_BITS )
				{
					/* 受信状態インクリメント */
					uc_read_status = READSTATUS_READ_END;
					/* 1stデータのデータ長をセット */
					uc_first_data_bit = (unsigned char)(ui_data_pos & 0xFF);
				}
			}		
	

			/* カウンタリセット */
			ui_on_count = 1;	/* 今回ONなので0ではなく1をセット */
			ui_off_count = 0;
		}
		/* データ受信待ち？ */
		else if ( uc_read_status == READSTATUS_2ND_READERCODE_WAIT )
		{
			/* 家電協判定 */
			if( uc_format_type == FORMATCODE_KADEN )
			{
				if(READERCODE_MIN_KADEN <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_KADEN)
				{
//					if( (ui_off_count*(READERCODE_ON_T_KADEN-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_KADEN+1)))
//					{
//						uc_format_type = FORMATCODE_KADEN;
//					}
					
					/* カウンタリセット */
					ui_on_count = 1;	/* 今回ONなので0ではなく1をセット */
					ui_off_count = 0;

					/* 受信状態インクリメントを2ndデータ受信待ちへ */
					uc_read_status = READSTATUS_2ND_DATA_WAIT;
				}
			}
			/* NEC判定 */
			else if( uc_format_type == FORMATCODE_NEC )
			{
				if(READERCODE_MIN_NEC <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_NEC)
				{
//					if( (ui_off_count*(READERCODE_ON_T_NEC-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_NEC+1)))
//					{
//						uc_format_type = FORMATCODE_NEC;
//					}
					
					/* カウンタリセット */
					ui_on_count = 1;	/* 今回ONなので0ではなく1をセット */
					ui_off_count = 0;

					/* 受信状態インクリメントを2ndデータ受信待ちへ */
					uc_read_status = READSTATUS_2ND_DATA_WAIT;
				}	
			}
			/* DAIKIN判定 */
			else if( uc_format_type == FORMATCODE_DAIKIN )
			{
				if(READERCODE_MIN_DAI <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_DAI)
				{
//					if( (ui_off_count*(READERCODE_ON_T_DAI-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_DAI+1)))
//					{
//						uc_format_type = FORMATCODE_DAIKIN;
//					}

					/* カウンタリセット */
					ui_on_count = 1;	/* 今回ONなので0ではなく1をセット */
					ui_off_count = 0;

					/* 受信状態インクリメントを2ndデータ受信待ちへ */
					uc_read_status = READSTATUS_2ND_DATA_WAIT;
				}	
			}
			/* DAIKIN2判定 */
			else if( uc_format_type == FORMATCODE_DAIKIN2 )
			{
				if(READERCODE_MIN_DAI2 <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_DAI2)
				{
//					if( (ui_on_count*(READERCODE_OFF_T_DAI2-1)) < ui_off_count && ui_off_count < (ui_on_count*(READERCODE_OFF_T_DAI2+1)))
//					{
//						uc_format_type = FORMATCODE_DAIKIN2;
//					}
					
					/* カウンタリセット */
					ui_on_count = 1;	/* 今回ONなので0ではなく1をセット */
					ui_off_count = 0;

					/* 受信状態インクリメントを2ndデータ受信待ちへ */
					uc_read_status = READSTATUS_2ND_DATA_WAIT;
				}
			}


			if( ui_off_count > READCODE_END_CNT || ui_data_pos == DATA_MAX_BITS )
			{
				/* 受信状態インクリメント */
				uc_read_status = READSTATUS_READ_END;
			}
		}
		/* データ受信待ち？ */
		else if ( uc_read_status == READSTATUS_2ND_DATA_WAIT )
		{
			set_bit_data = 0xFF;

			/* 家電協判定 */
			if( uc_format_type == FORMATCODE_KADEN )
			{
				if(DATACODE_MIN_KADEN <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_KADEN)
				{
					if( ui_off_count >= DATACODE_DATA1_KADEN )
					{	/* DATA 1 */
						set_bit_data = ON;
					}
					else
					{	/*DATA 0*/
						set_bit_data = OFF;
					}
				}
			}
			/* NEC判定 */
			else if( uc_format_type == FORMATCODE_NEC )
			{
				if(DATACODE_MIN_NEC <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_NEC)
				{
					if( ui_off_count >= DATACODE_DATA1_NEC )
					{	/* DATA 1 */
						set_bit_data = ON;
					}
					else
					{	/*DATA 0*/
						set_bit_data = OFF;
					}
				}
			}
			/* DAIKIN判定 */
			else if( uc_format_type == FORMATCODE_DAIKIN || uc_format_type == FORMATCODE_DAIKIN2 )
			{
				if(DATACODE_MIN_DAI <= (ui_on_count + ui_off_count)
					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_DAI)
				{
					if( ui_off_count >= DATACODE_DATA1_DAI )
					{	/* DATA 1 */
						set_bit_data = ON;
					}
					else
					{	/*DATA 0*/
						set_bit_data = OFF;
					}
				}
			}


			if( set_bit_data == ON || set_bit_data == OFF )
			{
				byte_pos = ui_data_pos / 8;
				bit_pos = ui_data_pos % 8;
				
				if(uc_read_code_type == CODE_TYPE_EXTENSION)
				{	// 拡張の時
					if( 0 <= byte_pos && byte_pos < READBUFFER_DATA_AREA_SIZE_EX )
					{
						if( set_bit_data == ON )
						{
							uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] = uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] | (bit_mask_data << bit_pos);
						}
						else if ( set_bit_data == OFF )
						{
							uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] = uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] & ~(bit_mask_data << bit_pos);
						}		
					}
				}
				else
				{	// 標準のときは
					if( 0 <= byte_pos && byte_pos < READBUFFER_DATA_SIZE )
					{
						if( set_bit_data == ON )
						{
							uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] = uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] | (bit_mask_data << bit_pos);
						}
						else if ( set_bit_data == OFF )
						{
							uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] = uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] & ~(bit_mask_data << bit_pos);
						}		
					}
				}
				
				ui_data_pos++;
			}

			if( ui_off_count > READCODE_END_CNT || ui_data_pos == DATA_MAX_BITS )
			{
				/* 受信状態インクリメント */
				uc_read_status = READSTATUS_READ_END;
			}

			/* カウンタリセット */
			ui_on_count = 1;	/* 今回ONなので0ではなく1をセット */
			ui_off_count = 0;
		}
	}

	/* データ読み込み完了？ */
	if ( uc_read_status == READSTATUS_READ_END ){
		/* データ確定 */
		if(uc_read_code_type == CODE_TYPE_EXTENSION)
		{	// 拡張の時はそのまま

			if(uc_first_data_bit == (unsigned char)(ui_data_pos & 0xFF))
			{
				set_size_data = uc_first_data_bit;
				set_2nd_size_data = 0;
			}
			else
			{
				set_size_data = uc_first_data_bit;
				set_2nd_size_data = (unsigned char)ui_data_pos - uc_first_data_bit;
				
#if 0
				// First DataとSecond Data数が一致している場合は同じデータかをチェック
				if(set_size_data == set_2nd_size_data )
				{
					samae_bit_count = 0;
					for(fi = 0; fi < set_size_data; fi++)
					{
						byte_pos = fi / 8;
						bit_pos = fi % 8;
						first_bit = (uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] >> bit_pos) & 0x01;
						byte_pos = (set_size_data + fi) / 8;
						bit_pos = (set_size_data + fi) % 8;
						second_bit = (uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] >> bit_pos) & 0x01;
						if(first_bit != second_bit)
						{
							break;
						}
						else
						{
							samae_bit_count++;
						}
					}
					if(set_size_data == samae_bit_count)
					{
						set_2nd_size_data = 0;
					}		
				}
#endif
			}
			// 8bit未満はノイズとして無視
			if(set_size_data < 8)
			{
				set_size_data = 0;
				set_2nd_size_data = 0;
			}
		}
		else
		{	// 標準のときは、1/4
			set_size_data = (unsigned char)((ui_data_pos>>2) & 0xFF);
		}
		
		/* データありの場合（リーダコードなしデータの場合にノイズをひろうことがある） */
		if( set_size_data > 0 )
		{
			for( fi = 0 ; fi < READBUFFER_SIZE_EX ; fi++ ){
				uc_fix_read_buff[fi] = uc_read_buff[fi];
				uc_read_buff[fi] = 0;
			}

			if(uc_read_code_type == CODE_TYPE_EXTENSION)
			{	// 拡張のときは 1バイト目と2バイト目にデータ長をセット
				uc_fix_read_buff[1] = set_size_data;
				uc_fix_read_buff[2] = set_2nd_size_data;
			}
			else
			{	// 標準のとき
				/* [0]の上位4byteにデータ長をセット */
				uc_fix_read_buff[0] |= (unsigned char)((set_size_data << 4) & 0xFF);
			}
		}
		else
		{	/* ノイズをひろった場合は、データクリアして再読み込み */
			for( fi = 0 ; fi < READBUFFER_SIZE_EX ; fi++ ){
				uc_read_buff[fi] = 0;
			}
		}

		/* 初期化 */
		uc_read_status = READSTATUS_NEXT_DATA_WAIT;
		ui_next_data_wait_cnt = 0;
		ui_data_pos = 0;
		uc_first_data_bit = 0;
		uc_format_type = FORMATCODE_UNKNOWN;
	}
	/* 次データ待ち */
	if ( uc_read_status == READSTATUS_NEXT_DATA_WAIT ){
		
		/* OFFが一定時間継続したら次のデータ読み込み */
		if ( uc_now_signal == OFF )
		{
			ui_next_data_wait_cnt++;
		}
		else
		{
			ui_next_data_wait_cnt = 0;
		}	
		/* OFF一定時間継続？ */
		if( ui_next_data_wait_cnt >= NEXT_DATA_WAIT )
		{
			/* リーダコード待ちへ */
			uc_read_status = READSTATUS_READERCODE_WAIT;
			/* カウンタリセット */
			ui_on_count = 0;
			ui_off_count = 0;
		}
	}
		
	/* 今回の信号値を保存 */	
	uc_pre_signal = uc_now_signal;
			
    return ret_code; 
}
/** EOF main.c *************************************************/
#endif
