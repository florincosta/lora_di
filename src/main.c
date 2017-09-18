/*
 / _____)             _              | |
 ( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
 (______/|_____)_|_|_| \__)_____)\____)_| |_|
 (C)2013 Semtech

 Description: Ping-Pong implementation

 License: Revised BSD License, see LICENSE.TXT file include in the project

 Maintainer: Miguel Luis and Gregory Cristian
 */
/******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    27-February-2017
 * @brief   this is the main!
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "delay.h"
#include "low_power.h"
#include "vcom.h"

#if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( USE_BAND_915 )

#define RF_FREQUENCY                                915000000 // Hz

#else
#error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
#error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE            3000
#define BUFFER_SIZE                 64 // Define the payload size here
#define LED_PERIOD_MS               200

#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;
#define THIRTHY_MIN 1800000L

#define RTC_ASYNCH_PREDIV    0x7C
#define RTC_SYNCH_PREDIV     0x0127
/* subsecond number of bits */
#define N_PREDIV_S                 10
/* Synchonuous prediv  */
#define PREDIV_S                  ((1<<N_PREDIV_S)-1)
/* Asynchonuous prediv   */
#define PREDIV_A                  (1<<(15-N_PREDIV_S))-1

typedef enum {
	ST_SLEEP, ST_RUN,
} fsmStates;

uint8_t buffer[BUFFER_SIZE];
volatile uint32_t rxTimeoutFlag;
volatile uint32_t rxDataFlag;
static fsmStates states = ST_SLEEP;

static volatile uint8_t txDone = false;

/* Led Timers objects*/
static TimerEvent_t timerLed;

/* Private function prototypes -----------------------------------------------*/
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

/*!
 * \brief Function executed on when led timer elapses
 */

static void OnledEvent(void);

/*!
 * \brief Function executed on when led timer elapses
 */
static void OnUserButtonEvent(void);
/**
 * @brief  System Clock Configuration */
static void devSystemClock_ConfigLPM(void);
static void SystemPower_Config(void);

static uint16_t crcTable[] = {

    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,

    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,

    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,

    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,

    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,

    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,

    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,

    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,

    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,

    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,

    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,

    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,

    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,

    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,

    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,

    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,

    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,

    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,

    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,

    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,

    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,

    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,

    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,

    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,

    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,

    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,

    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,

    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,

    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,

    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,

    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,

    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040

};



/**

  * @brief  Calculates CRC on a byte stream.

  * @param  data: pointer to byte stream

  * @param  length: length of the bytes stream

  * @retval crc value

  */

uint16_t calc(const uint8_t* const data, const uint32_t length) {

    uint16_t crc;

    uint32_t index; /* index for CRC lookup table */

    uint32_t i;

    crc = 0xFFFF;

    for(i = 0; i < length; i++) {

        index = (crc & 0xFF) ^ data[i];

        crc >>= 8;

        crc ^= crcTable[index];

    }
    return crc;
}
/**
 * Main application entry point.
 */
int main(void) {
    uint32_t i;
    uint16_t batteryVoltage =0x00UL;
    uint16_t chksum;



	HAL_Init();
	/* Configure the system clock to 2 MHz */
	devSystemClock_ConfigLPM();

	HW_Init();

    __HAL_RCC_ADC1_CLK_SLEEP_DISABLE();
    __HAL_RCC_USART2_CLK_SLEEP_DISABLE();
    __HAL_RCC_GPIOA_CLK_SLEEP_DISABLE();
    __HAL_RCC_GPIOB_CLK_SLEEP_DISABLE();

	// Radio initialization
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;

	Radio.Init(&RadioEvents);
	Radio.SetChannel(RF_FREQUENCY);

#if defined(USE_MODEM_LORA)

	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
	LORA_SPREADING_FACTOR, LORA_CODINGRATE,
	LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
	true, 0, 0, LORA_IQ_INVERSION_ON, 3000000);

	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
	LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
	LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
	LORA_IQ_INVERSION_ON, true);

#elif defined(USE_MODEM_FSK)

	Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
			FSK_DATARATE, 0,
			FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
			true, 0, 0, 0, 3000000 );

	Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
			0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
			0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
			0, 0,false, true );

#else
#error "Please define a frequency band in the compiler options."
#endif

	//__HAL_FLASH_PREFETCH_BUFFER_DISABLE();
#define SENDER
#ifdef SENDER

    /* Insert 3 seconds delay */
    HAL_Delay(3000);

	while (true) {

	        switch(states){
	            case ST_SLEEP:{

	                /* User push-button (External lines 4 to 15) will be used to wakeup the system from SLEEP mode */
	                BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
	                BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
	                HW_GPIO_SetIrq(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0,
	                OnUserButtonEvent);

	                Radio.Sleep();
	                HW_RTC_SetAlarm(THIRTHY_MIN);
                    states = ST_RUN;
                    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	            }
	            break;

              case ST_RUN: {
                  /* Reconfigures system clock after wake-up from STOP */
                  devSystemClock_ConfigLPM();
                  HW_RTC_StopAlarm();
                  /* Resume Tick interrupt if disabled prior to sleep mode entry*/
                  HAL_ResumeTick();

                  HAL_Delay(20);

                  batteryVoltage = HW_GetBatteryVoltage();
                  buffer[0] = BSP_PB_GetState(BUTTON_USER);
                  buffer[1] = (uint8_t)((batteryVoltage & 0xFF00) >> 8);
                  buffer[2] = (uint8_t)(batteryVoltage & 0x00FF);

                  /*checksum computation modbus*/
                  chksum = 0x00UL;
                  chksum = calc(&buffer[0],3);
                  buffer[3] = (chksum & 0xFF00)>>8;
                  buffer[4] = (chksum & 0x00FF);

                  // send  button state
                  PRINTF(buffer);
                  PRINTF("\n");

                  txDone = false;
                  Radio.Send(buffer, BUFFER_SIZE);

                  while(!txDone) {
                      DelayMs(1);
                  }

                  states = ST_SLEEP;
              }
                  break;
	        }
	}

#endif

//#define RECEIVER
#ifdef RECEIVER
	while(true) {
		rxTimeoutFlag = false;
		rxDataFlag = false;
		Radio.Rx(RX_TIMEOUT_VALUE);
		while(!rxTimeoutFlag && !rxDataFlag);
	}
#endif
}

/*
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = MSI
 *            SYSCLK(Hz)                     = 2000000
 *            HCLK(Hz)                       = 2000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            APB2 Prescaler                 = 1
 *            Flash Latency(WS)              = 0
 *            Main regulator output voltage  = Scale3 mode
 * @retval None
 */
void devSystemClock_ConfigLPM(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };

	/* Enable MSI Oscillator */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
	RCC_OscInitStruct.MSICalibrationValue = 0x00;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		/* Initialization Error */
		while (1);
	}

	/* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		/* Initialization Error */
		while (1);
	}
	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is
	 clocked below the maximum system frequency, to update the voltage scaling value
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/* Disable Power Control clock */
	__HAL_RCC_PWR_CLK_DISABLE();

}

void OnTxDone(void) {
//    Radio.Sleep();
    txDone = true;
	PRINTF("OnTxDone\n");
}

void OnTxTimeout(void) {
//    Radio.Sleep();
    txDone = true;
    PRINTF("OnTxTimeout\n");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
	char str[10] = { 0 };
	uint32_t i;

	//check for board unique ID

	for (i = 0; (i < size) && (i < 9); i++) {
		str[i] = payload[i];
	}

	rxDataFlag = true;
	PRINTF(str);
	PRINTF("\n");
	PRINTF("OnRxDone\n");
	PRINTF("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);
}



void OnRxTimeout(void) {
	Radio.Sleep();
	rxTimeoutFlag = true;
	PRINTF("OnRxTimeout\n");
}

void OnRxError(void) {
	Radio.Sleep();
	rxTimeoutFlag = true;
	PRINTF("OnRxError\n");
}

static void OnledEvent(void) {
	LED_Toggle( LED_BLUE ) ;LED_Toggle( LED_RED1 ) ;LED_Toggle( LED_RED2 ) ;LED_Toggle( LED_GREEN ) ;

	TimerStart(&timerLed);
}

void OnUserButtonEvent(void) {
	PRINTF("UserButton\n");
}

/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
  *            + Regulator in LP mode
  *            + VREFINT OFF, with fast wakeup enabled
  *            + HSI as SysClk after Wake Up
  *            + No IWDG
  *            + Automatic Wakeup using RTC clocked by LSI (after ~4s)
  * @param  None
  * @retval None
  */
static void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  /* Note: Debug using ST-Link is not possible during the execution of this   */
  /*       example because communication between ST-link and the device       */
  /*       under test is done through UART. All GPIO pins are disabled (set   */
  /*       to analog input mode) including  UART I/O pins.           */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

//  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  //HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  //HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  //HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  //HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
  //HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);


  /* Disable GPIOs clock */
//  __HAL_RCC_GPIOA_CLK_DISABLE();
//  __HAL_RCC_GPIOB_CLK_DISABLE();
//  __HAL_RCC_GPIOC_CLK_DISABLE();
//  __HAL_RCC_GPIOD_CLK_DISABLE();
//  __HAL_RCC_GPIOE_CLK_DISABLE();
//  __HAL_RCC_GPIOH_CLK_DISABLE();


//  /* Configure RTC */
//  RTCHandle.Instance = RTC;
//  /* Configure RTC prescaler and RTC data registers as follow:
//    - Hour Format = Format 24
//    - Asynch Prediv = Value according to source clock
//    - Synch Prediv = Value according to source clock
//    - OutPut = Output Disable
//    - OutPutPolarity = High Polarity
//    - OutPutType = Open Drain */
//  RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
//  RTCHandle.Init.AsynchPrediv = PREDIV_A;
//  RTCHandle.Init.SynchPrediv = PREDIV_S;
//  RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
//  RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//  RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//  if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler();
//  }
}

