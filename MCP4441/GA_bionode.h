// MASTER
// MCP4441 Digital Potentiometer
// Georgia Lawlor
// 02/02/2021

#include <nRF.h>

//#define BIONODE_5V3
#define BIONODE_5V0_DEVKIT

#ifndef BIONODE_H
#define BIONODE_H
#endif

#define DEFAULT_RADIO_FREQ 26

//Handshake Packet register positions
#define POS_PID_H 0
#define POS_PID_L 1
#define POS_TX_ID 2
#define POS_RX_ID 3
#define POS_TYPE 4
#define POS_TARGET_ID 5
#define POS_DIGIPOT 6              //VGDP - used for indicating digipot value
#define POS_HANDSHAKE_INTERVAL_H 7 //CONSIDER REMOVING THIS REGISTER
#define POS_HANDSHAKE_INTERVAL_L 8 //CONSIDER REMOVING THIS REGISTER
#define POS_STIM_AMP_H 9
#define POS_STIM_AMP_L 10
#define POS_PULSE_REPEAT_TIME_3 11
#define POS_PULSE_REPEAT_TIME_2 12
#define POS_PULSE_REPEAT_TIME_1 13
#define POS_PULSE_REPEAT_TIME_0 14
#define POS_PULSE_WIDTH_3 15
#define POS_PULSE_WIDTH_2 16
#define POS_PULSE_WIDTH_1 17
#define POS_PULSE_WIDTH_0 18
#define POS_STIM_CYCLE_H 19
#define POS_STIM_CYCLE_L 20
#define POS_ADC_TIMER1_PRESCALE_REG 21
#define POS_ADC_TIMER1_CC0_REG 22
#define POS_ACTIVE_CHANNELS_H 23
#define POS_ACTIVE_CHANNELS_L 24
#define POS_ADC_RESOLUTION 25
#define POS_STIM_TIMER_CONFIG 26         // 0 = Ignore, 1 = Start, 2 = Stop
#define POS_IMPEDANCE_TEST_1_CONFIG 27   // 0 = Ignore, 1 = Start, 2 = Stop
#define POS_IMPEDANCE_TEST_2_CONFIG 28   // 0 = Ignore, 1 = Start, 2 = Stop
#define POS_THERMAL_TEST_CONFIG 29       // 0 = Ignore, 1 = Start, 2 = Stop
#define POS_ADC_CHANNEL_SWITCH_CONFIG 30 //Consider removing this. Legacy // 0 = 1:1, 1 = 1:3, 2 = 1:39
#define POS_POSITIVE_CALIBRATION 31
#define POS_NEGATIVE_CALIBRATION 32
#define POS_POSITIVE_PW_CALIBRATION 33
#define POS_NEGATIVE_PW_CALIBRATION 34
#define POS_ZERO_CALIBRATION 35
#define POS_STIM_WAVEFORM_CONFIG 36 //0 = alternating phase, 1 = biphasic ////VGVG commented to be replaced by decimation
//#define DECIMATION_ON                  36 //VGVG added for decimation variable
#define POS_INTER_PHASIC_DELAY_3 37
#define POS_INTER_PHASIC_DELAY_2 38
#define POS_INTER_PHASIC_DELAY_1 39
#define POS_INTER_PHASIC_DELAY_0 40
#define POS_BATTERY_ON 49

#define POS_STIM_ON_TIME_CNT_3 41
#define POS_STIM_ON_TIME_CNT_2 42
#define POS_STIM_ON_TIME_CNT_1 43
#define POS_STIM_ON_TIME_CNT_0 44

#define POS_STIM_OFF_TIME_CNT_3 45
#define POS_STIM_OFF_TIME_CNT_2 46
#define POS_STIM_OFF_TIME_CNT_1 47
#define POS_STIM_OFF_TIME_CNT_0 48

#ifdef BIONODE_ASIC
#define LARGE_PACKET_LENGTH 190
#define SMALL_PACKET_LENGTH 50
#define RADIO_DATA_OFFSET 5

//ASIC MUX Control Pins
#define ASIC_M0_Pin 14
#define ASIC_M1_Pin 13
#define ASIC_M2_Pin 12
#define ASIC_M3_Pin 11

//ASIC AFE Controls, Ch1-8, S1 & S2 sets gain, S3-S6 Sets low pass cutoff
#define ASIC_S1L_Pin 28 //1
#define ASIC_S2L_Pin 27 //0
#define ASIC_S3L_Pin 10 //1
#define ASIC_S4L_Pin 9  //0
#define ASIC_S5L_Pin 8  //0
#define ASIC_S6L_Pin 7  //0

//ASIC AFE Controls, Ch9-16, S1 & S2 sets gain, S3-S6 Sets low pass cutoff
#define ASIC_S1R_Pin 15
#define ASIC_S2R_Pin 16
#define ASIC_S3R_Pin 17
#define ASIC_S4R_Pin 18
#define ASIC_S5R_Pin 19
#define ASIC_S6R_Pin 20

//ASIC AFE Outputs
#define ASIC_VOUT_Pin 30  //AIN6
#define ASIC_VOUT8_Pin 31 //AIN7
//#define ASIC_VOUT16_Pin  ??

//ASIC Controls. Requires external analog voltages. set to input her only to avoid interference from MCU. Remove if hardware is fixed
#define ASIC_V1L_Pin 4
#define ASIC_V2L_Pin 2
#define ASIC_V1R_Pin 29
#define ASIC_V2R_Pin 3

#endif /*BIONODE_ASIC*/

#ifdef BIONODE_4V0
//Wireless communication constants
#define LARGE_PACKET_LENGTH 190
#define SMALL_PACKET_LENGTH 50
#define RADIO_DATA_OFFSET 5
#define DEFAULT_HANDSHAKE_INTERVAL 100

//Pin numbers
#define STIM_GLITCH_SWITCH 21
#define STIM_FEEDBACK_INPUT 24

//ADC Input channels
#define REC1_AIN 4
#define REC2_AIN 1
#define OPT1_AIN 5
#define OPT2_AIN 0
#define RS_H_AIN 3
#define RS_L_AIN 6

//SPI Pins
#define BIONODE_SPI_CS 15
#define BIONODE_SPI_CLK 25
#define BIONODE_SPI_MOSI 22

#endif /*BIONODE_4V0*/

#ifdef BIONODE_5V0
//Wireless communication constants
#define LARGE_PACKET_LENGTH 190
#define SMALL_PACKET_LENGTH 50
#define HANDSHAKE_PACKET_LENGTH 50
#define RADIO_DATA_OFFSET 5
#define STIM_STATUS_LENGTH 5
#define DEFAULT_HANDSHAKE_INTERVAL 100

#define Data_8bit_Small 0
#define Data_10bit_Small 1
#define Data_8bit_Large 9
#define Data_10bit_Large 10

//Pin numbers
#define STIM_GLITCH_SWITCH 20
#define STIM_FEEDBACK_INPUT 11
#define DAC_POWERED_ON 12
#define CAP_DISCHARGE_SWITCH 22
#define HCP_POWER_SWITCH 23
#define BATT_SW_CTRL 24
//#define TEST_GPIO_OUT 10
//#define TEST2_GPIO_OUT 16
//#define TEST3_GPIO_OUT 15

//ADC Input channels
#define REC1_AIN 2
#define REC2_AIN 0
#define OPT1_AIN 1
#define OPT2_AIN 3
#define RS_H_AIN 6
#define RS_L_AIN 5

//SPI Pins
#define BIONODE_SPI_CS 10
#define BIONODE_SPI_CLK 9
#define BIONODE_SPI_MOSI 8
#endif /*BIONODE_5V0*/

#ifdef BIONODE_5V0_DEVKIT
//Wireless communication constants

#define LARGE_PACKET_LENGTH 190
#define SMALL_PACKET_LENGTH 50
#define HANDSHAKE_PACKET_LENGTH 50
#define RADIO_DATA_OFFSET 5
#define STIM_STATUS_LENGTH 5
#define DEFAULT_HANDSHAKE_INTERVAL 100

#define Data_8bit_Small 0
#define Data_10bit_Small 1
#define Data_12bit_Small 12 //VG12
#define Data_8bit_Large 9
#define Data_10bit_Large 10
#define Data_12bit_Large 11 //VG12


//ADC Input channels
#define REC1_AIN 0
#define WPR0_AIN 1 //GA
#define REC2_AIN 2

#define OPT1_AIN 4
#define RS_H_AIN 5
#define RS_L_AIN 6
#define OPT2_AIN 7


//Pin numbers
#define BATT_SW_CTRL 10
#define CAP_DISCHARGE_SWITCH 11
#define HCP_POWER_SWITCH 12


#define TEST2_GPIO_OUT 15 //15
#define TEST1_GPIO_OUT 16 //16
#define STIM_GLITCH_SWITCH 17

#define BIONODE_SPI_CS 19 //VGDP
#define TEST3_GPIO_OUT 20 //20

#define BIONODE_SPI_CLK 22 //VGDP
#define BIONODE_SPI_MOSI 23 //VGDP
#define DAC_POWERED_ON 24
#define STIM_FEEDBACK_INPUT 25
// #define BIONODE_SPI_VGDP_CS 28 //VGDP
// #define BIONODE_SPI_VGDP_MOSI 04 //VGDP
// #define BIONODE_SPI_VGDP_CLK 29 //VGDP
//GA

#define HVC_A0 13
#define A1 14
#define RST 18
#define WP_LOCK 14





#endif /*BIONODE_5V0*/

#ifdef BIONODE_5V3
//Wireless communication constants
#define LARGE_PACKET_LENGTH 190
#define SMALL_PACKET_LENGTH 50
#define HANDSHAKE_PACKET_LENGTH 50
#define RADIO_DATA_OFFSET 5
#define STIM_STATUS_LENGTH 5
#define DEFAULT_HANDSHAKE_INTERVAL 100

#define Data_8bit_Small 0
#define Data_10bit_Small 1
#define Data_8bit_Large 9
#define Data_10bit_Large 10
#define Data_12bit_Large 11 //VG12
#define Data_12bit_Small 12 //VG12

//Pin numbers
#define STIM_GLITCH_SWITCH 12
#define STIM_FEEDBACK_INPUT 11
// #define DAC_POWERED_ON // 12  This pin removed with 1.8 V DAC
#define CAP_DISCHARGE_SWITCH 22
#define HCP_POWER_SWITCH 23
#define BATT_SW_CTRL 24
#define BATT_VOLTAGE 7
#define BIONODE_GPIO_1 14
#define BIONODE_GPIO_2 15
// #define BIONODE_GPIO_3 16
#define BIONODE_GPIO_4 17
// #define BIONODE_GPIO_5 18
#define BIONODE_GPIO_6 19
#define BIONODE_GPIO_7 20
// #define BIONODE_GPIO_8 21
#define TEST1_GPIO_OUT 18 //SCL
#define TEST2_GPIO_OUT 16 //SDA
#define TEST3_GPIO_OUT 21 //CS

//ADC Input channels
#define REC1_AIN 2 // HW Name: Analog 1
#define REC2_AIN 0 // HW Name: Analog 2
#define OPT1_AIN 1 // HW Name: Analog 3
#define OPT2_AIN 3 // HW Name: Analog 4
#define OPT3_AIN 7 // HW Name: Analog 5
#define OPT4_AIN 4 // HW Name: Analog 6
#define RS_H_AIN 6 // HW Name: IMP_V
#define RS_L_AIN 5 // HW Name: IMP_I

//SPI Pins
#define BIONODE_SPI_CS 10
#define BIONODE_SPI_CLK 9
#define BIONODE_SPI_MOSI 8
#endif /*BIONODE_5V3*/
