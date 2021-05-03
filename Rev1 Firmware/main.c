// MASTER
///WORKING in 8bit and 10-bit, high sample. - VG 06/30
// HIGH SAMPLE
// NEW DAC 2.17.2020 - VG
// Bionode 5V3 Dual Sample
#include <__cross_studio_io.h>
#include <nrf.h>
#include "spi_master_config.h"
#include "spi_master.h"
#include "bionode.h"

#include "twi_master.h" //GA
// #include "nrf_delay.h"
// #include "nrf_gpio.h"


//Function declarations
int ADC_Init(void);
int Radio_Init(uint8_t Freq);
int Timer0_Init(void);
int Timer1_Init(void);
int Timer2_Init(void);
int Timer3_Init(void);
int GPIO_Init(void);
int PPI_Init(void);
int RTC0_Init(void);
int RTC1_Init(void);
int I2C_Init(void); //GA
void Stop_Stim(void);
void SetStimGlitchSwitch(_Bool on);
void I2C_PP(void); //GA

//Global variables
bool initializer; //GA
bool successful; //GA
uint16_t DP_chCounter = 0; //GA
uint16_t DP_maxVal; //GA
uint16_t DP_minVal; //GA
uint8_t reg_cmd[1]; //GA

_Bool startup = 0;
_Bool stimFbPin;
// uint16_t DacZeroLevel = 2048; old dac
uint16_t DacZeroLevel = 128; //VG NEW DAC
uint16_t positiveAmplitudeCode = 0;
uint16_t negativeAmplitudeCode = 0;
uint16_t positiveStimCode = 0;
uint16_t negativeStimCode = 0;
uint32_t PRT = 3000;
uint32_t PW = 1000;
uint32_t IPD = 100;
int8_t zeroCalibration = 0;
int8_t posCalibration = 0;
int8_t negCalibration = 0;
uint32_t stimCount = 0;
uint32_t duration = 50;
static _Bool Reset_ADC_Counter_Flag = 0; //Indicates whether ADC counter should be reset. This will occur when user switches recording channels
uint16_t ADC_Active_Ch = 0x2121; //(AIN4, AIN1,AIN4,AIN1)
uint8_t RADIO_PACKET_LENGTH = SMALL_PACKET_LENGTH;  // Reserve enough memory for the largest packet size possible
static uint32_t LARGE_PACKET_PAYLOAD = LARGE_PACKET_LENGTH - RADIO_DATA_OFFSET - STIM_STATUS_LENGTH; // Number of payload data points in a Large packet
static uint32_t LARGE_PACKET_PAYLOAD_10Bit = (LARGE_PACKET_LENGTH - RADIO_DATA_OFFSET - STIM_STATUS_LENGTH) * 4 / 5;
static uint32_t SMALL_PACKET_PAYLOAD = SMALL_PACKET_LENGTH - RADIO_DATA_OFFSET - STIM_STATUS_LENGTH; // Number of payload data points in a Small packet
static uint32_t SMALL_PACKET_PAYLOAD_10Bit = (SMALL_PACKET_LENGTH - RADIO_DATA_OFFSET - STIM_STATUS_LENGTH) * 4 / 5;
_Bool changePktLen = 0;
_Bool resetPID = 0;
uint8_t ADC_Results_A[LARGE_PACKET_LENGTH]; //Radio buffer A
uint8_t ADC_Results_B[LARGE_PACKET_LENGTH]; //Radio buffer B
uint16_t ADC_Results;
uint32_t PACKET_PAYLOAD_LENGTH = LARGE_PACKET_LENGTH - RADIO_DATA_OFFSET - STIM_STATUS_LENGTH; // Initally default to (large, VG) small packets (Legacy)
volatile uint8_t RadioRxData[LARGE_PACKET_LENGTH]; //Radio RX buffer, reserved space for largest packet
static uint8_t ADC_Resolution = 8; //Current resolution of the ADC.
const uint8_t Mask1[4]={0xff, 0x3f, 0x0f, 0x3}; //Mask1 used for packet stuffing 10-bit samples
const uint8_t Mask2[4]={0xc0, 0xf0, 0xfc, 0xff}; //Mask2 used for packet stuffing 10-bit samples
uint16_t PID_Global = 0;
static _Bool isHandshakeOK = 0;
uint16_t Handshake_Interval = 1; // Set handshake to 1 for startup so it always listens for first handshake
static uint8_t isRxEvent = 0;
uint8_t tx_data[TX_RX_MSG_LENGTH]; //!< SPI TX buffer
uint8_t rx_data[TX_RX_MSG_LENGTH]; //!< SPI RX buffer
uint32_t *spi_base_address;
_Bool isStimOn = 0;
uint8_t PESLP_REGISTERS[8] = {SAADC_CH_PSELP_PSELP_AnalogInput0, SAADC_CH_PSELP_PSELP_AnalogInput1, SAADC_CH_PSELP_PSELP_AnalogInput2, SAADC_CH_PSELP_PSELP_AnalogInput3, SAADC_CH_PSELP_PSELP_AnalogInput4, SAADC_CH_PSELP_PSELP_AnalogInput5, SAADC_CH_PSELP_PSELP_AnalogInput6, SAADC_CH_PSELP_PSELP_AnalogInput7};
uint32_t SAADC_Active_PSELP_Registers[4];
_Bool DACIsOn = false;
uint16_t DACLimit = 100;
_Bool DACZeroCalDirection_Up = true;
uint16_t lastMaxValue = 0;
uint16_t lastMinValue = 0;
uint32_t stimOnTimeCnt = 655360;
uint32_t stimOffTimeCnt = 1310720;

void main(void)
{
 /* //Disabling NFC protection to let us use P09 and P10 as GPIOs.

  //Watchdog disable
  NRF_MWU->REGIONENCLR = ((MWU_REGIONENCLR_RGN0WA_Clear << MWU_REGIONENCLR_RGN0WA_Pos) | (MWU_REGIONENCLR_PRGN0WA_Clear << MWU_REGIONENCLR_PRGN0WA_Pos));

  
  uint32_t data = 0xFFFFFFFE;
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
  NRF_UICR->NFCPINS = data;
  //*(uint32_t *)0x1000120C = data;
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}


  //Watchdog enable
  NRF_MWU->REGIONENSET = ((MWU_REGIONENSET_RGN0WA_Set << MWU_REGIONENSET_RGN0WA_Pos) | (MWU_REGIONENSET_PRGN0WA_Set << MWU_REGIONENSET_PRGN0WA_Pos));

*/
  //First, start up the high frequency clock, and wait for it to start before moving on
  NRF_CLOCK->TASKS_HFCLKSTART = 0x01;
  while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0){
  }

  //Tell the NRF52832 to use the high frequency clock to synthesize the low frequency clock.
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Synth;

  //Start the low frequency clock, and wait for it to start before moving on.
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
  while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0){
  }

  //Ensure that the processor returns to sleep after executing all pending interrupt handlers
  SCB->SCR = 1 << SCB_SCR_SLEEPONEXIT_Pos;

  //Initialize all peripherals
  ADC_Init();
  Radio_Init(DEFAULT_RADIO_FREQ);
  Timer1_Init();
  Timer0_Init();
  Timer2_Init();
  GPIO_Init();
  PPI_Init();
  RTC0_Init();
  RTC1_Init();
  Timer3_Init();
  I2C_Init();  //GA


  spi_base_address = spi_master_init(0, SPI_MODE0, false);
  // tx_data[0] = (DacZeroLevel>>8);
  // tx_data[1] = DacZeroLevel&0xFF; // old dac
  tx_data[0] = (DacZeroLevel>>2); //VG NEW DAC
  tx_data[1] = (6<<DacZeroLevel)&0xFF; //VG NEW DAC

  spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);

  //Set interrupt handler priorities.
  NVIC_SetPriority(TIMER0_IRQn, 0);
  NVIC_SetPriority(RADIO_IRQn, 1);
  NVIC_SetPriority(SAADC_IRQn, 2);
  NVIC_SetPriority(TIMER1_IRQn, 3);
  NVIC_SetPriority(TIMER2_IRQn, 3);
  NVIC_SetPriority(RTC0_IRQn, 3);
  NVIC_SetPriority(RTC1_IRQn, 3);
  NVIC_SetPriority(TIMER3_IRQn, 3);

  //testing priority setup
  //for (int i = 0; i < 39; ++ i)
  //  NVIC_SetPriority(i,1);

  NVIC_SetPriority(TIMER0_IRQn, 0);

  //Enable interrupt handlers
  NVIC_EnableIRQ(SAADC_IRQn);
  NVIC_EnableIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RTC0_IRQn);
  NVIC_EnableIRQ(RTC1_IRQn);
  NVIC_EnableIRQ(TIMER3_IRQn);
  
  //Start the ADC and the timer that triggers it as well as the real-time counter
  NRF_SAADC->TASKS_START = 1;
  NRF_TIMER1->TASKS_START = 1;
  NRF_RTC0->TASKS_START = 1;

  SetStimGlitchSwitch(false);

  // Set timer3 to delay stim glitch swich conneciton (DAC glitch)
  NRF_TIMER3->TASKS_STOP = 1;
  NRF_TIMER3->TASKS_CLEAR = 1;
  NRF_TIMER3->TASKS_START = 1;


  //Now, wait for interrupts. The processor should sleep while waiting.
  while(1)
  {
    __WFI();
  }
}

int ADC_Init(void)
{
  //Put initial values into the SAADC_Active_PSELP_Register list
  SAADC_Active_PSELP_Registers[0] = PESLP_REGISTERS[RS_H_AIN];
  SAADC_Active_PSELP_Registers[1] = PESLP_REGISTERS[RS_L_AIN]; 
  SAADC_Active_PSELP_Registers[2] = PESLP_REGISTERS[RS_H_AIN];
  SAADC_Active_PSELP_Registers[3] = PESLP_REGISTERS[RS_L_AIN];

  //Set the initial ADC resolution to be 8-bit
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_8bit << SAADC_RESOLUTION_VAL_Pos;

  //Set the analog input to the first entry in the active saadc channel list
  NRF_SAADC->CH[0].PSELP = SAADC_Active_PSELP_Registers[0] << SAADC_CH_PSELN_PSELN_Pos;

  //Save all data from the ADC into the global ADC_Result buffer
  NRF_SAADC->RESULT.PTR = (int)&ADC_Results;

  //Set the maximum number of buffer words to transfer to 1
  NRF_SAADC->RESULT.MAXCNT = 1;

  //Instruct the ADC to generate an interrupt whenever it finishes taking a sample
  //TODO: Set up DMA to make the ADC load the radio buffer without needing an interrupt to do so
  NRF_SAADC->INTENSET = SAADC_INTEN_END_Enabled << SAADC_INTEN_END_Pos;

  //Configure channel 0 to be:
  //Mode:               single ended
  //Acquisition time:   3 us
  //Reference:          internal reference
  //Gain:               1/3 gain
  //Negative Resistor:  Bypass
  //Positive Resistor:  Bypass
  NRF_SAADC->CH[0].CONFIG =(SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
                           (SAADC_CH_CONFIG_TACQ_3us << SAADC_CH_CONFIG_TACQ_Pos) |
                           (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                           (SAADC_CH_CONFIG_GAIN_Gain1_3 << SAADC_CH_CONFIG_GAIN_Pos) |
                           (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) | 
                           (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos);
  
  //Enable the ADC
  NRF_SAADC->ENABLE = 1;

  return 0;
}

int Radio_Init(uint8_t frequency)
{
  //Power on the radio
  NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;

  //Enable shortcuts between ready/start and end/disable. 
  //This causes the radio to automatically start a transmission when the radio is ready, 
  //and automatically disable itself when it ends a transmission.
  NRF_RADIO->SHORTS = ( RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos |
                        RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

  //Enable a radio interrupt for an END event 
  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Enabled << RADIO_INTENSET_END_Pos;

  //Set the starting address of the payload to be the global ADC_Results_A buffer
  NRF_RADIO->PACKETPTR = (int)&ADC_Results_A;

  //Set the TX power to +4dBm
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos;

  //Set the on-air data rate to 2Mbps
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;

  //Set S0 and S1 length to be 0. Commented out because I don't think we need this.
  NRF_RADIO->PCNF0 = 0x00;


  //Set the Packet Configure Register 1 to:
  //Maximum Packet Length:    RADIO_PACKETLSENGTH bytes
  //Static Packet Length:     RADIO_PACKETLSENGTH bytes
  //Base address length:      2 bytes
  //Endian:                   Big, meaning MSB on air first
  //Packet Whiteening:        Disabled
  //declare space for a Large packet length
  NRF_RADIO->PCNF1 = (LARGE_PACKET_LENGTH << RADIO_PCNF1_MAXLEN_Pos) |
                     (LARGE_PACKET_LENGTH << RADIO_PCNF1_STATLEN_Pos)|
                     (0x04 << RADIO_PCNF1_BALEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
                     (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);

  //Set the base 0 address to 0x80201234
  NRF_RADIO->BASE0 = 0x80201234<<RADIO_BASE0_BASE0_Pos;

  //Set the prefix for AP0 to 0x80
  NRF_RADIO->PREFIX0 = 0x80 << RADIO_PREFIX0_AP0_Pos;

  //Set the TX Address to 0x00
  NRF_RADIO->TXADDRESS = 0x00<<RADIO_TXADDRESS_TXADDRESS_Pos;

  //Enable RX Address ADDR0
  NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled<<RADIO_RXADDRESSES_ADDR0_Pos;

  //Set a 2-byte CRC
  NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos;

  //Set the initial value of the CRC to 0xFFFF
  NRF_RADIO->CRCINIT = 0xFFFF << RADIO_CRCINIT_CRCINIT_Pos;

  //Set the CRC Polynomial to x^16 + x^12 + x^5 + x
  NRF_RADIO->CRCPOLY = 0x11021 << RADIO_CRCPOLY_CRCPOLY_Pos;

  //Set the communication frequency of the radio
  NRF_RADIO->FREQUENCY = frequency << RADIO_FREQUENCY_FREQUENCY_Pos;

  //don't know what this does...commenting it out for now.
  NRF_RADIO->EVENTS_BCMATCH = 0;

  return 0;
}

int RTC0_Init(void)
{
  //Set the counter clock to equal 32768/(1+2^0) = 32.768 kHz
  NRF_RTC0->PRESCALER = 0;

  //Set the compare event register value to trigger an event 100 times per second
  NRF_RTC0->CC[0] = 3277;

  //Enable Compare 0 routing
  NRF_RTC0->EVTEN = RTC_EVTEN_COMPARE0_Enabled << RTC_EVTEN_COMPARE0_Pos;

  //Enable the Compare 0 event interrupt
  NRF_RTC0->INTENSET = RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;

  return 0;
}

int RTC1_Init(void)
{
  NRF_RTC1->PRESCALER = 0;
  NRF_RTC1->CC[0]= stimOnTimeCnt;
  NRF_RTC1->EVTEN= 1<<16;
  NRF_RTC1->INTENSET =1<<16;
  return 0;
}

int Timer0_Init(void)
{
  //This is the stimulation timer. It controls the pulse width (PW) and pulse-repeat time timing (PRT) of the stimulator.

  //Cause the Compare[0] event register to automatically reset Timer 0
  NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

  //Set the timer mode to timer.
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

  //Set the timer clock to equal 16MHz/2^4 = 1MHz.
  NRF_TIMER0->PRESCALER = 4<<TIMER_PRESCALER_PRESCALER_Pos;

  //Set the timer to be a 32-bit timer to allow for long PRTs.
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;

  //Set initial compare event register values.
  NRF_TIMER0->CC[0] = PRT;
  NRF_TIMER0->CC[1] = PRT-PW;
  NRF_TIMER0->CC[2] = PRT>>1;
  NRF_TIMER0->CC[3] = (PRT>>1)-PW;

  //Enable Compare0, Compare1, Compare2, and Compare3 events
  NRF_TIMER0->INTENSET =  (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos) |
                          (TIMER_INTENSET_COMPARE1_Set << TIMER_INTENSET_COMPARE1_Pos) |
                          (TIMER_INTENSET_COMPARE2_Set << TIMER_INTENSET_COMPARE2_Pos) |
                          (TIMER_INTENSET_COMPARE3_Set << TIMER_INTENSET_COMPARE3_Pos);
  
  return 0;
}

int Timer1_Init(void)
{
  //Timer1 is used to initiate ADC samples.
  
  //Cause the Compare[0] event register to automatically reset Timer 1
  NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

  //Set the timer mode to timer.
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

  //Set the timer clock to equal 16MHz/2^2 = 1 MHz, VGVGVG
  NRF_TIMER1->PRESCALER = 2 << TIMER_PRESCALER_PRESCALER_Pos;

  //Set the timer to be a 32-bit timer
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;

  //Initially set the sample rate to be 20kHz, VGVG
  NRF_TIMER1->CC[0] = 200; //was 125 VGVG

  //Note: We do not want to enable the Compare0 interrupt because it is already connected
  //to the ADC via the PPI.

  return 0;
}

int Timer2_Init(void)
{
  //Timer2 is used to time the handshake interval.
  
  //Cause the Compare[0] event register to automatically stop Timer 2
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_STOP_Enabled << TIMER_SHORTS_COMPARE0_STOP_Pos;

  //Set the timer mode to timer.
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

  //Set the timer clock to equal 16MHz/2^7 = 125 kHz
  NRF_TIMER2->PRESCALER = 7 << TIMER_PRESCALER_PRESCALER_Pos;

  //Set the timer to be a 16-bit timer
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;

  //This will cause a CC[0] event 1.92 ms after the timer is started, which will disable the radio
  //if it hasn't heard back from the Base Station. Lower limit is 51, some time is added for safety
  NRF_TIMER2->CC[0] = 65;

  //Note: We do not want to enable the Compare0 interrupt because it is already connected
  //to the Radio via the PPI.

  return 0;
}

int Timer3_Init(void)
{
  //Timer3 is used to force a delay between sensing the DAC is powered on and connecting its output to the HCP.
  
  //Cause the Compare[0] event register to automatically stop and clear Timer 3
  NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos | 
                        TIMER_SHORTS_COMPARE0_STOP_Enabled << TIMER_SHORTS_COMPARE0_STOP_Pos;

  //Set the timer mode to timer.
  NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

  //Set the timer clock to equal 16MHz/2^7 = 125 kHz
  NRF_TIMER3->PRESCALER = 7 << TIMER_PRESCALER_PRESCALER_Pos;

  //Set the timer to be a 16-bit timer
  NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;

  //Count to 125 in order to make a 1-ms delay between the DACPOWERED flag going high and bypassing the stim-glitch-switch
  NRF_TIMER3->CC[0] = 12500;

  //Enable the Compare0 event.
  NRF_TIMER3->INTENSET =  (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos);

  return 0;
}

int PPI_Init(void)
{
  //First, set up the Timer1 compare 0 event to trigger an ADC sample event using the
  //Programmable Peripheral Interconnect (PPI) channel 2. shortcuts an external event to trigger another interrupt without waking up CPU. VG

  //Set the channel 2 event endpoint to the Timer1 Event_Compare[0] event register.
  NRF_PPI->CH[2].EEP = (int)&NRF_TIMER1->EVENTS_COMPARE[0];

  //Set the channel 2 task end-point to the ADC Sample task register
  NRF_PPI->CH[2].TEP = (int)&NRF_SAADC->TASKS_SAMPLE;

  //Enable the channel 2 PPI
  NRF_PPI->CHENSET = PPI_CHENSET_CH2_Enabled << PPI_CHENSET_CH2_Pos;

  //Now, we'll set up Timer2 compare 0 event to force the radio to shutdown Rx, which
  //ends a handshake. We'll use PPI channel 1 to do this.

  //Set the channel 1 event endpoint to the Timer2 Event_Compare[0] event register
  NRF_PPI->CH[1].EEP = (int)&NRF_TIMER2->EVENTS_COMPARE[0];
 
  //Set the channel 1 task end-point to the Radio disable task register
  NRF_PPI->CH[1].TEP = (int)&NRF_RADIO->TASKS_DISABLE;

  //Enable the channel 1 PPI
  NRF_PPI->CHENSET = PPI_CHENSET_CH1_Enabled << PPI_CHENSET_CH1_Pos;

  return 0;
}

int GPIO_Init(void)
{
  //Set the Stim Glitch Swith GPIO to be an output pin
  NRF_GPIO->PIN_CNF[STIM_GLITCH_SWITCH] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << STIM_GLITCH_SWITCH;

  //Set the CAP_DISCHARGE_SWITCH to be an output pin
  NRF_GPIO->PIN_CNF[CAP_DISCHARGE_SWITCH] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << CAP_DISCHARGE_SWITCH;

  //Set the HCP_POWER_SWITCH to be an output pin
  NRF_GPIO->PIN_CNF[HCP_POWER_SWITCH] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << HCP_POWER_SWITCH;

  //Set the BATT_SW_CTRL output pin to be an output, 
  NRF_GPIO->PIN_CNF[BATT_SW_CTRL] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << BATT_SW_CTRL;
  NRF_GPIO->OUT &= ~(1<<BATT_SW_CTRL); // set low

  //Tell the Stim Glitch Switch to switch to initially not be bypassed
  //NRF_GPIO->OUT &= ~(1<<STIM_GLITCH_SWITCH);
  SetStimGlitchSwitch(false);

  //TESTING!!!!!!!!!!!!!!!!!!!!!!
  //Initially start up by turning on the HCP after closing the cap discharge switch
  NRF_GPIO->OUT &= ~(1<<HCP_POWER_SWITCH);

  //Set the stim feedback pin to be an input pin
  NRF_GPIO->PIN_CNF[STIM_FEEDBACK_INPUT] = GPIO_PIN_CNF_DIR_Input;

  // TM BN_5V3 Update: This GPIO Pin removed with 1.8 V DAC.  Left for reference for later dev
  // //Configure the GPIOTE register 0 to trigger an event on the next edge of the DAC_POWERED_ON pin
  // NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_IN0_Clear<<GPIOTE_INTENCLR_IN0_Pos;
  // NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Event<<GPIOTE_CONFIG_MODE_Pos |
  //                           DAC_POWERED_ON<<GPIOTE_CONFIG_PSEL_Pos |
  //                           GPIOTE_CONFIG_POLARITY_Toggle<<GPIOTE_CONFIG_POLARITY_Pos;
  // //Enable the event interrupt
  // NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Set<<GPIOTE_INTENSET_IN0_Pos;

  /* //GA */
  //Set the Wiper Lock GPIO to be an output pin
  //NRF_GPIO->PIN_CNF[DP_WP_LOCK] = GPIO_PIN_CNF_DIR_Output;
  //NRF_GPIO->DIRSET |= 1 << DP_WP_LOCK;
  //NRF_GPIO->OUTSET = (1 << DP_WP_LOCK); // set high

  ////Set up the Reset GPIO to be an output pin
  //NRF_GPIO->PIN_CNF[DP_RST] = GPIO_PIN_CNF_DIR_Output;
  //NRF_GPIO->DIRSET |= 1 << DP_RST;
  //NRF_GPIO->OUTSET = (1 << DP_RST); // set high

  ////Set up the A1 GPIO to be an output pin
  //NRF_GPIO->PIN_CNF[DP_A1] = GPIO_PIN_CNF_DIR_Output;
  //NRF_GPIO->DIRSET |= 1 << DP_A1;
  //NRF_GPIO->OUTCLR = (1 << DP_A1); // set low

  ////Set up the High Voltage Control/A0 GPIO to be an output
  //NRF_GPIO->PIN_CNF[DP_HVC_A0] = GPIO_PIN_CNF_DIR_Output;
  //NRF_GPIO->DIRSET |= 1 << DP_HVC_A0;
  //NRF_GPIO->OUTCLR = (1 << DP_HVC_A0); // set low

  //Set up the Transfer Success? GPIO to be an output
  NRF_GPIO->PIN_CNF[DP_SUCCESS] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << DP_SUCCESS; 
  NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low

  return 0;
}

int I2C_Init(void) //GA
{
  initializer = twi_master_init();
  DP_maxVal = DacZeroLevel;
  DP_minVal = DacZeroLevel;
  return 0;
}

void RADIO_IRQHandler(void)
{
  static uint32_t temp_ptr; //temporary radio buffer pointer
  //First, disable the radio interrupt
  NVIC_DisableIRQ(RADIO_IRQn);

  //Clear the radio end event register
  NRF_RADIO->EVENTS_END = 0;

  //We are not expecting a handshake packet, and we do not have to prepare to receive one, so do nothing.
  if (isRxEvent == 2)
  {
    NVIC_EnableIRQ(RADIO_IRQn);
    //NRF_GPIO->OUT &= ~(1<<TEST2_GPIO_OUT);
    return;
  }

  //This interrupt was generated by a Tx End event occurring after sending a packet that requires a handshake to follow.
  //Prepare the radio to receive a handshake.
  if (!isRxEvent)
  {
    //Set RxEvent flag to 1
    isRxEvent = 1;
    temp_ptr=NRF_RADIO->PACKETPTR;
    NRF_RADIO->PACKETPTR = (int)&RadioRxData;//Switch to Rx pointer

    // Wait for the previous packet to finish sending before preparing recieve
    while(NRF_RADIO->STATE != 0) { }

    // Set the radio to recieve a standard handshake packet length
    NRF_RADIO->PCNF1 = (NRF_RADIO->PCNF1 & ~(0xFFFF)) |
                       (HANDSHAKE_PACKET_LENGTH << RADIO_PCNF1_MAXLEN_Pos) |
                       (HANDSHAKE_PACKET_LENGTH << RADIO_PCNF1_STATLEN_Pos);

    //Restart the handshake timer
    NRF_TIMER2->TASKS_STOP = 0x01;
    NRF_TIMER2->TASKS_CLEAR = 0x01;
    NRF_TIMER2->TASKS_START = 0x01;

    //Set the radio to recieve to prepare for handshake
    //NRF_GPIO->OUT &= ~(1<<TEST2_GPIO_OUT);
    NRF_RADIO->TASKS_RXEN = 1;
  }

  //This interrupt was generated by a received handshake packet.
  else
  {
    //If this is a valid radio packet, continue
    if (NRF_RADIO->CRCSTATUS == RADIO_CRCSTATUS_CRCSTATUS_CRCOk)
    {
      //indicate that we got a good handshake packet
      //NRF_GPIO->OUT |=  (1<<TEST2_GPIO_OUT);
      isHandshakeOK = 1;

      if (RadioRxData[1] != 0) //make sure the PID is not zero. This would indicate a non-updating handshake
      {
        //Update all Bionode registers using handshake packet information
         
        Handshake_Interval =(RadioRxData[POS_HANDSHAKE_INTERVAL_H]<<8) | RadioRxData[POS_HANDSHAKE_INTERVAL_L];
        if (RadioRxData[POS_BATTERY_ON] == 0x1)
        {
          NRF_GPIO->OUT &= ~(1<<BATT_SW_CTRL);
        }
        else
        {
          NRF_GPIO->OUT |= 1<<BATT_SW_CTRL;
        }

        ////VGVGVG ON SWITCH
        // if (RadioRxData[DECIMATION_ON] == 0x1)
        // {
        //   NRF_GPIO->OUT &=  
        // }
        // else
        // {
          
        // }

        //////

        //If the   packet indicates that we should START stimulating,
        if(RadioRxData[POS_STIM_TIMER_CONFIG] == 1)
        {
          //TESTING!!!! Turn on HCP first
          NRF_GPIO->OUT |= 1<<HCP_POWER_SWITCH;
          //Stop and reset the stim timer
          NRF_TIMER0->TASKS_STOP = 1;
          NRF_TIMER0->TASKS_CLEAR = 1;
          stimCount = 0;

          //Calculate DAC amplitude values with calibrations
          positiveAmplitudeCode = (RadioRxData[POS_STIM_AMP_H] <<8) | RadioRxData[POS_STIM_AMP_L];
          // negativeAmplitudeCode = 4096 - positiveAmplitudeCode; // old dac
          negativeAmplitudeCode = 256 - positiveAmplitudeCode; //VG NEW DAC
          
          if (RadioRxData[POS_ZERO_CALIBRATION] != 0xFF)
          {
            zeroCalibration = RadioRxData[POS_ZERO_CALIBRATION];
          }
          else
          {
            zeroCalibration = (lastMaxValue+lastMinValue)>>1;
          }

          posCalibration = RadioRxData[POS_POSITIVE_CALIBRATION];
          negCalibration = RadioRxData[POS_NEGATIVE_CALIBRATION];

          // DacZeroLevel = 2048 + zeroCalibration; // old dac
          DacZeroLevel = 128 + zeroCalibration; //VG NEW DAC

          positiveStimCode = positiveAmplitudeCode + posCalibration + zeroCalibration;
          negativeStimCode= negativeAmplitudeCode + negCalibration + zeroCalibration;

          //Calculate timer compare values using PW and PRT.
          PRT = (RadioRxData[POS_PULSE_REPEAT_TIME_3] <<24) |(RadioRxData[POS_PULSE_REPEAT_TIME_2] <<16) | (RadioRxData[POS_PULSE_REPEAT_TIME_1] <<8) | RadioRxData[POS_PULSE_REPEAT_TIME_0]; 
          PW = (RadioRxData[POS_PULSE_WIDTH_3] <<24) |(RadioRxData[POS_PULSE_WIDTH_2] <<16) | (RadioRxData[POS_PULSE_WIDTH_1] <<8) | RadioRxData[POS_PULSE_WIDTH_0];
          duration = (RadioRxData[POS_STIM_CYCLE_H]<<8) | RadioRxData[POS_STIM_CYCLE_L];

          ////VGVG entire section here commented out since stimwaveformregister replaced with Decimation variable
          if (RadioRxData[POS_STIM_WAVEFORM_CONFIG] == 0)
          {
            //double the PRT
            //TODO: get rid of this doubling step maybe????
            PRT = PRT<<1;
            int8_t PW1Cal = RadioRxData[POS_NEGATIVE_PW_CALIBRATION];
            int8_t PW2Cal = RadioRxData[POS_POSITIVE_PW_CALIBRATION];

            //Update Timer0 compare event values
            NRF_TIMER0->CC[0] = PRT;
            NRF_TIMER0->CC[1] = PRT-(PW+PW1Cal);
            NRF_TIMER0->CC[2] = PRT>>1;
            NRF_TIMER0->CC[3] = (PRT>>1)-(PW+PW2Cal);

            //Removing corner case where a compare register set to zero does not generate an interrupt. RPT = reset, so it's basically zero
            if (NRF_TIMER0->CC[3] == 0)
              NRF_TIMER0->CC[3] = PRT;
          }
          else if (RadioRxData[POS_STIM_WAVEFORM_CONFIG] == 1)
          {
            IPD = (RadioRxData[POS_INTER_PHASIC_DELAY_3] <<24) |(RadioRxData[POS_INTER_PHASIC_DELAY_2] <<16) | (RadioRxData[POS_INTER_PHASIC_DELAY_1] <<8) | RadioRxData[POS_INTER_PHASIC_DELAY_0]; 
            int8_t PW1Cal = RadioRxData[POS_NEGATIVE_PW_CALIBRATION];
            int8_t PW2Cal = RadioRxData[POS_POSITIVE_PW_CALIBRATION];

            //Update Timer0 compare event values
            NRF_TIMER0->CC[0] = PRT;
            NRF_TIMER0->CC[1] = PRT-(PW+PW2Cal);
            NRF_TIMER0->CC[2] = PW+PW2Cal+IPD;
            NRF_TIMER0->CC[3] = IPD;

            //Check for too-small IPD here. Must characterize first!
          }
          ////////////////////////


          // tx_data[0] = (DacZeroLevel>>8);
          // tx_data[1] = DacZeroLevel&0xFF; // old dac
          tx_data[0] = (DacZeroLevel>>2); //VG NEW DAC
          tx_data[1] = (6<<DacZeroLevel)&0xFF; //VG NEW DAC

          spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);

          //Turn on the stim-glitch-switch
          //NRF_GPIO->OUTSET = 1<<STIM_GLITCH_SWITCH;

          stimOnTimeCnt = (RadioRxData[POS_STIM_ON_TIME_CNT_3] <<24) |(RadioRxData[POS_STIM_ON_TIME_CNT_2] <<16) | (RadioRxData[POS_STIM_ON_TIME_CNT_1] <<8) | RadioRxData[POS_STIM_ON_TIME_CNT_0]; 
          stimOffTimeCnt = (RadioRxData[POS_STIM_OFF_TIME_CNT_3] <<24) |(RadioRxData[POS_STIM_OFF_TIME_CNT_2] <<16) | (RadioRxData[POS_STIM_OFF_TIME_CNT_1] <<8) | RadioRxData[POS_STIM_OFF_TIME_CNT_0]; 
          NRF_RTC1->TASKS_STOP = 1;
          NRF_RTC1->TASKS_CLEAR = 1;
          NRF_RTC1->CC[0] = stimOnTimeCnt;
          NRF_RTC1->TASKS_START = 1;

          //Start the timer0 and set stim on.
          NRF_TIMER0->TASKS_START = 1;
          isStimOn = 1;
        }
        //Stop the stimulation
        else if (RadioRxData[POS_STIM_TIMER_CONFIG] == 2)
        {
          NRF_TIMER0->TASKS_STOP = 1;
          NRF_TIMER0->TASKS_CLEAR = 1;          
          NRF_RTC1->TASKS_STOP = 1;
          NRF_RTC1->TASKS_CLEAR = 1;
          Stop_Stim();
          stimCount = 0;
        }

        //Update sample timer
        if ((NRF_TIMER1->CC[0] != RadioRxData[POS_ADC_TIMER1_CC0_REG]) || (NRF_TIMER1->PRESCALER != RadioRxData[POS_ADC_TIMER1_PRESCALE_REG]))
        {
          // Packet Length determined bassed on the prescaler. Small Prescale => Faster => Larger packets needed
          if (RadioRxData[POS_ADC_TIMER1_PRESCALE_REG] == 0x02) {
            RADIO_PACKET_LENGTH = LARGE_PACKET_LENGTH;
          } else if (RadioRxData[POS_ADC_TIMER1_PRESCALE_REG] == 0x07) {
            RADIO_PACKET_LENGTH = SMALL_PACKET_LENGTH;
          }

          PACKET_PAYLOAD_LENGTH = RADIO_PACKET_LENGTH - RADIO_DATA_OFFSET - STIM_STATUS_LENGTH; //VG, 180 for long packet, 
          if (RadioRxData[POS_ADC_RESOLUTION] == 10){
            PACKET_PAYLOAD_LENGTH -= ((RADIO_PACKET_LENGTH - RADIO_DATA_OFFSET - 5) / 5); //VG 144
          }

          NRF_TIMER1->TASKS_STOP = 1;
          NRF_TIMER1->TASKS_CLEAR = 1;
          // Set timer to iterate at new sample rate, VG
          NRF_TIMER1->PRESCALER = RadioRxData[POS_ADC_TIMER1_PRESCALE_REG];
          NRF_TIMER1->CC[0] = RadioRxData[POS_ADC_TIMER1_CC0_REG];
          NRF_TIMER1->TASKS_START = 1;
        }

        //Update AIN channels
        //This byte is in the format of XXBBBAAA, XX are don't cares, AAA selects one of the 8 Ch's and will go to the 
        //even indices of the radio packet, BBB selects one of the 8 ch's  and goes to the odd indices
        //Also updates ADC Resolution
        if ( (ADC_Active_Ch != ((RadioRxData[POS_ACTIVE_CHANNELS_H] <<8) | RadioRxData[POS_ACTIVE_CHANNELS_L])) || (ADC_Resolution != RadioRxData[POS_ADC_RESOLUTION]))
        {
          ADC_Active_Ch = (RadioRxData[POS_ACTIVE_CHANNELS_H] <<8) | RadioRxData[POS_ACTIVE_CHANNELS_L];
          ADC_Resolution = RadioRxData[POS_ADC_RESOLUTION];
          NRF_SAADC->TASKS_STOP = 1;
          NVIC_DisableIRQ(SAADC_IRQn);
          NVIC_ClearPendingIRQ(SAADC_IRQn);

          //Update all 4 channel selections
          for (int i = 0; i < 4; ++i)
          {
            switch (0x07 & ADC_Active_Ch>>(4*i))
            {
              case 1: //Rec1
                SAADC_Active_PSELP_Registers[i] = PESLP_REGISTERS[REC1_AIN];
                break;
              case 2: //Rec2
                SAADC_Active_PSELP_Registers[i] = PESLP_REGISTERS[REC2_AIN];
                break;
              case 3: //Opt1
                SAADC_Active_PSELP_Registers[i] = PESLP_REGISTERS[OPT1_AIN];
                break;
              case 4: //Opt2
                SAADC_Active_PSELP_Registers[i] = PESLP_REGISTERS[OPT2_AIN];
                break;
              case 5: //Rs_H
                SAADC_Active_PSELP_Registers[i] = PESLP_REGISTERS[RS_H_AIN];
                break;
              case 6: //Rs_L
                SAADC_Active_PSELP_Registers[i] = PESLP_REGISTERS[RS_L_AIN];
                break;
              default:
                break;
            }
          }

          //Update the resolution
          NRF_SAADC->RESOLUTION = ADC_Resolution == 8 ? SAADC_RESOLUTION_VAL_8bit << SAADC_RESOLUTION_VAL_Pos : SAADC_RESOLUTION_VAL_10bit << SAADC_RESOLUTION_VAL_Pos;

          PACKET_PAYLOAD_LENGTH = RADIO_PACKET_LENGTH - RADIO_DATA_OFFSET - STIM_STATUS_LENGTH ;
           
           //VG 10 bit stuff verified
          if (ADC_Resolution == 10){
            PACKET_PAYLOAD_LENGTH -= ((RADIO_PACKET_LENGTH - RADIO_DATA_OFFSET - 5) / 5);
          }
          // VG verified

          Reset_ADC_Counter_Flag = 1;
          NRF_SAADC->CH[0].PSELP = SAADC_Active_PSELP_Registers[0] << SAADC_CH_PSELN_PSELN_Pos;
          NVIC_EnableIRQ(SAADC_IRQn);
        }
      }
      NRF_RADIO->PACKETPTR = temp_ptr; //Set the radio pointer back to the TX pointer.
      //NRF_GPIO->OUT &= ~(1<<TEST2_GPIO_OUT);
      isRxEvent = 2;
    }
  }
  NVIC_EnableIRQ(RADIO_IRQn);
}

void SAADC_IRQHandler(void)
{
  //First, disable the ADC interrupt request
  NVIC_DisableIRQ(SAADC_IRQn);
  //NRF_GPIO->OUT |=  (1<<TEST1_GPIO_OUT);
  
  static uint16_t tempVal = 0; //Temporary value that holds the current value in the ADC buffer
  static uint8_t ADC_Counter = 0; //ADC counter that indicates how many samples we've acquired since the last transmission
  static uint8_t Cnt_I = 0; //Counter for 10-bit packet stuffing
  static uint8_t Cnt_O = 0; //Counter for 10-bit packet stuffing
  static _Bool Buffer_A_Select = 1; //Flag indicating which of the two radio buffers to fill.
  static uint16_t PID = 0; //local packet id number
  static uint8_t HandshakeRetry_cnt = 0; //handshake retry counter.
  static uint8_t *Radio_Buffer;
  //NVIC_DisableIRQ(SAADC_IRQn);
  //Tell the ADC to start the ADC and prepare the result buffer in RAM
  NRF_SAADC->TASKS_START = 1;

  //Clear the ADC end event register that triggered this interrupt
  NRF_SAADC->EVENTS_END = 0;

  //Save the current result into the temporary value holder.
  tempVal = 0x3ff & *((uint32_t *)NRF_SAADC->RESULT.PTR);

  //GA Check one channel peak to peak voltage, only for 8 bit and 5k sampling freq, assumign 100Hz sine wave.
  // Should be able to do 2 and 4 channel mode
  if (NRF_SAADC->CH[0].PSELP == PESLP_REGISTERS[OPT2_AIN])
  {    
    NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    if (tempVal > DP_maxVal) // new max
    {
      DP_maxVal = tempVal;
    }
    else if (tempVal < DP_minVal) // new min
    {
      DP_minVal = tempVal;
    }
    ++DP_chCounter; // increment counter
    
    if (DP_chCounter == 500) // 10 periods of 100Hz at 4 channel mode. 20 periods for 2 channel mode
    {
      NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
      I2C_PP();
    }
  }

  //Clear the ADC counter if necessary
  if (Reset_ADC_Counter_Flag == 1)
  {
    ADC_Counter = 0;
    Cnt_I = 0;
    Cnt_O = 0;
    Reset_ADC_Counter_Flag = 0;
  }
 
  //NRF_GPIO->OUT |=  (1<<TEST3_GPIO_OUT);
  //Swap the active SAADC channel so that the next sample will be from the next entry in the SAADC channel, VG
  if ((ADC_Counter & 0x3) == 0)
    NRF_SAADC->CH[0].PSELP = SAADC_Active_PSELP_Registers[1] << SAADC_CH_PSELP_PSELP_Pos;
  else if ((ADC_Counter & 0x3) == 1) //bitwise AND, modulus function or hard coded to have all the counts. 0-1-2-3-4-5. go back to zero, then track youre going back. 
    NRF_SAADC->CH[0].PSELP = SAADC_Active_PSELP_Registers[2] << SAADC_CH_PSELP_PSELP_Pos;
  else if ((ADC_Counter & 0x3) == 2)
    NRF_SAADC->CH[0].PSELP = SAADC_Active_PSELP_Registers[3] << SAADC_CH_PSELP_PSELP_Pos;
  else if ((ADC_Counter & 0x3) == 3)
    NRF_SAADC->CH[0].PSELP = SAADC_Active_PSELP_Registers[0] << SAADC_CH_PSELP_PSELP_Pos;

  //NRF_GPIO->OUT &= ~(1<<TEST3_GPIO_OUT);

  //Save the newest sample into the radio buffer
  Radio_Buffer = Buffer_A_Select ? ADC_Results_A : ADC_Results_B;
  
  //If resolution is equal to 8, just save the sample in the buffer.
  if (ADC_Resolution == 8)
  {
    Radio_Buffer[ADC_Counter + RADIO_DATA_OFFSET] = 0xff & tempVal;
  }
  //If resolution is equal to 10, we need to packet-stuff., VG
  else if (ADC_Resolution == 10)
  {
    if ((ADC_Counter & 0x03) == 0x00) { //VGVG added for accounting for long packets that arent 40 samples long anymore
      //The if statement above looks at the two LSBs to see if its a multiple of 4. 
    //if ((ADC_Counter == 0)||(ADC_Counter == 4)||(ADC_Counter == 8)||(ADC_Counter == 12)||(ADC_Counter == 16)||(ADC_Counter == 20)||(ADC_Counter == 24)||(ADC_Counter == 28)){ //VGVG original statement
      Radio_Buffer[Cnt_I+Cnt_O*5+RADIO_DATA_OFFSET] = 0;
      Radio_Buffer[Cnt_I+Cnt_O*5+RADIO_DATA_OFFSET +1] = 0;
      Radio_Buffer[Cnt_I+Cnt_O*5+RADIO_DATA_OFFSET +2] = 0;
      Radio_Buffer[Cnt_I+Cnt_O*5+RADIO_DATA_OFFSET +3] = 0;
      Radio_Buffer[Cnt_I+Cnt_O*5+RADIO_DATA_OFFSET +4] = 0;
    }
    Radio_Buffer[Cnt_I+Cnt_O*5+RADIO_DATA_OFFSET] |= (uint8_t)((tempVal>>(2*(Cnt_I+1)))&Mask1[Cnt_I]);
    Radio_Buffer[Cnt_I+Cnt_O*5+RADIO_DATA_OFFSET+1] |=(uint8_t) ((tempVal<<(6-2*Cnt_I))&Mask2[Cnt_I]);
    Cnt_I++;
    if (Cnt_I == 4)
    {
      Cnt_I=0;
      Cnt_O++;
    }
  }

  ++ADC_Counter;

  // TODO TESTING ONLY, COMMENT OUT WHEN FINISHED
  // This will toggle the GPIO pins on every ADC sample. Toggle frequency should change according to sample frequency.
  // Even # pins will be syncronized, 
  // Odd  # pins will by syncronized but out of phase with even # pins
  // This is done to create some abnormal pattern that wouldn't happen by chance.
  // if (ADC_Counter %2 == 0) {
  //   NRF_GPIO->OUT &= ~(1<<BIONODE_GPIO_1);
  //   NRF_GPIO->OUT |=  (1<<BIONODE_GPIO_2);
  //   NRF_GPIO->OUT &= ~(1<<BIONODE_GPIO_3);
  //   NRF_GPIO->OUT |=  (1<<BIONODE_GPIO_4);
  //   NRF_GPIO->OUT &= ~(1<<BIONODE_GPIO_5);
  //   NRF_GPIO->OUT |=  (1<<BIONODE_GPIO_6);
  //   NRF_GPIO->OUT &= ~(1<<BIONODE_GPIO_7);
  //   NRF_GPIO->OUT |=  (1<<BIONODE_GPIO_8);
  // } else {
  //   NRF_GPIO->OUT |=  (1<<BIONODE_GPIO_1);
  //   NRF_GPIO->OUT &= ~(1<<BIONODE_GPIO_2);
  //   NRF_GPIO->OUT |=  (1<<BIONODE_GPIO_3);
  //   NRF_GPIO->OUT &= ~(1<<BIONODE_GPIO_4);
  //   NRF_GPIO->OUT |=  (1<<BIONODE_GPIO_5);
  //   NRF_GPIO->OUT &= ~(1<<BIONODE_GPIO_6);
  //   NRF_GPIO->OUT |=  (1<<BIONODE_GPIO_7);
  //   NRF_GPIO->OUT &= ~(1<<BIONODE_GPIO_8);
  // }

  //Now, figure out if we need to initiate a radio TX. Do this whenever the ADC buffer is full.
  if (ADC_Counter == PACKET_PAYLOAD_LENGTH) //VG this makes sense. Ensure packet payload length changes properly
  {
    //reset packet-stuffing counters.
    Cnt_I = 0;
    Cnt_O = 0;
    if (resetPID) {
      PID = 0;
      resetPID = 0;
    }

    //Handshake stuff//
    //Ask for another handshake if we did not get a successful handshake after the last packet was sent, and we had expected one.
    if(((PID%Handshake_Interval) ==1)&&(isHandshakeOK==0))
    {
      //decrement the PID to indicate that a handshake may need to be re-sent.
      PID--;
      //keep track of how many times we've tried to get a successful handshake
      HandshakeRetry_cnt++;

      //If we've had more than 10 handshake attempts, reset the PID to 0 and give up.
      //If waiting to change packets, must receve multiple handshakes without PID reset
      if ((HandshakeRetry_cnt>10) && (!changePktLen)) {
        HandshakeRetry_cnt=0;
        PID=0;
      }
    }

    //If we have successfully done a handshake, reset the HandshakeRetry_cnt counter.
    else if (isHandshakeOK == 1)
    {
      HandshakeRetry_cnt = 0;
    }

    //If we are supposed to do a handshake after this packet, reset handshake flags.
    if ((PID%Handshake_Interval) == 0)
    {
      isRxEvent = 0;
      isHandshakeOK = 0;
    }

    else
      isRxEvent = 2;
  
    //Create the Radio Packet
    Radio_Buffer = Buffer_A_Select ? ADC_Results_A : ADC_Results_B;
    //Set the PID, TXID, RXID, and Type packet bytes
    Radio_Buffer[0] = (uint8_t) (0xff&(PID>>8));
    Radio_Buffer[1] = (uint8_t) (0xff&PID);
    Radio_Buffer[2] = (uint8_t) 0x02;
    Radio_Buffer[3] = (uint8_t) 0x00;
    if ((ADC_Counter == SMALL_PACKET_PAYLOAD) || (ADC_Counter == SMALL_PACKET_PAYLOAD_10Bit)) {
      Radio_Buffer[4] = ADC_Resolution == 8 ? (uint8_t)Data_8bit_Small : (uint8_t)Data_10bit_Small;//VGVG verified
    } else {
      Radio_Buffer[4] = ADC_Resolution == 8 ? (uint8_t)Data_8bit_Large : (uint8_t)Data_10bit_Large; //VGVG verified
    }

    // ADD STIM STATUS CODE BELOW:
    //Radio_Buffer[RADIO_DATA_OFFSET+PACKET_PAYLOAD_LENGTH+0] = (uint8_t) (insert_data_here);
    //Radio_Buffer[RADIO_DATA_OFFSET+PACKET_PAYLOAD_LENGTH+1] = (uint8_t) (insert_data_here);
    //Radio_Buffer[RADIO_DATA_OFFSET+PACKET_PAYLOAD_LENGTH+2] = (uint8_t) (insert_data_here);
    //Radio_Buffer[RADIO_DATA_OFFSET+PACKET_PAYLOAD_LENGTH+3] = (uint8_t) (insert_data_here);
    //Radio_Buffer[RADIO_DATA_OFFSET+PACKET_PAYLOAD_LENGTH+4] = (uint8_t) (insert_data_here);

    NRF_RADIO->PACKETPTR = (int)Radio_Buffer;

    // Wait for any previous radio activity to finish
    while(NRF_RADIO->STATE != 0) { 
      if (NRF_RADIO->STATE == 0x03) {
        NRF_RADIO->TASKS_DISABLE = 1;
      }
    }

    // set the radio back to transmit length of data packet (needed if just recieved handshake)
    NRF_RADIO->PCNF1 = (NRF_RADIO->PCNF1 & ~(0xFFFF)) |
                       (RADIO_PACKET_LENGTH << RADIO_PCNF1_MAXLEN_Pos) |
                       (RADIO_PACKET_LENGTH << RADIO_PCNF1_STATLEN_Pos);

    //Enable the radio TX
    //NRF_GPIO->OUT |=  (1<<TEST2_GPIO_OUT);
    NRF_RADIO->TASKS_TXEN = 0x01;

    //reset the ADC counter, and increment the PID. If PID rolls over, set it equal to 1.
    ADC_Counter = 0;
    PID = PID == 0xffff ? 1 : PID+1;

    //toggle the buffer select flag
    Buffer_A_Select = !Buffer_A_Select;
  }

  //Set the global PID value
  PID_Global = PID;
  //NRF_GPIO->OUT &= ~(1<<TEST1_GPIO_OUT);

  //re-enable the SAADC interrupt request
  NVIC_EnableIRQ(SAADC_IRQn);
}

void RTC0_IRQHandler(void)
{
  //TODO: It would be nice to be able to DISABLE this for Bionodes that do not stimulate...
  //First, disable the RTC0 interrupt
  NVIC_DisableIRQ(RTC0_IRQn);

  //Clear the compare event flag that got us here and clear the counter
  NRF_RTC0->EVENTS_COMPARE[0] = 0;
  NRF_RTC0->TASKS_CLEAR = 1;

  if (isStimOn)
  {
    //Do nothing, calibration happens automatically during stim
  }

  else if (DACIsOn)
  {
    //Calibrate stim
    //TODO: Put this pin number into a .h file or something!
    //Read compare input pin
    stimFbPin = NRF_GPIO->IN & 1<<STIM_FEEDBACK_INPUT;

    //We need to lower the zero point
    if (stimFbPin)
    {
      if (DACZeroCalDirection_Up)
      {
        // lastMaxValue = 2048 + zeroCalibration; // old dac
        lastMaxValue = 128 + zeroCalibration; // VG NEW DAC
      }
      DACZeroCalDirection_Up = false;
      if (zeroCalibration > -DACLimit)
      {
        zeroCalibration -= 1;
        // DacZeroLevel = 2048 + zeroCalibration;
        DacZeroLevel = 128 + zeroCalibration; // VG NEW DAC
      }
    }

    //We need to raise the zero point
    else
    {
      if (!DACZeroCalDirection_Up)
      {
        // lastMinValue = 2048+zeroCalibration; // old dac
        lastMinValue = 128 +zeroCalibration; // VG NEW DAC
      }
      DACZeroCalDirection_Up = true;
      if (zeroCalibration < DACLimit)
      {
        zeroCalibration += 1;
        // DacZeroLevel = 2048 + zeroCalibration; // old dac
        DacZeroLevel = 128 + zeroCalibration; //VG NEW DAC
      }
    }

    //Update the output of the DAC with the new zero point.
    // tx_data[0] = DacZeroLevel>>8;
    // tx_data[1] = DacZeroLevel; // old dac
    tx_data[0] = DacZeroLevel>>2; //VG NEW DAC
    tx_data[1] = (6<<DacZeroLevel)&0xFF; //VG NEW DAC

    spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);
  }
  //Re-enable the RTC0 interrupt
  NVIC_EnableIRQ(RTC0_IRQn);
}

void RTC1_IRQHandler(void){
    NVIC_DisableIRQ(RTC1_IRQn);
    NRF_RTC1->EVENTS_COMPARE[0]=0;
    if (isStimOn){
      NRF_RTC1->CC[0] += stimOffTimeCnt;
      isStimOn = 0;
      NRF_TIMER0->TASKS_STOP = 1;
      NRF_TIMER0->TASKS_CLEAR = 1;
      Stop_Stim();
      stimCount = 0;
    }
    else{
      NRF_RTC1->CC[0] += stimOnTimeCnt;
      //TESTING!!!! Turn on HCP first
      NRF_GPIO->OUT |= 1<<HCP_POWER_SWITCH;
      //Stop and reset the stim timer
      NRF_TIMER0->TASKS_STOP = 1;
      NRF_TIMER0->TASKS_CLEAR = 1;
      stimCount = 0;
      //Start the timer0 and set stim on.
      NRF_TIMER0->TASKS_START = 1;
      isStimOn = 1;

    }

    NVIC_EnableIRQ(RTC1_IRQn);
}

void TIMER0_IRQHandler(void)
{
  //First, disable the timer interrupt
  NVIC_DisableIRQ(TIMER0_IRQn);
  //NRF_GPIO->OUT |= (1<<TEST_GPIO_OUT);

  //If the stimCount is greater than or equal to the set stim duration, stop and clear the timer and reset the stim counter
  if ((stimCount >= duration) && (duration != 0xFFFF))
  {
    Stop_Stim();
    NRF_TIMER0->TASKS_STOP = 0x01;
    NRF_TIMER0->TASKS_CLEAR = 0x01;
    stimCount = 0;
  }

  else
  {
    //Positive or Negative stimulation required. Connect the DAC output to the HCP and open up the cap discharge switch
    if ((NRF_TIMER0->EVENTS_COMPARE[1] == 1) || (NRF_TIMER0->EVENTS_COMPARE[3] == 1))
    {
      SetStimGlitchSwitch(true);
    }

    //We just finished a negative stim. Disconnect the DAC output from the HCP, and close the cap discharge switch.
    //Set the DAC output to the positive stim level in anticipation for the positive stim pulse
    else if (NRF_TIMER0->EVENTS_COMPARE[0] == 1)
    {
      SetStimGlitchSwitch(false);
      // tx_data[0] = positiveStimCode>>8;
      // tx_data[1] = positiveStimCode; // old dac
      tx_data[0] = positiveStimCode>>2; //VG NEW DAC
      tx_data[1] = (6<<positiveStimCode)&0xFF; //VG NEW DAC

      spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);
      stimCount += 1;
    }

    //We just finished a positive stim. Disconnect the DAC output from the HCP, and close the cap discharge switch.
    //Set the DAC output to the negative stim level in anticipation for the positive stim pulse
    else if (NRF_TIMER0->EVENTS_COMPARE[2] == 1)
    {
      SetStimGlitchSwitch(false);
      // tx_data[0] = negativeStimCode>>8;
      // tx_data[1] = negativeStimCode; // old dac
      tx_data[0] = negativeStimCode>>2; //VG NEW DAC
      tx_data[1] = (6<<negativeStimCode)&0xFF; //VG NEW DAC

      spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);
    }


  /*
    //This interrupt indicates that a positive stim is needed
    if (NRF_TIMER0->EVENTS_COMPARE[1] == 1)
    {
      //Close the stim_glitch_switch and output the positive stim voltage.
      //NRF_GPIO->OUTSET = 1 << STIM_GLITCH_SWITCH;
      SetStimGlitchSwitch(true);
      //NRF_GPIO->OUT &= ~(1<<CAP_DISCHARGE_SWITCH);
      tx_data[0] = positiveStimCode>>8;
      tx_data[1] = positiveStimCode;
      spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);
      stimCount += 1;
    }

    //This interrupt indicates that a negative stim is needed
    else if (NRF_TIMER0->EVENTS_COMPARE[3] == 1)
    {
      //Before setting the negative stim output voltage, run stim feedback calibration

      //Close the stim_glitch_switch and output the negative stim voltage.
      //NRF_GPIO->OUTSET = 1<<STIM_GLITCH_SWITCH;
      SetStimGlitchSwitch(true);
      //NRF_GPIO->OUT &= ~(1<<CAP_DISCHARGE_SWITCH);
      tx_data[0] = negativeStimCode >> 8;
      tx_data[1] = negativeStimCode;
      spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);
      stimCount = stimCount + 1;
    }

    //This interrupt indicates that we are between negative and positive stim pulses
    else
    {
      //We no longer disconnect the DAC from the HCP to allow for calibration between stim sessions
      //NRF_GPIO->OUTCLR = 1 << STIM_GLITCH_SWITCH;
      SetStimGlitchSwitch(false);
      //NRF_GPIO->OUT |= (1<<CAP_DISCHARGE_SWITCH);
      tx_data[0] = (DacZeroLevel>>8);
      tx_data[1] = (DacZeroLevel&0xFF);
      spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);
    }*/
  }

  //Clear out all compare events
  NRF_TIMER0->EVENTS_COMPARE[3] = 0;
  NRF_TIMER0->EVENTS_COMPARE[2] = 0;
  NRF_TIMER0->EVENTS_COMPARE[1] = 0;
  NRF_TIMER0->EVENTS_COMPARE[0] = 0;

  //NRF_GPIO->OUT &= ~(1<<TEST_GPIO_OUT);
  //Re-enable the interrupt
  NVIC_EnableIRQ(TIMER0_IRQn);
}

void TIMER3_IRQHandler(void)
{
  //First, disable the timer interrupt
  NVIC_DisableIRQ(TIMER3_IRQn);
  //Clear out compare event
  NRF_TIMER3->EVENTS_COMPARE[0] = 0;

  //Turn on the stim-glitch-switch
  //NRF_GPIO->OUT |= (1<<STIM_GLITCH_SWITCH);
  //SetStimGlitchSwitch(true);
  DACIsOn = true;
  NVIC_EnableIRQ(TIMER3_IRQn);
}

// TM BN_5V3 Update: This GPIO Pin removed with 1.8 V DAC
// Left for reference in later development
// void GPIOTE_IRQHandler(void)
// {
//   NVIC_DisableIRQ(GPIOTE_IRQn);
//   NRF_GPIOTE->EVENTS_IN[0] = 0;

//   NVIC_EnableIRQ(GPIOTE_IRQn);
// }

__inline void Stop_Stim(void)
{
  SetStimGlitchSwitch(false);
  //Set DAC Out to zero
  // tx_data[0] = (DacZeroLevel>>8);
  // tx_data[1] = DacZeroLevel&0xFF; // old dac
  tx_data[0] = (DacZeroLevel>>2); //VG NEW DAC
  tx_data[1] = (6<<DacZeroLevel)&0xFF; //VG NEW DAC
  
  spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data);
  //NRF_GPIO->OUT &= ~(1<<STIM_GLITCH_SWITCH);
  SetStimGlitchSwitch(false);
  //NRF_GPIO->OUT |= (1<<CAP_DISCHARGE_SWITCH);
  isStimOn = 0;
  //TESTING!!! Turn off the HCP
  NRF_GPIO->OUT &= ~(1<<HCP_POWER_SWITCH);
  return;
}

__inline void SetStimGlitchSwitch(_Bool on)
{
  if (on && (DACIsOn))
  {
    NRF_GPIO->OUT |= (1<<STIM_GLITCH_SWITCH) | (1<<CAP_DISCHARGE_SWITCH);
    //NRF_GPIO->OUT |= (1<<CAP_DISCHARGE_SWITCH);
  }
  else
  {
    NRF_GPIO->OUT &= ~(1<<STIM_GLITCH_SWITCH | 1<<CAP_DISCHARGE_SWITCH);
    //NRF_GPIO->OUT &= ~(1<<CAP_DISCHARGE_SWITCH);
  }
}

void I2C_PP(void) //GA
{
  if ((DP_minVal > 12) | (DP_maxVal < 243)) // if signal is too small, increase gain
  {
    // Increment VWIPER 0
    reg_cmd[0] = ((DP_VWIPER0 << 4) | (DP_INCR_CMD << 2));
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd, sizeof(reg_cmd), true);  
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }
  }
  else if ((DP_minVal < 3) | (DP_maxVal > 253)) // if signal is too high, decrease gain
  {
    // Decrement VWIPER 0
    reg_cmd[0] = ((DP_VWIPER0 << 4) | (DP_DECR_CMD << 2));
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd, sizeof(reg_cmd), true);  
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }
  }
  DP_maxVal = DacZeroLevel; // Reset DP_maxVal and DP_minVal
  DP_minVal = DacZeroLevel;
  DP_chCounter = 0;

  /*
    // Read from status register
    uint8_t reg_cmd[1] = {((DP_STATUS << 4) | (DP_READ_CMD << 2))};
    uint8_t status_reg[2];
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd, sizeof(reg_cmd), false);  
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_READ_BIT), status_reg, sizeof(status_reg), true);
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }
    
    // Increment VWIPER 0
    uint8_t reg_cmd1[1] = {((DP_VWIPER0 << 4) | (DP_DECR_CMD << 2))};
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd1, sizeof(reg_cmd1), true);  
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }

    // Read from VWIPER 0
    uint8_t DP_VWIPER0_reg[2];
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_READ_BIT), DP_VWIPER0_reg, sizeof(DP_VWIPER0_reg), true);
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }
    
    // Write to NVWIPER 0
    uint8_t reg_cmd2[2] = {((DP_NVWIPER0 << 4) | (DP_WRITE_CMD << 2)), (DP_VWIPER0_reg[1])};
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd2, sizeof(reg_cmd2), true);
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }
    */
}