#include "Arduino.h"


#define DEBUG 1

#define PZT_RRR 15  // PZT rising ramp rate
#define PZT_FRR -15 // PZT falling ramp rate
#define PZT_TIMESTEP 100  // microseconds; dt between PZT waveform updates

// these set when the ADC activates DMA. If the ADC value goes above HIGH_THRESHOLD, 
// the DMA begins. If it then falls below LOW_THRESHOLD, it stops and will increment to 
// the next buffer.
#define LOW_THRESHOLD 930 
#define HIGH_THRESHOLD 1000 

// Number of scans to average over for deciding "peak height"
// #define SCANS_TO_AVERAGE 10

#define BUFFER_SIZE 1024    // ADC buffer for single peak (triggered by threshold above)
#define MAX_PEAK_CT 16      // number of peaks to allocate memory for

#define RAILED_OVERHEAD 100 // how close can we get to rails during locked mode?

uint16_t buf[MAX_PEAK_CT][BUFFER_SIZE];  // allocate the buffer for ADC DMA



// these keep track of how many points/peak and how many peaks were read during a given 
// scan.
volatile uint16_t pk_count = 0; // peak counter for given scan
volatile uint16_t pt_count[MAX_PEAK_CT];  // number of points per peak
volatile uint16_t pk_position[MAX_PEAK_CT]; // track voltage position of peak.
                                        // if too close to zero, can skip

volatile int16_t pzt_o = 0;  // piezo output voltage
volatile int16_t pzt_min = 0;   // pzt min voltage
volatile int16_t pzt_max = 4095;    // pzt max voltage

uint8_t scans_to_average = 10;


void ADC_Handler(){
  // only run for right interrupt reason... ie, out of windowed range.
  if ((adc_get_status(ADC) & ADC_ISR_COMPE) == ADC_ISR_COMPE){
    
    // toggle comparison mode. 
    // ADC_EMR register 0b00 -> LOW mode, 0b01 -> HIGH mode
    ADC->ADC_EMR ^= 0x01; 
      
    NVIC_ClearPendingIRQ(ADC_IRQn);
    if (!(ADC->ADC_EMR & 0x01)){
      // if rising, turn on DMA
        REG_ADC_RPR = (uint32_t) buf[pk_count];  // set recieve pointer register
        REG_ADC_RCR = BUFFER_SIZE;  // set recieve counter register
        REG_ADC_PTCR = ADC_PTCR_RXTEN;  // enable DMA by setting RX Enable on Transfer Control Register
        pk_position[pk_count] = pzt_o;  // start of peak
    } else {
        REG_ADC_PTCR = ADC_PTCR_RXTDIS;  // disable DMA
        // store the number of points read in to DMA for given peak
        pt_count[pk_count++] = BUFFER_SIZE - REG_ADC_RCR;
    }
  }
}


//ADC & DAC Configuration
void adc_dac_setup(){
  analogWriteResolution(12);    // 12 bits on DAC
  
  // ADC configuration
  analogReadResolution(12);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);

  ADC->ADC_CHER = ADC_CHER_CH7; // enable ADC on channel 7, which is A0 for DUE.
  ADC->ADC_MR |= 0x80;  // ADC mode register; set free running bit (will sample as fast as possible)
  adc_set_comparison_channel(ADC, ADC_CHANNEL_7);   // set up for comparison window interrupts 
  adc_set_comparison_mode(ADC, ADC_EMR_CMPMODE_HIGH); // triggers interrupt when conversion > high threshold
  adc_set_comparison_window(ADC, LOW_THRESHOLD, HIGH_THRESHOLD);  // LOW_THRESHOLD and HIGH_THRESHOLD are both const, so need to be defined at compile... see top of file.
  adc_enable_interrupt(ADC, ADC_IER_COMPE); // turn on interrupt on comparison event
  adc_start(ADC); // start the ADC!
  
  pmc_enable_periph_clk(ID_ADC); // power management controller; enables clock for ADC 
  
  // ADC Interrupt NVIC Enable; clear and start.
  NVIC_DisableIRQ(ADC_IRQn);      
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, 0);
  NVIC_EnableIRQ(ADC_IRQn);
}




