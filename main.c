#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "waveform_data.h"

void I2C1_Write(uint8_t addr, uint16_t value);   // PROTOTYPE
void Timer0A_Handler(void);

volatile uint32_t idx = 0;


// I2C1 INIT (PA6=SCL, PA7=SDA)

void I2C1_Init(void)
{
    SYSCTL_RCGCI2C_R |= 0x02;   // enable I2C1 clock
    SYSCTL_RCGCGPIO_R |= 0x01;  // enable port A

    while((SYSCTL_PRGPIO_R & 0x01) == 0);

    GPIO_PORTA_AFSEL_R |= 0xC0;  // PA6, PA7 alternate function
    GPIO_PORTA_DEN_R   |= 0xC0;
    GPIO_PORTA_ODR_R   |= 0x80;  // PA7 open-drain
    GPIO_PORTA_PCTL_R  &= ~0xFF000000;
    GPIO_PORTA_PCTL_R  |=  0x33000000;   // I2C1 SCL/SDA

    I2C1_MCR_R = 0x10;    // Master mode
    I2C1_MTPR_R = 7;      // 100kHz
}


// WRITE TO MCP4725

void I2C1_Write(uint8_t addr, uint16_t value)
{
    uint8_t high = (value >> 4) & 0xFF;
    uint8_t low  = (value & 0x0F) << 4;

    I2C1_MSA_R = (addr << 1);   // write

    I2C1_MDR_R = 0x40;          // fast write command
    I2C1_MCS_R = 0x03;
    while(I2C1_MCS_R & 1);

       I2C1_MDR_R = high;
       I2C1_MCS_R = 0x01;
       while(I2C1_MCS_R & 1);

       I2C1_MDR_R = low;
       I2C1_MCS_R = 0x05;
       while(I2C1_MCS_R & 1);



}
// TIMER0A @ 20 kHz Sample Rate

void Timer0A_Init(void)
{
    SYSCTL_RCGCTIMER_R |= 1;
    TIMER0_CTL_R &= ~1;
    TIMER0_CFG_R = 0;
    TIMER0_TAMR_R = 0x02;
    TIMER0_TAILR_R = 4000;   // 20kHz for 80MHz
    TIMER0_IMR_R |= 1;
    NVIC_EN0_R |= 1 << 19;
    TIMER0_CTL_R |= 1;
 }

void Timer0A_Handler(void)
{
    TIMER0_ICR_R = 1;

    I2C1_Write(0x60, waveform_samples[idx]);

    idx++;
    if(idx >= WAVEFORM_LEN) idx = 0;
}

// MAIN

int main(void)
{
    I2C1_Init();
    Timer0A_Init();

    while(1);
}very good
