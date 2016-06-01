#include "shiftReg.h"
#include "nrf_delay.h"

//Initialize shift register
void shiftRegInit() {
	nrf_gpio_cfg_output(QA1);
	nrf_gpio_cfg_output(QA2);
	nrf_gpio_cfg_output(QA3);
	nrf_gpio_cfg_output(RCLK);
	nrf_gpio_cfg_output(SRCLK);
	nrf_gpio_cfg_output(SRCLR);
	nrf_gpio_cfg_output(OE);
	nrf_gpio_cfg_input(btn1, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(btn2, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(btn3, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(btn4, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(btn5, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(btn6, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_pin_set(SRCLR);
    nrf_gpio_pin_clear(OE);
}


//Function for pulsing the shift register						
void shiftRegPulse() {
	nrf_gpio_pin_set(SRCLK);  //SRCLK
    nrf_delay_ms(1);
	nrf_gpio_pin_clear(SRCLK);
    nrf_delay_ms(1);
	
}

//Function for latching the shift register
void shiftRegLatch() {
	nrf_gpio_pin_set(RCLK); //RCLK
	nrf_delay_ms(1);
	nrf_gpio_pin_clear(RCLK);
	nrf_delay_ms(1);
	
}

//Function for shifting out the stored bits
void shiftRegWrite(uint8_t reg1, uint8_t reg2, uint8_t reg3) {
	for(uint8_t i = 0; i < 8; i++) {
		if(reg1 & 128) {
			nrf_gpio_pin_clear(QA1);			
		}
		else if(!(reg1 & 128)) {
			nrf_gpio_pin_set(QA1);
		}
		
		if(reg2 & 128) {
			nrf_gpio_pin_clear(QA2);
		}
		else {
			nrf_gpio_pin_set(QA2);
		}
		if(reg3 & 128) {
			nrf_gpio_pin_clear(QA3);
		}
		else {
			nrf_gpio_pin_set(QA3);
		}
		shiftRegPulse();
		reg1 = (reg1 << 1);
		reg2 = (reg2 << 1);
		reg3 = (reg3 << 1);
	}
	
	shiftRegLatch();
}

//Function for tracking button presses
uint8_t btnCounter(uint8_t btn, uint8_t btncounter) {
	while(nrf_gpio_pin_read(btn));
	btncounter++;
	if(btncounter > 6){
		btncounter = 1;
	}
	return btncounter;
}
