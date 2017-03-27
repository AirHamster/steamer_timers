
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#define LEDPORT GPIOE
#define LED GPIO15
#define COMPARE_STEP 1400

uint16_t compare_time = 14000;
uint8_t	upcount = 1;

static void gpio_setup(void)
{
	/* Enable GPIOE clock. */
	rcc_periph_clock_enable(RCC_GPIOE);

	/* Set GPIO12 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(LEDPORT, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, LED);
}

static void rcc_setup(void)
{
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);


	 /* Set the PLL multiplication factor to 8. */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL8);

	/* Select HSI/2 as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);
	
	/* Enable timer 1 clocking */
	rcc_periph_clock_enable(RCC_TIM1);	
}

static void iwdg_setup(void)
{
	iwdg_set_period_ms(2000);
	iwdg_start();
}

static void timer1_setup(void)
{

	/* Reset TIM1 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM1);
	timer_set_period(TIM1, 43000);
	

	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_set_prescaler(TIM1, 256);

	/* Disable preload. */
	timer_disable_preload(TIM1);
	timer_continuous_mode(TIM1);
	timer_enable_oc_preload(TIM1,TIM_OC1);

	/* Set the initual output compare value for OC1. */
	timer_set_oc_value(TIM1, TIM_OC1, compare_time); 

	/* Enable TIM1 interrupt. */
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM1_UP_IRQ);

	/*Enable timer 1 overflow and compare int */
	timer_enable_irq(TIM1, (TIM_DIER_UIE));
	timer_enable_irq(TIM1, (TIM_DIER_CC1IE));
	/* Counter enable. */
	timer_enable_counter(TIM1);

	gpio_toggle(LEDPORT, LED);	/* LED on/off */
}

void tim1_up_isr(void)
{
	//i2c1_read(MPU_ADDRESS, MPU_WHO_AM_I, &temp, 1);	
	/* Clear update interrupt flag. */
	timer_clear_flag(TIM1, TIM_SR_UIF);

	//usart_send_blocking(USART1, I2C1_SR1);
	/* Toggle LED to indicate compare event. */
	gpio_set(LEDPORT, LED);
}
void tim1_cc_isr (void)
{
	/* Clear compare interrupt flag. */
	timer_clear_flag(TIM1, TIM_SR_CC1IF);
		
	gpio_clear(LEDPORT, LED);
	
	/* Code for smooth inc/decrement of led light duration */

	if (upcount ==1){
	compare_time += COMPARE_STEP;
	}else{
	compare_time -= COMPARE_STEP;
	}

	if (compare_time == 42000){
		upcount = 0;
	}
	if (compare_time == 0){
		upcount = 1;
	}
	timer_set_oc_value(TIM1, TIM_OC1, compare_time); 
}


int main(void)
{
	int i;

	gpio_setup();

	/* wait a bit after reset to indicate that watchdog is working fine */
	for (i = 0; i < 800000; i++)	/* Wait a bit. */
		__asm__("nop");

	rcc_setup();
	iwdg_setup();
	timer1_setup();	

	while (1) {
		for (i = 0; i < 800000; i++){	/* Wait a bit. */
			/* resetting watchdog */
			iwdg_reset();
			__asm__("nop");
			}
	}

	return 0;
}
