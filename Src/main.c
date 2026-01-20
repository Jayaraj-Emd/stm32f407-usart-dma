
#define HIGH             ENABLE
#define BUTTON_PRESED    HIGH
#include "stm32f407.h"
#include <string.h>
char msg[1024] = "UART Tx testing...\n\r";
USART_Handle_t usart2_handle;

#define Is_it_HT()       DMA1->HISR & (1 << 20)
#define Is_it_FT()       DMA1->HISR & (1 << 21)
#define Is_it_TE()       DMA1->HISR & (1 << 19)
#define Is_it_FE()       DMA1->HISR & (1 << 16)
#define Is_it_DME()      DMA1->HISR & (1 << 18)


/*
 * interrupt call back funtions
 */
void extern HT_complete_callback();
void extern FT_complete_callback();
void extern TE_error_callback();
void extern FE_error_callback();
void extern DME_error_callback();
void button_init(void)
{
	GPIO_Handle_t gpio_btn = {0};

	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTPUT_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PIN_NO_PUPD;
	GPIO_Init(&gpio_btn);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
}

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}
void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpio = {0};

	usart_gpio.pGPIOx = GPIOA;
	usart_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	usart_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	usart_gpio.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PIN_PULL_UP;
	usart_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTPUT_SPEED_FAST;

	/* TX */
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpio);

	/* RX */
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpio);
}


void Dma1_init(void){
	DMA_StreamRegDef_t *pSTREAM6;
	pSTREAM6 = DMA1_STREAM6;
	USART_RegDef_t *pUSART2;
	pUSART2= USART2;
	DMA1_PCLK_EN();
	pSTREAM6->CR &= ~(1 << 0);
	pSTREAM6->CR &= ~(7 << 25);
	pSTREAM6->CR |= (4 << 25);
	pSTREAM6->M0AR = (uint32_t)msg;
	pSTREAM6->PAR = (uint32_t) &pUSART2->DR;
	uint32_t len = sizeof(msg);
	pSTREAM6->NDTR = len;
	pSTREAM6->CR |= (1 << 6);
	pSTREAM6->CR &= ~(0x3 << 13);
	pSTREAM6->CR &= ~(0x3 << 11);
	pSTREAM6->CR |= (1 << 10);
	pSTREAM6->FCR |= (1 << 2);
	pSTREAM6->FCR &= ~(0x3 << 0);
	pSTREAM6->FCR |=  (0x3 << 0);

}
void dma1_enable_stream6(){
	DMA_StreamRegDef_t *pSTREAM6;
	pSTREAM6 = DMA1_STREAM6;
	pSTREAM6->CR |= (1 << 0);
}
void DMA_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

void dma1_interrupt_configuration(){
	DMA_StreamRegDef_t *pSTREAM6;
	pSTREAM6 = DMA1_STREAM6;
	pSTREAM6->CR |= (1 << 3);
	pSTREAM6->CR |= (1 << 4);
	pSTREAM6->CR |= (1 << 2);
	pSTREAM6->FCR |= (1 << 7);
	pSTREAM6->CR |= (1 << 2);

	DMA_IRQInterruptConfig(IRQ_NO_DMA1_STREAM6, ENABLE);
}
int main(void){
	button_init();
	USART2_GPIOInit();
	USART2_Init();
	USART_PeripheralControl(USART2,ENABLE);
	Dma1_init();
	dma1_interrupt_configuration();
	dma1_enable_stream6();
	while(1);
}


void EXTI0_IRQHandler(){
	USART_RegDef_t *pUSART2;
	pUSART2= USART2;
	pUSART2->CR3 |= ( 1 << 7);
}
void DMA1_Stream6_IRQHandler(){
	if(Is_it_HT()){
		DMA1->HIFCR = (1<<20);
		HT_complete_callback();
	}
	else if(Is_it_FT()){
		DMA1->HIFCR = (1 << 21);
		FT_complete_callback();
	}
	else if(Is_it_FE()){
		DMA1->HIFCR = (1 << 16);
		FE_error_callback();
	}
	else if(Is_it_TE()){
		DMA1->HIFCR = (1 << 19);
		TE_error_callback();
	}
	else if(Is_it_DME()){
		DMA1->HIFCR = (1 << 18);
		DME_error_callback();
	}
}
void HT_complete_callback(){

}
void FT_complete_callback(){
	DMA_StreamRegDef_t *pSTREAM6;
	pSTREAM6 = DMA1_STREAM6;

	uint32_t len = sizeof(msg);
	pSTREAM6->NDTR = len;

	USART_RegDef_t *pUSART2;
	pUSART2= USART2;

	pUSART2->CR3 &= ~( 1 << 7);

	dma1_enable_stream6();
}
void FE_error_callback(){

}
void TE_error_callback(){

}
void DME_error_callback(){

}
