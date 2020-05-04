
//������: �������������� ���� �������, ���������� �� ���������� ������ 25 �����(W25Q16, ��������� SPI) � ������� DAC 
//STM32F407xx .����� ����� ������� ����������. ������ ������� �� ������� �� 256 ����(2048 ���).������ � DAC ������ 
//� ������� DMA.������ ������ ������ ������� � ������������� ������,������ ������ ����� DMA.DAC ����������� TIM6.
// ���� ������, �� � ���� ������������ ����������� ������� ������������� � 2 ���� ��� ������� ������� �� �������
//��� ������ �������� SPI ��������� ������ ������ ������� ������������� ��� ������ ���������� TIM6

#include "stm32f0xx.h"
#include "delay.h" //��������� ����, ��� �������,������ ����� �� ���� ��������
#include "spi25.h"//��������� ��� ������ � flash
#include "WavHead.h"

#define reset_CS GPIOB->BSRR |= GPIO_BSRR_BS_6;//����� ��������� ���������(��� 1)

#define F_CLK 48000000// ������� ������������ TIM6

unsigned char DAC_Buff[512]; //������������� ����� ��� ������ ������� ��� ���������� � ���
uint32_t length;//���������� ������� � Wav �����
uint32_t byteRate;//���������� ����, ���������� �� ������� ���������������.
uint8_t Ncanal;//����� �������
uint32_t SampleRate;//������� �������������

 void init_RCC()//��������� ������������,���������� ��������� CUBE MX
{
RCC->CR|=RCC_CR_HSEON; //��������� ��������� HSE,������� �����
while (!(RCC->CR & RCC_CR_HSERDY)) {}; // ���� ����������
	
FLASH->ACR |= FLASH_ACR_PRFTBE; // Enable Prefetch Buffer.
FLASH->ACR &= ~FLASH_ACR_LATENCY; // �����������.
FLASH->ACR |= FLASH_ACR_LATENCY; // if 24 MHz < SYSCLK < 48 MHz

RCC->CR &=~RCC_CR_PLLON;//	
RCC->CFGR &=~RCC_CFGR_SW; // �������� ���� SW0, SW1
RCC->CFGR &=~ RCC_CFGR_PPRE;	//000: HCLK clock divided by 1 for APB
RCC->CFGR &=~ RCC_CFGR_HPRE;	//SYSCLK DIV BY 1 FOR HCLK
RCC->CFGR &=~ RCC_CFGR_PLLXTPRE;	//��������� ������������ PLL

//RCC->CFGR= (0b111<<28);	//�������� ��� ��� =128,���������� � 051
//RCC->CFGR |= RCC_CFGR_MCO_2;// System clock
	
//system clock �� ��� ����� 48���!!!!!���������	
 //Config PLL ������ ��� F051:
// RCC->PLLCFGR=(MCOPRE<<28) | (MCO<<24) | (PLLMUL<<18) | PLLXTPRE |(PLLSRC<<15)|( PPRE<<8)|(HPRE<<4)|SW; //page 162	
//	MCOPRE-������������ ������ MCO,MCO-����� ������ ��������,PLLXTPRE-����� �� ������� ������������ PLL(�������� � RCC_CFGR2)
//PLLSRC-������ ��������� PLL,PPRE-�������� ��� APB,HPRE-�������� ��� AHB,SW �������� ������������
RCC->CFGR= (0b111<<28)|(4<<18) | (0b10<<15)| RCC_CFGR_SW_PLL|RCC_CFGR_MCO_2; //   ������������� ��� ��������� ���� 	PLLON  � CR		
RCC->CR |=    RCC_CR_PLLON;//��
while (!(RCC->CR & RCC_CR_PLLRDY)){}; //wait for PLL ready 	

}//end init_RCC()
//*****************************************************************************************************************
void init_DAC_TIM6()//������������� ��� � ���6,������������ ���,PA4-���� ������
{
RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
GPIOA->MODER |= GPIO_MODER_MODER4;//01-output,00-input(after reset),10-AF,11-analog.	
RCC->APB1ENR |= RCC_APB1ENR_DACEN;             //������������ ����

DAC->CR   &=(DAC_CR_BOFF1                  //��������� �������� �����
             | DAC_CR_TSEL1);              //000-�������� ������� �������������� TIM6
DAC->CR   |= (DAC_CR_TEN1                  //���������� ����������� �������� �������������� �� ������� �� TIM6
             |DAC_CR_DMAEN1                 //��������� ����� ������ ������ �1 �� ���
             |DAC_CR_EN1); 

RCC->APB1ENR        |=  RCC_APB1ENR_TIM6EN;    //������ ������������ TIM6
TIM6->PSC            =  0;                     //������ ������� ������������� 
TIM6->ARR            =  F_CLK/(SampleRate*Ncanal); //65536max                  
TIM6->CR2         |=  TIM_CR2_MMS_1;//MMS = 010 : update event is selected as a trigger output (TRGO)
}//end init_DAC_TIM6
//*******************************************************************************************************************
void init_DMA()//������������� ��� ����������� ������ �� ������� � ���
{	
  RCC->AHBENR  |= RCC_AHBENR_DMA1EN;	//������������ DMA
	//�����7 ����� 5 = dac1//111 -7 �����   
 DMA1_Channel3->CCR &=~ DMA_CCR_EN; //������������� ����� �����������������
	DMA1_Channel3->CCR=0;//����� CCR ��������
	
DMA1_Channel3->CPAR  = (uint32_t)&DAC->DHR8R1; //��������� �� ������� ���������???????channel1 8-bit right-aligned data
//DAC_DHR8Rx - ������� ��� ������ 8 ��� ������. ������������� �� ������� ����.
DMA1_Channel3->CMAR   = (uint32_t)&DAC_Buff; //��������� �� ������ ������ � ������ DAC 
DMA1_Channel3->CNDTR =  512;//������ ������,����� ����������
         
                
DMA1_Channel3->CCR &= DMA_CCR_PINC;        //����� ��������� �� ��������������
DMA1_Channel3->CCR |=  DMA_CCR_MINC;         //����� ������ ��������������
DMA1_Channel3->CCR &= ~DMA_CCR_PSIZE;        //����������� ������ ��������� - 8 ���.
DMA1_Channel3->CCR &= ~DMA_CCR_MSIZE;        //����������� ������ ������    - 8 ���
DMA1_Channel3->CCR |=  DMA_CCR_CIRC;         //�������� ����������� ����� �������� ������
 DMA1_Channel3->CCR |=  DMA_CCR_PL_1;         //��������� ����� �������

DMA1_Channel3->CCR  |=  DMA_CCR_DIR; 	       //1-����������� �������� - �� ������ � ���������

}//end init_DMA

void init_MCO(void)//������������� ������ Microcontroller clock output 
{//� 051 ��� PA8(pin 18)
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

   GPIOA->MODER &=~ GPIO_MODER_MODER8;//�����
	 GPIOA->MODER |= GPIO_MODER_MODER8_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->AFR[1]  &=~ GPIO_AFRH_AFSEL8 ;//�����,AF0 MCO
	 GPIOA->OSPEEDR |=(GPIO_OSPEEDER_OSPEEDR8_1|
	                  GPIO_OSPEEDER_OSPEEDR8_0);//10-������� �������� 10���,01-2���,11-50���


}
void no_noise()
{
RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
GPIOA->MODER |= GPIO_MODER_MODER4_0;//01-output,00-input(after reset),10-AF,11-analog.
GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_0;//10-������� �������� 10���,01-2���,11-50���	|
GPIOA->BSRR |= GPIO_BSRR_BS_4;	                 
}


//**********************************************************************************************************************

int main(void)
{
	
init_RCC();
//init_MCO();//	
no_noise();
spi_conf();	

init_DMA();	
NVIC_EnableIRQ (SPI1_IRQn);// ������� CMSIS ����������� ���������� � NVIC
__enable_irq ();// ��������� ���������� ����������
Read_Head();//������ ��������� Wav
init_DAC_TIM6(); 	// ������ ������� ��������
	
	
Read_SPI(512);	//��������� ���� �����

 DMA1_Channel3->CCR |= DMA_CCR_EN;  	//��������� ������ 3 ������ DMA	
 TIM6->CR1 |= TIM_CR1_CEN;//��������� ��������������

		
	
while(1)
  {
      while(!(DMA1->ISR & DMA_ISR_HTIF3)) {}   //���� ������������ ������ ����� ������ 5 ������	
				length=length-256;
				if ((length-256)<256){break;} //���� �������� ������������� ������ 256 ����	�������
			Read_SPI(256);//���������� ��� ���������� �� ���������,������� ������� �� 512	
			DMA1->IFCR |= DMA_IFCR_CHTIF3; //�������� ����
               
     
 
       while(!(DMA1->ISR & DMA_ISR_TCIF3)) {}   //���� ������������ ������ ����� ������ 5 ������
				length=length-256;
				if ((length-256)<256){break;} //���� �������� ������������� ������ 256 ����	�������
				Read_SPI(256); 
		 	 	DMA1->IFCR |= DMA_IFCR_CTCIF3;			//�������� ����
          
    
  }
  
	DMA1_Channel3->CCR &=~ DMA_CCR_EN;
	DAC->CR   &= ~DAC_CR_EN1;
  TIM6->CR1 &= ~TIM_CR1_CEN;
  reset_CS;//� ��������,������������� ������

 while(1){}//��������




}
	
