#include "stm32f0xx.h"
#include "delay.h"


#define READ  0x03//������� ������ ������. ����� ��� ���� �������� 3 ����� ������ � �������� �� ������ ������. 
                   //����� ������ ����� ����������� �� ��� ��� ���� �� �� ����������� �������� ����������� ������� �� CLK 
									 //����� SPI (����������� ��� ������ ������, � ��� ���������� max ������ ������ ���������� � 
									  //����� ���������� ������ ���� � ��������� �����). 

#define set_CS GPIOA->BSRR |= GPIO_BSRR_BS_15//����� ��������� �������(��� 0)
#define reset_CS GPIOA->BSRR |= GPIO_BSRR_BR_15;//����� ��������� �� �������(��� 1)


extern unsigned char DAC_Buff[512];//�����, � ������� ��������� ���������� ����������
//extern-��������� �� ������� ����������(� main.c ���������� ��� ������� ��� ��� ����� � ������) � �� �������� ������
uint16_t i = 0;//����������,������ ������� � ������� ����� ������


void spi_conf()//��������� spi
{	
  //�������� ������������ SPI1 � GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN ;//� h ����� ���� ��������� �� 2,� datasheet ��� ����
	
	  //����������� ������ SPI1:
	//PA15-FCS-����� ���������
	 GPIOA->MODER &=~ GPIO_MODER_MODER15;//�����
	 GPIOA->MODER |= GPIO_MODER_MODER15_0;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->OTYPER &=~ GPIO_OTYPER_OT_15;//1-�������� ���������,0-�����������(����� ������)
	 GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR15_1;//10-������� �������� 10���,01-2���,11-50���
	 GPIOA->BSRR |= GPIO_BSRR_BS_15;//����� ��������� �������(��� 0)
	
  //PA5-SCK: 
   GPIOA->MODER &=~ GPIO_MODER_MODER5;//�����
	 GPIOA->MODER |= GPIO_MODER_MODER5_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->AFR[0]  &=~ GPIO_AFRL_AFSEL5 ;//�����,AF0 -SPI1
	 GPIOA->OSPEEDR |=(GPIO_OSPEEDER_OSPEEDR5_1|
	                  GPIO_OSPEEDER_OSPEEDR5_0);//10-������� �������� 10���,01-2���,11-50���
		  
  //PA6-MISO:
	 GPIOA->MODER &=~ GPIO_MODER_MODER6;//�����
	 GPIOA->MODER |= GPIO_MODER_MODER6_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->AFR[0]  &=~ GPIO_AFRL_AFSEL6 ;//�����,AF0 -SPI1
   /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 //GPIOB->AFR[0]  |= (GPIO_AFRL_AFSEL4_2 | GPIO_AFRL_AFSEL4_0);//AF=5 -SPI1	 
	 //GPIOA->AFR[0] |=(1<<(16)) |(1<<(18));-������� ���������,H ���� �� ����� ������� �������� ��������� GPIO_AFRL_AFSEL4_0
   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
	
  //PA7-MOSI:
	 GPIOA->MODER &=~ GPIO_MODER_MODER7;//�����
	 GPIOA->MODER |= GPIO_MODER_MODER7_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->AFR[0]  &=~ GPIO_AFRL_AFSEL7 ;//�����,AF0 -SPI1
	 GPIOA->OSPEEDR |=(GPIO_OSPEEDER_OSPEEDR7_1|
	                  GPIO_OSPEEDER_OSPEEDR7_0);//10-������� �������� 10���,01-2���,11-50���
		
  // SPI1->CR1 &= ~SPI_CR1_DFF;  //������ ����� 8 ��� def=0,def=1-16 ���
	
	SPI1->CR2 &=~ SPI_CR2_DS;
	SPI1->CR2 |= (SPI_CR2_DS_2|SPI_CR2_DS_1|SPI_CR2_DS_0);//����� 8 ���
	
   SPI1->CR1 &= ~SPI_CR1_LSBFIRST ;    //MSB ���������� ������,��� =1-LSB ���������� ������(�������)
   SPI1->CR1 |=SPI_CR1_SSM ;         //����������� ���������� ������� ������ NSS ���������� ��������� ���� SSI.
   SPI1->CR1 |=SPI_CR1_SSI ;         //SSi � ������� ���������
   SPI1->CR1 |=(SPI_CR1_BR_1|SPI_CR1_BR_0) ;//SPI_CR1_BR_2|0x04�������� ��������: F_PCLK/4,����� 0 F_PCLK/2
   SPI1->CR1 |=SPI_CR1_MSTR ;        //����� Master (�������)
   SPI1->CR1 &=~(SPI_CR1_CPOL | SPI_CR1_CPHA); //����� ������ SPI: 0
	 SPI1->CR2 |= SPI_CR2_RXNEIE;
	 SPI1->CR1 |= SPI_CR1_SPE; //�������� SPI
}	//end spi_conf
//*******************************************************************************************************************

void SPI_Write(uint8_t data)//
{
  while(!(SPI1->SR & SPI_SR_TXE)){}// ����,����� ����� ����� ��������� ������ � DR
                                    // TXE ���� ����������� ������ ��������
  SPI1->DR = data;//��������� ����� �����������
	while(SPI1->SR & SPI_SR_BSY){}//���� ��������� ��������(�������������� �������)
 
}
//*********************************************************************************************************************
	void SPI1_IRQHandler(void)//���������� ���������� 
{
 if (SPI1->SR & SPI_SR_RXNE)//���� ������ ������(������ ���� RXNE)..,����� � F4 ����� ������ DR
	{
		DAC_Buff[i] = SPI1->DR;
		i++;
		if (i==512) i=0;
	}//������� ���� RXNE ����������� ��� ������ �������� SPI_DR.
	 }
//************************************************************************************************************************

void Read_SPI(uint16_t N)//������ � ����� N ���������� ������,���������� ��������� SPI,� ���������� ������ � ����� 
{ 
	for(uint16_t n=0;n<N;n++)//����� uint8_t =256,�� ��� �������������� ��� 0
	{	SPI_Write(0xff);	}
	
	
}//end Read_SPI
//***********************************************************************************************************************

void Read_En(void)//��������� �������� � ����� ������ ����������
{
  set_CS;
	delay_ms(1);
	SPI_Write(READ);//0x03,������� ������ ����	
	SPI_Write(0);//������� ���� ������ ��������
	SPI_Write(0);//������� ���� ������ ��������	
	SPI_Write(0);//����� ������� �����
  i=0;//������ �������� � ������ ������

}

