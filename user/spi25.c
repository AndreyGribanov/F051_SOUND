#include "stm32f0xx.h"
#include "delay.h"


#define READ  0x03//������� ������ ������. ����� ��� ���� �������� 3 ����� ������ � �������� �� ������ ������. 
                   //����� ������ ����� ����������� �� ��� ��� ���� �� �� ����������� �������� ����������� ������� �� CLK 
									 //����� SPI (����������� ��� ������ ������, � ��� ���������� max ������ ������ ���������� � 
									  //����� ���������� ������ ���� � ��������� �����). 

#define set_CS GPIOB->BSRR |= GPIO_BSRR_BR_6//����� ��������� �������(��� 0)
#define reset_CS GPIOB->BSRR |= GPIO_BSRR_BS_6;//����� ��������� �� �������(��� 1)


extern unsigned char DAC_Buff[512];//�����, � ������� ��������� ���������� ����������
//extern-��������� �� ������� ����������(� main.c ���������� ��� ������� ��� ��� ����� � ������) � �� �������� ������
uint16_t i = 0;//����������,������ ������� � ������� ����� ������


void spi_conf()//��������� spi ��� F051
{	
  //�������� ������������ SPI1 � GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN ;//� h ����� ���� ��������� �� 2,� datasheet ��� ����
	
	  //����������� ������ SPI1:
	//PB6-FCS-����� ���������
	 GPIOB->MODER &=~ GPIO_MODER_MODER6;//�����
	 GPIOB->MODER |= GPIO_MODER_MODER6_0;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOB->OTYPER &=~ GPIO_OTYPER_OT_6;//1-�������� ���������,0-�����������(����� ������)
	 GPIOB->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR6_1;//10-������� �������� 10���,01-2���,11-50���
	 GPIOB->BSRR |= GPIO_BSRR_BS_6;//����� ��������� ���������(��� 1)
	
  //PB3-SCK: 
   GPIOB->MODER &=~ GPIO_MODER_MODER3;//�����
	 GPIOB->MODER |= GPIO_MODER_MODER3_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOB->AFR[0]  &=~ GPIO_AFRL_AFSEL3 ;//�����,AF0 -SPI1
	 GPIOB->OSPEEDR |=(GPIO_OSPEEDER_OSPEEDR3_1|
	                  GPIO_OSPEEDER_OSPEEDR3_0);//10-������� �������� 10���,01-2���,11-50���
		  
  //PB4-MISO:
	 GPIOB->MODER &=~ GPIO_MODER_MODER4;//�����
	 GPIOB->MODER |= GPIO_MODER_MODER4_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOB->AFR[0]  &=~ GPIO_AFRL_AFSEL4 ;//�����,AF0 -SPI1
   /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 //GPIOB->AFR[0]  |= (GPIO_AFRL_AFSEL4_2 | GPIO_AFRL_AFSEL4_0);//AF=5 -SPI1	 
	 //GPIOA->AFR[0] |=(1<<(16)) |(1<<(18));-������� ���������,H ���� �� ����� ������� �������� ��������� GPIO_AFRL_AFSEL4_0
   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
	
  //PB5-MOSI:
	 GPIOB->MODER &=~ GPIO_MODER_MODER5;//�����
	 GPIOB->MODER |= GPIO_MODER_MODER5_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOB->AFR[0]  &=~ GPIO_AFRL_AFSEL5 ;//�����,AF0 -SPI1
	 GPIOB->OSPEEDR |=(GPIO_OSPEEDER_OSPEEDR5_1|
	                  GPIO_OSPEEDER_OSPEEDR5_0);//10-������� �������� 10���,01-2���,11-50���
		
   SPI1->CR2 |= SPI_CR2_FRXTH;  //��������� rxne ��� ������� 8 ��� � FIFO
	
	SPI1->CR2 &=~ SPI_CR2_DS;
	SPI1->CR2 |= (SPI_CR2_DS_2|SPI_CR2_DS_1|SPI_CR2_DS_0);//����� 8 ���
	
   SPI1->CR1 &= ~SPI_CR1_LSBFIRST ;    //MSB ���������� ������,��� =1-LSB ���������� ������(�������)
   SPI1->CR1 |=SPI_CR1_SSM ;         //����������� ���������� ������� ������ NSS ���������� ��������� ���� SSI.
   SPI1->CR1 |=SPI_CR1_SSI ;         //SSi � ������� ���������
   SPI1->CR1 |=(SPI_CR1_BR_2);//|SPI_CR1_BR_1);//|SPI_CR1_BR_0) ;//SPI_CR1_BR_2|0x04�������� ��������: F_PCLK/4,����� 0 F_PCLK/2
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
 
 *(uint8_t *)&(SPI1->DR) = data;//�� ��������� ������ � SPIx-�DR 16���!!���������� ������ � F051,�����,data ����� �������� � � ����� ff00
	//	SPI1->DR = data;//��������� ����� �����������/��� ���� � f103,f407
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
	delay_us(500);
	//delay_us(1000);
	SPI_Write(READ);//0x03,������� ������ ����	
	SPI_Write(0);//������� ���� ������ ��������
	SPI_Write(0);//������� ���� ������ ��������	
	SPI_Write(0);//����� ������� �����
  i=0;//������ �������� � ������ ������

}

