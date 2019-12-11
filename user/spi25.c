#include "stm32f0xx.h"
#include "delay.h"


#define READ  0x03//команда чтения памяти. После нее надо передать 3 байта адреса с которого вы хотите читать. 
                   //После память будет считываться до тех пор пока вы не перестанете подавать тактирующие сигналы по CLK 
									 //линии SPI (считываются все адреса подряд, а при достижении max адреса памяти микросхемы — 
									  //адрес становится равным нулю — кольцевой буфер). 

#define set_CS GPIOA->BSRR |= GPIO_BSRR_BS_15//выбор кристалла активен(лог 0)
#define reset_CS GPIOA->BSRR |= GPIO_BSRR_BR_15;//выбор кристалла не активен(лог 1)


extern unsigned char DAC_Buff[512];//буфер, в который считывает обработчик прерываний
//extern-указывает на внешнюю переменную(в main.c компилятор уже выделил под нее место в памяти) и не выделяет память
uint16_t i = 0;//глобальная,индекс массива в который БУДЕМ читать


void spi_conf()//настройка spi
{	
  //Включаем тактирование SPI1 и GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN ;//в h файле шина разделена на 2,в datasheet она одна
	
	  //Настраиваем выводы SPI1:
	//PA15-FCS-выбор кристалла
	 GPIOA->MODER &=~ GPIO_MODER_MODER15;//сброс
	 GPIOA->MODER |= GPIO_MODER_MODER15_0;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->OTYPER &=~ GPIO_OTYPER_OT_15;//1-открытый коллектор,0-двухтактный(после сброса)
	 GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR15_1;//10-высокая скорость 10МГц,01-2МГц,11-50Мгц
	 GPIOA->BSRR |= GPIO_BSRR_BS_15;//выбор кристалла активен(лог 0)
	
  //PA5-SCK: 
   GPIOA->MODER &=~ GPIO_MODER_MODER5;//сброс
	 GPIOA->MODER |= GPIO_MODER_MODER5_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->AFR[0]  &=~ GPIO_AFRL_AFSEL5 ;//сброс,AF0 -SPI1
	 GPIOA->OSPEEDR |=(GPIO_OSPEEDER_OSPEEDR5_1|
	                  GPIO_OSPEEDER_OSPEEDR5_0);//10-высокая скорость 10МГц,01-2МГц,11-50Мгц
		  
  //PA6-MISO:
	 GPIOA->MODER &=~ GPIO_MODER_MODER6;//сброс
	 GPIOA->MODER |= GPIO_MODER_MODER6_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->AFR[0]  &=~ GPIO_AFRL_AFSEL6 ;//сброс,AF0 -SPI1
   /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 //GPIOB->AFR[0]  |= (GPIO_AFRL_AFSEL4_2 | GPIO_AFRL_AFSEL4_0);//AF=5 -SPI1	 
	 //GPIOA->AFR[0] |=(1<<(16)) |(1<<(18));-ВАРИАНТ НАВЕРНЯКА,H ФАЙЛ НЕ ИМЕЕТ АДРЕСОВ ПОБИТНОЙ НАСТРОЙКИ GPIO_AFRL_AFSEL4_0
   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
	
  //PA7-MOSI:
	 GPIOA->MODER &=~ GPIO_MODER_MODER7;//сброс
	 GPIOA->MODER |= GPIO_MODER_MODER7_1;//01-output,00-input(after reset),10-AF,11-analog.
	 GPIOA->AFR[0]  &=~ GPIO_AFRL_AFSEL7 ;//сброс,AF0 -SPI1
	 GPIOA->OSPEEDR |=(GPIO_OSPEEDER_OSPEEDR7_1|
	                  GPIO_OSPEEDER_OSPEEDR7_0);//10-высокая скорость 10МГц,01-2МГц,11-50Мгц
		
  // SPI1->CR1 &= ~SPI_CR1_DFF;  //Размер кадра 8 бит def=0,def=1-16 бит
	
	SPI1->CR2 &=~ SPI_CR2_DS;
	SPI1->CR2 |= (SPI_CR2_DS_2|SPI_CR2_DS_1|SPI_CR2_DS_0);//ДЛИНА 8 БИТ
	
   SPI1->CR1 &= ~SPI_CR1_LSBFIRST ;    //MSB передается первым,при =1-LSB передается первым(младший)
   SPI1->CR1 |=SPI_CR1_SSM ;         //Программное управление ведомым сигнал NSS заменяется значением бита SSI.
   SPI1->CR1 |=SPI_CR1_SSI ;         //SSi в высоком состоянии
   SPI1->CR1 |=(SPI_CR1_BR_1|SPI_CR1_BR_0) ;//SPI_CR1_BR_2|0x04Скорость передачи: F_PCLK/4,когда 0 F_PCLK/2
   SPI1->CR1 |=SPI_CR1_MSTR ;        //Режим Master (ведущий)
   SPI1->CR1 &=~(SPI_CR1_CPOL | SPI_CR1_CPHA); //Режим работы SPI: 0
	 SPI1->CR2 |= SPI_CR2_RXNEIE;
	 SPI1->CR1 |= SPI_CR1_SPE; //Включаем SPI
}	//end spi_conf
//*******************************************************************************************************************

void SPI_Write(uint8_t data)//
{
  while(!(SPI1->SR & SPI_SR_TXE)){}// ждем,когда можно будет загружать данные в DR
                                    // TXE флаг опустошения буфера передачи
  SPI1->DR = data;//заполняем буфер передатчика
	while(SPI1->SR & SPI_SR_BSY){}//ждем окончания передачи(необязательное условие)
 
}
//*********************************************************************************************************************
	void SPI1_IRQHandler(void)//обработчик прерываний 
{
 if (SPI1->SR & SPI_SR_RXNE)//если пришли данные(поднят флаг RXNE)..,сброс в F4 после чтения DR
	{
		DAC_Buff[i] = SPI1->DR;
		i++;
		if (i==512) i=0;
	}//Очистка бита RXNE выполняется при чтении регистра SPI_DR.
	 }
//************************************************************************************************************************

void Read_SPI(uint16_t N)//чтение в буфер N количества данных,фактически тактирует SPI,а обработчик читает в буфер 
{ 
	for(uint16_t n=0;n<N;n++)//когда uint8_t =256,то оно воспринимается как 0
	{	SPI_Write(0xff);	}
	
	
}//end Read_SPI
//***********************************************************************************************************************

void Read_En(void)//процедура перевода в режим чтения микросхемы
{
  set_CS;
	delay_ms(1);
	SPI_Write(READ);//0x03,команда чтения флеш	
	SPI_Write(0);//старший байт номера страницы
	SPI_Write(0);//младший байт номера страницы	
	SPI_Write(0);//номер первого байта
  i=0;//читать начинаем в начало буфера

}

