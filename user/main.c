
//Задача: Вооспроизвести файл сэмплов, хранящихся на микросхеме памяти 25 серии(W25Q16, интерфейс SPI) с помощью DAC 
//STM32F407xx .Длина файла заранее неизвестна. Память состоит из страниц по 256 Байт(2048 бит).Данные в DAC грузим 
//с помощью DMA.Майном читаем память сначала в промежуточный массив,откуда данные берет DMA.DAC тактируется TIM6.
// если стерео, то в моно переделываем увеличением частоты дискритизации в 2 раза без разбора сэмплов по каналам
//при назкой скорости SPI возникает эффект низкой частоты дискритизации при верных настройках TIM6

#include "stm32f0xx.h"
#include "delay.h" //Генерация пауз, как правило,всегда нужна во всех проектах
#include "spi25.h"//процедуры для работы с flash
#include "WavHead.h"

#define reset_CS GPIOA->BSRR |= GPIO_BSRR_BR_15;//выбор кристалла не активен(лог 1)

#define F_CLK 48000000// частота тактирования TIM6

unsigned char DAC_Buff[512]; //промежуточный буфер для данных которые ДМА перекидает в ЦАП
uint32_t length;//количество фреймов в Wav файле
uint32_t byteRate;//Количество байт, переданных за секунду воспроизведения.
uint8_t Ncanal;//Число каналов
uint32_t SampleRate;//частота дискритизации

 void init_RCC()//настройка тактирования,пользуемся картинкой CUBE MX
{
RCC->CR|=RCC_CR_HSEON; //Запускаем генератор HSE,внешний кварц
while (!(RCC->CR & RCC_CR_HSERDY)) {}; // Ждем готовности
	
FLASH->ACR |= FLASH_ACR_PRFTBE; // Enable Prefetch Buffer.
FLASH->ACR &= ~FLASH_ACR_LATENCY; // Предочистка.
FLASH->ACR |= FLASH_ACR_LATENCY; // if 24 MHz < SYSCLK < 48 MHz

RCC->CR &=~RCC_CR_PLLON;//	
RCC->CFGR &=~RCC_CFGR_SW; // Очистить биты SW0, SW1
RCC->CFGR &=~ RCC_CFGR_PPRE;	//000: HCLK clock divided by 1 for APB
RCC->CFGR &=~ RCC_CFGR_HPRE;	//SYSCLK DIV BY 1 FOR HCLK
RCC->CFGR &=~ RCC_CFGR_PLLXTPRE;	//ОТКЛЮЧАЕМ ПРЕДДЕЛИТЕЛЬ PLL


	
 //Config PLL только для F051:
// RCC->PLLCFGR=(MCOPRE<<28) | (MCO<<24) | (PLLMUL<<18) | PLLXTPRE |(PLLSRC<<15)|( PPRE<<8)|(HPRE<<4)|SW; //page 162	
//	MCOPRE-предделитель выхода MCO,MCO-какой сигнал выводить,PLLXTPRE-будет ли включен предделитель PLL(значение в RCC_CFGR2)
//PLLSRC-откуда тактируем PLL,PPRE-делитель для APB,HPRE-делитель для AHB,SW источник тактирования
RCC->CFGR= (4<<18) | (0b10<<15)| RCC_CFGR_SW_PLL; //   настраевается при сброшеном бите 	PLLON  в CR		
RCC->CR |=    RCC_CR_PLLON;//да
while (!(RCC->CR & RCC_CR_PLLRDY)){}; //wait for PLL ready 	

}//end init_RCC()
//*****************************************************************************************************************
void init_DAC_TIM6()//инициализация ЦАП И ТИМ6,тактирующего ЦАП,PA4-порт выхода
{
RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
GPIOA->MODER |= GPIO_MODER_MODER4;//01-output,00-input(after reset),10-AF,11-analog.	
RCC->APB1ENR |= RCC_APB1ENR_DACEN;             //тактирование ЦАПа

DAC->CR   &=(DAC_CR_BOFF1                  //ОТключить выходной буфер
             | DAC_CR_TSEL1);              //000-источник запуска преобразования TIM6
DAC->CR   |= (DAC_CR_TEN1                  //обновление содержимого регистра преобразования по событию от TIM6
             |DAC_CR_DMAEN1                 //разрешить прием данных канала №1 от ДМА
             |DAC_CR_EN1); 

RCC->APB1ENR        |=  RCC_APB1ENR_TIM6EN;    //подаем тактирование TIM6,на APB1 таймеры работают с удвоенной частотой APB1 те 84МГц
TIM6->PSC            =  0;                     //Задаем частоту дискретизации 
TIM6->ARR            =  F_CLK/(SampleRate*Ncanal); //65536max                  //(при тактовой 84000000)
TIM6->CR2         |=  TIM_CR2_MMS_1;//MMS = 010 : update event is selected as a trigger output (TRGO)
}//end init_DAC_TIM6
//*******************************************************************************************************************
void init_DMA()//инициализация ДМА передающего данные из массива в ЦАП
{	
  RCC->AHBENR  |= RCC_AHBENR_DMA1EN;	//тактирование DMA
	//канал7 поток 5 = dac1//111 -7 канал   
 DMA1_Channel3->CCR &=~ DMA_CCR_EN; //останавливаем перед конфигурированием
	DMA1_Channel3->CCR=0;//сброс CCR регистра
	
DMA1_Channel3->CPAR  = (uint32_t)&DAC->DHR8R1; //указатель на регистр периферии???????channel1 8-bit right-aligned data
//DAC_DHR8Rx - регистр для записи 8 бит данных. Выравниваение по правому краю.
DMA1_Channel3->CMAR   = (uint32_t)&DAC_Buff; //указатель на начало буфера в памяти DAC 
DMA1_Channel3->CNDTR =  512;//размер буфера,число транзакций
         
                
DMA1_Channel3->CCR &= DMA_CCR_PINC;        //адрес периферии не инкрементируем
DMA1_Channel3->CCR |=  DMA_CCR_MINC;         //адрес памяти инкрементируем
DMA1_Channel3->CCR &= ~DMA_CCR_PSIZE;        //размерность данных периферии - 8 бит.
DMA1_Channel3->CCR &= ~DMA_CCR_MSIZE;        //размерность данных памяти    - 8 бит
DMA1_Channel3->CCR |=  DMA_CCR_CIRC;         //включить циклический режим передачи данных
 DMA1_Channel3->CCR|=  DMA_CCR_PL_1;         //приоритет очень высокий

DMA1_Channel3->CCR  |=  DMA_CCR_DIR; 	       //1-направление передачи - из памяти в периферию

}//end init_DMA

//**********************************************************************************************************************

int main(void)
{
	
init_RCC();
spi_conf();	

init_DMA();	
NVIC_EnableIRQ (SPI1_IRQn);// Функции CMSIS разрешающие прерывания в NVIC
__enable_irq ();// Разрешаем глобальные прерывания
Read_Head();//читаем заголовок Wav
init_DAC_TIM6(); 	// теперь битрейд известен
	
	
Read_SPI(512);	//заполняем весь буфер

 DMA1_Channel3->CCR |= DMA_CCR_EN;  	//разрешаем работу 3 канала DMA	
 TIM6->CR1 |= TIM_CR1_CEN;//запустить преобразование

		
	
while(1)
  {
      while(!(DMA1->ISR & DMA_ISR_HTIF3)) {}   //ждем освобождение первой части буфера 5 потока	
				length=length-256;
				if ((length-256)<256){break;} //если осталось воспроизвести меньше 256 байт	выходим
			Read_SPI(256);//обработчик сам расскидает по половинам,счетчик индекса до 512	
			DMA1->IFCR |= DMA_IFCR_CHTIF3; //сбросить флаг
               
     
 
       while(!(DMA1->ISR & DMA_ISR_TCIF3)) {}   //ждем освобождение второй части буфера 5 потока
				length=length-256;
				if ((length-256)<256){break;} //если осталось воспроизвести меньше 256 байт	выходим
				Read_SPI(256); 
		 	 	DMA1->IFCR |= DMA_IFCR_CTCIF3;			//сбросить флаг
          
    
  }
  
	DMA1_Channel3->CCR &=~ DMA_CCR_EN;
	DAC->CR   &= ~DAC_CR_EN1;
  TIM6->CR1 &= ~TIM_CR1_CEN;
  reset_CS;//в принципе,необязательно делать

 while(1){}//зависаем




}
	
