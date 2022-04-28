/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define     LCM_OUT1     	GPIOB->ODR
#define     LCM_OUT0        GPIOC->ODR
#define     LCM_RS         	GPIO_PIN_0        	// PC0
#define     LCM_RW         	GPIO_PIN_1        	// PC1
#define     LCM_E          	GPIO_PIN_2        	// PC2
#define     LCM_D7         	GPIO_PIN_7        	// PB7
#define     LCM_D6        	GPIO_PIN_6        	// PB6
#define     LCM_D5        	GPIO_PIN_5          // PB5
#define     LCM_D4        	GPIO_PIN_4          // PB4
#define     LCM_D3        	GPIO_PIN_3          // PB3
#define     LCM_D2        	GPIO_PIN_2          // PB2
#define     LCM_D1        	GPIO_PIN_1          // PB1
#define     LCM_D0        	GPIO_PIN_0          // PB0
#define     LCM_MASK  (LCM_D7|LCM_D6|LCM_D5|LCM_D4|LCM_D3|LCM_D2|LCM_D1| LCM_D0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
int volatile valADC; //переменная для считывания значение АЦП, чтобы избежать оптимизации
char col;		// номер колонки на дисплее
char addInd;	//адрес позиции курсора (строка +колонка)
char data;	// данные индикатора 0-7
int str;	// строка дисплея
int rs; 	// индекс команда или данные при записи в дисплей
int flenter;// флаг нажати кнопки ввод
int  flVal; // флаг изменения значения позиции курсора
int N; //
int tn=0;//установленная температура нагрева
int y;  //переменная преобразования кода АЦП в напряжение
int Tc1; //установленная температура нагрева для первой ст. программы
int tn1; //время для нагрева до установленной температуры
int ty1;// время удержания температуры установленной
int Tc2; //установленная температура нагрева для первой ст. программы
int tn2; //время для нагрева до установленной температуры
int ty2;// время удержания температуры установленной
int T;//перемення принимающая заначения Tc1 или Tc2
float Ti=0;				//расчет прироста температуры в единицу времени
int ti=0;		//текущее время нагрева
float  Td=0;  //расчетная текущая температура
uint8_t tim6_flag=0;//вводим флаг для обратного вызова в прерывании
int a6=0; //переменная нажатия кнопки
int t6=0; //промежуточная для времени по таймеру 6
uint8_t tim7_flag=0;//вводим флаг для обратного вызова в прерывании
int a7=0; //переменная нажатия кнопки
int tt2=0; //переменная температуры удержания
int tt1=0; //переменная температуры нагрева
float E, E0, Intg, Ki,Kd, p, cIntg, cDif, PID;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Takt(void)
{
	LCM_OUT0 &= ~LCM_E;
	HAL_Delay(10);
	LCM_OUT0 |= LCM_E;
	HAL_Delay(10);
	LCM_OUT0 &= (~LCM_E);
	HAL_Delay(45);
}
//------------------------------------------------------------------------------
void Write_DC(char data, int rs) //функция записии команды или данных
{
	if(rs==1) {
		LCM_OUT0=LCM_RS;
	}
	else {
		LCM_OUT0 &= ~LCM_RS;
	}

	LCM_OUT1 = data;
	Takt();
}
//-----------------------------------------------
void Cursor (char col, int str)
{

	if (str ==1) {
	addInd=0x80|col;
	LCM_OUT0 &= ~LCM_RS;
	LCM_OUT1 = addInd;
	Takt();
	}
	else {
		addInd=0x80|0x40|col;
		LCM_OUT0 &= ~LCM_RS;
		LCM_OUT1 = addInd;
		Takt();
	}

}
//-----------------------------------------
void InitializeLCD(void)
{
    LCM_OUT1 &= ~(LCM_MASK);
    HAL_Delay(100);
    LCM_OUT0 &= ~LCM_RS;
    LCM_OUT0 &= ~LCM_RW;
    LCM_OUT0 &= ~LCM_E;

    Takt();
    Write_DC(0x3C, 0);
    Write_DC(0x3C, 0);
    Write_DC(0x0E, 0);
    Write_DC(0x01, 0);
    Write_DC(0x06, 0);
}
//------------------------------
void Dsp1(void)
{
	  Write_DC(0x01, 0);
	  Cursor (0x01, 1);
	    Write_DC(0xA8, 1);//П
	    Write_DC(0x70, 1);//р
	    Write_DC(0x6F, 1);//о
	    Write_DC(0xB4, 1);//г
	    Write_DC(0x20, 1);//
	    Write_DC(0x4E, 1);//N
	    if (N==0)
	    {
	    Write_DC(0x31, 1); //при установке параметров 1 программы
	    }
	    else
	    {
	    	 Write_DC(0x32, 1); //при установке параметров 2 программы
	    }

	  Cursor (0x01, 2);
	  Write_DC(0x54, 1);//Т
	  Write_DC(0xBD, 1);//н
	  Write_DC(0x3D, 1);//=
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0
}
//----------------------------------------------------
void Dsp2(void)
{
	  Write_DC(0x01, 0);
	  Cursor (0x01, 1);
	  Write_DC(0x74, 1);//t
	  Write_DC(0xBD, 1);//н
	  Write_DC(0x3D, 1);//=
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0


	  Cursor (0x01, 2);
	  Write_DC(0x74, 1);//t
	  Write_DC(0x79, 1);//у
	  Write_DC(0x3D, 1);//=
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0
}
//-------------------------- Установка занчения для первой строки
void ChVal (void)
{
	int tn1=1;
	int tn2=0;
	int tn3=0;
	char tN1=0;
	char tN2=0;
	char tN3=0;
	char x=0;
while(flVal==1)
	{
       if(str==2){x=(0x40);}
       else {x=(0x00);}
		if(addInd==(0x80|x|0x06))
			{
			if((GPIOC->IDR & GPIO_PIN_12)==0)//если нажата кнопка С9
			{
			  a6=1;  // выставляем флаг нажатия
			}
				if(tim6_flag & a6) // если флаг прерывания по Т6 и флаг нажатия
				{
				tim6_flag = 0;
				a6=0;

				if(tn1<=9)
					{

					tN1=0x30|tn1;
					Write_DC(tN1, 1);// единицы температуры
					tn1=tn1+1;
					Cursor (0x06, str);
					}
				else
					{
					tn1=1;
					Write_DC(0x30, 1);
					tn2=tn2+1;
					if(tn2<=9)
						{
						tN2=0x30|tn2;
						Cursor (0x05, str);
						Write_DC(tN2, 1);// десятки температуры
						Cursor (0x06, str);
						}
					else
						{
						tn2=0;
						Cursor (0x05, str);
						Write_DC(0x30, 1);
						tn3=tn3+1;
						if(tn3<=9)
							{

							tN3=0x30|tn3;
							Cursor (0x04, str);
							Write_DC(tN3, 1);// десятки температуры

							Cursor (0x06, str);
							}
							else
							{
								tn3=0;
							}
						}
					}


				}
			}
		if((GPIOC->IDR & GPIO_PIN_10)==0) // если нажата кнопка С10
		{
		  a6=1;
		}
		if(tim6_flag & a6) // если флаг прерывания по т6 300мс и флаг нажатия кнопки

		{
			tim6_flag = 0;
			a6=0;
			flVal=0;
		}
		}
		tn=100*tn3+10*tn2+tn1;
	}
//------------------------------
void ValLCD (int n) // распознавание значения и кодировка для дисплея
{
	  switch (n) {
	      case 0:
	          data=(0x30);
	          break;
	      case 1:
	    	  data=(0x31);
	    	  break;
	      case 2:
	      	  data=(0x32);
	      	 break;
	      case 3:
	      	  data=(0x33);
	      	 break;
	      case 4:
	      	  data=(0x34);
	      	 break;
	      case 5:
	      	  data=(0x35);
	      	 break;
	      case 6:
	      	  data=(0x36);
	      	 break;
	      case 7:
	      	  data=(0x37);
	      	 break;
	      case 8:
	      	  data=(0x38);
	      	 break;
	      case 9:
	      	  data=(0x39);
	      	 break;
	  }
	 Write_DC(data, rs);
}
//----------------------------
void AcdReadLcd (int str)
{
	ADC1->CR |=  ADC_CR_ADSTART;  // запускаем преобразование

		  while((ADC1->CR &=ADC_ISR_EOC)==0) {} 	// пока не выставит флаг окончания преобразования
		  valADC=ADC1->DR;							// записываем содержимое регистра данных в переменную
		  ADC1->CR |=  ADC_CR_ADSTP;				// останавливаем преобразование
		  y=((3.3/4095)*valADC)*100;				// приводим к напряжению

		  Cursor (0x04, str);							// переводим курсор в позицию первого нуля
		  int y1=y/100;								// делим получившееся число после преобразования на 100 до единиц
		  int n=y1;									//целое число присваиваем переменной подпрограммы
		  ValLCD (n);								// распознаем для кодов дисплея сотеную позицию

		  Cursor (0x05, str);
		  int y2=(y-y1*100)/10;
		  n=y2;
		  ValLCD (n);								// распознаем для кодов дисплея десятичную позицию

		  Cursor (0x06, str);
		  y2=(y-y1*100)-y2*10;
		  n=y2;
		  ValLCD (n);								// распознаем для кодов дисплея единиц позицию

		  ADC1->DR=0x00;
		  HAL_Delay(1000);

}
//-----------------------------------------
void TimeLCD (ti)
{
	Cursor (0x04, 2);							// переводим курсор в позицию первого нуля
		 		  int ti1=ti/1000;					// делим получившееся число на 1000 до единиц
		 		  int n=ti1;						//целое число присваиваем переменной подпрограммы
		 		  ValLCD (n);						// распознаем для кодов дисплея сотеную позицию

		 		  Cursor (0x05, 2);
		 		  int ti2=(ti-ti1*1000)/100;
		 		  n=ti2;
		 		  ValLCD (n);								// распознаем для кодов дисплея десятичную позицию

		 		  Cursor (0x06, 2);
		 		  int ti3=((ti-ti1*1000)-ti2*100)/10;
		 		  n=ti3;
		 		  ValLCD (n);

		 		 Cursor (0x07, 2);
		 		 ti3=(((ti-ti1*1000)-ti2*100)-ti3*10);
		 		 n=ti3;
		 		 ValLCD (n);

}

//----------------------

void   Disptnok(void)
{Cursor (0x01, 2);
   	  Write_DC(0x74, 1);//t
   	  Write_DC(0x6E, 1);//n
   	  Write_DC(0x3D, 1);//=
   	  Write_DC(0x4F, 1);//O
   	  Write_DC(0x6B, 1);//k
   	  Write_DC(0x20, 1);
   	  Write_DC(0x20, 1);
   	  }
//---------------------------------------------
void DispNagr(void)
{
	   	  Write_DC(0x54, 1);//Т
	   	  Write_DC(0x69, 1);//i
	   	  Write_DC(0x3D, 1);//=

	   	Cursor (0x01, 2);
	   	   	  Write_DC(0x74, 1);//t
	   	   	  Write_DC(0x6E, 1);//n
	   	   	  Write_DC(0x3D, 1);//=

}
//--------------------------------------------------------
void TIM6_Callback(void)// обратный вызов из прерывания для кнопок
{
	tim6_flag = 1;
	TIM6->SR&=(~TIM_SR_UIF);
    if(t6<=4) {t6++;}
    else {ti++;t6=0;}
}
//----------------------------------------------------------------

//----------------------------------------------------------------
void Ten (void)
{
	p = 240;
	Ki = 19;
	Kd = 600;
		E0=E;  //Предыдущенее значение ошибки
		E=y-Td;	//текущая ошибка в градусах
		Intg=Intg+(E+E0)/2; //интеграл рассогласования
		cIntg = Intg*(1/Ki); // Интегральный вклад в регулирование
		cDif = Kd*(E-E0); // Дифференциальный вклад в регулирование
		PID = (p*E + cIntg + cDif);  // Регулятор
		TIM3->CCR2=PID*20;

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  InitializeLCD(); //инициализация дисплея

      ADC1->CR &= ~(1U<<29UL); //сбрасываем бит выключение регулятора напряжени АЦП в 00
      ADC1->CR |= (1U<<28UL);   // включаем регулятор напряжения АЦП установив в 01
      HAL_Delay(1);

      ADC1->CR &= ~(ADC_CR_ADCALDIF);  // устанавливаем бит калибровки диф канала в 0
      ADC1->CR |=  ADC_CR_ADCAL;    // устанавливаем в бит калибровки несимметричного канала в 1
      while((ADC1->CR &  ADC_CR_ADCAL)!=0)  { }  // ждем выполнения калибровки пока станет равен 0

      ADC1->IER |= ADC_IER_ADRDYIE;				// устанавливаем разрешение прерывания по флагу включения АЦП
      ADC1->IER |= ADC_IER_EOCIE;
      ADC1->CFGR |= ADC_CFGR_CONT; 	//разрешение непрерывного преобразования для обычных преобразований
      ADC1->CR |=  ADC_CR_ADEN;					// включаем АЦП
      while((ADC1->ISR &  ADC_ISR_ADRDY)==0)  { } // ждем установки флага включения АЦП

       TIM6->DIER |= TIM_DIER_UIE;   //разрешение прерывания по Т6
       NVIC_EnableIRQ(TIM6_DAC1_IRQn);//Включение прерывания
       TIM6->CR1 |= TIM_CR1_CEN;//включение таймера 6


       TIM3->CCER|= (TIM_CCER_CC2E);// разрешение вывода ШИМ на ногу 2-го канала А7
       TIM3->CR1 |= TIM_CR1_CEN;//включение таймера 3

       Dsp1();		 	//	выводим 1 дисплей с номером программы и температурой нагрева
           str=2;			//  для 2 строки дисплея
           Cursor (0x06, str);
           flVal=1;
           ChVal ();		//	подпрограмма установки числа во второй строке
           Tc1=tn;			// присваеваем число переменной температуре нагрева
           tn=0;			// обнуляем
           HAL_Delay(1000);

           Dsp2();			// выводим 2 дисплей
           str=1;			// переходми в 1 строку
           Cursor (0x06, str);
           flVal=1;
           ChVal ();		// вводим значение для времени нагрева
           tn1=tn;			// присваиваем значение переменной времени нагрева
           tn=0;			// обнуляем
           HAL_Delay(1000);

           str=2;
           Cursor (0x06, str);
           flVal=1;
           ChVal ();		// вводим число для времени удержания температуры
           ty1=tn;			// присваиваем значение переменной
           tn=0;
           HAL_Delay(1000);

           N=2;  			// переход к установке параметров программы 2
             Dsp1();		//	выводим 1 дисплей с номером программы и температурой нагрева
             str=2;			//  для 2 строки дисплея
             Cursor (0x06, str);
             flVal=1;
             ChVal ();		//	подпрограмма установки числа во второй строке
             Tc2=tn;			// присваеваем число переменной температуре нагрева
             tn=0;			// обнуляем
             HAL_Delay(1000);

             Dsp2();			// выводим 2 дисплей
             str=1;			// переходми в 1 строку
             Cursor (0x06, str);
             flVal=1;
             ChVal ();		// вводим значение для времени нагрева
             tn2=tn;			// присваиваем значение переменной времени нагрева
             tn=0;			// обнуляем
             HAL_Delay(1000);

             str=2;
             Cursor (0x06, str);
             flVal=1;
             ChVal ();		// вводим число для времени удержания температуры
             ty2=tn;			// присваиваем значение переменной
             tn=0;
             HAL_Delay(1000);

             while (a6==0)				//кнопка запуска исполнения программы
            {
            	 int a7=0;
            	if((GPIOC->IDR & GPIO_PIN_9)==0)
            {
            		a7=1;
            }
            if((tim6_flag & a7)==1) // если флаг прерывания по т6 300мс и флаг нажатия кнопки
            	{
             			tim6_flag = 0;
             			a6=1;
            	}
            }
             a6=0;

             Write_DC(0x01, 0);// очистка дисплея

             Cursor (0x00, 1);
        	 Write_DC(0x31, 1);//1
             DispNagr();

             str=1;
             rs=1;
             AcdReadLcd (str);  // определяем текущую температуру начальную и выводим на дисплей. Здесь есть задержка 1секунда

                   Ti= (Tc1-y)/(tn1);				//расчет прироста температуры в минуту
                   Ti= Ti/60;  //расчет прироста температуры в секунду
                   Td=y+Ti;//температура которая будет через 1 секунду после включения
                   ti=0;
                   TimeLCD (ti);

  /* USER CODE END 2 */
                   tt1=tn1*60;

  /* USER CODE BEGIN WHILE */
  while (y<Tc1)
  {

     AcdReadLcd (str); // преобразования текущей температуры АЦП и вывод на дисплей

     Ten();   // вызов ПИД регулятора
if(ti<tt1)
{
	TimeLCD (ti);					//вывод на дисплеий текущего времен нагрева
}
else
{
	Disptnok();					// при окончании установленного времени нагрева вывод Ок
}

Td=Td+Ti;
}


ty1=ty1-1;
if (ty1>0)   // нагрев закончился и начинается удержание если время удержания больше 0
{
Cursor (0x01, 2);
Write_DC(0x74, 1);//t
Write_DC(0x79, 1);//y
Write_DC(0x3D, 1);//=


tt2=ty1*60;
Td=Tc1;
ti=0;
while(ti<tt2)
{
TimeLCD (ti);					//вывод на дисплеий текущего времен нагрева
AcdReadLcd (str);				//определение текущей температуры
Ten();							// вызов ПИД регулятора
}

}

//-------------------------------------- программа 2 нагрева
Tc2=Tc2-1;
if(Tc2<=0){
Write_DC(0x01, 0);// очистка дисплея
Cursor (0x00, 1);
Write_DC(0x32, 1);//2
DispNagr();		//

str=1;
rs=1;
AcdReadLcd (str);  			// определяем текущую температуру начальную и выводим на дисплей. Здесь есть задержка 1секунда

Ti= (Tc2-y)/(tn2*60);			//расчет прироста температуры в единицу времени
Td=y+Ti;			// ожидаемая температура в i момент времени из расчета времени нагрева
tn2=tn2-1;
tt1=tn2*60;
ti=0;
while (y<Tc2)
{

AcdReadLcd (str);
TimeLCD ();					//вывод на дисплеий текущего времен нагрева
Ten();							// вызов ПИД регулятора
if(ti<tt1)
{
	TimeLCD ();
}
else
{
Disptnok();				// при окончании установленного времени нагрева вывод Ок
}
Td=Td+Ti;
}
// ------------ окончание нагрева по 2 программе
ty2=ty2-1;
if (ty2>0)    // если время удержания по 2 программе не равно 0
{
Cursor (0x01, 2);
Write_DC(0x74, 1);//t
Write_DC(0x79, 1);//y
Write_DC(0x3D, 1);//=

tt2=ty2*60;
Td=Tc2;
ti=0;
while(ti<tt2)
{
TimeLCD (ti);					//вывод на дисплеий текущего времен нагрева
AcdReadLcd (str);
Ten();
}
}
}
Write_DC(0x01, 0);// очистка дисплея
Cursor (0x01, 1);
Write_DC(0x65, 1);//e
Write_DC(0x6E, 1);//n
Write_DC(0x64, 1);//d
    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 150;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 63999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
