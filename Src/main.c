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
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "flash_wrt.h"
#include "delay_micros.h"
#include "ds18b20.h"
#include "math.h"
#include "usart_ring.h"
#include "tm1637.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNT_REQUEST     40 // размер буфера ацп для adc_33_volt, adc_25_volt, adc_ntc, adc_vrefint (должно быть чётное число)
#define CH_ADC            4  // количество каналов (если тут не чётное число, то COUNT_REQUEST должен делится на него без остатка)
#define DIV_ADC           (COUNT_REQUEST / CH_ADC)

#define EC_COUNT_ADC      16 // количество опросов EC (должно быть чётное число)

#define BUF_UART          256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
char trans_str[BUF_UART] = {0,};

volatile uint8_t adc_flag_full = 0;
volatile uint8_t flag_ds18b20 = 1;

//////////////////////////////////////////////////
//uint32_t interval_uart = 500;

uint32_t interval_ds18 = 800; // миллисекунды, min 800                             [A800]
uint32_t interval_ec = 500;   // миллисекунды, min 500                             [B

uint32_t referenceVoltage = 3187;  // 2500mv (напряжение питания в попугаях)       [C

uint32_t ntcR1 = 9600;    // 10kΩ voltage divider resistor value                   [D
uint32_t ntcRo = 10000;   // 10kΩ R of Thermistor at 25 degree                     [E
uint32_t ntcTo = 25;      // 25 Temperature in Kelvin for 25 degree                [F
uint32_t ntcKoefB = 3950; // 3950 Beta value                                       [G

int32_t ecRo = 1000;      //  Ω Voltage divider resistor value 500Ω / 1000Ω        [K
int32_t ecKoefA = 54790;  //  Alfa value                                           [L
int32_t ecKoefB = 90;     //  Beta value                                           [M
int32_t ecKoefC = 34;     //  С-value                                              [N
int32_t ecKoefT = 0;      //  Ноль Koef Temperature                                [P

uint32_t ec_Hz = 9;       // Частота PWM (в микросек, min 9, max 65535)            [Q
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if DEBUG_USART1
void trans_to_usart1(char *buf)
{
  uint8_t len = strlen((char*)buf);

  for(uint8_t i = 0; i < len; i++)
  {
    while((USART1->SR & USART_SR_TXE) == 0){}
    USART1->DR = buf[i];
  }

  while((USART1->SR & USART_SR_TXE) == 0){}
  USART1->DR = '\n';
}
#endif

/////////// Таймер измерения времени для скорости & rpm ///////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2) // ds18b20
  {
    flag_ds18b20 = 2;
  }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    adc_flag_full = 1;
    HAL_ADC_Stop(&hadc1);
  }
}


int32_t rawNtcToTemperature(uint16_t Vout)
{
  uint32_t Rt;
  double kelvin, celsius;

  uint32_t R1 = ntcR1; // 9600  | voltage divider resistor value
  uint32_t Ro = ntcRo; // 10000 | R of Thermistor at 25 degree
  uint32_t To = ntcTo; // 25 | Temperature in Kelvin for 25 degree
  uint32_t koefB = ntcKoefB; // 3950  | Beta value
  uint32_t mv = (referenceVoltage - Vout);

  Rt = R1 * mv / (referenceVoltage - mv);
  kelvin = (double)Ro / (double)Rt;              // R/Ro
  kelvin = log(kelvin);                          // ln(R/Ro)
  kelvin = (1 / (double)koefB) * kelvin;         // 1/B * ln(R/Ro)
  kelvin = (1 / ((double)To + 273.15)) + kelvin; // 1/To + 1/B * ln(R/Ro)
  kelvin = 1 / kelvin; // 1/( 1/To + 1/B * ln(R/Ro) )​

  celsius = kelvin - 273.15; // Convert Kelvin to Celsius.
  return round(celsius * 1000);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Enable power.
  HAL_GPIO_WritePin(EC_PWR_GPIO_Port, EC_PWR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NTC_PWR_GPIO_Port, NTC_PWR_Pin, GPIO_PIN_RESET);
  ///////////////////// READ FLASH ////////////////////////
  uint32_t data[13] = {0,};

  Read_flash_data(ADDR_FLASH_PAGE_31, data);

  #if DEBUG_USART1 // дефаин в файле main.h
  snprintf(trans_str, BUF_UART, "A %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",\
      data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12]);
  trans_to_usart1(trans_str);
  #endif

  // @todo: проверка целостности вмсето пустой флешки
  if(1 || data[0] == 0xFFFFFFFF) // если флеш пустая, записываем туда дефолтные значения
  {
    data[0] = interval_ds18;    // 800
    data[1] = interval_ec;      // 500
    data[2] = referenceVoltage; // 2500
    data[3] = ntcR1;            // 9600
    data[4] = ntcRo;            // 10000
    data[5] = ntcTo;            // 25
    data[6] = ntcKoefB;         // 3950
    data[7] = ecRo;             // 1000
    data[8] = ecKoefA;          // 54790
    data[9] = ecKoefB;          // 90
    data[10] = ecKoefC;         // 34
    data[11] = ecKoefT;         // 0
    data[12] = ec_Hz;           // 9

    Write_flash_data(ADDR_FLASH_PAGE_31, data);
  }
  else
  {
    interval_ds18 = data[0];
    interval_ec = data[1];
    referenceVoltage = data[2];
    ntcR1 = data[3];
    ntcRo = data[4];
    ntcTo = data[5];
    ntcKoefB = data[6];
    ecRo = data[7];
    ecKoefA = data[8];
    ecKoefB = data[9];
    ecKoefC = data[10];
    ecKoefT = data[11];
    ec_Hz = data[12];
  }

  #if DEBUG_USART1
  snprintf(trans_str, BUF_UART, "B %lu %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %lu",\
      interval_ds18, interval_ec, referenceVoltage, ntcR1, ntcRo, ntcTo, ntcKoefB, ecRo, ecKoefA, ecKoefB, ecKoefC, ecKoefT, ec_Hz);
  trans_to_usart1(trans_str);
  #endif


  /////////////////////// DWT /////////////////////////////
  DWT_Init();

  /////////////////////// tm1637/////////////////////////////
  set_brightness(7); // 0 - 7 яркость
  clearDisplay();    // очистка дисплея
  int8_t data_to_disp[4] = {0,};

  ///////////////////// DS18B20 ///////////////////////////
  setResolution(DS18B20_Resolution_12_bit);

  ///////////////////// ADC1 /////////////////////////////
  HAL_ADCEx_Calibration_Start(&hadc1); // калибровка АЦП1
  HAL_ADCEx_Calibration_Start(&hadc2); // калибровка АЦП2
  uint16_t adc_buf[COUNT_REQUEST] = {0,}; // буфер для adc_33_volt, adc_25_volt, adc_ntc, adc_vrefint, по 10 значений каждого (#define COUNT_REQUEST)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, COUNT_REQUEST); // опрашивает adc_33_volt, adc_25_volt, adc_ntc, adc_vrefint

  /////// переменные АЦП1 ///////
  uint32_t tmp_adc_33_volt = 0;
  uint32_t tmp_adc_25_volt = 0;
  uint32_t tmp_adc_ntc = 0;
  uint32_t tmp_adc_vrefint = 0;

  uint16_t adc_33_volt = 0;
  uint16_t adc_25_volt = 0;
  uint16_t adc_ntc = 0;
  uint16_t adc_vrefint = 0;


  ///////////////////// ADC2 и PWM ////////////////////////
  TIM1->ARR = ec_Hz;
  TIM1->CCR1 = TIM1->ARR;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = ~TIM_SR_UIF;

  TIM3->ARR = TIM1->ARR;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = ~TIM_SR_UIF;

  HAL_ADC_Start(&hadc2);      // включаем АЦП2 (преобразование запустится по триггеру)
  HAL_TIM_Base_Start(&htim3); // подчинённый таймер, запускается от тим1 и работает с ним синхронно "толкая" АЦП2 сразу после переключения фронтов тим1
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1); // запускает PWM, и посылает триггер для запуска тим3
  HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);

  /////// переменные АЦП2 ///////
  uint32_t tmp_adc_ec_positive = 0; // собирает пачку позитивных значений ЕС (по умолчанию 10 - #define EC_COUNT_ADC)
  uint16_t adc_ec_positive = 0; // среднее позитив = (tmp_adc_ec_positive / EC_COUNT_ADC)

  uint32_t tmp_adc_ec_negative = 0; // собирает пачку негативных значений ЕС (по умолчанию 10 - #define EC_COUNT_ADC)
  uint16_t adc_ec_negative = 0; // среднее негатив = (tmp_adc_ec_negative / EC_COUNT_ADC)
  // Посчитанные значения EC.
  int16_t adc_ec_delta = 0;
  uint16_t adc_ec_raw = 0;
  uint16_t ec = 0;

  ///////////////// Готовые температуры ////////////////////
  int32_t temp_ntc = 0;
  int16_t temp_ds18 = 0;


  ///////////////////////// tick ///////////////////////////
  uint32_t tim_ds18 = 0;
  //uint32_t tim_uart = 0;
  uint32_t tim_ec = 0;


  //////////////// Вкл прерывания UART /////////////////////
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // приём команд происходит в файле stm32f1xx_it.c

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(adc_flag_full == 1) // ждём флаг, который установится в прерывании АЦП1 по окончанию работы ДМА
    {
        adc_flag_full = 0; // обнуляем флаг

        // обнуляем временные переменные
        tmp_adc_33_volt = 0;
        tmp_adc_25_volt = 0;
        tmp_adc_ntc = 0;
        tmp_adc_vrefint = 0;

        // разбираем данные с АЦП1 полученые через ДМА
      for(uint16_t i = 0; i < (COUNT_REQUEST / CH_ADC); i++)
      {
        tmp_adc_33_volt += adc_buf[CH_ADC * i + 0];
        tmp_adc_25_volt += adc_buf[CH_ADC * i + 1];
        tmp_adc_ntc += adc_buf[CH_ADC * i + 2];
        tmp_adc_vrefint += adc_buf[CH_ADC * i + 3];
      }

      // усредняем
      adc_33_volt = (tmp_adc_33_volt / DIV_ADC);
      adc_25_volt = (tmp_adc_25_volt / DIV_ADC);
      adc_ntc = (tmp_adc_ntc / DIV_ADC);
      adc_vrefint = (tmp_adc_vrefint / DIV_ADC);

      temp_ntc = rawNtcToTemperature(adc_ntc); // получаем температуру с NTC

      //sprintf(trans_str, "A %d %d %d %d %ld", adc_33_volt, adc_25_volt, adc_ntc, adc_vrefint, ntc);
      //trans_to_usart1(trans_str);
      //HAL_Delay(500);

      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, COUNT_REQUEST); // перезапускаем АЦП1
    }


    //////////////////////////////// DS18B20 ////////////////////////////////////
    if((uwTick - tim_ds18) > interval_ds18)
    {
      if(flag_ds18b20 == 1)
      {
        flag_ds18b20 = 0;

        DB18B20_PORT->BSRR = (uint32_t)DB18B20_PIN << 16u;
        delay_us(DELAY_RESET);
        DB18B20_PORT->BSRR = DB18B20_PIN;
        delay_us(DELAY_RESET);

        writeByte(SKIP_ROM);
        writeByte(CONVERT_T);

        TIM2->DIER |= TIM_IT_UPDATE;
        TIM2->CR1 |= TIM_CR1_CEN;
      }
      else if(flag_ds18b20 == 2) // когда таймер отсчитает 750мс, он установит flag == 2
      {
        DB18B20_PORT->BSRR = (uint32_t)DB18B20_PIN << 16u;
        delay_us(DELAY_RESET);
        DB18B20_PORT->BSRR = DB18B20_PIN;
        delay_us(DELAY_RESET);

        writeByte(SKIP_ROM);
        writeByte(READ_SCRATCHPAD);

        temp_ds18 = 0;

        for(uint8_t i = 0; i < 16; i++) temp_ds18 += (int16_t)readBit() << i;

        temp_ds18 = (1000 * temp_ds18 / 16);
        //float t = temp_ds18 / 16.0;

        flag_ds18b20 = 1; // запускаем новое измерение.
      }

      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // просто мигалка, можно вставить куда захочется
      tim_ds18 = uwTick;
    }


    //////////////////////////// EC ///////////////////////////////
    if((uwTick - tim_ec) > interval_ec)
    {
      tmp_adc_ec_positive = 0;
      tmp_adc_ec_negative = 0;

      // данные с АЦП2 (ЕС) читаются постоянно, преобразование запускается по триггеру от таймера №3.
      // блогодаря тому, что АЦП2 работает постоянно, нам не нужно тратить время на его запуск.
      ADC2->DR; // читаем регистр чтоб обнулить его

      for(uint8_t i = 0; i < EC_COUNT_ADC; i++)
      {
        while(!(ADC2->SR & ADC_SR_EOC)); // ждём следуещего преобразования

        if((GPIOA->IDR & GPIO_PIN_8)) // проверяем пин РА8 (в каком он состоянии - позитивном или негативном)
        {
          tmp_adc_ec_positive += ADC2->DR;
          //trans_to_usart1("P");
        }
        else
        {
          tmp_adc_ec_negative += ADC2->DR;
          //trans_to_usart1("N");
        }
      }

      adc_ec_positive = (tmp_adc_ec_positive / (EC_COUNT_ADC / 2));
      adc_ec_negative = adc_25_volt - (tmp_adc_ec_negative / (EC_COUNT_ADC / 2));
      adc_ec_delta = adc_ec_positive - adc_ec_negative;
      adc_ec_raw = (adc_ec_positive + adc_ec_negative);
      ec = ecKoefA / (ecRo - ecKoefB) - ecKoefC;

      uint16_t vdd = 1210 * 4095 / adc_vrefint; // напряжение питания на основе Vrefint (не точно)

      ////////////////////////// Выводим все данные в УАРТ /////////////////////////////
      uint16_t len = snprintf(
          trans_str, BUF_UART, "%d %d %d %d %ld %d %d %d %d %d\n", adc_33_volt,
          adc_25_volt, adc_ntc, adc_vrefint, temp_ntc, temp_ds18,
          adc_ec_positive, adc_ec_negative, adc_ec_delta, vdd);

      while ((HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY))
        ;
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)trans_str, len);

      // if(adc_ec_positive < 2900 || adc_ec_positive > 3100) while(1){}
      // if(adc_ec_negative < 2900 || adc_ec_negative > 3100) while(1){}

      //////////////// tm1637 ///////////////
      if (vdd >= 1000) {
        data_to_disp[0] = (vdd / 1000 % 10);
        data_to_disp[1] = (vdd / 100 % 10);
        data_to_disp[2] = (vdd / 10 % 10);
        data_to_disp[3] = vdd % 10;
      }
      else if(vdd >= 100)
      {
        data_to_disp[0] = 0x7f; // ничего не выводит на сегмент
        data_to_disp[1] = (vdd / 100 % 10);
        data_to_disp[2] = (vdd / 10 % 10);
        data_to_disp[3] = vdd % 10;
      }
      else if(vdd >= 10)
      {
        data_to_disp[0] = 0x7f;
        data_to_disp[1] = 0x7f;
        data_to_disp[2] = (vdd / 10 % 10);
        data_to_disp[3] = vdd % 10;
      }
      else if(vdd >= 0)
      {
        data_to_disp[0] = 0x7f;
        data_to_disp[1] = 0x7f;
        data_to_disp[2] = 0x7f;
        data_to_disp[3] = vdd % 10;
      }
      //// Для отрицательных значений -1 -99 ////
      else if(vdd <= -10 && vdd >= -99)
      {
        int8_t tmp = vdd;
        tmp *= -1;

        data_to_disp[0] = 0x7f;
        data_to_disp[1] = 18; // выводит на сегмент знак "минус"
        data_to_disp[2] = (tmp / 10 % 10);
        data_to_disp[3] = tmp % 10;
      }
      else if(vdd <= -1 && vdd >= -9)
      {
        int8_t tmp = vdd;
        tmp *= -1;

        data_to_disp[0] = 0x7f;
        data_to_disp[1] = 0x7f;
        data_to_disp[2] = 18;
        data_to_disp[3] = tmp % 10;
      }

      display_mass(data_to_disp);

      //snprintf(trans_str, BUF_UART, "Vref: %d Vdd: %d mV\n", adc_vrefint, vdd);
      //trans_to_usart1(trans_str);

      tim_ec = uwTick;
    }


    //////////////////////////// UART //////////////////////////////
    /*if((uwTick - tim_uart) > interval_uart) // кидаем данные у УАРТ с указанным интервалом
    {
      uint16_t len = snprintf(trans_str, BUF_UART, "%d %d %d %d %ld %d %d %d\n", \
          adc_33_volt, adc_25_volt, adc_ntc, adc_vrefint, temp_ntc, temp_ds18, adc_ec_positive, adc_ec_negative);
      trans_to_usart_data(trans_str, len);

      //snprintf(trans_str, 128, "Len %d", len);
      //trans_to_usart1(trans_str);
      tim_uart = uwTick;
    }*/


    //////////////////////////// UART Приём //////////////////////////////
    if(uart_available()) // есть ли что-то в приёмном буфере, тогда читаем
    {
      char str[16] = {0,};
      uint8_t i = 0;
      uint8_t flag = 0;

      while(uart_available())
      {
        str[i] = uart_read();

        if(str[i] == ']')
        {
          str[i] = '\0';
          flag = 1;
          break;
        }
        else if(i == 15) break;

        i++;
        delay_us(150);
      }


      if(flag && str[0] == '[')
      {
      switch(str[1])
      {
        case 'A':
          interval_ds18 = strtoul(&str[2], NULL, 0);
          if(interval_ds18 < 800) interval_ds18 = 800;
        break;

        case 'B':
          interval_ec = strtoul(&str[2], NULL, 0); // интервал вывода в УАРТ и опрос ЕС
          if(interval_ec < 500) interval_ec = 500;
        break;

        case 'C':
          referenceVoltage = strtoul(&str[2], NULL, 0);
        break;

        case 'D':
          ntcR1 = strtoul(&str[2], NULL, 0);
        break;

        case 'E':
          ntcRo = strtoul(&str[2], NULL, 0);
        break;

        case 'F':
          ntcTo = strtoul(&str[2], NULL, 0);
        break;

        case 'G':
          ntcKoefB = strtoul(&str[2], NULL, 0);
        break;

        case 'K':
          ecRo = strtoul(&str[2], NULL, 0);
        break;

        case 'L':
          ecKoefA = strtoul(&str[2], NULL, 0);
        break;

        case 'M':
          ecKoefB = strtoul(&str[2], NULL, 0);
        break;

        case 'N':
          ecKoefC = strtoul(&str[2], NULL, 0);
        break;

        case 'P':
          ecKoefT = strtoul(&str[2], NULL, 0);
        break;

        case 'Q':
          ec_Hz = strtoul(&str[2], NULL, 0);
          if(ec_Hz < 9) ec_Hz = 9;
        break;

        default:
        break;
      }

      data[0] = interval_ds18;
      data[1] = interval_ec;
      data[2] = referenceVoltage;
      data[3] = ntcR1;
      data[4] = ntcRo;
      data[5] = ntcTo;
      data[6] = ntcKoefB;
      data[7] = ecRo;
      data[8] = ecKoefA;
      data[9] = ecKoefB;
      data[10] = ecKoefC;
      data[11] = ecKoefT;
      data[12] = ec_Hz;

      Write_flash_data(ADDR_FLASH_PAGE_31, data); // сохраняем данные (сохранение в одну страницу флеша, поэтому все значения перезаписываются)

      if(str[1] == 'Q') NVIC_SystemReset(); // если изменилась частота, тогда ресетим камень
      }
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 20;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7600;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_Pin|DIO_Pin|EC_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NTC_PWR_GPIO_Port, NTC_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin DIO_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EC_PWR_Pin NTC_PWR_Pin */
  GPIO_InitStruct.Pin = EC_PWR_Pin|NTC_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

