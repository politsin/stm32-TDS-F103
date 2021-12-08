# stm32-TDS-F103

## Output data:
- ec (посчитанное значение EC)
- R (Сопротивление)
- adc_ec_raw (EC в попугаях)
- adc_ec_delta (разница между измерениями)
- adc_ec_positive (в штуках)
- adc_ec_negative (в штуках)
- adc_33_volt (в штуках)
- adc_25_volt (в штуках)
- adc_ntc (в штуках)
- adc_vrefint (в штуках)
- temp_ntc
- temp_ds18
- vdd (напряжение питания в миливольтах)

## RoadMAP
- [ ] 3 кнопочки настроек или енкодер
- [ ] i2c
- [ ] stm32-G0

## TODO:
- [ ] Проверка целостности конфигурации.
  - `if(data[0] == 0xFFFFFFFF) // если флеш пустая`
- [ ] Конфигурирование UART по 3 точкам
- [ ] FIR для всех АЦП

## HAL
| Функция     | Описание       |     V2    |     V3    |
| ----------- | -------------- | --------- | --------- |
| LED         | BluePill LED   | PC13      | PC13      |
| NTC_PWR     | Power          | PB8       | PB8 [!]   |
| I2C1        | Not used DA/CL | PB7/PB8   | _         |
| EC [1|2]    | TIM1 CH1/CH1N  | PB4/PB4   | PA8/PB13  |
| EC_PWR      | Power          | PB3       | PB3 [!]   |
| SWO         | CLC/DIO        | PA14/PA13 | PA14/PA13 |
| DS18B20     |                | PA11      | PA11      |
| USART       | RX/TX          | PA10/PA9  | PA10/PA9  |
| tm1637      | DIO/CLK        | PB10/PB11 | PB10/PB11 |
| ADC 3300    | ADC1_33        | _         | PA0 [!]   |
| ADC 2500    | ADC1_25        | PA7       | PA1 [!]   |
| ADC NTC     | ADC1_NTC       | PA6       | PA2 [!]   |
| ADC EC      | ADC2_EC        | PA5       | PA5       |
