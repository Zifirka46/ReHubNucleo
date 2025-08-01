#pragma once

//Хуй пойми что за сервоприводы
#define SERVO_RIGHT_STEP PA0
#define SERVO_RIGHT_DIR PF5
#define SERVO_RIGTH_EN PE13

#define SERVO_LEFT_STEP PC6
#define SERVO_LEFT_DIR PB12
#define SERVO_LEFT_EN PA4 

// Распиновка звеньев

//все что подписано как левое в комментариях это правые, потому что армянская диаспора
//ебанула левую ногу, как правую в распиновке и перед показом меняли 

// Подключение драйвера левого бедра
#define PIN_PWM_HIP_RIGHT PB9
#define PIN_DIR_HIP_RIGHT PF10
#define PIN_ENB_HIP_RIGHT PE7

// Подключение драйвера левой голени
#define PIN_PWM_KNEE_RIGHT PB11
#define PIN_DIR_KNEE_RIGHT PF4
#define PIN_ENB_KNEE_RIGHT PE8

// Подключение драйвера левой стопы
#define PIN_PWM_FOOT_RIGHT PD12
#define PIN_DIR_FOOT_RIGHT PB2
#define PIN_ENB_FOOT_RIGHT PB1

// Подключение драйвера правого бедра
#define PIN_PWM_HIP_LEFT PE10 
#define PIN_DIR_HIP_LEFT PG5
#define PIN_ENB_HIP_LEFT PG4

// Подключение драйвера правой голени
#define PIN_PWM_KNEE_LEFT PE12
#define PIN_DIR_KNEE_LEFT PG8
#define PIN_ENB_KNEE_LEFT PD10

// Подключение драйвера правой стопы
#define PIN_PWM_FOOT_LEFT PE14
#define PIN_DIR_FOOT_LEFT PG14
#define PIN_ENB_FOOT_LEFT PE0

// Распиновка регулировок

//Кнопки регулировок
#define BUTTON_HIP_LEFT_UP 0x01
#define BUTTON_HIP_LEFT_DOWN 0x02

#define BUTTON_KNEE_LEFT_UP 0x03
#define BUTTON_KNEE_LEFT_DOWN 0x04

#define BUTTON_FOOT_LEFT_UP 0x05
#define BUTTON_FOOT_LEFT_DOWN 0x06

#define BUTTON_HIP_RIGHT_UP 0x07
#define BUTTON_HIP_RIGHT_DOWN 0x08

#define BUTTON_KNEE_RIGHT_UP 0x09
#define BUTTON_KNEE_RIGHT_DOWN 0x10

#define BUTTON_FOOT_RIGHT_UP 0x11
#define BUTTON_FOOT_RIGHT_DOWN 0x12

#define BUTTON_WIDTH_UP 0x13
#define BUTTON_WIDTH_DOWN 0x14

#define BUTTON_HEIGHT_UP 0x15
#define BUTTON_HEIGHT_DOWN 0x16

// Подключение регулировок левго бедра
#define PIN_DRIVER_HIP_RIGHT_INA PE9
#define PIN_DRIVER_HIP_RIGHT_INB PE11
#define PIN_DRIVER_HIP_RIGHT_PWM PC8

// Подключение регулировок левой голени
#define PIN_DRIVER_KNEE_RIGHT_INA PD15
#define PIN_DRIVER_KNEE_RIGHT_INB PE4
#define PIN_DRIVER_KNEE_RIGHT_PWM PC9

// Подключение регулировок левой стопы
#define PIN_DRIVER_FOOT_RIGHT_PWM PB8
#define PIN_DRIVER_FOOT_RIGHT_DIR PD14

// Подключение регулировок правого бедра
#define PIN_DRIVER_HIP_LEFT_INA PG3
#define PIN_DRIVER_HIP_LEFT_INB PF12
#define PIN_DRIVER_HIP_LEFT_PWM PB5

// Подключение регулировок правой голени
#define PIN_DRIVER_KNEE_LEFT_INA PF3
#define PIN_DRIVER_KNEE_LEFT_INB PF13
#define PIN_DRIVER_KNEE_LEFT_PWM PB10

// Подключение регулировок правой стопы
#define PIN_DRIVER_FOOT_LEFT_PWM PD13
#define PIN_DRIVER_FOOT_LEFT_DIR PF11

//PCA9535PW
#define SCL_EXPANSION PF14
#define SDA_EXPANSION PF15

//Тензодатчики

//Левая нога
#define PIN_TENZO_HIP_RIGHT_SCK PF6
#define PIN_TENZO_HIP_RIGHT_DT PF7

#define PIN_TENZO_KNEE_RIGHT_SCK PC10
#define PIN_TENZO_KNEE_RIGHT_DT PC12

#define PIN_TENZO_FOOT_RIGHT_SCK1 PD4
#define PIN_TENZO_FOOT_RIGHT_DT1 PD5 

#define PIN_TENZO_FOOT_RIGHT_SCK2 PC3
#define PIN_TENZO_FOOT_RIGHT_DT2 PD3

#define PIN_TENZO_FOOT_RIGHT_SCK3 PC2
#define PIN_TENZO_FOOT_RIGHT_DT3 PC0

//Правая нога
#define PIN_TENZO_HIP_LEFT_SCK PE3
#define PIN_TENZO_HIP_LEFT_DT PG2

#define PIN_TENZO_KNEE_LEFT_SCK PD6 
#define PIN_TENZO_KNEE_LEFT_DT PD7 

#define PIN_TENZO_FOOT_LEFT_SCK1 PE1
#define PIN_TENZO_FOOT_LEFT_DT1 PG9 

#define PIN_TENZO_FOOT_LEFT_SCK2 PD0
#define PIN_TENZO_FOOT_LEFT_DT2 PG0

#define PIN_TENZO_FOOT_LEFT_SCK3 PE2
#define PIN_TENZO_FOOT_LEFT_DT3 PD1

//Датчики угла поворота

//Энкодеры звеньев
#define PIN_BURNS_CLOCK PG10
#define PIN_BURNS_DATA PG15

//Левая нога
#define PIN_CS_HIP_RIGHT PF8
#define PIN_CS_KNEE_RIGHT PF2
#define PIN_CS_FOOT_RIGHT PE5

//Правая нога
#define PIN_CS_HIP_LEFT PE6
#define PIN_CS_KNEE_LEFT PG1
#define PIN_CS_FOOT_LEFT PF9


//Оптопары(энкодеры регулировки)
#define PIN_OPTO_ENCODER_HIP_RIGHT PC7
#define PIN_OPTO_ENCODER_KNEE_RIGHT PB6

#define PIN_OPTO_ENCODER_HIP_LEFT PD11
#define PIN_OPTO_ENCODER_KNEE_LEFT PA3

//Потенциометры стоп
#define PIN_ANGLE_FOOT_RIGHT PA5
#define PIN_ANGLE_FOOT_LEFT PA6

//Концевики (в распиновки они просто пронумерованы от 1 до 4 так что может не совпасть с реальностью)
//Были подписаны только какие 2 левые и какие 2 правые
#define PIN_SWITCH_HIP_RIGHT PD2
#define PIN_SWITCH_KNEE_RIGHT PC11

#define PIN_SWITCH_HIP_LEFT PH_2
#define PIN_SWITCH_KNEE_LEFT PG12

//Реле
#define PIN_RELAY_ADJ PB4 //Регулеровки
#define PIN_RELAY_MAX PA15 //Звенья(максоны)

// Настройка пинов
void setupPins()
{

    pinMode(PIN_RELAY_ADJ, OUTPUT);
    pinMode(PIN_RELAY_MAX, OUTPUT);

    // Запуск реле
    digitalWrite(PIN_RELAY_ADJ, HIGH);
    digitalWrite(PIN_RELAY_MAX, HIGH);
}