#pragma once

#include <math.h>

// Enum состояний системы (верхний уровень КА)
enum SYSTEM_STATES
{
    SS_MAIN_MENU,     // Ожидание команды запуска от ППО
    SS_ANGLE_DESIRED_LINK, // Полуавтомат. желаемые углы
    SS_START_POSING_LINK,  // Перевод звеньев в положение по умолчанию
    SS_EXE_DEMO_LINK,      // Упражнение, перемещение левого бедра
    SS_SEDENTARY_LINK,     // Сидячее положение
    SS_WALK_1_LINK,        // Походка
    SS_WALK_2_LINK,
    SS_HMN_GAIT_LINK,
    SS_POSITION_DESIRED_ADJ,
    SS_ZEROING_ADJ,     // Обнуление энкодеров
    SS_DEMO_ADJ
};

SYSTEM_STATES system_state = SS_MAIN_MENU;
int system_state_last = -1;

// Если сменилось состояние системы, вернет единицу на 1 цикл
bool isSSChange()
{
    if (system_state != system_state_last)
    {
        system_state_last = system_state;
        return true;
    }
    else
        return false;
}

float deg2rad(float angle)
{
    float new_angle = angle * (PI / 180);
    return new_angle;
}

float rad2deg(float angle)
{
    float new_angle = angle * (180 / PI);
    return new_angle;
}

float lyambda = 0.99;          // Коэффициент показывающий насколько нога будет прямая
float angle_des = deg2rad(-1); // Желаемый угол
float lengthHip = 0.39;         // Длина бедра
float lengthKnee = 0.39;        // Длина голени
float ls = 0.2;
float hs = 0.1;
const int hmn_gt_time = 1; // Пар-тр для 3 походки(время шага)
float hmn_gt_scaling = 0.88;
float hmn_gt_scaling_z = 0.89;

float x_ankle_start_des; // Желаемая точка по иксу
float y_ankle_start_des; // Желаемая точка по игрику
float x_ankle_start;
float y_ankle_start;
float step_x;
float step_y;
float _x_targ;
float _y_targ;
float q1;
float q21;
float q1_des;
float q21_des;
double x_offset = -0.3598;
double y_offset = -0.7919;
float dt = 0.0;
float t;
int k = 0;
const int hmn_gt_size = hmn_gt_time * 83;
unsigned long last_time_hmn_gait;

bool running = 0;
float start_time = 0.0;
float current_time = 0.0;

void getApproxHmnGt(float &x_aprox, float &y_aprox, float x)
{

    static float w, a0, a1, b1, a2, b2, a3, b3, a4, b4, a5, b5, a6, b6, a7, b7, a8, b8;

    a0 =      0.3179 * hmn_gt_scaling; 
    a1 =     -0.2687 * hmn_gt_scaling;
    b1 =      0.1288 * hmn_gt_scaling;
    a2 =    -0.05454* hmn_gt_scaling;
    b2 =    -0.05705* hmn_gt_scaling;
    a3 =     0.01883* hmn_gt_scaling;
    b3 =   -0.008654 * hmn_gt_scaling;
    a4 =  -0.0002569* hmn_gt_scaling;
    b4 =     0.01156* hmn_gt_scaling;
    w =        4.539 * (1.34f / hmn_gt_time);

    x_aprox = a0 + a1*cos(x*w) + b1*sin(x*w) + a2*cos(2*x*w) + b2*sin(2*x*w) + 
              a3*cos(3*x*w) + b3*sin(3*x*w) + a4*cos(4*x*w) + b4*sin(4*x*w);

    a0 =     0.09677 * hmn_gt_scaling_z;
    a1 =     0.09289 * hmn_gt_scaling_z;
    b1 =    -0.02086 * hmn_gt_scaling_z;
    a2 =     0.01216 * hmn_gt_scaling_z;
    b2 =    0.008177 * hmn_gt_scaling_z;
    a3 =    -0.01752 * hmn_gt_scaling_z;
    b3 =     0.02908 * hmn_gt_scaling_z;
    a4 =    -0.01223 * hmn_gt_scaling_z;
    b4 =     0.02525 * hmn_gt_scaling_z;
    a5 =   -0.005644 * hmn_gt_scaling_z;
    b5 =    0.009501 * hmn_gt_scaling_z;
    a6 =   -0.002904 * hmn_gt_scaling_z;
    b6 =   0.0009237 * hmn_gt_scaling_z;
    w =        3.624 * (1.34f / hmn_gt_time);
    
    y_aprox =   a0 + a1*cos(x*w) + b1*sin(x*w) + a2*cos(2*x*w) + b2*sin(2*x*w) + 
                a3*cos(3*x*w) + b3*sin(3*x*w) + a4*cos(4*x*w) + b4*sin(4*x*w) + a5*cos(5*x*w) + 
                b5*sin(5*x*w) + a6*cos(6*x*w) + b6*sin(6*x*w);
}

//.......................................................................................................................
// регулировки

  // Состояния обнуления регулировок с ДУП - энкодером (второй уровень КА)
  enum ZEROING_STATES{
    ZS_HIP_LEFT,   // Поиск нуля ДУП левого бедра
    ZS_KNEE_LEFT,  // Поиск нуля ДУП левого колена
    ZS_HIP_RIGHT,  // Поиск нуля ДУП правого бедра
    ZS_KNEE_RIGHT, // Поиск нуля ДУП правого колена
  };
  
  // Текущее состояние обнуления
  ZEROING_STATES zeroing_state = ZS_HIP_LEFT;
  int zeroing_state_last = -1;
  
  // Если сменилось состояние обнуление, вернет единицу на 1 цикл
  bool isZSChange(){
    if(zeroing_state != zeroing_state_last){
      zeroing_state_last = zeroing_state;
      return true;
    }
    else return false;  
  }
  
  // Флаг, поднимаемый, если прошла процедура инициализации
  bool is_zeroed_flag = false; 
  
  enum DEMO_STATES{
    DS_HIP_LEFT,
    DS_KNEE_LEFT,
    DS_FOOT_LEFT,
    DS_HIP_RIGHT,
    DS_KNEE_RIGHT,
    DS_FOOT_RIGHT,
    DS_HIP_LEFT_BACK,
    DS_KNEE_LEFT_BACK,
    DS_HIP_RIGHT_BACK,
    DS_KNEE_RIGHT_BACK
  };
  
  DEMO_STATES demo_state = DS_HIP_RIGHT;
  int demo_state_last = -1;
  
  bool isDSChange(){
    if(demo_state != demo_state_last){
      demo_state_last = demo_state;
      return true;
    }
    else return false;  
  }