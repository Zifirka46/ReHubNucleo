#pragma once

#include <HX711.h>
#include <DFilters.h>

// Класс для упрощенного работа с АЦП HX711
class DHX711 : private HX711
{
public:
    DHX711(int pin_dt, int pin_sck, float scale) 
    {
        // Запускаем модуль, настраиваем и тарируем

        begin(pin_dt, pin_sck);
        set_gain(128);
        set_scale(scale);
        tare(10);
    }

    // Возвращаем отфильтрованные значения с АЦП
    float getForce()
    {
        if (is_ready())
            return get_units(1);
    }
};