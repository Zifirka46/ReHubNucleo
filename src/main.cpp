#include <Arduino.h>
#include<Wire.h>

#include "GyverPID.h"

#include "DFilters.h"
#include "DButton.h"
#include "DPolynomial.h"
#include "DFilters.h"
#include "DTimer.h"
#include "DOptoEncoder.h"

#include "pinmap.h"
#include "variables.h"
#include "sensors.h"
#include "drive.h"
#include "controlling.h"
#include "communication.h"

void setup(){
    analogReadResolution(10);

    Serial.begin(9600);
    setupEncod();

    setEncodScales();

    pinMode(PIN_RELAY_ADJ, OUTPUT);
    pinMode(PIN_RELAY_MAX, OUTPUT);

    digitalWrite(PIN_RELAY_ADJ, HIGH);
    digitalWrite(PIN_RELAY_MAX, HIGH);
}

void loop(){
  updateEncod();
  mainControl();
  sendingData();
}