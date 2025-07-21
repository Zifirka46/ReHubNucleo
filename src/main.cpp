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
    setupPins();
    analogReadResolution(10);

    Serial.begin(9600);
    setupEncod();

    setEncodScales();
}

void loop(){
  updateEncod();
  mainControl();
  sendingData();
  // Serial.println("penis");
}