#include "main.h"

Adafruit_SH1106G OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

TaskHandle_t MotorController = NULL;
TaskHandle_t OLEDandPeripheral = NULL;
SemaphoreHandle_t MutualExclusion;

const uint8_t numberofsensor = 8;

uint8_t SensorInput[numberofsensor] = {39, 36, 35, 34, 33, 32, 27, 26};
uint8_t RightMotorModulation, LeftMotorModulation, MotorBaseModulation, BaseSpeedPercentage;

uint16_t SensorReading[numberofsensor] = {0};
uint16_t MotorLeftSpeed, MotorRightSpeed, MaxModulation;

int8_t MotorBias = 0;

int16_t barheight[numberofsensor] = {0};

int sensorweight[numberofsensor] = {343, 194, 144, 1, 1, -109, -196, -384};

// ======================================================
// BUTTON
// ======================================================


// 0 = KANAN
// 1 = KIRI
// 2 = MAJU
// 3 = MUNDUR
uint8_t selectedOption = 0;

void setup()
{
  OLED.begin(I2C_ADDRESS, OLED_RESET);
  OLED.clearDisplay();
  MaxModulation = 0xFF;

  // Button
  pinMode(BUTTON_NEXT, INPUT_PULLUP);
  pinMode(BUTTON_BACK, INPUT_PULLUP);
  ledcSetup(LEFT_CHANNEL, 1000, 16);
  ledcSetup(RIGHT_CHANNEL, 1000, 16);
  ledcAttachPin(LEFTMOTOR, LEFT_CHANNEL);
  ledcAttachPin(RIGHTMOTOR, RIGHT_CHANNEL);

  xTaskCreatePinnedToCore(
    CoreZero,
    "ProccessingVariable",
    4096,
    NULL,
    0,
    &MotorController,
    0
  );

  xTaskCreatePinnedToCore(
    CoreOne,
    "ProccessingVariable",
    4096,
    NULL,
    0,
    &OLEDandPeripheral,
    1
  );
}

void loop()
{
}

float scanline() {
  int32_t totalsensorvalue = 0;
  int32_t totalweight = 0;

  for(uint8_t point = 0; point < numberofsensor; point++) {
    SensorReading[point] = digitalRead(SensorInput[point]);
    totalsensorvalue += SensorReading[point];
    totalweight += (SensorReading[point] * sensorweight[point]);
  }

  if(totalsensorvalue == 0) {
    return NAN;
  }
  else {
    return (float)totalweight / totalsensorvalue;
  }
}


// ======================================================
// MOTOR TASK - CORE 0
// ======================================================
void CoreZero(void *pvParameters){
  while(1) {
    float weight = scanline();
    uint16_t MotorBaseSpeed = (BaseSpeedPercentage * MOTORMAXSPEED) / 100.0;
    MotorLeftSpeed = MotorBaseSpeed + weight;
    MotorRightSpeed = MotorBaseSpeed - weight;
    LeftMotorModulation = (MotorLeftSpeed / (float)MOTORMAXSPEED) * MaxModulation;
    RightMotorModulation = (MotorRightSpeed / (float)MOTORMAXSPEED) * MaxModulation;
    if(IsRunning) {
      ledcWrite(
        LEFT_CHANNEL,
        LeftMotorModulation
      );
      ledcWrite(
        RIGHT_CHANNEL,
        RightMotorModulation
      );
    }
    else {
      ledcWrite(LEFT_CHANNEL, 0);
      ledcWrite(RIGHT_CHANNEL, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// ======================================================
// OLED DRAW
// ======================================================

void drawOLED() {
  OLED.clearDisplay();

  // --------------------------------------------------
  // BEL0K
  // --------------------------------------------------
  OLED.setTextSize(1);
  OLED.setCursor(60, 19);
  OLED.setTextColor(SH110X_WHITE);
  OLED.print("Belok :");


  // --------------------------------------------------
  // KECEPATAN
  // --------------------------------------------------
  OLED.setCursor(45, 27);
  OLED.setTextColor(SH110X_WHITE);
  OLED.print("Kecepatan :");


  // --------------------------------------------------
  // WAKTU
  // --------------------------------------------------
  OLED.setCursor(60, 35);
  OLED.setTextColor(SH110X_WHITE);
  OLED.print("Waktu :");


  // --------------------------------------------------
  // KANAN
  // --------------------------------------------------
  OLED.setCursor(8, 48);
  if(selectedOption == 0) {
    OLED.setTextColor(
      SH110X_BLACK,
      SH110X_WHITE
    );
  }
  else {
    OLED.setTextColor(SH110X_WHITE);
  }
  OLED.print("KANAN");


  // --------------------------------------------------
  // KIRI
  // --------------------------------------------------
  OLED.setCursor(72, 48);
  if(selectedOption == 1) {
    OLED.setTextColor(
      SH110X_BLACK,
      SH110X_WHITE
    );
  }
  else {
    OLED.setTextColor(SH110X_WHITE);
  }
  OLED.print("KIRI");


  // --------------------------------------------------
  // MAJU
  // --------------------------------------------------
  OLED.setCursor(8, 56);
  if(selectedOption == 2) {
    OLED.setTextColor(
      SH110X_BLACK,
      SH110X_WHITE
    );
  }
  else {
    OLED.setTextColor(SH110X_WHITE);
  }
  OLED.print("MAJU");


  // --------------------------------------------------
  // MUNDUR
  // --------------------------------------------------
  OLED.setCursor(68, 56);
  if(selectedOption == 3) {
    OLED.setTextColor(
      SH110X_BLACK,
      SH110X_WHITE
    );
  }
  else {
    OLED.setTextColor(SH110X_WHITE);
  }
  OLED.print("MUNDUR");


  // --------------------------------------------------
  // DISPLAY
  // --------------------------------------------------
  OLED.display();
}


// ======================================================
// OLED + BUTTON TASK - CORE 1
// ======================================================
void CoreOne(void *pvParameters)
{
  // Tampilkan OLED pertama kali
  drawOLED();

  while(1) {
    // ==================================================
    // BUTTON NEXT
    // ==================================================

    if(digitalRead(BUTTON_NEXT) == LOW) {
      selectedOption++;

      // Kalau melewati MUNDUR,
      // kembali ke KANAN
      if(selectedOption > 3) {
        selectedOption = 0;
      }

      drawOLED();
      
      // Debounce
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    // ==================================================
    // BUTTON BACK
    // ==================================================
    if(digitalRead(BUTTON_BACK) == LOW) {
      // Kalau sedang di KANAN,
      // BACK akan menuju MUNDUR
      if(selectedOption == 0) {
        selectedOption = 3;
      }
      else {
        selectedOption--;
      }
      drawOLED();

      // Debounce
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    // ==================================================
    // SENSOR BAR
    // ==================================================
    for(uint8_t pointer = 0; pointer < numberofsensor; pointer++) {
      barheight[pointer] = (int16_t)(((float)SensorReading[pointer] / 4096) * 10);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}