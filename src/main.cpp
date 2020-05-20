#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp32-hal.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include "SSD1306Wire.h"
#include <inttypes.h> 

#define MOTOR_POLES 14
#define MINIQUADTESTBENCH

long thrust = 0;

TaskHandle_t Task1;

rmt_data_t dshotPacket[16];
rmt_obj_t* rmt_send = NULL;

hw_timer_t * timer = NULL;

HardwareSerial MySerial(1);

SSD1306Wire display(0x3c, 21, 22);  // 21 and 22 are default pins

uint8_t receivedBytes = 0;
volatile bool requestTelemetry = false;
bool printTelemetry = true;
uint16_t dshotUserInputValue = 0;
uint16_t dshotmin = 48;
uint16_t dshotmax = 2047;
uint16_t dshotidle = dshotmin + round(3.5 * (dshotmax - dshotmin) / 100); // 3.5%
uint16_t dshot50 =   dshotmin + round(50 * (dshotmax - dshotmin) / 100); // 50%
uint16_t dshot75 =   dshotmin + round(75 * (dshotmax - dshotmin) / 100); // 75%
int16_t ESC_telemetrie[5]; // Temperature, Voltage, Current, used mAh, eRpM
bool runMQTBSequence = true;

uint32_t currentTime;
uint8_t temperature = 0;
uint8_t temperatureMax = 0;
float voltage = 0;
float voltageMin = 99;
uint32_t current = 0;
uint32_t currentMax = 0;
uint32_t erpm = 0;
uint32_t erpmMax = 0;
uint32_t rpm = 0;
uint32_t rpmMAX = 0;
uint32_t kv = 0;
uint32_t kvMax = 0;

void gotTouch4() {
  temperatureMax = 0;
  voltageMin = 99;
  currentMax = 0;
  erpmMax = 0;
  rpmMAX = 0;
  kvMax = 0;
  runMQTBSequence = false;
  printTelemetry = true;
}
void resetMaxMinValues() {
  gotTouch4();
}
void updateDisplay() {
  display.clear();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0,  0, "Dshot Packet");
  display.drawString(0, 10, "Temp C");
  display.drawString(0, 20, "Volt");
  display.drawString(0, 30, "mA");
  display.drawString(0, 40, "eRPM");
  display.drawString(0, 50, "KV");

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(80, 10, String(temperature));
  display.drawString(80, 20, String(voltage));
  display.drawString(80, 30, String(current));
  display.drawString(80, 40, String(erpm));
  display.drawString(80, 50, String(kv));

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128,  0, String(dshotUserInputValue));
  display.drawString(128, 10, String(temperatureMax));
  display.drawString(128, 20, String(voltageMin));
  display.drawString(128, 30, String(currentMax));
  display.drawString(128, 40, String(erpmMax));
  display.drawString(128, 50, String(kvMax));

  display.display();
}

void dshotOutput(uint16_t value, bool telemetry) {

  uint16_t packet;

  // telemetry bit
  if (telemetry) {
    packet = (value << 1) | 1;
  } else {
    packet = (value << 1) | 0;
  }

  // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
  int csum = 0;
  int csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^=  csum_data;
    csum_data >>= 4;
  }
  csum &= 0xf;
  packet = (packet << 4) | csum;

  // durations are for dshot600
  // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
  // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
  // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
  // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
  for (int i = 0; i < 16; i++) {
    if (packet & 0x8000) {
      dshotPacket[i].level0 = 1;
      dshotPacket[i].duration0 = 100;
      dshotPacket[i].level1 = 0;
      dshotPacket[i].duration1 = 34;
    } else {
      dshotPacket[i].level0 = 1;
      dshotPacket[i].duration0 = 50;
      dshotPacket[i].level1 = 0;
      dshotPacket[i].duration1 = 84;
    }
    packet <<= 1;
  }

  rmtWrite(rmt_send, dshotPacket, 16);

  return;

}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i = 0; i < 8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen) {
  uint8_t crc = 0, i;
  for ( i = 0; i < BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
} 
void IRAM_ATTR getTelemetry() {
  requestTelemetry = true;
}

void startTelemetryTimer() {
  Serial.print("starting telemetry timer\n"); 
  timer = timerBegin(0, 80, true); // timer_id = 0; divider=80; countUp = true;
  timerAttachInterrupt(timer, &getTelemetry, true); // edge = true
  timerAlarmWrite(timer, 20000, true);  //1000 = 1 ms
  timerAlarmEnable(timer);
}

// Second core used to handle dshot packets
void secondCoreTask( void * pvParameters ) {
 Serial.print("Starting second core task\n"); 
  while (1) {

    dshotOutput(dshotUserInputValue, requestTelemetry);
    //Serial.print("sending dshot command with value %u", (unsigned)dshotUserInputValue); 
    if (requestTelemetry) {
      requestTelemetry = false;
      receivedBytes = 0;
    }

    delay(1);

  }
}

void receiveTelemtrie() {
  static uint8_t SerialBuf[10];

  if (MySerial.available()) {
    SerialBuf[receivedBytes] = MySerial.read();
    receivedBytes++;
  }

  if (receivedBytes > 9) { // transmission complete

    uint8_t crc8 = get_crc8(SerialBuf, 9); // get the 8 bit CRC

    if (crc8 != SerialBuf[9]) {
      //                Serial.println("CRC transmission failure");

      // Empty Rx Serial of garbage telemtry
      while (MySerial.available())
        MySerial.read();

      requestTelemetry = true;

      return; // transmission failure
    }

    // compute the received values
    ESC_telemetrie[0] = SerialBuf[0]; // temperature
    ESC_telemetrie[1] = (SerialBuf[1] << 8) | SerialBuf[2]; // voltage
    ESC_telemetrie[2] = (SerialBuf[3] << 8) | SerialBuf[4]; // Current
    ESC_telemetrie[3] = (SerialBuf[5] << 8) | SerialBuf[6]; // used mA/h
    ESC_telemetrie[4] = (SerialBuf[7] << 8) | SerialBuf[8]; // eRpM *100

    requestTelemetry = true;

    if (!runMQTBSequence) { // Do not update during MQTB sequence.  Slows serial output.
      updateDisplay();
    }


    if (printTelemetry) {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(dshotUserInputValue);
      Serial.print(",");
      //      Serial.print("Voltage (V): ");
      Serial.print(ESC_telemetrie[1] / 100.0);
      Serial.print(",");
      //      Serial.print("Current (A): ");
      Serial.print(ESC_telemetrie[2] / 10.0);
      Serial.print(",");
      //      Serial.print("RPM : ");
      Serial.print(ESC_telemetrie[4] * 100 / (MOTOR_POLES / 2));
      Serial.print(",");
      // Thrust
      Serial.println(thrust);
    }

    temperature = 0.9 * temperature + 0.1 * ESC_telemetrie[0];
    if (temperature > temperatureMax) {
      temperatureMax = temperature;
    }

    voltage = 0.9 * voltage + 0.1 * (ESC_telemetrie[1] / 100.0);
    if (voltage < voltageMin) {
      voltageMin = voltage;
    }

    current = 0.9 * current + 0.1 * (ESC_telemetrie[2] * 100);
    if (current > currentMax) {
      currentMax = current;
    }
  }
}
 

void setup() {
 Serial.begin(115200);
  Serial.print("init starting\n");
  //MySerial.begin(115200, SERIAL_8N1, 16, 17);

  if ((rmt_send = rmtInit(5, true, RMT_MEM_64)) == NULL) {
    Serial.println("init sender failed\n");
  }
  Serial.print("init sender success\n");
  float realTick = rmtSetTick(rmt_send, 12.5); // 12.5ns sample rate
  Serial.printf("rmt_send tick set to: %fns\n", realTick);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // Output disarm signal while esc initialises and do some display stuff.
  uint8_t xbeep = random(15, 100);
  uint8_t ybeep = random(15, 50);
  uint8_t ibeep = 0;
  while (millis() < 3500) {
    dshotOutput(0, false);
    delay(1000);

    display.clear();
    ibeep++;
    if (ibeep == 100) {
      ibeep = 0;
      xbeep = random(15, 50);
      ybeep = random(15, 50);
    }
    display.drawString(xbeep, ybeep, "beep");
    if (millis() < 500) {
      display.drawString(0, 0, "Initialising ESC... 4s");
    } else if (millis() < 1500) {
      display.drawString(0, 0, "Initialising ESC... 3s");
    } else if (millis() < 2500) {
      display.drawString(0, 0, "Initialising ESC... 2s");
    } else {
      display.drawString(0, 0, "Initialising ESC... 1s");
    }
    display.display();
  }


  // Empty Rx Serial of garbage telemtry
  //while (MySerial.available())
  //  MySerial.read();

  requestTelemetry = false;


  startTelemetryTimer(); // Timer used to request tlm continually in case ESC rcv bad packet

  xTaskCreatePinnedToCore(secondCoreTask, "Task1", 10000, NULL, 1, &Task1, 0);

  Serial.print("Time (ms)");
  Serial.print(",");
  Serial.print("dshot");
  Serial.print(",");
  Serial.print("Voltage (V)");
  Serial.print(",");
  Serial.print("Current (A)");
  Serial.print(",");
  Serial.print("RPM");
  Serial.print(",");
  Serial.println("Thrust (g)");

  dshotUserInputValue = dshotidle; 
  
#ifdef MINIQUADTESTBENCH
  dshotUserInputValue = dshotidle;
  runMQTBSequence = true;
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0,  0, "Running MQTB Sequence...");
  display.display();
#endif

}

void loop() {
  Serial.print("entering loop\n"); 
  printTelemetry = false ;
  //dshotUserInputValue = 500;
  
  // while (millis() < 3500) {
  //   dshotOutput(0, false);
  //   delay(100);
  // }
  printTelemetry = false;
  
runMQTBSequence = true;


#ifdef MINIQUADTESTBENCH
  if (runMQTBSequence) {
    Serial.print("entering miniquadtest\n");
    currentTime = millis();
   // if (currentTime >= 4000 && currentTime < 6000) {
          dshotUserInputValue = dshot50;
          delay(5000);
    // } else if (currentTime >= 6000 && currentTime < 8000) {
    //   dshotUserInputValue = dshotidle;
    // } else if (currentTime >= 8000 && currentTime < 10000) {
    //   dshotUserInputValue = dshot75;
    // } else if (currentTime >= 10000 && currentTime < 12000) {
    //   dshotUserInputValue = dshotidle;
    // } else if (currentTime >= 12000 && currentTime < 14000) {
    //   dshotUserInputValue = dshotmax;
    // } else if (currentTime >= 14000 && currentTime < 16000) {
    //   dshotUserInputValue = dshotmin;
    // } else if (currentTime >= 16000 && currentTime < 22000) {
    //   dshotUserInputValue = dshotmin + (currentTime - 16000) * (dshotmax - dshotmin) / 6000.0;
    // } else if (currentTime >= 24000 && currentTime < 26000) {
    //   dshotUserInputValue = dshotidle;
    // } else if (currentTime >= 26000 && currentTime < 28000) {
    //   printTelemetry = false;
    //   dshotUserInputValue = 0;
    //}
  }
#endif
//resetMaxMinValues();
}
