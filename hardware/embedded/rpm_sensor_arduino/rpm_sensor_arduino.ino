/*
 * Arduino firmware for the rpm sensor.
 * It simply counts interrupts and returns the counters on I2C requests.
 * Use Arduino Mega board.
 * If you want to use a different board other than Mega,
 * be careful about interrupt numbers.
 * Author: Jongwoon Yoo (jongwoon.yoo@nasa.gov)
 */

#include <Wire.h>

// Currently, it support 4 motors.
#define NUM_MOTORS 4
#define NUM_POLES 12
#define I2C_ADDR 0x0A

// Interrupt counter.
long int count_[NUM_MOTORS];

void setup() {
  int i = 0;
  for (i = 0; i < NUM_MOTORS; i++)
    count_[i] = 0;

  /* 
   * Arduino Mega external interrupts.
   * interrupt 0: 2
   * interrupt 1: 3
   * interrupt 5: 18 
   * interrupt 4: 19
   */
  attachInterrupt(0, update0, RISING);
  attachInterrupt(1, update1, RISING);
  attachInterrupt(4, update2, RISING);
  attachInterrupt(5, update3, RISING);

  Wire.begin(I2C_ADDR);
  Wire.onRequest(handleEvent);
}

void loop() {
}

void update0() {
  count_[0]++;
}

void update1() {
  count_[1]++;
}

void update2() {
  count_[2]++;
}

void update3() {
  count_[3]++;
}

void handleEvent() {
  // Convert 'long int' to 'byte []'.
  // The size of 'long int' in Arduino is 4 bytes.
  // The last byte is for checksum (XOR).
  byte data[4 * NUM_MOTORS + 1];
  byte checksum = 0x00;

  int i, j;
  for (i = 0; i < NUM_MOTORS; i++) {
    for (j = 0; j < 4; j++) {
      data[i * NUM_MOTORS + j] = (byte) (count_[i] >> (j * 8));
      checksum ^= data[i * NUM_MOTORS + j];
    }
  }

  data[4 * NUM_MOTORS] = checksum;

  Wire.write(data, 4 * NUM_MOTORS + 1);
}



