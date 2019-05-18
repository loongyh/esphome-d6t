#include "esphome.h"
#include "math.h"

using namespace esphome;

#define I2C_ADDR_D6T 0x0a
#define CMD 0x4c

static const int SOBEL_X[3][3] =
    { { -1, 0, 1 },
      { -2, 0, 2 },
      { -1, 0, 1 } };

static const int SOBEL_Y[3][3] =
  { { -1, -2, -1 },
    { 0,  0,  0 },
    { 1,  2,  1 } };

static const float THRESHOLD = 4.0;

class D6T44L06BinarySensor : public PollingComponent, public binary_sensor::BinarySensor {
 public:
  D6T44L06BinarySensor() : PollingComponent(500) {}

  void setup() override {
    Wire.begin();
  }

  void update() override {
    float temps[4][4];
    float gradients[4][4];
    bool state;

    readTemps(temps);
    applySobel(temps, gradients);
    state = meetsThreshold(gradients);

    publish_state(state);
  }

 private:
  void readTemps(float temps[4][4]) {
    uint8_t buf[35];
    int i = 0;

    Wire.beginTransmission(I2C_ADDR_D6T);
    Wire.write(CMD);
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDR_D6T, 35);
    while (Wire.available() && i < 35){
      buf[i++] = Wire.read();
    }
    Wire.endTransmission();

    for (int y = 0; y < 4; y++) {
      for (int x = 0; x < 4; x++) {
        i = x + (y * 4);
        temps[y][x] = (buf[(i * 2 + 2)] + (buf[(i * 2 + 3)] << 8)) * 0.1;
      }
    }
  }

  void applySobel(float temps[4][4], float gradients[4][4]) {
    for (int y = 0; y < 4; y++) {
      for (int x = 0; x < 4; x++) {
        float magX = 0;
        float magY = 0;
        float gradient = 0;

        for (int yk = 0; yk < 3; yk++) {
          for (int xk = 0; xk < 3; xk++) {
            magX += temps[clamp(y + yk - 1, 0, 3)][clamp(x + xk - 1, 0, 3)] * SOBEL_X[yk][xk];
            magY += temps[clamp(y + yk - 1, 0, 3)][clamp(x + xk - 1, 0, 3)] * SOBEL_Y[yk][xk];
          }
        }

        gradients[y][x] = sqrt(pow(magX, 2) + pow(magY, 2));
      }
    }
  }

  bool meetsThreshold(float values[4][4]) {
    for (int y = 0; y < 4; y++){
      for (int x = 0; x < 4; x++){
        if (values[y][x] >= THRESHOLD) {
          return true;
        }
      }
    }

    return false;
  }

  int clamp(int d, int min, int max) {
    const int t = d < min ? min : d;
    return t > max ? max : t;
  }
};