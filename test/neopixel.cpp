#include <Arduino.h>
#include <unity.h>
#include <Adafruit_NeoPixel.h>

#define LED_RED 0x00FF0000
#define LED_ORANGE 0x00FF7F00
#define LED_YELLOW 0x00FFFF00
#define LED_GREEN 0x0000FF00
#define LED_BLUE 0x000000FF
#define LED_INDIGO 0x004B0082
#define LED_VIOLET 0x008000FF

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setUp(void)
{
  // set stuff up here
}

void tearDown(void)
{
  // clean stuff up here
}

void test_neopixel_pin_number(void)
{
  TEST_ASSERT_EQUAL(PIN_NEOPIXEL, pixels.getPin());
}

void test_neopixel_state_high(void)
{
  uint32_t color = pixels.Color(255, 255, 255); // white color
  pixels.setPixelColor(0, color);
  pixels.show();
  TEST_ASSERT_EQUAL_UINT32(color, pixels.getPixelColor(0));
}

void test_neopixel_state_low(void)
{
  pixels.clear(); // turn off the NeoPixel
  pixels.show();
  TEST_ASSERT_EQUAL_UINT32(pixels.Color(0, 0, 0), pixels.getPixelColor(0));
}

void test_neopixel_color(uint32_t color) {
  pixels.setPixelColor(0, color);
  pixels.show();
  TEST_ASSERT_EQUAL_UINT32(color, pixels.getPixelColor(0));
}

void test_neopixel_colors(void) {
  test_neopixel_color(LED_RED);
  delay(500);
  test_neopixel_color(LED_ORANGE);
  delay(500);
  test_neopixel_color(LED_YELLOW);
  delay(500);
  test_neopixel_color(LED_GREEN);
  delay(500);
  test_neopixel_color(LED_BLUE);
  delay(500);
  test_neopixel_color(LED_INDIGO);
  delay(500);
  test_neopixel_color(LED_VIOLET);
  delay(500);
}

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);
  pixels.begin(); // This initializes the NeoPixel library.

  UNITY_BEGIN(); // IMPORTANT LINE!
  RUN_TEST(test_neopixel_pin_number);
}

uint8_t i = 0;
uint8_t max_blinks = 3;

void loop()
{
  if (i < max_blinks)
  {
    RUN_TEST(test_neopixel_state_high);
    delay(500);
    RUN_TEST(test_neopixel_state_low);
    delay(500);
    RUN_TEST(test_neopixel_colors);
    delay(500);
    i++;
  }
  else if (i == max_blinks)
  {
    UNITY_END(); // stop unit testing
  }
}
