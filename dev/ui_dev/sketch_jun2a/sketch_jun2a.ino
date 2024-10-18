#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <EncoderButton.h>

// Define the pins for the encoder and button
#define ENCODER_PIN_A   21
#define ENCODER_PIN_B   22
#define BUTTON_PIN      4

// Define the pins for the display
#define TFT_CS     31
// #define TFT_RST    9  // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC     30

// Initialize the display
Adafruit_GC9A01A tft = Adafruit_GC9A01A(&SPI1, TFT_CS, TFT_DC);

// Initialize the encoder button
EncoderButton eb1(ENCODER_PIN_A, ENCODER_PIN_B, BUTTON_PIN);

// Define the shape states
enum Shape {CIRCLE, SQUARE, TRIANGLE};
Shape currentShape = CIRCLE;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  
  // Initialize the display
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(GC9A01A_BLACK);
  
  // Initialize the encoder button
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
}

void loop() {
  // Check if the button was pressed
  eb1.update();
}

void drawShape(Shape shape) {
  int16_t tftWidth = tft.width();
  int16_t tftHeight = tft.height();
  int16_t centerX = tftWidth / 2;
  int16_t centerY = tftHeight / 2;
  int16_t size = min(tftWidth, tftHeight) / 3;

  switch (shape) {
    case CIRCLE:
      tft.drawCircle(centerX, centerY, size, GC9A01A_WHITE);
      break;
    case SQUARE:
      tft.drawRect(centerX - size, centerY - size, size * 2, size * 2, GC9A01A_WHITE);
      break;
    case TRIANGLE:
      tft.drawLine(centerX, centerY - size, centerX - size, centerY + size, GC9A01A_WHITE);
      tft.drawLine(centerX - size, centerY + size, centerX + size, centerY + size, GC9A01A_WHITE);
      tft.drawLine(centerX + size, centerY + size, centerX, centerY - size, GC9A01A_WHITE);
      break;
  }
}

void onEb1Clicked(EncoderButton& eb) {
  Serial.print("eb1 clickCount: ");
  Serial.println(eb.clickCount());
}

void onEb1Encoder(EncoderButton& eb) {
  Serial.print("eb1 incremented by: ");
  Serial.println(eb.increment());
  Serial.print("eb1 position is: ");
  Serial.println(eb.position());

  // Cycle through the shapes
  currentShape = static_cast<Shape>((currentShape + eb.increment()) % 3);
  Serial.printf("Current shape: %i", currentShape);
  // Clear the screen
  tft.fillScreen(GC9A01A_BLACK);
  // Draw the new shape
  drawShape(currentShape);
}