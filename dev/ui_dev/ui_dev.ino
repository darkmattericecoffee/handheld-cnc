#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <EncoderButton.h>
#include <fonts/FreeMonoBold12pt7b.h>
#include <fonts/FreeMonoBold9pt7b.h>
#include <fonts/FreeMono12pt7b.h>
#include <fonts/FreeMono9pt7b.h>

// Define the pins for the encoder and button
#define ENCODER_PIN_A   21
#define ENCODER_PIN_B   22
#define BUTTON_PIN      4

// Define the pins for the display
#define TFT_CS     31
// #define TFT_RST    9  // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC     30

// State values
typedef enum State {
  POWER_ON,
  MACHINE_X_ZERO,
  WORKSPACE_Z_ZERO,
  DESIGN_SELECTED,
  READY
} State;

// Define the shape states
enum Shape {CIRCLE, SQUARE, TRIANGLE};
int shapeCount = 3;
Shape currentShape = CIRCLE;

// Router state
State state = POWER_ON;

// Initialize the display
Adafruit_GC9A01A tft = Adafruit_GC9A01A(&SPI1, TFT_CS, TFT_DC);

// Initialize the encoder button
EncoderButton eb1(ENCODER_PIN_A, ENCODER_PIN_B, BUTTON_PIN);

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);

  // Pin modes
  pinMode(BUTTON_PIN, INPUT);
  
  // Initialize the display
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(GC9A01A_BLACK);
  drawCenteredText("Powering on...",1);
  delay(1000);
  tft.fillScreen(GC9A01A_BLACK);
  drawCenteredText("Press button to zero x-axis",1);
  while(digitalRead(BUTTON_PIN) == HIGH) {
    delay(10);
  }
  tft.fillScreen(GC9A01A_BLACK);
  drawCenteredText("Move tool to surface then press button",1);
  while(digitalRead(BUTTON_PIN) == HIGH) {
    delay(10);
  }
  tft.fillScreen(GC9A01A_BLACK);
  drawShape(currentShape);
  
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
  if (eb.increment() > 0) {
    currentShape = static_cast<Shape>((currentShape + 1) % shapeCount);
  } else {
    currentShape = static_cast<Shape>((currentShape + shapeCount - 1) % shapeCount); // To cycle back correctly
  }
  Serial.printf("Current shape: %i\n", currentShape);
  // Clear the screen
  tft.fillScreen(GC9A01A_BLACK);
  // Draw the new shape
  drawShape(currentShape);
}

void drawCenteredText(const char* text, int size) {
  int16_t tftWidth = tft.width();
  int16_t tftHeight = tft.height();
  int16_t centerX = tftWidth / 2;
  int16_t centerY = tftHeight / 2;
  int16_t x1, y1;
  uint16_t w, h;
  tft.setFont(&FreeMonoBold9pt7b);
  tft.setTextSize(size);
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  // Calculate the top-left corner to start the text so that it gets centered
  int16_t xStart = centerX - w / 2;
  int16_t yStart = centerY - h / 2;

  tft.setCursor(xStart, yStart);
  tft.setTextColor(GC9A01A_WHITE);
  tft.println(text);
}