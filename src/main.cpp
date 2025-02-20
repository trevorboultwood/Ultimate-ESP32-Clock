/*******************************************************************
    Tetris clock that fetches its time Using the EzTimeLibrary

    For use with the ESP32 or TinyPICO
 *                                                                 *
    Written by Brian Lough
    YouTube: https://www.youtube.com/brianlough
    Tindie: https://www.tindie.com/stores/brianlough/
    Twitter: https://twitter.com/witnessmenow
 *******************************************************************/

// ----------------------------
// Standard Libraries - Already Installed if you have ESP32 set up
// ----------------------------


// yawn ---- lets try and recover what we made, god damn it!



// Wifi portal
//merging time and text.
//class that stores the time and in some instances the time changes to a block colour.



//rotating cube
//plasma
//spiral
//snake
//caledoscope

#include <Wire.h>
#include <WiFi.h>
#include <FastLED.h>
#include <WiFiManager.h>
#include <Arduino.h>
#include <currentDisplay.h>
uint16_t time_counter = 0, cycles = 0, fps = 0;
unsigned long fps_timer;

CRGB currentColor;
CRGBPalette16 palettes[] = {HeatColors_p, LavaColors_p, RainbowColors_p, RainbowStripeColors_p, CloudColors_p};
CRGBPalette16 currentPalette = palettes[0];

CRGB ColorFromCurrentPalette(uint8_t index = 0, uint8_t brightness = 255, TBlendType blendType = LINEARBLEND)
{
  return ColorFromPalette(currentPalette, index, brightness, blendType);
}

// ----------------------------
// Additional Libraries - each one of these will need to be installed.
// ----------------------------

// Enabling this is meant to have a performance
// improvement but its worse for me.
// https://github.com/2dom/PxMatrix/pull/103
//#define double_buffer

#include <PxMatrix.h>
// The library for controlling the LED Matrix
// At time of writing this my changes for the TinyPICO
// Have been merged into the main PxMatrix library,
// but have not been released, so you will need to install
// from Github
//
// If you are using a regular ESP32 you may be able to use
// the library manager version
// https://github.com/2dom/PxMatrix

// Adafruit GFX library is a dependancy for the PxMatrix Library
// Can be installed from the library manager
// https://github.com/adafruit/Adafruit-GFX-Library

#include <TetrisMatrixDraw.h>
// This library draws out characters using a tetris block
// amimation
// Can be installed from the library manager
// https://github.com/toblum/TetrisAnimation

#include <ezTime.h>
#include <Effects.h>
// Library used for getting the time and adjusting for DST
// Search for "ezTime" in the Arduino Library manager
// https://github.com/ropg/ezTime

// ---- Stuff to configure ----

// Initialize a default Wifi connection to the router
char ssid[] = "";          // your network SSID (name)
char password[] = ""; // your network key

#define MYTIMEZONE "Europe/London" // not "Europe/Dublin"!
const char *TZ_INFO = "GMT+0BST-1,M3.5.0/01:00:00,M10.5.0/02:00:00";
// Sets whether the clock should be 12 hour format or not.
bool twelveHourFormat = true;

// If this is set to false, the number will only change if the value behind it changes
// e.g. the digit representing the least significant minute will be replaced every minute,
// but the most significant number will only be replaced every 10 minutes.
// When true, all digits will be replaced every minute.
bool forceRefresh = true;
// -----------------------------

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = NULL;
hw_timer_t *animationTimer = NULL;
// PxMATRIX display(32,16,P_LAT, P_OE,P_A,P_B,P_C);
// PxMATRIX display(64,32,P_LAT, P_OE,P_A,P_B,P_C,P_D);
TetrisMatrixDraw tetris(display);  // Main clock
TetrisMatrixDraw tetris2(display); // The "M" of AM/PM
TetrisMatrixDraw tetris3(display); // The "P" or "A" of AM/PM

Timezone myTZ;
unsigned long oneSecondLoopDue = 0;

bool showColon = true;
volatile bool finishedAnimating = false;
bool displayIntro = true;

String lastDisplayedTime = "";
String lastDisplayedAmPm = "";
bool changedfinishedAnimating = false;

// Effects

Effects effects;
#define MATRIX_WIDTH 64
#define MATRIX_HEIGHT 64

#define PattenQuant 10
struct timerSpiral
{
  unsigned long takt;
  unsigned long lastMillis;
  unsigned long count;
  int delta;
  byte up;
  byte down;
};
timerSpiral multiTimer[5];
byte theta1 = 0;
byte theta2 = 0;
byte hueoffset = 0;

uint8_t radiusx = MATRIX_WIDTH / 4;
uint8_t radiusy = MATRIX_HEIGHT / 4;
uint8_t minx = MATRIX_CENTER_X - radiusx;
uint8_t maxx = MATRIX_CENTER_X + radiusx + 1;
uint8_t miny = MATRIX_CENTER_Y - radiusy;
uint8_t maxy = MATRIX_CENTER_Y + radiusy + 1;

uint8_t spirocount = 1;
uint8_t spirooffset = 256 / spirocount;
boolean spiroincrement = false;

boolean handledChange = false;
int timers = sizeof(multiTimer) / sizeof(multiTimer[0]);

// Some standard colors
uint16_t myRED = display.color565(255, 0, 0);
uint16_t myGREEN = display.color565(0, 255, 0);
uint16_t myBLUE = display.color565(0, 0, 255);
uint16_t myWHITE = display.color565(255, 255, 255);
uint16_t myYELLOW = display.color565(255, 255, 0);
uint16_t myCYAN = display.color565(0, 255, 255);
uint16_t myMAGENTA = display.color565(255, 0, 255);
uint16_t myBLACK = display.color565(0, 0, 0);

void spiralStart()
{ // set all counting directions positive for the beginning
  for (int i = 0; i < timers; i++)
    multiTimer[i].delta = 1;

  // set range (up/down), speed (takt=ms between steps) and starting point of all oszillators

  unsigned long now = millis();

  multiTimer[0].lastMillis = now;
  multiTimer[0].takt = 42; // x1
  multiTimer[0].up = MATRIX_WIDTH - 1;
  multiTimer[0].down = 0;
  multiTimer[0].count = 0;

  multiTimer[1].lastMillis = now;
  multiTimer[1].takt = 55; // y1
  multiTimer[1].up = MATRIX_HEIGHT - 1;
  multiTimer[1].down = 0;
  multiTimer[1].count = 0;

  multiTimer[2].lastMillis = now;
  multiTimer[2].takt = 3; // color
  multiTimer[2].up = 255;
  multiTimer[2].down = 0;
  multiTimer[2].count = 0;

  multiTimer[3].lastMillis = now;
  multiTimer[3].takt = 71; // x2
  multiTimer[3].up = MATRIX_WIDTH - 1;
  multiTimer[3].down = 0;
  multiTimer[3].count = 0;

  multiTimer[4].lastMillis = now;
  multiTimer[4].takt = 89; // y2
  multiTimer[4].up = MATRIX_HEIGHT - 1;
  multiTimer[4].down = 0;
  multiTimer[4].count = 0;
}

// This method is needed for driving the display
void IRAM_ATTR display_updater()
{
  portENTER_CRITICAL_ISR(&timerMux);
  display.display(10);
  portEXIT_CRITICAL_ISR(&timerMux);
}

// This method is for controlling the tetris library draw calls
int firstNewAnimation = 0;
void animationHandler()
{

  // Not clearing the display and redrawing it when you
  // dont need to improves how the refresh rate appears

  if (!finishedAnimating)
  {
    if (firstNewAnimation == 1)
    {
      firstNewAnimation = 0;
      currentBuffer.clearDisplay();
    }

    display.clearDisplay();
    effects.ClearFrame(); // empty

    // display.fillScreen(tetris.tetrisBLACK);
    if (displayIntro)
    {
      finishedAnimating = tetris.drawText(1, 21);
    }
    else
    {
      if (twelveHourFormat)
      {
        // if this executes then we are currently drawing pattern.
        changedfinishedAnimating = true;

        // Place holders for checking are any of the tetris objects
        // currently still animating.
        bool tetris1Done = false;
        bool tetris2Done = false;
        bool tetris3Done = false;

        // Trevor - I have increased y by 15 on these as well as a colon VVV
        tetris1Done = tetris.drawNumbers(-6, 41, showColon); // was 26 before i cock it up
        tetris2Done = tetris2.drawText(56, 40);

        // Only draw the top letter once the bottom letter is finished.
        if (tetris2Done)
        {
          tetris3Done = tetris3.drawText(56, 30);
        }

        finishedAnimating = tetris1Done && tetris2Done && tetris3Done;
        if (finishedAnimating)
        {
          firstNewAnimation = 1;
        }
      }
      else
      {
        finishedAnimating = tetris.drawNumbers(2, 26, showColon);
      }
    }
  }
}

void drawIntro(int x = 0, int y = 0)
{
  tetris.drawChar("P", x, y, tetris.tetrisCYAN);
  tetris.drawChar("o", x + 5, y, tetris.tetrisMAGENTA);
  tetris.drawChar("w", x + 11, y, tetris.tetrisYELLOW);
  tetris.drawChar("e", x + 17, y, tetris.tetrisGREEN);
  tetris.drawChar("r", x + 22, y, tetris.tetrisBLUE);
  tetris.drawChar("e", x + 27, y, tetris.tetrisRED);
  tetris.drawChar("d", x + 32, y, tetris.tetrisWHITE);
  tetris.drawChar(" ", x + 37, y, tetris.tetrisMAGENTA);
  tetris.drawChar("b", x + 42, y, tetris.tetrisYELLOW);
  tetris.drawChar("y", x + 47, y, tetris.tetrisGREEN);
}

void drawConnecting(int x = 0, int y = 0)
{
  tetris.drawChar("C", x, y, tetris.tetrisCYAN);
  tetris.drawChar("o", x + 5, y, tetris.tetrisMAGENTA);
  tetris.drawChar("n", x + 11, y, tetris.tetrisYELLOW);
  tetris.drawChar("n", x + 17, y, tetris.tetrisGREEN);
  tetris.drawChar("e", x + 22, y, tetris.tetrisBLUE);
  tetris.drawChar("c", x + 27, y, tetris.tetrisRED);
  tetris.drawChar("t", x + 32, y, tetris.tetrisWHITE);
  tetris.drawChar("i", x + 37, y, tetris.tetrisMAGENTA);
  tetris.drawChar("n", x + 42, y, tetris.tetrisYELLOW);
  tetris.drawChar("g", x + 47, y, tetris.tetrisGREEN);
}

// new cube stuff

struct vec3d
{
  float x, y, z;
};

struct triangle
{
  vec3d p[3];
};

struct mat4x4
{
  float m[4][4];
};

void MultiplyMatrixVector(vec3d &i, vec3d &o, mat4x4 &m)
{
  o.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + m.m[3][0];
  o.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + m.m[3][1];
  o.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + m.m[3][2];
  float w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + m.m[3][3];

  if (w != 0.0f)
  {
    o.x /= w;
    o.y /= w;
    o.z /= w;
  }
}

void drawTri(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  CRGB color = ColorFromCurrentPalette((30 - 2) * (240 / (MATRIX_WIDTH / 2)));
  effects.BresenhamLine(x0, y0, x1, y1, color);
  effects.BresenhamLine(x1, y1, x2, y2, color);
  effects.BresenhamLine(x2, y2, x0, y0, color);
}

const static triangle cube[12] PROGMEM = {
    // SOUTH
    {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f},

    // EAST
    {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f},
    {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f},

    // NORTH
    {1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f},
    {1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f},

    // WEST
    {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},

    // TOP
    {0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    {0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f},

    // BOTTOM
    {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
};

triangle tri;
mat4x4 matProj, matRotZ, matRotX;
float fTheta;

#define NUM_LAYERS 1

// used for the random based animations
int16_t dx;
int16_t dy;
int16_t dz;
int16_t dsx;
int16_t dsy;

void setup()
{
  Serial.begin(115200);

  // Attempt to connect to Wifi network:

  /* old style of connecting

  // Set WiFi to station mode and disconnect from an AP if it was Previously
  // connected
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  */

//Display a message about connecting


//then disable the screen and try and connect

//then reanable.

  WiFiManager wifiManager;
  if (!wifiManager.autoConnect("WiFiManagerAP", "password"))
  {
    Serial.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
  }

  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Do not set up display before WiFi connection
  // as it will crash!

  // Intialise display library
  // display.begin(16, SPI_BUS_CLK, 27, SPI_BUS_MISO, SPI_BUS_SS); // TinyPICO
  display.begin(32); // Generic ESP32 including Huzzah
  display.flushDisplay();

  // Setup timer for driving display
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &display_updater, true);
  timerAlarmWrite(timer, 2000, true);
  timerAlarmEnable(timer);
  yield();
  display.clearDisplay();
  // "connecting"
  drawConnecting(5, 10);
  // Setup EZ Time
  setDebug(INFO);
  waitForSync();

  Serial.println();
  Serial.println("UTC:             " + UTC.dateTime());

  myTZ.setLocation(F(MYTIMEZONE));
  Serial.print(F("Time in your set timezone:         "));
  Serial.println(myTZ.dateTime());

  display.clearDisplay();
  // "Powered By"
  // drawIntro(6, 12);
  // delay(2000);

  // Start the Animation Timer
  tetris.setText("CLOCK ESP");
  animationTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(animationTimer, &animationHandler, true);
  timerAlarmWrite(animationTimer, 50000, true); // 100000
  timerAlarmEnable(animationTimer);

  // Wait for the animation to finish
  while (!finishedAnimating)
  {
    delay(10); // waiting for intro to finish
  }
  delay(2000);
  firstNewAnimation = 1;
  finishedAnimating = false;
  displayIntro = false;
  tetris.scale = 2;
  effects.targetPalette = palettes[random(0, sizeof(palettes) / sizeof(palettes[0]))]; // just to load a palette
  spiralStart();

  float fNear = 0.1f;
  float fFar = 1000.0f;
  float fFov = 90.0f;
  float fAspectRatio = (float)64 / (float)64;
  float fFovRad = 1.0f / tanf(fFov * 0.5f / 180.0f * 3.14159f);

  matProj.m[0][0] = fAspectRatio * fFovRad;
  matProj.m[1][1] = fFovRad;
  matProj.m[2][2] = fFar / (fFar - fNear);
  matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
  matProj.m[2][3] = 1.0f;
  matProj.m[3][3] = 0.0f;

  // set to reasonable values to avoid a black out
  noisesmoothing = 200;

  // just any free input pin
  // random16_add_entropy(analogRead(18));

  // fill coordinates with random values
  // set zoom levels
  noise_x = random16();
  noise_y = random16();
  noise_z = random16();
  noise_scale_x = 6000;
  noise_scale_y = 6000;

  // for the random movement
  dx = random8();
  dy = random8();
  dz = random8();
  dsx = random8();
  dsy = random8();
}
float fElapsedTime = 0;
void setMatrixTime()
{
  String timeString = "";
  String AmPmString = "";
  if (twelveHourFormat)
  {
    // Get the time in format "1:15" or 11:15 (12 hour, no leading 0)
    // Check the EZTime Github page for info on
    // time formatting
    timeString = myTZ.dateTime("g:i");

    // If the length is only 4, pad it with
    //  a space at the beginning
    if (timeString.length() == 4)
    {
      timeString = " " + timeString;
    }

    // Get if its "AM" or "PM"
    AmPmString = myTZ.dateTime("A");
    if (lastDisplayedAmPm != AmPmString)
    {
      Serial.println(AmPmString);
      lastDisplayedAmPm = AmPmString;
      // Second character is always "M"
      // so need to parse it out
      tetris2.setText("M", forceRefresh);

      // Parse out first letter of String
      tetris3.setText(AmPmString.substring(0, 1), forceRefresh);
    }
  }
  else
  {
    // Get time in format "01:15" or "22:15"(24 hour with leading 0)
    timeString = myTZ.dateTime("H:i");
  }

  // Only update Time if its different
  if (lastDisplayedTime != timeString)
  {
    Serial.println(timeString);
    lastDisplayedTime = timeString;
    tetris.setTime(timeString, forceRefresh);

    // Must set this to false so animation knows
    // to start again
    finishedAnimating = false;
  }
}

void handleColonAfterAnimation()
{

  // It will draw the colon every time, but when the colour is black it
  // should look like its clearing it.
  uint16_t colour = showColon ? tetris.tetrisWHITE : tetris.tetrisBLACK;
  // The x position that you draw the tetris animation object
  int x = twelveHourFormat ? -6 : 2;
  // The y position adjusted for where the blocks will fall from
  // (this could be better!)

  // trevor has increased the y by 15.
  int y = 41 - (TETRIS_Y_DROP_DEFAULT * tetris.scale);
  tetris.drawColon(x, y, colour);
}

bool direction = false;
byte theta = 0;
byte hueoffset2 = 0;
int patternIndex = 7;
// snake stuff.

static const byte SNAKE_LENGTH = 16;

CRGB colors[SNAKE_LENGTH];
uint8_t initialHue;

enum Direction
{
  UP,
  DOWN,
  LEFT,
  RIGHT
};

struct Pixel
{
  uint8_t x;
  uint8_t y;
};
struct Snake
{
  Pixel pixels[SNAKE_LENGTH];

  Direction direction;

  void newDirection()
  {
    switch (direction)
    {
    case UP:
    case DOWN:
      direction = random(0, 2) == 1 ? RIGHT : LEFT;
      break;

    case LEFT:
    case RIGHT:
      direction = random(0, 2) == 1 ? DOWN : UP;

    default:
      break;
    }
  }
  void shuffleDown()
  {
    for (byte i = SNAKE_LENGTH - 1; i > 0; i--)
    {
      pixels[i] = pixels[i - 1];
    }
  }
  void reset()
  {
    direction = UP;
    for (int i = 0; i < SNAKE_LENGTH; i++)
    {
      pixels[i].x = 0;
      pixels[i].y = 0;
    }
  }
  void move()
  {
    switch (direction)
    {
    case UP:
      pixels[0].y = (pixels[0].y + 1) % MATRIX_HEIGHT;
      break;
    case LEFT:
      pixels[0].x = (pixels[0].x + 1) % MATRIX_WIDTH;
      break;
    case DOWN:
      pixels[0].y = pixels[0].y == 0 ? MATRIX_HEIGHT - 1 : pixels[0].y - 1;
      break;
    case RIGHT:
      pixels[0].x = pixels[0].x == 0 ? MATRIX_WIDTH - 1 : pixels[0].x - 1;
      break;
    }
  }
  void clearSnake(void) // Removing snake from screen.
  {
    for (byte i = 0; i < SNAKE_LENGTH; i++)
    {
      effects.leds[XY(pixels[i].x, pixels[i].y)] = CRGB(0, 0, 0);
    }
  }

  void draw(CRGB colors[SNAKE_LENGTH])
  {
    for (byte i = 0; i < SNAKE_LENGTH; i++)
    {
      // effects.leds[XY(pixels[i].x, pixels[i].y)] = ColorFromCurrentPalette(100); //this line works. but boring colours

      effects.leds[XY(pixels[i].x, pixels[i].y)] = colors[i] %= (255 - i * (255 / SNAKE_LENGTH)); // this one doesnt
    }
  }
};

static const int snakeCount = 10;
Snake snakes[snakeCount];
// counts all variables with different speeds linear up and down
void UpdateTimers()
{
  unsigned long now = millis();
  for (int i = 0; i < timers; i++)
  {
    while (now - multiTimer[i].lastMillis >= multiTimer[i].takt)
    {
      multiTimer[i].lastMillis += multiTimer[i].takt;
      multiTimer[i].count = multiTimer[i].count + multiTimer[i].delta;
      if ((multiTimer[i].count == multiTimer[i].up) || (multiTimer[i].count == multiTimer[i].down))
      {
        multiTimer[i].delta = -multiTimer[i].delta;
      }
    }
  }
}
void ShowNoiseLayer(byte layer, byte colorrepeat, byte colorshift)
{
  for (uint8_t i = 0; i < MATRIX_WIDTH; i++)
  {
    for (uint8_t j = 0; j < MATRIX_HEIGHT; j++)
    {
      uint8_t pixel = noise[i][j];

      // assign a color depending on the actual palette
      effects.leds[XY(i, j)] = effects.ColorFromCurrentPalette(colorrepeat * (pixel + colorshift), pixel);
    }
  }
}
int hue;
int centerX = 0;
int centerY = 0;
int step = -1;
int maxSteps = 16;
float fadeRate = 0.8;
int diff;

// cube stuff
int zOff = 150;
int xOff = 0;
int yOff = 0;
int cSize = 40;
int view_plane = 64;
float angle = PI / 100;
int rsteps = 100;
float pyrimid3d[8][3] = {
    {-cSize, cSize, zOff - cSize}, // 4
    {cSize, cSize, zOff - cSize},  // 1
    {-cSize, yOff - cSize, zOff - cSize},
    {0, yOff - cSize / 2, zOff},   // top point
    {-cSize, cSize, zOff + cSize}, // 2
    {cSize, cSize, zOff + cSize},  // 3
    {-cSize, -cSize, zOff + cSize},
    {cSize, -cSize, zOff + cSize}};

float cube3d[8][3] = {
    {xOff - cSize, yOff + cSize, zOff - cSize},
    {xOff + cSize, yOff + cSize, zOff - cSize},
    {xOff - cSize, yOff - cSize, zOff - cSize},
    {xOff + cSize, yOff - cSize, zOff - cSize},
    {xOff - cSize, yOff + cSize, zOff + cSize},
    {xOff + cSize, yOff + cSize, zOff + cSize},
    {xOff - cSize, yOff - cSize, zOff + cSize},
    {xOff + cSize, yOff - cSize, zOff + cSize}};

unsigned char cube2d[8][2];

// cube
void draw_pyrimid()
{
  CRGB color = ColorFromCurrentPalette((30 - 2) * (240 / (MATRIX_WIDTH / 2)));

  effects.BresenhamLine(cube2d[1][0], cube2d[1][1], cube2d[3][0], cube2d[3][1], color); // top point connected to one at the bottom
  effects.BresenhamLine(cube2d[4][0], cube2d[4][1], cube2d[3][0], cube2d[3][1], color);
  effects.BresenhamLine(cube2d[0][0], cube2d[0][1], cube2d[3][0], cube2d[3][1], color);
  effects.BresenhamLine(cube2d[5][0], cube2d[5][1], cube2d[3][0], cube2d[3][1], color);
  // Square at base
  effects.BresenhamLine(cube2d[0][0], cube2d[0][1], cube2d[1][0], cube2d[1][1], color);
  effects.BresenhamLine(cube2d[0][0], cube2d[0][1], cube2d[4][0], cube2d[4][1], color);
  effects.BresenhamLine(cube2d[1][0], cube2d[1][1], cube2d[5][0], cube2d[5][1], color);
  effects.BresenhamLine(cube2d[4][0], cube2d[4][1], cube2d[5][0], cube2d[5][1], color);
}

void draw_cube()
{

  CRGB color = ColorFromCurrentPalette((30 - 2) * (240 / (MATRIX_WIDTH / 2)));
  effects.BresenhamLine(cube2d[1][0], cube2d[1][1], cube2d[3][0], cube2d[3][1], color); // top point
  effects.BresenhamLine(cube2d[0][0], cube2d[0][1], cube2d[1][0], cube2d[1][1], color);
  effects.BresenhamLine(cube2d[0][0], cube2d[0][1], cube2d[2][0], cube2d[2][1], color);
  effects.BresenhamLine(cube2d[0][0], cube2d[0][1], cube2d[4][0], cube2d[4][1], color);
  effects.BresenhamLine(cube2d[1][0], cube2d[1][1], cube2d[5][0], cube2d[5][1], color);
  effects.BresenhamLine(cube2d[2][0], cube2d[2][1], cube2d[6][0], cube2d[6][1], color);
  effects.BresenhamLine(cube2d[2][0], cube2d[2][1], cube2d[3][0], cube2d[3][1], color);
  effects.BresenhamLine(cube2d[4][0], cube2d[4][1], cube2d[6][0], cube2d[6][1], color);
  effects.BresenhamLine(cube2d[4][0], cube2d[4][1], cube2d[5][0], cube2d[5][1], color);
  effects.BresenhamLine(cube2d[7][0], cube2d[7][1], cube2d[6][0], cube2d[6][1], color);
  effects.BresenhamLine(cube2d[7][0], cube2d[7][1], cube2d[3][0], cube2d[3][1], color);
  effects.BresenhamLine(cube2d[7][0], cube2d[7][1], cube2d[5][0], cube2d[5][1], color);
  // delay(500);
}
void printcube()
{
  effects.ClearFrame();
  // calculate 2d points
  for (byte i = 0; i < 8; i++)
  {
    cube2d[i][0] = (unsigned char)((cube3d[i][0] * view_plane / cube3d[i][2]) + (64 / 2));
    cube2d[i][1] = (unsigned char)((cube3d[i][1] * view_plane / cube3d[i][2]) + (64 / 2));
  }
  draw_cube();
  effects.ShowFrame();
}

void zrotate(float q)
{
  float tx, ty, temp;
  for (byte i = 0; i < 8; i++)
  {
    tx = cube3d[i][0] - xOff;
    ty = cube3d[i][1] - yOff;
    temp = tx * cos(q) - ty * sin(q);
    ty = tx * sin(q) + ty * cos(q);
    tx = temp;
    cube3d[i][0] = tx + xOff;
    cube3d[i][1] = ty + yOff;
  }
}

void yrotate(float q)
{
  float tx, tz, temp;
  for (byte i = 0; i < 8; i++)
  {
    tx = cube3d[i][0] - xOff;
    tz = cube3d[i][2] - zOff;
    temp = tz * cos(q) - tx * sin(q);
    tx = tz * sin(q) + tx * cos(q);
    tz = temp;
    cube3d[i][0] = tx + xOff;
    cube3d[i][2] = tz + zOff;
  }
}

void xrotate(float q)
{
  float ty, tz, temp;
  for (byte i = 0; i < 8; i++)
  {
    ty = cube3d[i][1] - yOff;
    tz = cube3d[i][2] - zOff;
    temp = ty * cos(q) - tz * sin(q);
    tz = ty * sin(q) + tz * cos(q);
    ty = temp;
    cube3d[i][1] = ty + yOff;
    cube3d[i][2] = tz + zOff;
  }
}

void scalePoints(float n)
{
  for (int i = 0; i < 8; i++)
  {
    cube3d[i][0] = cube3d[i][0] * n;
    cube3d[i][1] = cube3d[i][1] * n;
    cube3d[i][2] = cube3d[i][2] * n;
  }
}

void patten_init()
{
  switch (patternIndex) // pattern specific randomness and init
  {
  case 6:
  {
    ;
  }

  default:
    break;
  }
}
// uint8_t blockPattern = 0x00; //indicates the current effect takes the whole screen.
byte previousSecond = 0;
byte previousMinute = 0;
byte previousHour = 0;
byte clockConfig = 1;
void check_status()
{
  bool displayColon = true;                                          // An animation might not want to draw the colon.
  if (changedfinishedAnimating == true && finishedAnimating == true) // This all gets executed once.
  {
    changedfinishedAnimating = false;
    // captured it, do once after tetris, eg change some effects up etc
    direction = !direction;

    // clear led buffer
    effects.ClearFrame();
    // change pattern
    patternIndex++;
    if (patternIndex > PattenQuant - 1)
      patternIndex = 0;

    // currentDisplay *currentBuffer;
    // currentBuffer = effects.getCurrentBuffer();
    if (patternIndex == 2 || patternIndex == 8)
    {
      currentBuffer.blockPattern = 0x01;
      effects.changeTetrisText();
    }
    else
      currentBuffer.blockPattern = 0x00;

    if (patternIndex == 9) // analog clock
    {
      currentBuffer.clearDisplay(); // remove background limitations.
      effects.ClearFrame();
      effects.ShowFrameComplete();
    }

    Serial.print("Pattern: ");
    Serial.println(patternIndex);
    patten_init(); // init for some patterns
  }
  // patterns that dont need a colon.
  if (patternIndex == 9)
    displayColon = false;

  unsigned long now = millis();
  if (now > oneSecondLoopDue) // Colon blink
  {
    // We can call this often, but it will only
    // update when it needs to
    setMatrixTime();
    showColon = !showColon;
    if (finishedAnimating)
    {

      if (displayColon)
        handleColonAfterAnimation(); // Dont need this now as we have patterns... might introduce later
    }
    oneSecondLoopDue = now + 1000;
  }
  clockConfig = 0x01;
}

void ShowNoiseLayer2(byte layer, byte colorrepeat, byte colorshift)
{
  for (uint8_t i = 0; i < MATRIX_WIDTH; i++)
  {
    for (uint8_t j = 0; j < MATRIX_HEIGHT; j++)
    {

      uint8_t color = noise[i][j];

      uint8_t bri = color;

      // assign a color depending on the actual palette
      CRGB pixel = ColorFromCurrentPalette(colorrepeat * (color + colorshift));

      effects.leds[XY(i, j)] = pixel;
    }
  }
}
float deg2rad(float deg)
{
  float rad = (deg * 3.1415 / 180);
  return rad;
}

void loop()
{
  // Calculates when we have swapped finishedAnimating

  // patternIndex = 9; // force pattern //dont forget this gets incremented by one within status
  check_status();

  if (finishedAnimating)
  {

    switch (patternIndex)
    {
    case 0: // radarPattern
    {
      effects.DimAll(245);
      effects.ShowFrame();

      for (int offset = 0; offset < MATRIX_CENTER_X; offset++)
      {
        byte hue = 255 - (offset * 16 + hueoffset);
        CRGB color = ColorFromCurrentPalette(hue);
        uint8_t x = mapcos8(theta, offset, (MATRIX_WIDTH - 1) - offset);
        uint8_t y = mapsin8(theta, offset, (MATRIX_HEIGHT - 1) - offset);
        uint16_t xy = XY(x, y);
        effects.leds[xy] = color;

        EVERY_N_MILLIS(25)
        {
          theta += 2;
          hueoffset += 1;
        }
      }

      break;
    }
    case 1: // incremental drift pattern.
    {
      uint8_t dim = beatsin8(2, 230, 250);
      effects.DimAll(dim);
      effects.ShowFrame();

      for (int i = 2; i <= MATRIX_WIDTH / 2; i++)
      {
        CRGB color = ColorFromCurrentPalette((i - 2) * (240 / (MATRIX_WIDTH / 2)));

        uint8_t x = beatcos8((17 - i) * 2, MATRIX_CENTER_X - i, MATRIX_CENTER_X + i);
        uint8_t y = beatsin8((17 - i) * 2, MATRIX_CENTER_Y - i, MATRIX_CENTER_Y + i);

        effects.savePixel(x, y, color);
      }
      break;
    }
    case 2:
    { // plasma
      for (int x = 0; x < 64; x++)
      {
        for (int y = 0; y < 64; y++)
        {
          int16_t v = 0;
          uint8_t wibble = sin8(time_counter);
          v += sin16(x * wibble * 3 + time_counter);
          v += cos16(y * (128 - wibble) + time_counter);
          v += sin16(y * x * cos8(-time_counter) / 8);
          CRGB color = ColorFromCurrentPalette((v >> 8) + 127);
          effects.savePixel(x, y, color);
          // effects.drawBackgroundFastLEDPixelCRGB(x, y, (v >> 8) + 127); working but boring
          // efect.pixel not working?!
        }
      }
      effects.ShowFrame();
      break;
    }
    case 3:
    {
      // Snake pattern

      fill_palette(colors, SNAKE_LENGTH, initialHue++, 5, currentPalette, 255, LINEARBLEND);

      for (int i = 0; i < snakeCount; i++)
      {
        Snake *snake = &snakes[i];
        snake->clearSnake();
        snake->shuffleDown();

        if (random(10) > 7)
        {
          snake->newDirection();
        }

        snake->move();
        snake->draw(colors);
      }

      effects.ShowFrame();

      break;
    }
    case 4:
    { // infinity could do with two more of these.

      effects.DimAll(250);
      effects.ShowFrame();
      effects.MoveOscillators();
      int x = (MATRIX_WIDTH - 1) - effects.p[1];
      int y = map8(sin8(effects.osci[3]), 8, 23);
      byte hue = sin8(effects.osci[5]);

      effects.savePixel(x, y, hue);

      // add more here.
      // effects.Pixel(x, y, hue);
      break;
    }
    case 5: // spiral pattern
    {

      // manage the Oszillators
      UpdateTimers();

      // draw just a line defined by 5 oszillators
      effects.BresenhamLine(
          multiTimer[3].count,  // x1
          multiTimer[4].count,  // y1
          multiTimer[0].count,  // x2
          multiTimer[1].count,  // y2
          multiTimer[2].count); // color

      // manipulate the screen buffer
      // with fixed parameters (could be oszillators too)
      // Params: center x, y, radius, scale color down
      // --> NOTE: Affects always a SQUARE with an odd length
      //   effects.SpiralStream(15, 15, 10, 128);

      effects.SpiralStream(31, 15, 64, 128); // for 64 pixel wide matrix!
                                             //  effects.SpiralStream(47, 15, 10, 128);        // for 64 pixel wide matrix!

      // why not several times?!
      // effects.SpiralStream(16, 6, 6, 128);
      // effects.SpiralStream(10, 24, 10, 128);

      // increase the contrast
      effects.DimAll(250);
      effects.ShowFrame();
      break;
    }
    case 6:
    { // spiro
      effects.DimAll(254);
      effects.ShowFrame();

      boolean change = false;

      for (int i = 0; i < spirocount; i++)
      {
        uint8_t x = mapsin8(theta1 + i * spirooffset, minx, maxx);
        uint8_t y = mapcos8(theta1 + i * spirooffset, miny, maxy);

        uint8_t x2 = mapsin8(theta2 + i * spirooffset, x - radiusx, x + radiusx);
        uint8_t y2 = mapcos8(theta2 + i * spirooffset, y - radiusy, y + radiusy);

        CRGB color = ColorFromCurrentPalette(hueoffset2 + i * spirooffset);
        effects.leds[XY(x2, y2)] = color;

        if ((x2 == MATRIX_CENTER_X && y2 == MATRIX_CENTER_Y) ||
            (x2 == MATRIX_CENTRE_X && y2 == MATRIX_CENTRE_Y))
          change = true;
      }

      theta2 += 1;

      EVERY_N_MILLIS(25)
      {
        theta1 += 1;
      }

      EVERY_N_MILLIS(100)
      {
        if (change && !handledChange)
        {
          handledChange = true;

          if (spirocount >= MATRIX_WIDTH || spirocount == 1)
            spiroincrement = !spiroincrement;

          if (spiroincrement)
          {
            if (spirocount >= 4)
              spirocount *= 2;
            else
              spirocount += 1;
          }
          else
          {
            if (spirocount > 4)
              spirocount /= 2;
            else
              spirocount -= 1;
          }

          spirooffset = 256 / spirocount;
        }

        if (!change)
          handledChange = false;
      }

      EVERY_N_MILLIS(33)
      {
        hueoffset2 += 1;
      }

      break;
    }
    case 7:
    { // pyrimid pattern,
      switch (random(6))
      {
      case 0:
        for (int i = 0; i < rsteps; i++)
        {
          zrotate(angle);
          printcube();
        }
        break;
      case 1:
        for (int i = 0; i < rsteps; i++)
        {
          zrotate(2 * PI - angle);
          printcube();
        }
        break;
      case 2:
        for (int i = 0; i < rsteps; i++)
        {
          xrotate(angle);
          printcube();
        }
        break;
      case 3:
        for (int i = 0; i < rsteps; i++)
        {
          xrotate(2 * PI - angle);
          printcube();
        }
        break;
      case 4:
        for (int i = 0; i < rsteps; i++)
        {
          yrotate(angle);
          printcube();
        }
        break;
      case 5:
        for (int i = 0; i < rsteps; i++)
        {
          yrotate(2 * PI - angle);
          printcube();
        }
        break;
      }
      break;
    }
    case 8: // caleidoscope
    {
      noise_y += dy;
      noise_x += dx;
      noise_z += dz;

      effects.FillNoise();
      ShowNoiseLayer2(0, 1, 0);

      effects.Caleidoscope3();
      effects.Caleidoscope1();

      effects.ShowFrame();
      break;

    }
    case 9: // analog clock
    {
      int cx = 32;
      int cy = 32;
      display.drawCircle(cx, cy, 31, myCYAN);

      display.setTextColor(myBLUE);

      display.setCursor(27, 4);
      display.print("12");

      display.setCursor(4, 29);
      display.print("9");

      display.setCursor(56, 29);
      display.print("3");

      display.setCursor(30, 54);
      display.print("6");

      String theTimeNow = dateTime("l, d-M-y H:i:s.v T");

      byte seconds = ((theTimeNow[24] - 48) * 10) + theTimeNow[25] - 48;
      byte minutes = ((theTimeNow[21] - 48) * 10) + theTimeNow[22] - 48;
      byte hours = ((theTimeNow[18] - 48) * 10) + theTimeNow[19] - 48;

      if (hours > 12)
        hours = hours - 12;

      int r_sec = 28; // Length of second-hand
      int r_min = 26; // Length of minute-hand
      int r_hr = 20;  // Length of hour-hand
      int x_sec, y_sec, x_min, y_min, x_hr, y_hr;

      // remove old stuff
      x_sec = r_sec * cos(deg2rad((previousSecond - 15) * (360 / 60))) + cx;
      y_sec = r_sec * sin(deg2rad((previousSecond - 15) * (360 / 60))) + cy;
      x_min = r_min * cos(deg2rad((previousMinute - 15) * (360 / 60) + previousSecond / 10)) + cx;
      y_min = r_min * sin(deg2rad((previousMinute - 15) * (360 / 60) + previousSecond / 10)) + cy;
      x_hr = r_hr * cos(deg2rad((previousHour - 3) * (360 / 12) + previousMinute / 2)) + cx;
      y_hr = r_hr * sin(deg2rad((previousHour - 3) * (360 / 12) + previousMinute / 2)) + cy;

      display.drawLine(cx, cy, x_sec, y_sec, myBLACK);
      display.drawLine(cx, cy, x_min, y_min, myBLACK);
      display.drawLine(cx, cy, x_hr, y_hr, myBLACK);
      display.drawCircle(x_hr, y_hr, 2, myBLACK);
      // some notes
      /*
      - 15,3 is because we want to start at the top not the right (think sin and cos)
      */

      // print new stuff
      x_sec = r_sec * cos(deg2rad((seconds - 15) * (360 / 60))) + cx;
      y_sec = r_sec * sin(deg2rad((seconds - 15) * (360 / 60))) + cy;

      x_min = r_min * cos(deg2rad((minutes - 15) * (360 / 60) + seconds / 10)) + cx;
      y_min = r_min * sin(deg2rad((minutes - 15) * (360 / 60) + seconds / 10)) + cy;
      x_hr = r_hr * cos(deg2rad((hours - 3) * (360 / 12) + minutes / 2)) + cx;
      y_hr = r_hr * sin(deg2rad((hours - 3) * (360 / 12) + minutes / 2)) + cy;

      display.drawLine(cx, cy, x_sec, y_sec, myRED);
      display.drawLine(cx, cy, x_min, y_min, myWHITE);
      display.drawLine(cx, cy, x_hr, y_hr, myWHITE);
      display.drawCircle(x_hr, y_hr, 2, myMAGENTA);

      display.drawCircle(cx, cy, 0, myBLACK);
      display.drawCircle(cx, cy, 1, myBLACK);
      display.drawCircle(cx, cy, 2, myCYAN);

      // clear some pixels in the middle.

      // display.fillRect(28,28,5,5,myWHITE);

      display.writePixel(31, 31, myGREEN);
      display.writePixel(31, 32, myGREEN);
      display.writePixel(31, 33, myGREEN);

      display.writePixel(32, 31, myGREEN);
      display.writePixel(32, 32, myGREEN);
      display.writePixel(32, 33, myGREEN);

      display.writePixel(33, 31, myGREEN);
      display.writePixel(33, 32, myGREEN);
      display.writePixel(33, 33, myGREEN);

      display.showBuffer();
      previousHour = hours;
      previousMinute = minutes;
      previousSecond = seconds;
      delay(100);
      break;

      //      delay(1000);
    }

    default:
    {
      break;
    }
    }
    ++time_counter;
    ++cycles;
    ++fps;
    // Changing Palettes

    if (cycles >= 512) // 1024)
    {                  // something not right with this.... Need to establish differences between effect palletes and current....
      time_counter = 0;
      cycles = 0;
      currentPalette = palettes[random(0, sizeof(palettes) / sizeof(palettes[0]))];
      effects.targetPalette = palettes[random(0, sizeof(palettes) / sizeof(palettes[0]))];
    }
  }
}
