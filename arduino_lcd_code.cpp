/*
  DISPLAY NODE - Arduino Mega/Uno + 3.5" TFT ILI9488 + Touch Kill Switch
  UPDATED: Confirmation Popup for Kill Command with User Calibration (ID=0x9488)
*/

#include <MCUFRIEND_kbv.h>
#include <Adafruit_GFX.h>
#include <TouchScreen.h>

// --- 1. TOUCH PINS (Based on your Calibration: ID=0x9488) ---
const int XP = 9, XM = A3, YP = A2, YM = 8; 
const int TS_LEFT = 155, TS_RT = 925, TS_TOP = 57, TS_BOT = 967;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
#define MINPRESSURE 200
#define MAXPRESSURE 1000

MCUFRIEND_kbv tft;

// Color Definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define DARKGREY 0x7BEF

// --- DATA STRUCTURE ---
struct __attribute__((packed)) SensorPacket {
  float temp;
  float hum;
  float volt;
};

SensorPacket data;
SensorPacket old_data;

// --- STATE MACHINE ---
bool is_connected = false;
bool waiting_for_confirm = false; 
unsigned long last_packet_time = 0;
unsigned long last_touch_check = 0;

// --- FUNCTION PROTOTYPES ---
void drawInterface();
void drawConfirmPopup();
void drawLabel(int x, int y, const char* label);
void updateValues();
void checkTouch();

void setup() {
  Serial.begin(115200);
  
  uint16_t ID = tft.readID();
  if (ID == 0xD3D3) ID = 0x9488; 
  tft.begin(ID);
  tft.setRotation(1); 
  
  tft.fillScreen(BLACK);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.setCursor(80, 140);
  tft.print("WAITING FOR ROS...");
}

void loop() {
  // 1. Check Touch (Throttled to 10Hz)
  if (millis() - last_touch_check > 100) {
    checkTouch();
    last_touch_check = millis();
  }

  // 2. Handshake Phase
  if (!is_connected) {
    static unsigned long last_boot_msg = 0;
    if (millis() - last_boot_msg > 1000) {
      Serial.println("BOOT_OK");
      last_boot_msg = millis();
    }
    
    while (Serial.available() > 0) {
      if (Serial.read() == 0xA5) { 
        is_connected = true;
        waiting_for_confirm = false;
        drawInterface();
        last_packet_time = millis(); 
        return; 
      }
    }
    return;
  }

  // 3. Data Receiving Phase (Robust parsing to prevent crashes)
  while (Serial.available() >= 14 && !waiting_for_confirm) { 
    if (Serial.peek() == 0xAA) {
      Serial.read(); // Consume 0xAA
      if (Serial.peek() == 0xBB) {
        Serial.read(); // Consume 0xBB
        Serial.readBytes((char*)&data, sizeof(data));
        last_packet_time = millis();
        updateValues();
      }
    } else {
      // Discard misaligned byte to prevent buffer overflow and UI freeze
      Serial.read();
    }
  }

  // 4. Timeout Check
  if (millis() - last_packet_time > 2000) {
    is_connected = false;
    tft.fillScreen(BLACK);
    tft.setCursor(80, 140);
    tft.print("CONNECTION LOST");
  }
}

// --- GUI & TOUCH LOGIC ---

void checkTouch() {
  TSPoint p = ts.getPoint();
  
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
    int x = map(p.y, TS_TOP, TS_BOT, 0, 480);
    int y = map(p.x, TS_RT, TS_LEFT, 0, 320);

    if (!waiting_for_confirm) {
      if (x > 340 && y > 200) {
        waiting_for_confirm = true;
        drawConfirmPopup();
        delay(300); 
      }
    } else {
      if (x > 60 && x < 220 && y > 180 && y < 260) {
        waiting_for_confirm = false;
        drawInterface();
        delay(300);
      }
      
      if (x > 260 && x < 420 && y > 180 && y < 260) {
        tft.fillScreen(RED);
        tft.setTextColor(WHITE);
        tft.setTextSize(3);
        tft.setCursor(50, 140);
        tft.print("SHUTTING DOWN...");
        
        Serial.println("KILL_SYSTEM"); 
        delay(5000); 
      }
    }
  }
}

void drawConfirmPopup() {
  tft.fillRect(40, 60, 400, 220, DARKGREY);
  tft.drawRect(40, 60, 400, 220, WHITE);

  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(65, 100);
  tft.print("CONFIRM SHUTDOWN?");

  tft.fillRect(60, 180, 160, 80, BLUE);
  tft.setCursor(85, 210);
  tft.print("CANCEL");

  tft.fillRect(260, 180, 160, 80, RED);
  tft.setCursor(315, 210);
  tft.print("OK");
}

void drawInterface() {
  tft.fillScreen(BLACK);
  
  tft.fillRect(0, 0, 480, 40, BLUE);
  tft.setTextColor(WHITE); tft.setTextSize(2);
  tft.setCursor(10, 12); tft.print("ROV TELEMETRY");

  drawLabel(20, 60, "BATTERY");
  drawLabel(20, 140, "TEMP / HUM");
  
  tft.setTextColor(WHITE);
  tft.setCursor(160, 90); tft.print("V");
  tft.setCursor(160, 170); tft.print("C");
  tft.setCursor(160, 200); tft.print("%");

  tft.fillRect(340, 200, 120, 100, RED);
  tft.setTextColor(WHITE); tft.setTextSize(3);
  tft.setCursor(360, 240); tft.print("KILL");
  
  old_data.volt = -1.0;
  old_data.temp = -1.0;
  old_data.hum = -1.0;
}

void drawLabel(int x, int y, const char* label) {
  tft.setTextColor(CYAN); 
  tft.setTextSize(2);
  tft.setCursor(x, y);
  tft.print(label);
}

void updateValues() {
  if (abs(data.volt - old_data.volt) > 0.1) {
    tft.fillRect(20, 90, 130, 25, BLACK);
    tft.setTextColor(data.volt > 11.0 ? GREEN : RED);
    tft.setTextSize(3); tft.setCursor(20, 90); tft.print(data.volt, 1);
    old_data.volt = data.volt;
  }
  if (abs(data.temp - old_data.temp) > 0.5) {
    tft.fillRect(20, 170, 130, 25, BLACK);
    tft.setTextColor(WHITE); tft.setTextSize(3); tft.setCursor(20, 170); tft.print(data.temp, 1);
    old_data.temp = data.temp;
  }
  if (abs(data.hum - old_data.hum) > 1.0) {
    tft.fillRect(20, 200, 130, 25, BLACK);
    tft.setTextColor(WHITE); tft.setTextSize(3); tft.setCursor(20, 200); tft.print(data.hum, 0);
    old_data.hum = data.hum;
  }
}