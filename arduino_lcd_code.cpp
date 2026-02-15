/*
  DISPLAY NODE - Arduino Mega/Uno + 3.5" TFT ILI9488
  - Receives Binary Data from ROS 2
  - Draws Dashboard
*/

#include <MCUFRIEND_kbv.h>
#include <Adafruit_GFX.h>

// --- SCREEN SETUP ---
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
  float roll;
  float pitch;
  float yaw;
  float temp;
  float hum;
  float volt;
};

SensorPacket data;
SensorPacket old_data; // To check for changes

// --- STATE MACHINE ---
bool is_connected = false;
unsigned long last_packet_time = 0;

// --- FUNCTION PROTOTYPES ---
void drawInterface();
void drawLabel(int x, int y, const char* label); // FIXED: Added 'const'
void updateValues();
void drawAngle(int x, int y, float val, float *old_val);

void setup() {
  Serial.begin(115200);
  
  // LCD Init
  uint16_t ID = tft.readID();
  if (ID == 0xD3D3) ID = 0x9486; // Write-only controller fix
  tft.begin(ID);
  tft.setRotation(1); // Landscape
  
  // Draw Loading Screen
  tft.fillScreen(BLACK);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.setCursor(80, 140);
  tft.print("WAITING FOR ROS...");
}

void loop() {
  // 1. Handshake Phase
  if (!is_connected) {
    static unsigned long last_boot_msg = 0;
    if (millis() - last_boot_msg > 1000) {
      Serial.println("BOOT_OK");
      last_boot_msg = millis();
    }
    
    if (Serial.available()) {
      byte cmd = Serial.read();
      if (cmd == 0xA5) { // START CMD received
        is_connected = true;
        drawInterface(); // Draw the static UI once
      }
    }
    return;
  }

  // 2. Data Receiving Phase
  // Look for Header 0xAA 0xBB
  if (Serial.available() >= 26) { // Header(2) + Data(24)
    if (Serial.read() == 0xAA) {
      if (Serial.read() == 0xBB) {
        
        // Read the struct bytes
        Serial.readBytes((char*)&data, sizeof(data));
        last_packet_time = millis();
        
        // Update Screen
        updateValues();
      }
    }
  }

  // 3. Timeout Check (If ROS crashes, go back to waiting)
  if (millis() - last_packet_time > 2000) {
    is_connected = false;
    tft.fillScreen(BLACK);
    tft.setCursor(80, 140);
    tft.print("CONNECTION LOST");
  }
}

// --- GUI FUNCTIONS ---

void drawInterface() {
  tft.fillScreen(BLACK);
  
  // Header
  tft.fillRect(0, 0, 480, 40, BLUE);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 12);
  tft.print("ROV TELEMETRY");

  // Labels
  drawLabel(20, 60, "BATTERY");
  drawLabel(20, 140, "TEMP / HUM");
  drawLabel(240, 60, "ORIENTATION");
  
  // Static Units
  tft.setTextColor(WHITE); // Reset color
  tft.setTextSize(2);
  tft.setCursor(160, 90); tft.print("V");
  tft.setCursor(160, 170); tft.print("C");
  tft.setCursor(160, 200); tft.print("%");
  
  tft.setCursor(380, 90); tft.print("R");
  tft.setCursor(380, 120); tft.print("P");
  tft.setCursor(380, 150); tft.print("Y");
}

// FIXED: Changed char* to const char* and Cyan to CYAN
void drawLabel(int x, int y, const char* label) {
  tft.setTextColor(CYAN); 
  tft.setTextSize(2);
  tft.setCursor(x, y);
  tft.print(label);
}

void updateValues() {
  // We use "old_data" to only redraw pixels if values changed significantly
  // This reduces flicker.

  // --- BATTERY ---
  if (abs(data.volt - old_data.volt) > 0.1) {
    tft.fillRect(20, 90, 130, 25, BLACK); // Clear previous text
    tft.setTextSize(3);
    
    // Color Logic
    if(data.volt > 11.0) tft.setTextColor(GREEN);
    else if(data.volt > 10.0) tft.setTextColor(YELLOW);
    else tft.setTextColor(RED);
    
    tft.setCursor(20, 90);
    tft.print(data.volt, 1);
    old_data.volt = data.volt;
  }

  // --- TEMP ---
  if (abs(data.temp - old_data.temp) > 0.5) {
    tft.fillRect(20, 170, 130, 25, BLACK);
    tft.setTextSize(3);
    tft.setTextColor(WHITE);
    tft.setCursor(20, 170);
    tft.print(data.temp, 1);
    old_data.temp = data.temp;
  }

  // --- HUMIDITY ---
  if (abs(data.hum - old_data.hum) > 1.0) {
    tft.fillRect(20, 200, 130, 25, BLACK);
    tft.setTextSize(3);
    tft.setTextColor(WHITE);
    tft.setCursor(20, 200);
    tft.print(data.hum, 0);
    old_data.hum = data.hum;
  }

  // --- IMU ---
  drawAngle(240, 90, data.roll, &old_data.roll);
  drawAngle(240, 120, data.pitch, &old_data.pitch);
  drawAngle(240, 150, data.yaw, &old_data.yaw);
}

void drawAngle(int x, int y, float val, float *old_val) {
  if (abs(val - *old_val) > 1.0) {
    tft.fillRect(x, y, 130, 25, BLACK);
    tft.setTextSize(3);
    tft.setTextColor(MAGENTA);
    tft.setCursor(x, y);
    tft.print(val, 0);
    *old_val = val;
  }
}