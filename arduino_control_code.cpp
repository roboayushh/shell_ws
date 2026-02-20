/*
 * ROBUST ARDUINO THRUSTER FIRMWARE + SENSOR TELEMETRY
 * Logic: IDLE -> Recv ARM CMD -> ARMING (Send Data) -> ARMED (Send Data + Listen PWM)
 */

#include <Servo.h>
#include <DHT.h>

// --- CONFIGURATION: THRUSTERS ---
const int THRUSTER_PINS[4] = {9, 10, 11, 12};
const int RELAY_PIN = 7;
const unsigned long WATCHDOG_TIMEOUT = 500; 
const unsigned long ESC_BOOT_DELAY = 3000;  
const int NEUTRAL_PWM = 1500;

// --- CONFIGURATION: SENSORS ---
#define DHTPIN A2
#define DHTTYPE DHT11
#define VOLT_PIN A0
const float DIVIDER_FACTOR = 3.0;
const unsigned long SENSOR_INTERVAL = 50; // Send data every 50ms (20Hz)

// --- OBJECTS ---
Servo thrusters[4];
DHT dht(DHTPIN, DHTTYPE);

// --- DATA STRUCTURES ---
struct __attribute__((packed)) SensorPacket {
  float temperature;
  float humidity;
  float voltage;
};

SensorPacket sensor_data;

// --- STATES ---
enum SystemState {
  STATE_IDLE,      // Waiting for Pi Handshake
  STATE_ARMING,    // Power ON, Booting ESCs, Sending Data
  STATE_ARMED      // Full Control, Watchdog Active, Sending Data
};

SystemState current_state = STATE_IDLE;
uint16_t current_pwm[4] = {NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM};

// --- BUFFERS ---
uint8_t rx_buffer[64];
int rx_index = 0;
unsigned long last_packet_time = 0;
unsigned long arming_start_time = 0;
unsigned long last_sensor_time = 0; // For non-blocking sensor read

// --- PROTOTYPES ---
void process_frame(uint8_t* buf, int len);
int cobs_decode(const uint8_t *ptr, int length, uint8_t *dst);
void arm_system();
void disarm_system();
void handle_sensors(); 

void setup() {
  Serial.begin(115200);
  
  // Thruster Init
  pinMode(RELAY_PIN, OUTPUT);
  disarm_system(); 
  for(int i=0; i<4; i++) {
    thrusters[i].attach(THRUSTER_PINS[i]);
    thrusters[i].writeMicroseconds(NEUTRAL_PWM);
  }

  // Sensor Init
  dht.begin();
  // Allow sensors to settle
  delay(500);
}

void loop() {
  unsigned long now = millis();

  // 1. Read Serial (Byte by Byte)
  while (Serial.available() > 0) {
    uint8_t byte_in = Serial.read();
    if (byte_in == 0x00) {
      process_frame(rx_buffer, rx_index);
      rx_index = 0; 
    } else {
      if (rx_index < 60) rx_buffer[rx_index++] = byte_in;
    }
  }

  // 2. State Machine
  switch (current_state) {
    case STATE_IDLE:
      static unsigned long last_boot_msg = 0;
      if (now - last_boot_msg > 500) {
        Serial.println("BOOT_OK");
        last_boot_msg = now;
      }
      break;

    case STATE_ARMING:
      if (now - arming_start_time > ESC_BOOT_DELAY) {
        current_state = STATE_ARMED;
        last_packet_time = millis(); 
        Serial.println("ARM_CONFIRMED"); 
      }
      break;

    case STATE_ARMED:
      if (now - last_packet_time > WATCHDOG_TIMEOUT) {
        disarm_system();
      }
      break;
  }

  // 3. Independent Task: Sensor Telemetry
  if (current_state != STATE_IDLE) {
    if (now - last_sensor_time > SENSOR_INTERVAL) {
      handle_sensors();
      last_sensor_time = now;
    }
  }

  // 4. Write Output
  for(int i=0; i<4; i++) {
    thrusters[i].writeMicroseconds(current_pwm[i]);
  }
}

// --- SENSOR LOGIC ---
void handle_sensors() {
  // A. Voltage & DHT
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) { h = 0; t = 0; }

  int rawVolt = analogRead(VOLT_PIN);
  float voltage = rawVolt * (5.0 / 1023.0) * DIVIDER_FACTOR;

  // B. Pack Data
  sensor_data.temperature = t;
  sensor_data.humidity = h;
  sensor_data.voltage = voltage;

  // C. Send Binary Packet (Header 0xAABB + Struct)
  Serial.write(0xAA); 
  Serial.write(0xBB);
  Serial.write((uint8_t*)&sensor_data, sizeof(sensor_data));
}

// --- LOGIC ---

void disarm_system() {
  current_state = STATE_IDLE;
  digitalWrite(RELAY_PIN, LOW);
  for(int i=0; i<4; i++) current_pwm[i] = NEUTRAL_PWM;
  Serial.println("STATUS_DISARMED");
}

void arm_system() {
  if (current_state == STATE_IDLE) {
    current_state = STATE_ARMING;
    digitalWrite(RELAY_PIN, HIGH);
    arming_start_time = millis();
    Serial.println("STATUS_ARMING_WAIT"); 
  }
}

// --- COBS & PACKET HANDLING ---

void process_frame(uint8_t* buf, int len) {
  uint8_t decoded[64];
  int decoded_len = cobs_decode(buf, len, decoded);

  if (decoded_len == 0) return;

  if (decoded_len == 4) {
    if (decoded[0] == 0xAA && decoded[1] == 0x55 && 
        decoded[2] == 0xAA && decoded[3] == 0x55) {
      arm_system();
      return;
    }
  }

  if (current_state == STATE_ARMED && decoded_len == 9) {
    uint8_t calc_sum = 0;
    for(int i=0; i<8; i++) calc_sum += decoded[i];

    if (calc_sum == decoded[8]) { 
      last_packet_time = millis(); 
      for(int i=0; i<4; i++) {
        uint16_t val = decoded[i*2] | (decoded[i*2+1] << 8);
        current_pwm[i] = constrain(val, 1100, 1900);
      }
    }
  }
}

int cobs_decode(const uint8_t *ptr, int length, uint8_t *dst) {
  const uint8_t *end = ptr + length;
  uint8_t *dst_start = dst;
  while (ptr < end) {
    int code = *ptr++;
    for (int i = 1; i < code; i++) {
      *dst++ = *ptr++;
      if (ptr > end) return 0;
    }
    if (code < 0xFF && ptr < end) *dst++ = 0;
  }
  return dst - dst_start;
}