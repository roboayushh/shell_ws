/*
 * ROBUST ARDUINO THRUSTER FIRMWARE
 * Logic: IDLE -> Recv ARM CMD -> ARMING (Wait 3s) -> ARMED (Listen for PWM)
 */

#include <Servo.h>

// --- CONFIGURATION ---
const int THRUSTER_PINS[4] = {9, 10, 11, 12};
const int RELAY_PIN = 7; // Power Relay/MOSFET for ESCs
const unsigned long WATCHDOG_TIMEOUT = 500; // ms (Disarms if Pi dies)
const unsigned long ESC_BOOT_DELAY = 3000;  // ms (Time for ESCs to init)
const int NEUTRAL_PWM = 1500;

// --- STATES ---
enum SystemState {
  STATE_IDLE,      // Waiting for Pi Handshake
  STATE_ARMING,    // Power ON, Waiting for ESCs to boot (Watchdog Ignored)
  STATE_ARMED      // Normal Operation (Watchdog Active)
};

SystemState current_state = STATE_IDLE;
Servo thrusters[4];
uint16_t current_pwm[4] = {NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM};

// --- BUFFERS ---
uint8_t rx_buffer[64];
int rx_index = 0;
unsigned long last_packet_time = 0;
unsigned long arming_start_time = 0;

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  disarm_system(); // Ensure we start safe

  for(int i=0; i<4; i++) {
    thrusters[i].attach(THRUSTER_PINS[i]);
    thrusters[i].writeMicroseconds(NEUTRAL_PWM);
  }
}

void loop() {
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
  unsigned long now = millis();

  switch (current_state) {
    case STATE_IDLE:
      // Spam BOOT_OK every 500ms so Pi knows we are here
      static unsigned long last_boot_msg = 0;
      if (now - last_boot_msg > 500) {
        Serial.println("BOOT_OK");
        last_boot_msg = now;
      }
      break;

    case STATE_ARMING:
      // Wait for ESCs to boot. DO NOT CHECK WATCHDOG HERE.
      // Pi is deliberately silent during this phase.
      if (now - arming_start_time > ESC_BOOT_DELAY) {
        current_state = STATE_ARMED;
        last_packet_time = millis(); // Reset watchdog
        Serial.println("ARM_CONFIRMED"); // Tell Pi to start sending data
      }
      break;

    case STATE_ARMED:
      // If we haven't received valid PWM data for 500ms, DISARM.
      if (now - last_packet_time > WATCHDOG_TIMEOUT) {
        disarm_system();
      }
      break;
  }

  // 3. Write Output
  for(int i=0; i<4; i++) {
    thrusters[i].writeMicroseconds(current_pwm[i]);
  }
}

// --- LOGIC ---

void disarm_system() {
  current_state = STATE_IDLE;
  digitalWrite(RELAY_PIN, LOW); // Cut Power
  for(int i=0; i<4; i++) current_pwm[i] = NEUTRAL_PWM;
  Serial.println("STATUS_DISARMED");
}

void arm_system() {
  // Only arm if we are currently idle
  if (current_state == STATE_IDLE) {
    current_state = STATE_ARMING;
    digitalWrite(RELAY_PIN, HIGH); // Power ON
    arming_start_time = millis();
    Serial.println("STATUS_ARMING_WAIT"); 
  }
}

// --- COBS & PACKET HANDLING ---

void process_frame(uint8_t* buf, int len) {
  uint8_t decoded[64];
  int decoded_len = cobs_decode(buf, len, decoded);

  if (decoded_len == 0) return;

  // A. Check for ARM Command (Magic Bytes: AA 55 AA 55)
  // Pi sends this in response to "BOOT_OK"
  if (decoded_len == 4) {
    if (decoded[0] == 0xAA && decoded[1] == 0x55 && 
        decoded[2] == 0xAA && decoded[3] == 0x55) {
      arm_system();
      return;
    }
  }

  // B. Check for PWM Data (8 bytes data + 1 byte checksum)
  // Only accept this if we are ARMED
  if (current_state == STATE_ARMED && decoded_len == 9) {
    uint8_t calc_sum = 0;
    for(int i=0; i<8; i++) calc_sum += decoded[i];

    if (calc_sum == decoded[8]) { // Checksum match
      last_packet_time = millis(); // Feed Watchdog
      
      // Unpack 4x uint16_t (Little Endian)
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