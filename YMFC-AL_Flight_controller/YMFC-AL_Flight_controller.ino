/*
  YMFC-AL - FIXED VERSION
  - Corrected axis mapping: Pitch = X (nose up/down), Roll = Y (right side up/down)
  - Fixed gyro integration to use raw axis values before rate conversion
  - Stable angle readings with proper complementary filter
*/

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

const int EE_PID_START = 100;

float pid_p_gain_roll  = 1.3;
float pid_i_gain_roll  = 0.04;
float pid_d_gain_roll  = 18.0;
int   pid_max_roll     = 400;

float pid_p_gain_pitch = 1.3;
float pid_i_gain_pitch = 0.04;
float pid_d_gain_pitch = 18.0;
int   pid_max_pitch    = 400;

float pid_p_gain_yaw   = 4.0;
float pid_i_gain_yaw   = 0.02;
float pid_d_gain_yaw   = 0.0;
int   pid_max_yaw      = 400;

byte eeprom_data[36];
int receiver_input[5];
volatile byte last_channel_1,last_channel_2,last_channel_3,last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

int gyro_address = 0x68;
bool mpu_present = false;

int acc_axis[4], gyro_axis[4];
long acc_x, acc_y, acc_z;
double gyro_axis_cal[4] = {0,0,0,0};

float angle_roll = 0, angle_pitch = 0;
float angle_roll_acc = 0, angle_pitch_acc = 0;

float gyro_roll_input=0, gyro_pitch_input=0, gyro_yaw_input=0;
float pid_roll_setpoint=0, pid_pitch_setpoint=0, pid_yaw_setpoint=0;
float pid_i_mem_roll=0, pid_last_roll_d_error=0;
float pid_i_mem_pitch=0, pid_last_pitch_d_error=0;
float pid_i_mem_yaw=0, pid_last_yaw_d_error=0;
float pid_output_roll=0, pid_output_pitch=0, pid_output_yaw=0;

int esc_1=1000, esc_2=1000, esc_3=1000, esc_4=1000;
unsigned long loop_timer;

bool serialPrintAngles = false;
char serialBuf[80];
int serialPos = 0;

float angle_roll_smooth = 0, angle_pitch_smooth = 0;

void set_gyro_registers();
void gyro_signalen();
int convert_receiver_channel(byte function);
void calculate_pid();
void savePIDsToEEPROM();
void loadPIDsFromEEPROM();
void printPIDStatus();
void processSerialLine(const char *line);

void setup(){
  Serial.begin(57600);
  delay(50);
  Serial.println(F("\n=== YMFC-AL FIXED ==="));

  Wire.begin();
  TWBR = 12;

  for (int i=0;i<36;i++) eeprom_data[i] = EEPROM.read(i);

  bool found = false;
  for (int addrCandidate = 0; addrCandidate <= 1; ++addrCandidate) {
    int addr = (addrCandidate == 0) ? 0x68 : 0x69;
    Wire.beginTransmission(addr);
    Wire.write(0x75);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(addr,1);
      if (Wire.available()){
        Wire.read();
        mpu_present = true;
        gyro_address = addr;
        Serial.print(F("MPU at 0x")); Serial.println(addr, HEX);
        found = true; break;
      }
    }
  }
  if (!found) Serial.println(F("No MPU"));

  if (mpu_present) {
    set_gyro_registers();
    Serial.print(F("Calibrating... "));
    double sumX=0,sumY=0,sumZ=0;
    for (int i=0;i<300;i++){
      gyro_signalen();
      sumX += gyro_axis[1];
      sumY += gyro_axis[2];
      sumZ += gyro_axis[3];
      delay(2);
    }
    gyro_axis_cal[1] = sumX / 300.0;
    gyro_axis_cal[2] = sumY / 300.0;
    gyro_axis_cal[3] = sumZ / 300.0;
    Serial.println(F("done"));
  }

  loadPIDsFromEEPROM();
  printPIDStatus();

  DDRD |= B11110000;
  DDRB |= B00110000;
  digitalWrite(12, LOW);

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  for (int i=0;i<5;i++) receiver_input[i]=1500;
  loop_timer = micros();
}

void loop(){
  while (micros() - loop_timer < 4000);
  loop_timer = micros();

  receiver_input_channel_1 = convert_receiver_channel(1);
  receiver_input_channel_2 = convert_receiver_channel(2);
  receiver_input_channel_3 = convert_receiver_channel(3);
  receiver_input_channel_4 = convert_receiver_channel(4);

  gyro_signalen();

  if (mpu_present) {
    gyro_axis[1] -= (int)gyro_axis_cal[1];
    gyro_axis[2] -= (int)gyro_axis_cal[2];
    gyro_axis[3] -= (int)gyro_axis_cal[3];
  }

  int pitchIdx = 1;
  int rollIdx = 2;
  int yawIdx = 3;

  int gyro_pitch_raw = gyro_axis[pitchIdx];
  int gyro_roll_raw = gyro_axis[rollIdx];
  int gyro_yaw_raw = gyro_axis[yawIdx];

  // INVERT pitch so nose up = positive
  gyro_pitch_raw *= -1;
  
  if (eeprom_data[28] & 0b10000000) gyro_roll_raw *= -1;
  if (eeprom_data[30] & 0b10000000) gyro_yaw_raw *= -1;

  // INVERT acc_x so nose up = positive pitch
  acc_x = -acc_axis[pitchIdx];
  
  acc_y = acc_axis[rollIdx];
  if (eeprom_data[28] & 0b10000000) acc_y *= -1;
  
  acc_z = acc_axis[yawIdx];
  if (eeprom_data[30] & 0b10000000) acc_z *= -1;

  if (mpu_present) {
    angle_pitch += gyro_pitch_raw * 0.0000611;
    angle_roll  += gyro_roll_raw  * 0.0000611;

    angle_pitch -= angle_roll * sin(gyro_yaw_raw * 0.000001066);
    angle_roll  += angle_pitch * sin(gyro_yaw_raw * 0.000001066);

    long acc_tot = sqrt((double)acc_x*acc_x + (double)acc_y*acc_y + (double)acc_z*acc_z);
    if (acc_tot > 0) {
      angle_pitch_acc = asin((float)acc_x / acc_tot) * 57.296;
      angle_roll_acc  = asin((float)acc_y / acc_tot) * -57.296;
      
      // FASTER complementary filter (was 0.9996/0.0004, now 0.98/0.02 for quicker response)
      angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;
      angle_roll  = angle_roll  * 0.98 + angle_roll_acc  * 0.02;

      // MINIMAL smoothing for instant response (was 0.5/0.5, now 0.2/0.8)
      angle_pitch_smooth = angle_pitch_smooth * 0.2 + angle_pitch * 0.8;
      angle_roll_smooth  = angle_roll_smooth  * 0.2 + angle_roll  * 0.8;
    }
  } else {
    angle_pitch = angle_roll = 0;
    angle_pitch_smooth = angle_roll_smooth = 0;
  }

  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch_raw / 65.5) * 0.3);
  gyro_roll_input  = (gyro_roll_input  * 0.7) + ((gyro_roll_raw  / 65.5) * 0.3);
  gyro_yaw_input   = (gyro_yaw_input   * 0.7) + ((gyro_yaw_raw   / 65.5) * 0.3);

  pid_roll_setpoint = 0;
  if (receiver_input_channel_1 > 1508) pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if (receiver_input_channel_1 < 1492) pid_roll_setpoint = receiver_input_channel_1 - 1492;
  pid_roll_setpoint /= 3.0;

  pid_pitch_setpoint = 0;
  if (receiver_input_channel_2 > 1508) pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if (receiver_input_channel_2 < 1492) pid_pitch_setpoint = receiver_input_channel_2 - 1492;
  pid_pitch_setpoint /= 3.0;

  pid_yaw_setpoint = 0;
  if (receiver_input_channel_3 > 1050) {
    if (receiver_input_channel_4 > 1508) pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492) pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }

  calculate_pid();

  int throttle = receiver_input_channel_3;
  if (throttle > 1800) throttle = 1800;

  if (throttle > 1000) {
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

    int tmp = esc_2; esc_2 = esc_4; esc_4 = tmp;

    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;
    if (esc_1 > 2000) esc_1 = 2000;
    if (esc_2 > 2000) esc_2 = 2000;
    if (esc_3 > 2000) esc_3 = 2000;
    if (esc_4 > 2000) esc_4 = 2000;
  } else {
    esc_1 = esc_2 = esc_3 = esc_4 = 1000;
  }

  unsigned long startPulse = micros();
  PORTD |= B11110000;
  unsigned long t1 = startPulse + esc_1;
  unsigned long t2 = startPulse + esc_2;
  unsigned long t3 = startPulse + esc_3;
  unsigned long t4 = startPulse + esc_4;
  while (PORTD >= 16) {
    unsigned long tnow = micros();
    if (tnow >= t1) PORTD &= B11101111;
    if (tnow >= t2) PORTD &= B11011111;
    if (tnow >= t3) PORTD &= B10111111;
    if (tnow >= t4) PORTD &= B01111111;
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n' || serialPos >= (int)sizeof(serialBuf)-2) {
      serialBuf[serialPos] = 0;
      if (serialPos > 0) processSerialLine(serialBuf);
      serialPos = 0;
    } else {
      serialBuf[serialPos++] = c;
    }
  }

  if (serialPrintAngles) {
    Serial.print(F("Pitch: "));
    Serial.print(angle_pitch_smooth, 1);
    Serial.print(F("  Roll: "));
    Serial.println(angle_roll_smooth, 1);
  }
}

void processSerialLine(const char *line) {
  char cmd[80];
  int len = min((int)strlen(line), 78);
  for (int i=0;i<len;i++) cmd[i] = toupper((unsigned char)line[i]);
  cmd[len] = 0;

  if (strncmp(cmd, "PRINT ON", 8) == 0) {
    serialPrintAngles = true; Serial.println(F("Print ON")); return;
  }
  if (strncmp(cmd, "PRINT OFF", 9) == 0) {
    serialPrintAngles = false; Serial.println(F("Print OFF")); return;
  }
  if (strncmp(cmd, "SAVE", 4) == 0) { savePIDsToEEPROM(); Serial.println(F("Saved")); return; }
  if (strncmp(cmd, "LOAD", 4) == 0) { loadPIDsFromEEPROM(); printPIDStatus(); return; }
  if (strncmp(cmd, "PID", 3) == 0) { printPIDStatus(); return; }

  char which[4]; char axis; float val;
  int scanned = sscanf(line, " %3s %c %f", which, &axis, &val);
  if (scanned == 3) {
    char w = toupper(which[0]); axis = toupper(axis);
    if (w == 'P'){
      if (axis == 'R') pid_p_gain_roll = val;
      else if (axis == 'P') pid_p_gain_pitch = val;
      else if (axis == 'Y') pid_p_gain_yaw = val;
      printPIDStatus(); return;
    } else if (w == 'I'){
      if (axis == 'R') pid_i_gain_roll = val;
      else if (axis == 'P') pid_i_gain_pitch = val;
      else if (axis == 'Y') pid_i_gain_yaw = val;
      printPIDStatus(); return;
    } else if (w == 'D'){
      if (axis == 'R') pid_d_gain_roll = val;
      else if (axis == 'P') pid_d_gain_pitch = val;
      else if (axis == 'Y') pid_d_gain_yaw = val;
      printPIDStatus(); return;
    }
  }
}

void printPIDStatus() {
  Serial.print(F("R P=")); Serial.print(pid_p_gain_roll,2);
  Serial.print(F(" I=")); Serial.print(pid_i_gain_roll,4);
  Serial.print(F(" D=")); Serial.println(pid_d_gain_roll,1);

  Serial.print(F("P P=")); Serial.print(pid_p_gain_pitch,2);
  Serial.print(F(" I=")); Serial.print(pid_i_gain_pitch,4);
  Serial.print(F(" D=")); Serial.println(pid_d_gain_pitch,1);

  Serial.print(F("Y P=")); Serial.print(pid_p_gain_yaw,2);
  Serial.print(F(" I=")); Serial.print(pid_i_gain_yaw,4);
  Serial.print(F(" D=")); Serial.println(pid_d_gain_yaw,1);
}

void savePIDsToEEPROM(){
  int addr = EE_PID_START;
  EEPROM.write(addr++, 0xA5);
  union { float f; uint8_t b[4]; } u;
  u.f = pid_p_gain_roll; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
  u.f = pid_i_gain_roll; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
  u.f = pid_d_gain_roll; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
  u.f = pid_p_gain_pitch; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
  u.f = pid_i_gain_pitch; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
  u.f = pid_d_gain_pitch; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
  u.f = pid_p_gain_yaw; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
  u.f = pid_i_gain_yaw; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
  u.f = pid_d_gain_yaw; for (int i=0;i<4;i++) EEPROM.write(addr++, u.b[i]);
}

void loadPIDsFromEEPROM(){
  int addr = EE_PID_START;
  if (EEPROM.read(addr) != 0xA5) { Serial.println(F("Using defaults")); return; }
  addr++;
  union { float f; uint8_t b[4]; } u;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_p_gain_roll = u.f;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_i_gain_roll = u.f;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_d_gain_roll = u.f;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_p_gain_pitch = u.f;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_i_gain_pitch = u.f;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_d_gain_pitch = u.f;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_p_gain_yaw = u.f;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_i_gain_yaw = u.f;
  for (int i=0;i<4;i++) u.b[i] = EEPROM.read(addr++); pid_d_gain_yaw = u.f;
}

void calculate_pid(){
  float pid_error_temp;

  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < -pid_max_roll) pid_i_mem_roll = -pid_max_roll;
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if (pid_output_roll < -pid_max_roll) pid_output_roll = -pid_max_roll;
  pid_last_roll_d_error = pid_error_temp;

  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < -pid_max_pitch) pid_i_mem_pitch = -pid_max_pitch;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < -pid_max_pitch) pid_output_pitch = -pid_max_pitch;
  pid_last_pitch_d_error = pid_error_temp;

  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < -pid_max_yaw) pid_i_mem_yaw = -pid_max_yaw;
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < -pid_max_yaw) pid_output_yaw = -pid_max_yaw;
  pid_last_yaw_d_error = pid_error_temp;
}

int convert_receiver_channel(byte function){
  byte channel = 0;
  if (function+23 < 36) channel = eeprom_data[function + 23] & 0b00000111;
  byte reverse = (function+23 < 36 && (eeprom_data[function + 23] & 0b10000000)) ? 1 : 0;
  int actual = receiver_input[channel];
  int low = 1000; int center = 1500; int high = 2000;
  if (channel * 2 + 15 < 36) low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
  if (channel * 2 - 1 < 36 && channel*2-2 >=0) center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
  if (channel * 2 + 7 < 36) high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];

  if (center == 0) center = 1500;
  if (low == 0) low = 1000;
  if (high == 0) high = 2000;

  if(actual < center){
    if(actual < low) actual = low;
    long difference = ((long)(center - actual) * (long)500) / (center - low);
    if(reverse) return 1500 + difference;
    else return 1500 - difference;
  } else if(actual > center){
    if(actual > high) actual = high;
    long difference = ((long)(actual - center) * (long)500) / (high - center);
    if(reverse) return 1500 - difference;
    else return 1500 + difference;
  } else return 1500;
}

void gyro_signalen(){
  if (!mpu_present) {
    acc_axis[1] = acc_axis[2] = acc_axis[3] = 0;
    gyro_axis[1] = gyro_axis[2] = gyro_axis[3] = 0;
    return;
  }

  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 14);
  if (Wire.available() < 14) return;

  acc_axis[1] = Wire.read()<<8 | Wire.read();
  acc_axis[2] = Wire.read()<<8 | Wire.read();
  acc_axis[3] = Wire.read()<<8 | Wire.read();
  int temp = Wire.read()<<8 | Wire.read(); (void)temp;
  gyro_axis[1] = Wire.read()<<8 | Wire.read();
  gyro_axis[2] = Wire.read()<<8 | Wire.read();
  gyro_axis[3] = Wire.read()<<8 | Wire.read();
}

void set_gyro_registers(){
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B); Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C); Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A); Wire.write(0x03);
  Wire.endTransmission();
}

ISR(PCINT0_vect){
  current_time = micros();
  if(PINB & B00000001){ 
    if(last_channel_1 == 0){ last_channel_1 = 1; timer_1 = current_time; } 
  } else if(last_channel_1 == 1){ 
    last_channel_1 = 0; receiver_input[1] = current_time - timer_1; 
  }

  if(PINB & B00000010){ 
    if(last_channel_2 == 0){ last_channel_2 = 1; timer_2 = current_time; } 
  } else if(last_channel_2 == 1){ 
    last_channel_2 = 0; receiver_input[2] = current_time - timer_2; 
  }

  if(PINB & B00000100){ 
    if(last_channel_3 == 0){ last_channel_3 = 1; timer_3 = current_time; } 
  } else if(last_channel_3 == 1){ 
    last_channel_3 = 0; receiver_input[3] = current_time - timer_3; 
  }

  if(PINB & B00001000){ 
    if(last_channel_4 == 0){ last_channel_4 = 1; timer_4 = current_time; } 
  } else if(last_channel_4 == 1){ 
    last_channel_4 = 0; receiver_input[4] = current_time - timer_4; 
  }
}