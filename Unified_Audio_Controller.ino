#include <Wire.h>
#include <EEPROM.h>
#include <TDA7439.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>

// ============================
// Shared hardware config
// ============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define IR_RECEIVE_PIN D4

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

// ============================
// PT2323 / PT2258 config
// ============================
#define PT2323_ADDRESS  0x4A
#define PT2258_ADDRESS  0x44

#define IR_POWER         0x8041270C
#define IR_MUTE          0x8041A70D
#define IR_VOL_UP        0x8041A710
#define IR_VOL_DOWN      0x8041A711
#define IR_FL_UP         0x804127B0
#define IR_FL_DOWN       0x8041A7D2
#define IR_FR_UP         0x8041279F
#define IR_FR_DOWN       0x8041A70F
#define IR_SL_UP         0x8041275A
#define IR_SL_DOWN       0x8041A75B
#define IR_SR_UP         0x80412758
#define IR_SR_DOWN       0x8041A759
#define IR_CN_UP         0x80412754
#define IR_CN_DOWN       0x8041A783
#define IR_SUB_UP        0x80412726
#define IR_SUB_DOWN      0x8041A750
#define IR_SPEAKER_MODE  0x80412737
#define IR_SURROUND      0x8041A700
#define IR_RESET         0x8041270A

int power_state = 0;             // 0=Standby, 1=On
int input_sel = 2;               // fixed USB input
int mute_state = 0;
int surround_mode = 0;
int speaker_mode = 0;             // 0=5.1, 1=2.1
int channel_mute = 0;
int master_vol = 19;
int fl_vol = 0;
int fr_vol = 0;
int sl_vol = 0;
int sr_vol = 0;
int cn_vol = 0;
int sub_vol = 0;
int selected_channel = 0;
int display_timeout = 0;
int pt2258_mute = 0;

// ============================
// TDA7439 config
// ============================
TDA7439 tda;

#define EEPROM_VOL_L        0
#define EEPROM_VOL_R        1
#define EEPROM_BASS_L       2
#define EEPROM_BASS_R       3
#define EEPROM_MIDS_L       4
#define EEPROM_MIDS_R       5
#define EEPROM_TREB_L       6
#define EEPROM_TREB_R       7
#define EEPROM_BALANS_L     8
#define EEPROM_BALANS_R     9
#define EEPROM_GAIN3        10
#define EEPROM_SIZE         512

#define IR_TDA_VOL_UP       0x80412710
#define IR_TDA_VOL_DOWN     0x80412711
#define IR_TDA_BASS_UP      0x8041A721
#define IR_TDA_BASS_DOWN    0x8041A720
#define IR_TDA_MID_UP       0x8041A758
#define IR_TDA_MID_DOWN     0x80412759
#define IR_TDA_TREB_UP      0x8041275B
#define IR_TDA_TREB_DOWN    0x8041275A
#define IR_TDA_BAL_LEFT     0x8041A7D2
#define IR_TDA_BAL_RIGHT    0x8041A70F
#define IR_TDA_GAIN_UP      0x8041272A
#define IR_TDA_GAIN_DOWN    0x8041272B

int vol_L = 24, vol_R = 24;
int bass_L = 0, bass_R = 0;
int mids_L = 0, mids_R = 0;
int treb_L = 0, treb_R = 0;
int balans_L = 0, balans_R = 0;
int in_sel = 3;
int gain = 0;
int gain3 = 0;

byte menu = 0;
unsigned long time1 = 0;
unsigned long lastIRTime = 0;
const unsigned long IR_DISPLAY_TIMEOUT = 8000;
bool showIRControl = false;
bool showVUMeter = true;

// ============================
// FFT / spectrum config
// ============================
#define AUDIO_INPUT_PIN A0
#define SAMPLES 128
#define SAMPLING_FREQUENCY 40000
#define NUM_BARS 8

arduinoFFT FFT = arduinoFFT();
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned int samplingPeriodUs;
int barHeights[NUM_BARS] = {0};
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_REFRESH_INTERVAL = 50;

// ============================
// System mode
// ============================
int system_mode = 0;  // 0 = 5.1 / PT2323+PT2258, 1 = TDA7439 tone

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n========================================");
  Serial.println(" Unified Audio Amplifier Controller");
  Serial.println(" PT2323 + PT2258 + TDA7439 + IR + OLED");
  Serial.println("========================================\n");

  EEPROM.begin(EEPROM_SIZE);

  Wire.begin(D2, D1);
  Wire.setClock(100000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("ERROR: OLED initialization failed!");
    while (1) delay(100);
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  irrecv.enableIRIn();
  Serial.println("IR receiver enabled");

  // Load stored values
  eeprom_read_51();
  loadSettingsFromEEPROM();

  // Initial hardware setup
  startup_sequence_51();
  updateAudio();

  power_state = 0;
  input_sel = 2;
  system_mode = 0;

  samplingPeriodUs = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  Serial.println("Setup complete");
  Serial.println("Ready for IR control");
}

void loop() {
  handle_ir_remote();
  update_display();
  handle_display_timeout();
  update_eeprom_periodically();

  if (system_mode == 1) {
    collectSamples();
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    updateBars();

    if (showIRControl && (millis() - lastIRTime > IR_DISPLAY_TIMEOUT)) {
      showIRControl = false;
      showVUMeter = true;
      menu = 0;
    }
  }

  if (millis() - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL) {
    lastDisplayUpdate = millis();
    updateOLEDDisplay();
  }

  if (millis() - time1 > 60000) {
    saveSettingsToEEPROM();
    time1 = millis();
  }

  delay(5);
}

// ============================
// Unified IR handler
// ============================
void handle_ir_remote() {
  if (!irrecv.decode(&results)) {
    return;
  }

  uint32_t ir_cmd = results.value;
  lastIRTime = millis();
  display_timeout = 0;

  Serial.print("IR Command: 0x");
  Serial.println(ir_cmd, HEX);

  // Global power key always active
  if (ir_cmd == IR_POWER) {
    power_state = (power_state == 0) ? 1 : 0;
    if (power_state == 1) {
      power_up_51();
      updateAudio();
    } else {
      power_down_51();
    }
    irrecv.resume();
    return;
  }

  // Global mute key affects all active sections
  if (ir_cmd == IR_MUTE) {
    if (system_mode == 0) {
      mute_state = (mute_state == 0) ? 1 : 0;
      set_mute_51();
    } else {
      // TDA mute is represented by volume 0
      vol_L = constrain(vol_L, 0, 48);
      vol_R = constrain(vol_R, 0, 48);
      if (vol_L == 0 && vol_R == 0) {
        vol_L = 24;
        vol_R = 24;
      } else {
        vol_L = 0;
        vol_R = 0;
      }
      updateAudio();
    }
    irrecv.resume();
    return;
  }

  // If system is off, ignore all other commands
  if (power_state == 0) {
    irrecv.resume();
    return;
  }

  // Mode switch: use 5.1 speaker button as a system toggle
  if (ir_cmd == IR_SPEAKER_MODE) {
    system_mode = (system_mode == 0) ? 1 : 0;
    showIRControl = false;
    showVUMeter = (system_mode == 1);
    Serial.print("[MODE] ");
    Serial.println(system_mode == 0 ? "5.1 system" : "TDA tone system");
    irrecv.resume();
    return;
  }

  // PT2323/PT2258 5.1 control keys
  if (system_mode == 0) {
    handle_ir_remote_51(ir_cmd);
    irrecv.resume();
    return;
  }

  // TDA7439 control keys
  handle_ir_remote_tda(ir_cmd);
  irrecv.resume();
}

// ============================
// 5.1 remote functions
// ============================
void handle_ir_remote_51(uint32_t ir_cmd) {
  if (ir_cmd == IR_VOL_UP) {
    master_vol++;
    if (master_vol > 39) master_vol = 39;
    selected_channel = 0;
    update_all_volumes();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_VOL_DOWN) {
    master_vol--;
    if (master_vol < 19) master_vol = 19;
    selected_channel = 0;
    update_all_volumes();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_FL_UP) {
    fl_vol++;
    if (fl_vol > 25) fl_vol = 25;
    selected_channel = 1;
    set_fl();
    display_timeout = 0;
    return;
  }
  if (ir_cmd == IR_FL_DOWN) {
    fl_vol--;
    if (fl_vol < 0) fl_vol = 0;
    selected_channel = 1;
    set_fl();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_FR_UP) {
    fr_vol++;
    if (fr_vol > 25) fr_vol = 25;
    selected_channel = 2;
    set_fr();
    display_timeout = 0;
    return;
  }
  if (ir_cmd == IR_FR_DOWN) {
    fr_vol--;
    if (fr_vol < 0) fr_vol = 0;
    selected_channel = 2;
    set_fr();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_SL_UP && speaker_mode == 0) {
    sl_vol++;
    if (sl_vol > 25) sl_vol = 25;
    selected_channel = 3;
    set_sl();
    display_timeout = 0;
    return;
  }
  if (ir_cmd == IR_SL_DOWN && speaker_mode == 0) {
    sl_vol--;
    if (sl_vol < 0) sl_vol = 0;
    selected_channel = 3;
    set_sl();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_SR_UP && speaker_mode == 0) {
    sr_vol++;
    if (sr_vol > 25) sr_vol = 25;
    selected_channel = 4;
    set_sr();
    display_timeout = 0;
    return;
  }
  if (ir_cmd == IR_SR_DOWN && speaker_mode == 0) {
    sr_vol--;
    if (sr_vol < 0) sr_vol = 0;
    selected_channel = 4;
    set_sr();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_CN_UP && speaker_mode == 0) {
    cn_vol++;
    if (cn_vol > 25) cn_vol = 25;
    selected_channel = 5;
    set_cn();
    display_timeout = 0;
    return;
  }
  if (ir_cmd == IR_CN_DOWN && speaker_mode == 0) {
    cn_vol--;
    if (cn_vol < 0) cn_vol = 0;
    selected_channel = 5;
    set_cn();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_SUB_UP) {
    sub_vol++;
    if (sub_vol > 25) sub_vol = 25;
    selected_channel = 6;
    set_sub();
    display_timeout = 0;
    return;
  }
  if (ir_cmd == IR_SUB_DOWN) {
    sub_vol--;
    if (sub_vol < 0) sub_vol = 0;
    selected_channel = 6;
    set_sub();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_SURROUND) {
    surround_mode = (surround_mode == 0) ? 1 : 0;
    set_surround();
    display_timeout = 0;
    return;
  }

  if (ir_cmd == IR_RESET) {
    perform_reset();
    return;
  }
}

// ============================
// TDA7439 remote functions
// ============================
void handle_ir_remote_tda(uint32_t code) {
  if (code == IR_TDA_VOL_UP) {
    handleIRVolumeUp();
  }
  else if (code == IR_TDA_VOL_DOWN) {
    handleIRVolumeDown();
  }
  else if (code == IR_TDA_BASS_UP) {
    handleIRBassUp();
  }
  else if (code == IR_TDA_BASS_DOWN) {
    handleIRBassDown();
  }
  else if (code == IR_TDA_MID_UP) {
    handleIRMidsUp();
  }
  else if (code == IR_TDA_MID_DOWN) {
    handleIRMidsDown();
  }
  else if (code == IR_TDA_TREB_UP) {
    handleIRTrebleUp();
  }
  else if (code == IR_TDA_TREB_DOWN) {
    handleIRTrebleDown();
  }
  else if (code == IR_TDA_BAL_LEFT) {
    handleIRBalanceLeft();
  }
  else if (code == IR_TDA_BAL_RIGHT) {
    handleIRBalanceRight();
  }
  else if (code == IR_TDA_GAIN_UP) {
    handleIRGainUp();
  }
  else if (code == IR_TDA_GAIN_DOWN) {
    handleIRGainDown();
  }
}

// ============================
// 5.1 PT2323 / PT2258 implementation
// ============================
void power_up_51() {
  Serial.println("[POWER] Turning ON");

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20, 10);
  display.println("POWERING");
  display.setCursor(30, 30);
  display.println("ON...");
  display.display();
  delay(500);

  mute_state = 0;
  pt2258_mute = 0;
  selected_channel = 0;
  input_sel = 2;

  set_pt2258_mute();
  set_mute_51();
  set_input();
  set_surround();
  pt2258_init();
  update_all_volumes();

  delay(300);
  display_timeout = 0;
}

void power_down_51() {
  Serial.println("[POWER] Turning OFF - Standby");
  mute_state = 1;
  set_mute_51();
  display_timeout = 0;
  delay(100);
}

void set_input() {
  byte cmd = 0b11000111;
  pt2323_send(cmd);
  input_sel = 2;
  Serial.println("[INPUT] Fixed to USB");
}

void set_surround() {
  byte cmd = (surround_mode == 0) ? 0b11010000 : 0b11010001;
  pt2323_send(cmd);
  Serial.print("[SURROUND] Mode: ");
  Serial.println(surround_mode == 0 ? "ON" : "OFF");
}

void set_mute_51() {
  byte cmd = mute_state ? 0b11111111 : 0b11111110;
  pt2323_send(cmd);

  if (mute_state) {
    pt2258_mute = 1;
  } else {
    pt2258_mute = 0;
  }

  set_pt2258_mute();
  Serial.print("[MUTE] State: ");
  Serial.println(mute_state ? "MUTED" : "UNMUTED");
}

void update_all_volumes() {
  set_master_volume();
  set_fl();
  set_fr();
  set_sl();
  set_sr();
  set_cn();
  set_sub();
}

void set_master_volume() {
  if (master_vol > 39) master_vol = 39;
  if (master_vol < 19) master_vol = 19;

  pt2258_mute = (master_vol == 19) ? 1 : 0;
  set_pt2258_mute();

  Serial.print("[MASTER VOL] ");
  Serial.print(master_vol - 19);
  Serial.println(" dB");
}

void set_fl() {
  if (fl_vol > 25) fl_vol = 25;
  if (fl_vol < 0) fl_vol = 0;

  int fl;
  if (fl_vol == 0) {
    fl = 39 + fl_vol;
  } else {
    fl = master_vol + fl_vol;
  }
  int c = (fl_vol == 0) ? (113 - fl) : (65 - fl);
  byte a = c / 10;
  byte b = c - a * 10;
  pt2258_send(0b10100000 + a, 0b10110000 + b);
  Serial.print("[FL] ");
  Serial.print(fl_vol);
  Serial.println(" dB");
}

void set_fr() {
  if (fr_vol > 25) fr_vol = 25;
  if (fr_vol < 0) fr_vol = 0;

  int fr;
  if (fr_vol == 0) {
    fr = 39 + fr_vol;
  } else {
    fr = master_vol + fr_vol;
  }
  int c = (fr_vol == 0) ? (113 - fr) : (65 - fr);
  byte a = c / 10;
  byte b = c - a * 10;
  pt2258_send(0b01100000 + a, 0b01110000 + b);
  Serial.print("[FR] ");
  Serial.print(fr_vol);
  Serial.println(" dB");
}

void set_sl() {
  if (sl_vol > 25) sl_vol = 25;
  if (sl_vol < 0) sl_vol = 0;

  int sl;
  if (sl_vol == 0) {
    sl = 39 + sl_vol;
  } else {
    sl = master_vol + sl_vol;
  }

  int c = (sl_vol == 0) ? (113 - sl) : (65 - sl);
  byte a = c / 10;
  byte b = c - a * 10;
  pt2258_send(0b01000000 + a, 0b01010000 + b);

  channel_mute = (speaker_mode == 1) ? 1 : 0;
  byte x = (channel_mute == 0) ? 0b11111000 : 0b11111001;
  pt2323_send(x);

  Serial.print("[SL] ");
  Serial.print(sl_vol);
  Serial.println(" dB");
}

void set_sr() {
  if (sr_vol > 25) sr_vol = 25;
  if (sr_vol < 0) sr_vol = 0;

  int sr;
  if (sr_vol == 0) {
    sr = 39 + sr_vol;
  } else {
    sr = master_vol + sr_vol;
  }

  int c = (sr_vol == 0) ? (113 - sr) : (65 - sr);
  byte a = c / 10;
  byte b = c - a * 10;
  pt2258_send(0b10000000 + a, 0b10010000 + b);

  channel_mute = (speaker_mode == 1) ? 1 : 0;
  byte x = (channel_mute == 0) ? 0b11111010 : 0b11111011;
  pt2323_send(x);

  Serial.print("[SR] ");
  Serial.print(sr_vol);
  Serial.println(" dB");
}

void set_cn() {
  if (cn_vol > 25) cn_vol = 25;
  if (cn_vol < 0) cn_vol = 0;

  int cn;
  if (cn_vol == 0) {
    cn = 39 + cn_vol;
  } else {
    cn = master_vol + cn_vol;
  }

  int c = (cn_vol == 0) ? (90 - cn) : (65 - cn);
  byte a = c / 10;
  byte b = c - a * 10;
  pt2258_send(0b00100000 + a, 0b00110000 + b);

  channel_mute = (speaker_mode == 1) ? 1 : 0;
  byte x = (channel_mute == 0) ? 0b11110100 : 0b11110101;
  pt2323_send(x);

  Serial.print("[CENTER] ");
  Serial.print(cn_vol);
  Serial.println(" dB");
}

void set_sub() {
  if (sub_vol > 25) sub_vol = 25;
  if (sub_vol < 0) sub_vol = 0;

  int sub;
  if (sub_vol == 0) {
    sub = 16 + sub_vol;
  } else {
    sub = master_vol + sub_vol;
  }

  int c = (sub_vol == 0) ? (90 - sub) : (65 - sub);
  byte a = c / 10;
  byte b = c - a * 10;
  pt2258_send(0b00000000 + a, 0b00010000 + b);
  Serial.print("[SUB] ");
  Serial.print(sub_vol);
  Serial.println(" dB");
}

void set_speaker_mode_51() {
  channel_mute = (speaker_mode == 1) ? 1 : 0;
  set_sl();
  set_sr();
  set_cn();
  Serial.print("[SPEAKER MODE] ");
  Serial.println(speaker_mode == 0 ? "5.1" : "2.1");
}

void set_pt2258_mute() {
  Wire.beginTransmission(PT2258_ADDRESS);
  Wire.write(pt2258_mute ? 0b11111001 : 0b11111000);
  Wire.endTransmission();
}

void pt2323_send(byte data) {
  Wire.beginTransmission(PT2323_ADDRESS);
  Wire.write(data);
  Wire.endTransmission();
}

void pt2258_init() {
  Wire.beginTransmission(PT2258_ADDRESS);
  Wire.write(0b11000000);
  Wire.endTransmission();
}

void pt2258_send(byte data1, byte data2) {
  Wire.beginTransmission(PT2258_ADDRESS);
  Wire.write(data1);
  Wire.write(data2);
  Wire.endTransmission();
}

void perform_reset() {
  Serial.println("[RESET] Performing system reset...");
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(15, 10);
  display.println("RESETTING");
  display.setCursor(25, 30);
  display.println("SYSTEM");
  display.display();
  delay(1000);
  ESP.restart();
}

void handle_display_timeout() {
  if (power_state == 1 && mute_state == 0) {
    display_timeout++;
    if (display_timeout > 240) {
      display_timeout = 0;
    }
  }
}

void startup_sequence_51() {
  pt2258_mute = 1;
  set_pt2258_mute();

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Dolby");
  display.setCursor(0, 20);
  display.println("Atmos");
  display.display();
  delay(800);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("5.1");
  display.setCursor(0, 20);
  display.println("DTS");
  display.display();
  delay(800);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(20, 10);
  display.println("STANDBY MODE");
  display.setCursor(10, 25);
  display.println("Use remote to");
  display.setCursor(15, 35);
  display.println("power on");
  display.display();
  delay(1500);

  display.clearDisplay();
  input_sel = 2;
  set_input();
  set_surround();
  pt2258_init();
}

void eeprom_read_51() {
  input_sel = 2;
  master_vol = EEPROM.read(1);
  fl_vol = EEPROM.read(2) - 10;
  fr_vol = EEPROM.read(3) - 10;
  sl_vol = EEPROM.read(4) - 10;
  sr_vol = EEPROM.read(5) - 10;
  cn_vol = EEPROM.read(6) - 10;
  sub_vol = EEPROM.read(7) - 10;
  surround_mode = EEPROM.read(8);
  speaker_mode = EEPROM.read(9);

  if (master_vol < 19) master_vol = 19;
  if (master_vol > 39) master_vol = 39;
  if (speaker_mode > 1) speaker_mode = 0;
  if (surround_mode > 1) surround_mode = 0;

  Serial.println("[EEPROM] 5.1 settings loaded");
}

void update_eeprom_periodically() {
  static unsigned long last_update = 0;
  if (millis() - last_update > 5000) {
    EEPROM.write(1, master_vol);
    EEPROM.write(2, fl_vol + 10);
    EEPROM.write(3, fr_vol + 10);
    EEPROM.write(4, sl_vol + 10);
    EEPROM.write(5, sr_vol + 10);
    EEPROM.write(6, cn_vol + 10);
    EEPROM.write(7, sub_vol + 10);
    EEPROM.write(8, surround_mode);
    EEPROM.write(9, speaker_mode);
    EEPROM.commit();
    last_update = millis();
  }
}

// ============================
// OLED display for 5.1 system
// ============================
void update_display() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (power_state == 0) {
    display.setCursor(20, 10);
    display.setTextSize(2);
    display.println("STANDBY");
    display.setTextSize(1);
    display.setCursor(15, 35);
    display.println("Press POWER on");
    display.setCursor(20, 45);
    display.println("Remote to start");
    display.display();
    return;
  }

  if (mute_state == 1 && system_mode == 0) {
    display.setCursor(40, 25);
    display.setTextSize(2);
    display.println("MUTE");
    display.display();
    return;
  }

  if (system_mode == 0) {
    display_51_screen();
  } else {
    if (showVUMeter) {
      drawSpectrum();
    } else if (showIRControl) {
      displayIRControlScreen();
    } else {
      display_tone_screen();
    }
  }
  display.display();
}

void display_51_screen() {
  String mode_name = (speaker_mode == 0) ? "5.1" : "2.1";

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("IN: USB");

  display.setCursor(80, 0);
  display.print("MODE:");
  display.print(mode_name);

  String ch_name;
  int vol_value = 0;

  switch (selected_channel) {
    case 0: ch_name = "MASTER"; vol_value = master_vol - 19; break;
    case 1: ch_name = "FL"; vol_value = fl_vol; break;
    case 2: ch_name = "FR"; vol_value = fr_vol; break;
    case 3: ch_name = "SL"; vol_value = sl_vol; break;
    case 4: ch_name = "SR"; vol_value = sr_vol; break;
    case 5: ch_name = "CENTER"; vol_value = cn_vol; break;
    case 6: ch_name = "SUBWOOFER"; vol_value = sub_vol; break;
    default: ch_name = "MASTER"; vol_value = master_vol - 19; break;
  }

  display.setCursor(0, 15);
  display.print("Channel: ");
  display.println(ch_name);

  display.setCursor(0, 25);
  display.print("Level: ");
  display.print(vol_value);
  display.println(" dB");

  display.drawRect(10, 37, 108, 18, WHITE);
  int bar_width = map(vol_value, -25, 25, 0, 104);
  display.fillRect(12, 39, bar_width, 14, WHITE);

  display.setCursor(45, 42);
  display.print(vol_value);

  display.setCursor(0, 56);
  display.println("Use remote to adjust");
}

// ============================
// TDA7439 functions
// ============================
void updateAudio() {
  tda.setInput(3);
  tda.inputGain(gain);

  tda.setVolume(vol_L);
  tda.setVolume(vol_R);

  tda.setSnd(bass_L, 1);
  tda.setSnd(mids_L, 2);
  tda.setSnd(treb_L, 3);

  tda.setSnd(bass_R, 1);
  tda.setSnd(mids_R, 2);
  tda.setSnd(treb_R, 3);

  tda.spkAtt(4 + balans_L);
  tda.spkAtt(4 - balans_R);
}

void handleIRVolumeUp() {
  vol_L = constrain(vol_L + 1, 0, 48);
  vol_R = constrain(vol_R + 1, 0, 48);
  updateAudio();
  time1 = millis();
  menu = 1;
  showIRControl = true;
  showVUMeter = false;
  Serial.print("Volume UP - L:");
  Serial.print(-48 + vol_L);
  Serial.print(" dB R:");
  Serial.println(-48 + vol_R);
}

void handleIRVolumeDown() {
  vol_L = constrain(vol_L - 1, 0, 48);
  vol_R = constrain(vol_R - 1, 0, 48);
  updateAudio();
  time1 = millis();
  menu = 1;
  showIRControl = true;
  showVUMeter = false;
  Serial.print("Volume DOWN - L:");
  Serial.print(-48 + vol_L);
  Serial.print(" dB R:");
  Serial.println(-48 + vol_R);
}

void handleIRBassUp() {
  bass_L = constrain(bass_L + 1, -7, 7);
  bass_R = constrain(bass_R + 1, -7, 7);
  updateAudio();
  time1 = millis();
  menu = 2;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRBassDown() {
  bass_L = constrain(bass_L - 1, -7, 7);
  bass_R = constrain(bass_R - 1, -7, 7);
  updateAudio();
  time1 = millis();
  menu = 2;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRMidsUp() {
  mids_L = constrain(mids_L + 1, -7, 7);
  mids_R = constrain(mids_R + 1, -7, 7);
  updateAudio();
  time1 = millis();
  menu = 3;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRMidsDown() {
  mids_L = constrain(mids_L - 1, -7, 7);
  mids_R = constrain(mids_R - 1, -7, 7);
  updateAudio();
  time1 = millis();
  menu = 3;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRTrebleUp() {
  treb_L = constrain(treb_L + 1, -7, 7);
  treb_R = constrain(treb_R + 1, -7, 7);
  updateAudio();
  time1 = millis();
  menu = 4;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRTrebleDown() {
  treb_L = constrain(treb_L - 1, -7, 7);
  treb_R = constrain(treb_R - 1, -7, 7);
  updateAudio();
  time1 = millis();
  menu = 4;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRBalanceLeft() {
  balans_L = constrain(balans_L + 1, -4, 4);
  balans_R = constrain(balans_R + 1, -4, 4);
  updateAudio();
  time1 = millis();
  menu = 5;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRBalanceRight() {
  balans_L = constrain(balans_L - 1, -4, 4);
  balans_R = constrain(balans_R - 1, -4, 4);
  updateAudio();
  time1 = millis();
  menu = 5;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRGainUp() {
  gain3 = constrain(gain3 + 1, 0, 15);
  gain = gain3;
  updateAudio();
  time1 = millis();
  menu = 6;
  showIRControl = true;
  showVUMeter = false;
}

void handleIRGainDown() {
  gain3 = constrain(gain3 - 1, 0, 15);
  gain = gain3;
  updateAudio();
  time1 = millis();
  menu = 6;
  showIRControl = true;
  showVUMeter = false;
}

void display_tone_screen() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("MODE: TONE");
  display.setCursor(85, 0);
  display.print("IN3");

  display.setCursor(0, 12);
  display.print("VOL:");
  display.print(-48 + vol_L);
  display.print("dB");

  display.setCursor(0, 24);
  display.print("B: ");
  display.print(bass_L * 2);
  display.print("  M: ");
  display.print(mids_L * 2);
  display.print("  T: ");
  display.print(treb_L * 2);

  display.setCursor(0, 36);
  display.print("BAL:");
  display.print(balans_L);
  display.print(" GAIN:");
  display.print(gain3);

  display.setCursor(0, 52);
  display.println("Use IR to adjust tone");
}

void displayIRControlScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch (menu) {
    case 1: displayIRVolume(); break;
    case 2: displayIRBass(); break;
    case 3: displayIRMids(); break;
    case 4: displayIRTreble(); break;
    case 5: displayIRBalance(); break;
    case 6: displayIRGain(); break;
    default: display_tone_screen(); break;
  }

  display.display();
}

void displayIRVolume() {
  display.setCursor(15, 0);
  display.setTextSize(3);
  display.println("VOLUME");

  display.setTextSize(2);
  display.setCursor(20, 25);
  display.println("MASTER");

  display.setTextSize(4);
  display.setCursor(10, 40);
  display.print(-48 + vol_L);
  display.setTextSize(2);
  display.setCursor(90, 45);
  display.println("dB");

  drawVolumeBar(10, 55, vol_L, 48);
}

void displayIRBass() {
  display.setCursor(10, 0);
  display.setTextSize(3);
  display.println("BASS");

  display.setTextSize(2);
  display.setCursor(30, 27);
  display.println("MASTER");

  display.setTextSize(4);
  display.setCursor(20, 42);
  display.print(bass_L * 2);
  display.setTextSize(2);
  display.setCursor(95, 48);
  display.println("dB");

  drawEQBar(10, 60, bass_L);
}

void displayIRMids() {
  display.setCursor(15, 0);
  display.setTextSize(3);
  display.println("MIDS");

  display.setTextSize(2);
  display.setCursor(30, 27);
  display.println("MASTER");

  display.setTextSize(4);
  display.setCursor(20, 42);
  display.print(mids_L * 2);
  display.setTextSize(2);
  display.setCursor(95, 48);
  display.println("dB");

  drawEQBar(10, 60, mids_L);
}

void displayIRTreble() {
  display.setCursor(10, 0);
  display.setTextSize(3);
  display.println("TREBLE");

  display.setTextSize(2);
  display.setCursor(30, 27);
  display.println("MASTER");

  display.setTextSize(4);
  display.setCursor(20, 42);
  display.print(treb_L * 2);
  display.setTextSize(2);
  display.setCursor(95, 48);
  display.println("dB");

  drawEQBar(10, 60, treb_L);
}

void displayIRBalance() {
  display.setCursor(10, 0);
  display.setTextSize(3);
  display.println("BALANCE");

  display.setTextSize(2);
  display.setCursor(30, 27);
  display.println("MASTER");

  display.setTextSize(4);
  display.setCursor(40, 42);
  display.print(balans_L);

  drawBalanceSlider(10, 60, balans_L);
}

void displayIRGain() {
  display.setCursor(20, 0);
  display.setTextSize(3);
  display.println("GAIN");

  display.setTextSize(2);
  display.setCursor(30, 28);
  display.print("In: 3 (Fixed)");

  display.setCursor(20, 44);
  display.print("Gain: ");
  display.print(gain3);
  display.println("/15");

  display.drawRect(15, 56, 100, 8, SSD1306_WHITE);
  int gainWidth = (gain3 * 96) / 15;
  if (gainWidth > 0) {
    display.fillRect(16, 57, gainWidth, 6, SSD1306_WHITE);
  }
}

void drawVolumeBar(int x, int y, int value, int maxValue) {
  int barWidth = (value * 90) / maxValue;
  display.drawRect(x, y, 90, 8, SSD1306_WHITE);
  if (barWidth > 0) {
    display.fillRect(x + 1, y + 1, barWidth - 2, 6, SSD1306_WHITE);
  }
}

void drawEQBar(int x, int y, int value) {
  display.drawRect(x, y - 2, 100, 4, SSD1306_WHITE);
  int sliderPos = x + 50 + (value * 7);
  display.fillRect(sliderPos, y - 3, 4, 6, SSD1306_WHITE);
}

void drawBalanceSlider(int x, int y, int value) {
  display.drawRect(x, y - 2, 100, 4, SSD1306_WHITE);
  display.drawLine(x + 50, y - 4, x + 50, y + 2, SSD1306_WHITE);
  int sliderPos = x + 50 + (value * 12);
  display.fillRect(sliderPos, y - 3, 4, 6, SSD1306_WHITE);
}

// ============================
// Spectrum analyzer for TDA7439 mode
// ============================
void collectSamples() {
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long microseconds = micros();
    vReal[i] = analogRead(AUDIO_INPUT_PIN);
    vImag[i] = 0;
    while (micros() - microseconds < samplingPeriodUs) {
    }
  }
}

void updateBars() {
  for (int i = 0; i < NUM_BARS; i++) {
    int bin = (i + 2) * 2;
    if (bin >= SAMPLES / 2) bin = SAMPLES / 2 - 1;

    int height = (int)(vReal[bin] / 50);
    barHeights[i] = (barHeights[i] * 3 + height) / 4;

    if (barHeights[i] > SCREEN_HEIGHT - 14) {
      barHeights[i] = SCREEN_HEIGHT - 14;
    }
  }
}

void drawSpectrum() {
  display.clearDisplay();

  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(2, 2);
  display.print("VU METER | IN3 | VOL:");
  display.print(-48 + vol_L);
  display.print("dB");
  display.setTextColor(SSD1306_WHITE);

  int barWidth = (SCREEN_WIDTH - 2) / NUM_BARS;

  for (int i = 0; i < NUM_BARS; i++) {
    int x = 1 + (i * barWidth);
    int barHeight = barHeights[i];
    int y = SCREEN_HEIGHT - barHeight;
    display.fillRect(x, y, barWidth - 1, barHeight, SSD1306_WHITE);
  }

  display.drawLine(0, SCREEN_HEIGHT - 1, SCREEN_WIDTH, SCREEN_HEIGHT - 1, SSD1306_WHITE);
}

void updateOLEDDisplay() {
  if (system_mode == 0) {
    if (power_state == 0) {
      display.setTextSize(2);
      display.setCursor(20, 10);
      display.println("STANDBY");
      display.display();
    } else {
      display_51_screen();
      display.display();
    }
    return;
  }

  if (showVUMeter) {
    drawSpectrum();
  } else if (showIRControl) {
    displayIRControlScreen();
  } else {
    display_tone_screen();
  }

  display.display();
}

void saveSettingsToEEPROM() {
  EEPROM.write(EEPROM_VOL_L, vol_L);
  EEPROM.write(EEPROM_VOL_R, vol_R);
  EEPROM.write(EEPROM_BASS_L, bass_L + 7);
  EEPROM.write(EEPROM_BASS_R, bass_R + 7);
  EEPROM.write(EEPROM_MIDS_L, mids_L + 7);
  EEPROM.write(EEPROM_MIDS_R, mids_R + 7);
  EEPROM.write(EEPROM_TREB_L, treb_L + 7);
  EEPROM.write(EEPROM_TREB_R, treb_R + 7);
  EEPROM.write(EEPROM_BALANS_L, balans_L + 4);
  EEPROM.write(EEPROM_BALANS_R, balans_R + 4);
  EEPROM.write(EEPROM_GAIN3, gain3);
  EEPROM.commit();
  Serial.println("✓ Settings saved to EEPROM");
}

void loadSettingsFromEEPROM() {
  vol_L = EEPROM.read(EEPROM_VOL_L);
  vol_R = EEPROM.read(EEPROM_VOL_R);
  bass_L = EEPROM.read(EEPROM_BASS_L) - 7;
  bass_R = EEPROM.read(EEPROM_BASS_R) - 7;
  mids_L = EEPROM.read(EEPROM_MIDS_L) - 7;
  mids_R = EEPROM.read(EEPROM_MIDS_R) - 7;
  treb_L = EEPROM.read(EEPROM_TREB_L) - 7;
  treb_R = EEPROM.read(EEPROM_TREB_R) - 7;
  balans_L = EEPROM.read(EEPROM_BALANS_L) - 4;
  balans_R = EEPROM.read(EEPROM_BALANS_R) - 4;
  gain3 = EEPROM.read(EEPROM_GAIN3);

  if (vol_L > 48) vol_L = 24;
  if (vol_R > 48) vol_R = 24;
  if (gain3 > 15) gain3 = 0;

  gain = gain3;
  in_sel = 3;

  Serial.println("✓ TDA settings loaded from EEPROM");
}
