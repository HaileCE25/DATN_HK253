/*
 * Copyright (c) 2023 by Philipp Hafkemeyer
 * Qorvo DW3000 library for Arduino
 *
 * This project is licensed under the GNU GPL v3.0 License.
 * you may not use this file except in compliance with the License.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include "Arduino.h"
#include "SPI.h"
//#include "stdlib.h"
#include "DW3000_tx.h"

DW3000Class DW3000;

// Bus SPI riêng cho DW3000 (HSPI / SPI3) - đồng nhất với bản RX (DW3000_rx.cpp).
// Keyfob không có thiết bị SPI khác nên đây chỉ là parity, không bắt buộc.
SPIClass s_uwbSPI(HSPI);

#define SLOT2

#define CHIP_SELECT_PIN 21

// Chân đã chốt cho board keyfob (ESP32-S3) - GIỐNG HỆT car. RST đổi 17->5 để
// khớp và tránh xung đột GPIO17.
#define MOSI_PIN 38
#define MISO_PIN 6
#define SCK_PIN 7


#ifdef SLOT2  //Definition for the Makerfabs DW3000 solution
#define SLOT 7
#else  //Definition for any other chip, e.g. the DWM3000EVB shield with the Arduino Uno
#define SLOT 0
#endif

#define RST_PIN 5
#define DEBUG_PIN 4  // NOTE: was 25 -- GPIO22-25 don't exist on ESP32-S3, caused gpio_set_level errors
#define DEBUG_OUTPUT 0  //Turn to 1 to get all reads, writes, etc. as info in the console

UwbMsgPrepollData_t prepoll_data;
UwbMsgPrepollData_t Txprepoll_data;
UwbMsgFinalData_t finaldata_data;
UwbMsgFinalData_t Txfinaldata_data;
UwbTimestampData_t timestamp_data;

bool debug_seven_segment = 0;
bool seven_segment_used = 0;

int led_status = 0;

int device_id = 0;

int antenna_delay = 0x4015;  //for calibration purposes; the tinier the number, the longer the ranging results

int DW3000Class::config[] = {  //CHAN; PREAMBLE LENGTH; PREAMBLE CODE; PAC; DATARATE; PHR_MODE; PHR_RATE;
  CHANNEL_5,
  PREAMBLE_64,
  9,  //same for tx and rx
  PAC8,
  DATARATE_6_8MB,
  PHR_MODE_STANDARD,
  PHR_RATE_850KB
};

// key[16] cotain 16 byte key(128 bit) for encryption
uint8_t URSKkey[32] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };

// plaintext[16] contain the text we need to encrypt
uint8_t URSKinput_low[16] = { 0x00u, 0x00u, 0x00u, 0x01u, 0x55u, 0x52u, 0x53u, 0x4Bu, 0x00u,
                              0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x00u };

uint8_t URSKinput_high[16] = { 0x00u, 0x00u, 0x00u, 0x02u, 0x55u, 0x52u, 0x53u, 0x4Bu,
                               0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x00u };

uint8_t mURSKinput[32];

uint8_t URSKinputKT_low[48] = { 0x00u, 0x00u, 0x00u, 0x01u, 0x55u, 0x52u, 0x53u, 0x4Bu, 0x5fu,
                                0x4bu, 0x54u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x00u };

uint8_t URSKinputKT_high[48] = { 0x00u, 0x00u, 0x00u, 0x02u, 0x55u, 0x52u, 0x53u, 0x4Bu, 0x5fu,
                                 0x4bu, 0x54u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x00u };

uint8_t URSKoutputKT[32];

uint8_t Saltinput[16] = { 0x00u, 0x00u, 0x00u, 0x01u, 0x53u, 0x41u, 0x4Cu,
                          0x54u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u };

uint8_t PaddedSaltinput[32] = { 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
                                0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
                                0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
                                0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };

uint8_t SaltedHashinput[16] = { 0x94u, 0x54u, 0x3eu, 0xa1u, 0xacu, 0x37u, 0x5fu,
                                0xf4u, 0xb5u, 0x54u, 0x47u, 0xa6u, 0x5du, 0x0du, 0x4cu, 0x5du };

uint32_t SaltedHashinput_u32[4] = { 0xa13e5494u, 0xf45f37acu, 0xa64754b5u, 0x5d4c0d5du };

uint8_t rangingconfig[16];
// cypher[16] stores the encrypted text
uint8_t mURSK_low[16];

uint8_t mURSK_high[16];

uint8_t ktURSK_low[16];

uint8_t ktURSK_high[16];

uint8_t dURSK[16];

uint8_t dURSK_real[16];

uint8_t dUDSK[16];

uint8_t STS_IV[16];

uint32_t dURSK_respond[4];

uint32_t STS_IV_respond[4];

uint32_t dURSK_final[4];

uint32_t STS_IV_final[4];

uint32_t dURSK_poll[4];

uint32_t STS_IV_poll[4];

uint8_t dURSKinput[32] = { 0x00u, 0x00u, 0x00u, 0x01u, 0x55u, 0x52u, 0x53u, 0x4Bu, 0x00u,
                           0x94u, 0x54u, 0x3eu, 0xa1u, 0xacu, 0x37u, 0x5fu,
                           0xf4u, 0xb5u, 0x54u, 0x47u, 0xa6u, 0x5du, 0x0du, 0x4cu, 0x5du,
                           0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u };

uint8_t dUDSKinput[32] = { 0x00u, 0x00u, 0x00u, 0x01u, 0x55u, 0x44u, 0x53u, 0x4Bu, 0x00u,
                           0x94u, 0x54u, 0x3eu, 0xa1u, 0xacu, 0x37u, 0x5fu,
                           0xf4u, 0xb5u, 0x54u, 0x47u, 0xa6u, 0x5du, 0x0du, 0x4cu, 0x5du,
                           0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u };

uint8_t Salt[16];

AESTiny256 aes256;
AES_CMAC cmac(aes256);

bool DW3000Class::cmd_error = false;

static int32_t bytesdata_process(uint8_t high, uint8_t mid, uint8_t low) {

  int32_t value = ((int32_t)(high) << 16) | ((int32_t)mid << 8) | low;

  // if first 6 bits of high byte is 1, then it is a negative number
  if (high & 0x04) {
    value |= (int32_t)0xFFFC0000;  // set bits 31 to 18 (sign-extend 18-bit)
  }
  return value;
}



void DW3000Class::begin() {
  delay(5);
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  s_uwbSPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CHIP_SELECT_PIN);
  s_uwbSPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  delay(5);

  if (seven_segment_used) {
    //pinMode(15, INPUT); //INIT IRQ for display
    //attachInterrupt(digitalPinToInterrupt(15), DW3000Class::detectedIRQ, CHANGE);
  }

  spiSelect(CHIP_SELECT_PIN);
  memset(&Txprepoll_data,0x00,sizeof(Txprepoll_data));
  Txprepoll_data.StsCounter_u32 = 1;
  //Serial.println("[INFO] SPI ready");
}

void DW3000Class::hardReset() {
  pinMode(DEBUG_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);  // set reset pin active low to hard-reset DW3000 chip
  delay(10);
  digitalWrite(RST_PIN, HIGH);
  pinMode(RST_PIN, OUTPUT);
}

void DW3000Class::DebugToggle() {
  
  digitalWrite(DEBUG_PIN, LOW);  // set reset pin active low to hard-reset DW3000 chip
  digitalWrite(DEBUG_PIN, HIGH);
}
void DW3000Class::detectedIRQ() {
  debug_seven_segment = digitalRead(15);
}

bool DW3000Class::getSevenSegmentStatus() {
  return debug_seven_segment;
}

void DW3000Class::updateDisplay() {
  if (DW3000.getSevenSegmentStatus()) {
    //Serial.println("Seven segment on");
  }
  //TODO insert TM1637 code to display deviceID to seven segment display
}

/*
* This function uses 8 DIP switches connected to the pins that are declared in the below array.
* The device ID is set as a binary value through these switches.
* The LSB starts with the lowest defined pin in the pins array.
*/
void DW3000Class::updateDeviceID() {  // All pins are labeled as "IO[pin number]" on the board itself
  int pins_used = 8;
  int pins[] = { 25, 26, 32, 36, 39, 22, 21, 17 };
  for (int i = 0; i < pins_used; i++) {  //Set all used pins as input
    pinMode(pins[i], INPUT);
  }

  int tmp_device_id = 0;

  for (int i = 0; i < pins_used; i++) {
    //Serial.print("Pin ");
    //Serial.print(pins[i]);
    //Serial.print(": ");
    //Serial.println(digitalRead(pins[i]));
    tmp_device_id += (digitalRead(pins[i]) << i);
  }

  device_id = tmp_device_id;

  //Serial.print("[INFO] DeviceID updated to ");
  //Serial.println(device_id);
}

void DW3000Class::setSevenSegmentActivated(bool status) {
  seven_segment_used = status;
}

void DW3000Class::spiSelect(uint8_t cs) {
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);

  delay(5);
}

void DW3000Class::printFullConfig() {
  int tmp_size = 22;
  uint32_t tmp[tmp_size];
  Serial.println("\n\n#####     READING FULL CONFIG     #####\n");
  tmp[0] = read(0x00, 0x10);
  tmp[1] = read(0x00, 0x24);
  tmp[2] = read(0x00, 0x28);
  tmp[3] = read(0x00, 0x44);
  tmp[4] = read(0x01, 0x14);
  tmp[5] = read(0x02, 0x00);
  tmp[6] = read(0x03, 0x18);
  tmp[7] = read(0x04, 0x0C);
  tmp[8] = read(0x04, 0x20);
  tmp[9] = read(0x06, 0x00);
  tmp[10] = read(0x06, 0x0C);
  tmp[11] = read(0x07, 0x10);
  tmp[12] = read(0x07, 0x18);
  tmp[13] = read(0x07, 0x1C);
  tmp[14] = read(0x07, 0x50);
  tmp[15] = read(0x09, 0x00);
  tmp[16] = read(0x09, 0x08);
  tmp[17] = read(0x11, 0x04);
  tmp[18] = read(0x11, 0x08);
  tmp[19] = read(0x0E, 0x12);
  tmp[20] = read(0x01, 0x04);
  tmp[21] = read(0x0E, 0x00);
  Serial.println("\n\n#####     PRINTING FULL CONFIG     #####\n");
  Serial.println(" ");
  for (int i = 0; i < tmp_size; i++) {
    Serial.print(" ");
    Serial.print(tmp[i], HEX);
  }
}

void DW3000Class::writeSysConfig() {
  int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);
  usr_cfg |= 0x30000;
  write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg, 2);

  if (config[2] > 24) {
    //Serial.println("[ERROR] SCP ERROR! TX & RX Preamble Code higher than 24!");
  }

  int otp_write = 0x1400;

  if (config[1] >= 256) {
    otp_write |= 0x04;
  }


  write(OTP_IF_REG, 0x08, otp_write, 2);  //set OTP config
  write(DRX_REG, 0x00, 0x00, 1);          //reset DTUNE0_CONFIG

  write(DRX_REG, 0x0, config[3], 1);

  //64 = STS length
  write(STS_CFG_REG, 0x0, 128 / 8 - 1, 1);

  write(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1);
  //int dtune3_val[] = { 0x4C, 0x58, 0x5F, 0xAF }; //TODO if not working: change value back (p.147)
  write(DRX_REG, 0x0C, 0xAF5F584C, 4);
  int chan_ctrl_val = read(GEN_CFG_AES_HIGH_REG, 0x14);  //Fetch and adjust CHAN_CTRL data
  chan_ctrl_val &= (~0x1FFF);

  chan_ctrl_val |= config[0];  //Write RF_CHAN

  chan_ctrl_val |= 0x1F00 & (config[2] << 8);
  chan_ctrl_val |= 0xF8 & (config[2] << 3);
  chan_ctrl_val |= 0x06 & (0x00 << 1);

  write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_val, 4);  //Write new CHAN_CTRL data with updated values

  uint32_t tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24);

  tx_fctrl_val |= (config[1] << 12);  //Add preamble length
  tx_fctrl_val |= (config[4] << 10);  //Add data rate

  write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_val, 1);

  write(DRX_REG, 0x02, 0x81, 2);

  int rf_tx_ctrl_2 = 0x1C071134;
  int pll_conf = 0x0F3C;

  if (config[0]) {
    rf_tx_ctrl_2 &= ~0x00FFFF;
    rf_tx_ctrl_2 |= 0x000001;
    pll_conf &= 0x00FF;
    pll_conf |= 0x001F;
  }

  write(RF_CONF_REG, 0x1C, rf_tx_ctrl_2, 4);
  write(FS_CTRL_REG, 0x00, pll_conf, 2);

  write(RF_CONF_REG, 0x51, 0x14, 1);

  write(RF_CONF_REG, 0x1A, 0x0E, 1);

  write(FS_CTRL_REG, 0x08, 0x186, 2);

  write(GEN_CFG_AES_LOW_REG, 0x44, 0x02, 1);

  write(PMSC_REG, 0x04, 0x300200, 3);  //Set clock to auto mode

  write(PMSC_REG, 0x08, 0x0138, 2);
  int success = 0;
  for (int i = 0; i < 100; i++) {
    if (read(GEN_CFG_AES_LOW_REG, 0x0) & 0x2) {
      success = 1;
      break;
    }
  }

  if (!success) {
    //Serial.println("[ERROR] Couldn't lock PLL Clock!");
  } else {
    //Serial.println("[INFO] PLL is now locked.");
  }

  int otp_val = read(OTP_IF_REG, 0x08);
  otp_val |= 0x40;
  if (config[0]) otp_val |= 0x2000;


  write(OTP_IF_REG, 0x08, otp_val, 2);

  write(RX_TUNE_REG, 0x19, 0xF0, 1);

  int ldo_ctrl_val = read(RF_CONF_REG, 0x48);  //Save original LDO_CTRL data
  int tmp_ldo = (0x105 | 0x100 | 0x4 | 0x1);

  write(RF_CONF_REG, 0x48, tmp_ldo, 2);

  write(EXT_SYNC_REG, 0x0C, 0x020000, 3);  //Calibrate RX

  int l = read(0x04, 0x0C);

  delay(20);

  write(EXT_SYNC_REG, 0x0C, 0x11, 1);  //Enable calibration

  int succ = 0;
  for (int i = 0; i < 100; i++) {
    if (read(EXT_SYNC_REG, 0x20)) {
      succ = 1;
      break;
    }
    delay(10);
  }

  if (succ) {
    //Serial.println("[INFO] PGF calibration complete.");
  } else {
    //Serial.println("[ERROR] PGF calibration failed!");
  }

  write(EXT_SYNC_REG, 0x0C, 0x00, 1);
  write(EXT_SYNC_REG, 0x20, 0x01, 1);

  int rx_cal_res = read(EXT_SYNC_REG, 0x14);
  if (rx_cal_res == 0x1fffffff) {
    //Serial.println("[ERROR] PGF_CAL failed in stage I!");
  }
  rx_cal_res = read(EXT_SYNC_REG, 0x1C);
  if (rx_cal_res == 0x1fffffff) {
    //Serial.println("[ERROR] PGF_CAL failed in stage Q!");
  }

  write(RF_CONF_REG, 0x48, ldo_ctrl_val, 2);  //Restore original LDO_CTRL data

  write(0x0E, 0x02, 0x01, 1);  //Enable full CIA diagnostics to get signal strength information


  DW3000.write(0x02, 0x00, 0x07, 1);
}

void DW3000Class::configureAsTX() {
  write(RF_CONF_REG, 0x1C, 0x34, 1);  //write pg_delay
  write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xfdfdfdfd, 4);
}


void DW3000Class::setTXFrame(unsigned long long frame_data,uint16_t datalenght ,uint8_t const * data_buffer) {  // deprecated! use write(TX_BUFFER_REG, [...]);
  if (frame_data > ((pow(2, 8 * 8) - FCS_LEN))) {
    Serial.println("[ERROR] Frame is too long (> 1023 Bytes - FCS_LEN)!");
    return;
  }

  write_data(TX_BUFFER_REG, 0x00, data_buffer, datalenght);
}

void DW3000Class::printRoundTripInformation() {
  //Serial.println("\nRound Trip Information:");

  //Serial.print("TX Timestamp: ");
  ////Serial.println(tx_ts);
  //Serial.print("RX Timestamp: ");
  ////Serial.println(rx_ts);
}

int DW3000Class::getAnchorID() {
  return anchor_id;
}

uint32_t DW3000Class::sendBytes(uint8_t b[], int lenB, int recLen, uint8_t* Data) {  //WORKING
  digitalWrite(CHIP_SELECT_PIN, LOW);
  for (int i = 0; i < lenB; i++) {
    s_uwbSPI.transfer(b[i]);
  }
  int rec;
  uint32_t val, tmp;
  if (Data == NULL) {
    if (recLen > 0) {
      for (int i = 0; i < recLen; i++) {
        tmp = s_uwbSPI.transfer(0x00);
        if (i == 0) {
          val = tmp;  //Read first 4 octets
        } else {
          val |= (uint32_t)tmp << 8 * i;
        }
      }
    } else {
      val = 0;
    }
  } else {
    if (recLen > 0) {
      for (int i = 0; i < recLen; i++) {
        tmp = s_uwbSPI.transfer(0x00);
        Data[i] = tmp;  //Read first 4 octets
      }
    } else {
      val = 0;
    }
  }

  digitalWrite(CHIP_SELECT_PIN, HIGH);
  return val;
}

bool DW3000Class::checkForIDLE() {
  return (read(0x0F, 0x30) >> 16 & PMSC_STATE_IDLE) == PMSC_STATE_IDLE || (read(0x00, 0x44) >> 16 & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK) ? 1 : 0;
}

void DW3000Class::softReset() {
  clearAONConfig();

  write(PMSC_REG, 0x04, 0x1);  //force clock to FAST_RC/4 clock

  write(PMSC_REG, 0x00, 0x00, 2);  //init reset

  delay(100);

  write(PMSC_REG, 0x00, 0xFFFF, 2);  //return back

  write(PMSC_REG, 0x04, 0x00, 1);  //set clock back to Auto mode
}

void DW3000Class::clearAONConfig() {
  write(AON_REG, NO_OFFSET, 0x00, 2);
  write(AON_REG, 0x14, 0x00, 1);

  write(AON_REG, 0x04, 0x00, 1);  //clear control of aon reg

  write(AON_REG, 0x04, 0x02, 1);

  delay(1);
}

void DW3000Class::WritePayload(uint32_t base, uint32_t sub, uint32_t data_len, uint32_t* Data) {
  uint32_t payload_bytes = 0;
  uint32_t header = 0x00;
  header = header | 0x80;
  header = header | ((base & 0x1F) << 1);

  if (sub > 0) {
    header = header | 0x40;
    header = header << 8;
    header = header | ((sub & 0x7F) << 2);
  }

  uint32_t header_size = header > 0xFF ? 2 : 1;
  payload_bytes = data_len;

  uint8_t payload[header_size + payload_bytes];

  if (header_size == 1) {
    payload[0] = header;
  } else {
    payload[0] = (header & 0xFF00) >> 8;
    payload[1] = header & 0xFF;
  }


  //Serial.println();
  //Serial.print("dURSK payload:");
  for (int j = 0; j < data_len; j++) {
    payload[header_size + j] = *(Data + j);
    //Serial.print(payload[header_size + j], HEX);
  }
  (uint32_t) sendBytes(payload, 2 + payload_bytes, 0, NULL);
}

uint32_t DW3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit, uint8_t* Data, const uint8_t* data_buffer) {
  uint32_t header = 0x00;

  if (readWriteBit) header = header | 0x80;

  header = header | ((base & 0x1F) << 1);

  if (sub > 0) {
    header = header | 0x40;
    header = header << 8;
    header = header | ((sub & 0x7F) << 2);
  }

  uint32_t header_size = header > 0xFF ? 2 : 1;
  uint32_t res = 0;

  if (!readWriteBit) {
    uint8_t headerArr[header_size];

    if (header_size == 1) {
      headerArr[0] = header;
    } else {
      headerArr[0] = (header & 0xFF00) >> 8;
      headerArr[1] = header & 0xFF;
    }
    if (data_len == 0) {
      res = (uint32_t)sendBytes(headerArr, header_size, 4, NULL);
    } else {
      res = (uint32_t)sendBytes(headerArr, header_size, data_len, Data);
    }
    return res;
  } else {

    uint32_t payload_bytes = 0;
    if (data_len == 0) {
      if (data > 0) {
        uint32_t payload_bits = countBits(data);
        payload_bytes = (payload_bits - (payload_bits % 8)) / 8;  //calc the used bytes for transaction
        if ((payload_bits % 8) > 0) {
          payload_bytes++;
        }
      } else {
        payload_bytes = 1;
      }
    } else {
      payload_bytes = data_len;
    }
    uint8_t payload[header_size + payload_bytes];

    if (header_size == 1) {
      payload[0] = header;
    } else {
      payload[0] = (header & 0xFF00) >> 8;
      payload[1] = header & 0xFF;
    }
    if (data_buffer == NULL) {
      for (int i = 0; i < payload_bytes; i++) {
        payload[header_size + i] = (data >> i * 8) & 0xFF;
      }
    }
    else
    {
      memcpy(&payload[header_size], data_buffer , sizeof(uint8_t)*payload_bytes);
    }

    res = (uint32_t)sendBytes(payload, 2 + payload_bytes, 0, NULL);
    return res;
  }
}


uint32_t DW3000Class::readOrWriteFullAddress_seprate(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit, uint8_t* Data) {
  uint32_t header = 0x00;

  if (readWriteBit) header = header | 0x80;

  header = header | ((base & 0x1F) << 1);

  header = header | 0x40;
  header = header << 8;
  header = header | ((sub & 0x7F) << 2);
  uint32_t header_size = header > 0xFF ? 2 : 1;
  uint32_t res = 0;

  if (!readWriteBit) {
    int headerArr[2];
    headerArr[0] = ((header & 0xFF00) >> 8);
    headerArr[1] = header & 0xFF;


    digitalWrite(CHIP_SELECT_PIN, LOW);
    for (int i = 0; i < 2; i++) {
      s_uwbSPI.transfer(headerArr[i]);
    }

    for (int i = 0; i < data_len; i++) {
      Data[i] = s_uwbSPI.transfer(0x00);
    }
    digitalWrite(CHIP_SELECT_PIN, HIGH);
    return res;
  }
  return res;  // nhanh ghi chua cai dat: tra 0 thay vi roi khoi ham (UB)
}

uint32_t DW3000Class::read(int base, int sub) {
  uint32_t tmp;
  tmp = readOrWriteFullAddress(base, sub, 0, 0, 0, NULL, NULL);

  return tmp;
}

uint32_t DW3000Class::read_payload(int base, int sub, int data_len, uint8_t* Data) {
  uint32_t tmp;
  tmp = readOrWriteFullAddress_seprate(base, sub, 0, data_len, 0, Data);


  return tmp;
}

uint8_t DW3000Class::read8bit(int base, int sub) {
  return (uint8_t)(read(base, sub) >> 24);
}

void DW3000Class::write_data(int base, int sub, const uint8_t* data, int data_len) {
   readOrWriteFullAddress(base, sub, 0, data_len, 1, NULL,data);
}

uint32_t DW3000Class::write(int base, int sub, uint32_t data, int data_len) {
  return readOrWriteFullAddress(base, sub, data, data_len, 1, NULL, NULL);
}

uint32_t DW3000Class::write(int base, int sub, uint32_t data) {
  return readOrWriteFullAddress(base, sub, data, 0, 1, NULL, NULL);
}

uint32_t DW3000Class::readOTP(uint8_t addr) {
  write(OTP_IF_REG, 0x04, addr, 1);
  write(OTP_IF_REG, 0x08, 0x02, 1);

  return read(OTP_IF_REG, 0x10);
}


void DW3000Class::init() {
  //Serial.println("\n+++ DecaWave DW3000 Test +++\n");

  if (!checkForDevID()) {
    //Serial.println("[ERROR] Dev ID is wrong! Aborting!");
    return;
  }
  //Serial.println("yeah 3");
  setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4);


  while (!checkForIDLE()) {
    //Serial.println("[WARNING] IDLE FAILED (stage 1)");
    delay(100);
  }
  //Serial.println("yeah 4");
  //softReset();

  delay(200);

  while (!checkForIDLE()) {
    //Serial.println("[WARNING] IDLE FAILED (stage 2)");
    delay(100);
  }
  //Serial.println("yeah 1");
  uint32_t ldo_low = readOTP(0x04);
  uint32_t ldo_high = readOTP(0x05);
  uint32_t bias_tune = readOTP(0xA);
  bias_tune = (bias_tune >> 16) & BIAS_CTRL_BIAS_MASK;

  if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0) {
    write(0x11, 0x1F, bias_tune, 4);

    write(0x0B, 0x08, 0x0100, 2);
  }

  int xtrim_value = readOTP(0x1E);

  xtrim_value = xtrim_value == 0 ? 0x2E : xtrim_value;  //if xtrim_value from OTP memory is 0, choose 0x2E as default value

  write(FS_CTRL_REG, 0x14, xtrim_value, 1);


  writeSysConfig();

  write(0x00, 0x3C, 0xFFFFFFFF, 4);  //Set Status Enable
  write(0x00, 0x40, 0xFFFF, 2);

  write(0x0A, 0x00, 0x000900, 3);  //AON_DIG_CFG register setup; sets up auto-rx calibration and on-wakeup GO2IDLE  //0xA

  /*
     * Set RX and TX config channel 5 
     */
  if (config[0] == CHANNEL_5) {
    write(0x3, 0x1C, 0x10000240, 4);  //DGC_CFG0

    write(0x3, 0x20, 0x1B6DA489, 4);  //DGC_CFG1

    write(0x3, 0x38, 0x0001C0FD, 4);  //DGC_LUT_0

    write(0x3, 0x3C, 0x0001C43E, 4);  //DGC_LUT_1

    write(0x3, 0x40, 0x0001C6BE, 4);  //DGC_LUT_2

    write(0x3, 0x44, 0x0001C77E, 4);  //DGC_LUT_3

    write(0x3, 0x48, 0x0001CF36, 4);  //DGC_LUT_4

    write(0x3, 0x4C, 0x0001CFB5, 4);  //DGC_LUT_5

    write(0x3, 0x50, 0x0001CFF5, 4);  //DGC_LUT_6
  }
  if (config[0] == CHANNEL_9) {
    write(0x3, 0x1C, 0x10000240, 4);  //DGC_CFG0

    write(0x3, 0x20, 0x1b6da489, 4);  //DGC_CFG1

    write(0x3, 0x38, 0x0002A8FE, 4);  //DGC_LUT_0

    write(0x3, 0x3C, 0x0002AC36, 4);  //DGC_LUT_1

    write(0x3, 0x40, 0x0002A5FE, 4);  //DGC_LUT_2

    write(0x3, 0x44, 0x0002AF3E, 4);  //DGC_LUT_3

    write(0x3, 0x48, 0x0002AF7d, 4);  //DGC_LUT_4

    write(0x3, 0x4C, 0x0002AFB5, 4);  //DGC_LUT_5

    write(0x3, 0x50, 0x0002AFB5, 4);  //DGC_LUT_6
  }

  write(0x3, 0x18, 0xE5E5, 2);  //THR_64 value set to 0x32
  int f = read(0x4, 0x20);

  //SET PAC TO 32 (0x00) reg:06:00 bits:1-0, bit 4 to 0 (00001100) (0xC)
  write(0x6, 0x0, 0x81101C, 3);

  write(0x07, 0x34, 0x4, 1);  //enable temp sensor readings

  //Serial.println("yeah 7");
  /*
     * Things to do as documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
     */


  write(0x07, 0x48, 0x14, 1);        //LDO_RLOAD to 0x14 //0x7
  write(0x07, 0x1A, 0x0E, 1);        //RF_TX_CTRL_1 to 0x0E
  write(0x07, 0x1C, 0x1C071134, 4);  //RF_TX_CTRL_2 to 0x1C071134 (due to channel 5, else (9) to 0x1C010034)
  write(0x07, 0x10, 0x08B5A833, 4);
  write(0x09, 0x00, 0x1F3C, 2);  //PLL_CFG to 0x1F3C (due to channel 5, else (9) to 0x0F3C)  //0x9
  write(0x09, 0x80, 0x81, 1);    //PLL_CAL config to 0x81

  write(0x11, 0x04, 0xB48240, 3);

  write(0x11, 0x08, 0x80030739, 4);


  Serial.println("[INFO] Initialization finished.\n");
}

void DW3000Class::setupGPIO() {
  write(0x05, 0x08, 0xF0, 1);  //Set GPIO0 - GPIO3 as OUTPUT on DW3000
}

void DW3000Class::pullLEDHigh(int led) {  //led 0 - 2 possible
  if (led > 2) return;
  led_status = led_status + (1 << led);
  write(0x05, 0x0C, led_status, 2);
}

void DW3000Class::pullLEDLow(int led) {  //led 0 - 2 possible
  if (led > 2) return;
  led_status = led_status & ~((int)1 << led);  //https://stackoverflow.com/questions/47981/how-to-set-clear-and-toggle-a-single-bit
  write(0x05, 0x0C, led_status, 1);
}

void DW3000Class::writeTXDelay(uint32_t delay) {
  write(0x00, 0x2C, delay, 4);
}

void DW3000Class::writeRXDelay(uint32_t delay) {
  write(0x00, 0x2C, delay, 4);
}

void DW3000Class::TXThenRX() {  //Activates Transmission with delay programmed through writeTXDelay() and goes to receiver immediately
  writeFastCommand(0xC);
}

void DW3000Class::delayedTXThenRX() {  //Activates Transmission with delay programmed through writeTXDelay() and goes to receiver immediately
  writeFastCommand(0x0F);
}

void DW3000Class::delayedTX() {
  writeFastCommand(0x3);
}

void DW3000Class::delayedRX() {
  writeFastCommand(0x4);
}

unsigned long long DW3000Class::readRXTimestamp_Poll() {
  timestamp_data.PollTimestamp_L_u32 = read(0x0C, 0x00);
  timestamp_data.PollTimestamp_H_u32 = read(0x0C, 0x04) & 0xFF;

  unsigned long long rx_timestamp = ((unsigned long long)timestamp_data.PollTimestamp_H_u32 << 32) | timestamp_data.PollTimestamp_L_u32;

  return rx_timestamp;
}

unsigned long long DW3000Class::readRXTimestamp_Final() {
  timestamp_data.FinalTimestamp_L_u32 = read(0x0C, 0x00);
  timestamp_data.FinalTimestamp_H_u32 = read(0x0C, 0x04) & 0xFF;

  unsigned long long rx_timestamp = ((unsigned long long)timestamp_data.FinalTimestamp_H_u32 << 32) | timestamp_data.FinalTimestamp_L_u32;

  return rx_timestamp;
}
unsigned long long DW3000Class::readTXTimestamp() {
  unsigned long long ts_low = read(0x00, 0x74);
  unsigned long long ts_high = read(0x00, 0x78) & 0xFF;

  unsigned long long tx_timestamp = (ts_high << 32) + ts_low;

  return tx_timestamp;
}

unsigned long long DW3000Class::readRXTimestamp() {
  unsigned long long ts_low = read(0x0c, 0x00);
  unsigned long long ts_high = read(0x0c, 0x04) & 0xFF;

  unsigned long long tx_timestamp = (ts_high << 32) + ts_low;

  return tx_timestamp;
}

unsigned long long DW3000Class::readSysTimestamp() {
  unsigned long long ts_low = read(0x00, 0x1C);
  unsigned long long ts_high = read(0x00, 0x20) & 0xFF;

  unsigned long long sys_timestamp = (ts_high << 32) + ts_low;

  return sys_timestamp;
}

void DW3000Class::prepareDelayedTX() {
}


void DW3000Class::calculateTXRXdiff() {  //calc diff on PING side

  // unsigned long long ping_tx = timestamp_data.RespondTimestamp_L_u32[0];
  // unsigned long long ping_rx = timestamp_data.FinalTimestamp_L_u32[0];

  // long double clk_offset = getClockOffset();
  // long double clock_offset = 1.0 - clk_offset;

  // /*
  //      * PAYLOAD DESIGN:
  //      +------+-----------------------------------------------------------------------+-------------------------------+-------------------------------+------+------+------+-----+
  //      | Byte |                                 1 (0x00)                              |           2 (0x01)            |           3 (0x02)            |     4 - 6 (0x03-0x...)   |
  //      +------+-----+-----+----+----------+----------+----------+----------+----------+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+------+------+------+-----+
  //      | Bits |  1  |  2  |  3 |     4    |     5    |     6    |     7    |     8    | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |                          |
  //      +------+-----+-----+----+----------+----------+----------+----------+----------+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+--------------------------+
  //      |      | Mode bits:     | Reserved | Reserved | Reserved | Reserved | Reserved |           Sender ID           |         Destination ID        | Internal Delay / Payload |
  //      |      | 0 - Standard   |          |          |          |          |          |                               |                               |                          |
  //      |      |1-7 - See below |          |          |          |          |          |                               |                               |                          |
  //      +------+----------------+----------+----------+----------+----------+----------+-------------------------------+-------------------------------+--------------------------+
  //   *
  //   * Mode bits:
  //   * 0 - Standard
  //   * 1 - Double Sided Ranging
  //   * 2-6 - Reserved
  //   * 7 - Error
  //   */

  // long long t_reply = finaldata_data.Timestamp_FINAL_TX_u32 - finaldata_data.Timestamp_respond_u32;

  // /*
  //   * Calculate round trip time (see DW3000 User Manual page 248 for more)
  //   */

  // if (t_reply == 0) {  //t_reply is 0 when the calculation could not be done on the PONG side
  //   return;
  // }

  // long long t_round = ping_rx - ping_tx;
  // long long t_prop = lround((t_round - lround(t_reply * clock_offset)) / 2);

  // long double t_prop_ps = t_prop * PS_UNIT;

  // long double t_prop_cm = t_prop_ps * SPEED_OF_LIGHT;
  // Serial.println(" ");
  // printDouble(t_prop_cm, 100, false);  // second value sets the decimal places. 100 = 2 decimal places, 1000 = 3, 10000 = 4, ...
  // Serial.println(float(t_prop));
}

void DW3000Class::ds_sendFrame(int stage) {
  setMode(1);
  write(0x14, 0x03, stage & 0x7, 1);
  setFrameLength(4);

  TXInstantRX();  //Await response

  bool error = true;
  for (int i = 0; i < 50; i++) {
    if (sentFrameSucc()) {
      error = false;
      break;
    }
  };
  if (error) {
    //Serial.println("[ERROR] Could not send frame successfully!");
  }
}

void DW3000Class::ds_sendRTInfo(int t_roundB, int t_replyB) {
  setMode(1);
  write(0x14, 0x03, 4, 1);
  write(0x14, 0x04, t_roundB, 4);
  write(0x14, 0x08, t_replyB, 4);

  setFrameLength(12);

  TXInstantRX();
}

int DW3000Class::ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clk_offset) {  //returns ranging time in DW3000 ps units (~15.65ps per unit)


  int reply_diff = t_replyA - t_replyB;

  long double clock_offset = t_replyA > t_replyB ? 1.0 + getClockOffset(clk_offset) : 1.0 - getClockOffset(clk_offset);

  int first_rt = t_roundA - t_replyB;
  int second_rt = t_roundB - t_replyA;

  int combined_rt = (first_rt + second_rt - (reply_diff - (reply_diff * clock_offset))) / 2;
  int combined_rt_raw = (first_rt + second_rt) / 2;

  return combined_rt / 2;  // divided by 2 to get just one range
}

/*
* NOTE: If not using 64MHz PRF: See user manual capter 4.7.2 for an alternative calculation
*/
double DW3000Class::getSignalStrength() {  //Returns the signal strength of the received frame in dBm
  int CIRpower = read(0x0C, 0x2C) & 0x1FF;
  int PAC_val = read(0x0C, 0x58) & 0xFFF;
  unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
  double PRF_const = 121.7;

  /*//Serial.println("Signal Strength Data:");
    //Serial.print("CIR Power: ");
    //Serial.println(CIRpower);
    //Serial.print("PAC val: ");
    //Serial.println(PAC_val);
    //Serial.print("DGC decision: ");
    //Serial.println(DGC_decision);*/

  return 10 * log10((CIRpower * (1 << 21)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

double DW3000Class::getFirstPathSignalStrength() {
  float f1 = (read(0x0C, 0x30) & 0x3FFFFF) >> 2;
  float f2 = (read(0x0C, 0x34) & 0x3FFFFF) >> 2;
  float f3 = (read(0x0C, 0x38) & 0x3FFFFF) >> 2;

  int PAC_val = read(0x0C, 0x58) & 0xFFF;
  unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
  double PRF_const = 121.7;

  return 10 * log10((pow(f1, 2) + pow(f2, 2) + pow(f3, 2)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

void DW3000Class::setReceiverAddress(unsigned int addr) {  //Write receiver address to frame
  write(0x14, 0x02, addr & 0xFF, 4);                       //set receiver address
}

void DW3000Class::writeFastCommand(int cmd) {


  int header = 0;

  header = header | 0x1;
  header = header | (cmd & 0x1F) << 1;
  header = header | 0x80;



  uint8_t header_arr[] = { (uint8_t)header };

  sendBytes(header_arr, 1, 0, NULL);
}

int DW3000Class::receivedFrameSucc() {  //No frame received: 0; frame received: 1; error while receiving: 2
  uint32_t sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
  if ((sys_stat & SYS_STATUS_RX_ERR) > 0) {
    return 2;
  } else if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) && ((sys_stat & 0x400) != 0)) {

    return 1;
  }
  return 0;
}

int DW3000Class::receivedFrameSucc_noCIA() {  //No frame received: 0; frame received: 1; error while receiving: 2
  uint32_t sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
  if ((sys_stat & SYS_STATUS_RX_ERR) > 0) {
    return 2;
  } else if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC)) {

    return 1;
  }
  return 0;
}

int DW3000Class::sentFrameSucc() {  //No frame sent: 0; frame sent: 1; error while sending: 2
  int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
  if (((sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC)) {
    return 1;
  }
  return 0;
}

unsigned long long DW3000Class::readRXBuffer() {  // deprecated! Use read(RX_BUFFER_0_REG, [...]); instead
  unsigned long long buf0 = read(RX_BUFFER_0_REG, 0x0);
  buf0 = buf0 + ((unsigned long long)read(0x12, 0x20) << 32);

  return buf0;
}

void DW3000Class::putIdle() {
  DW3000Class::writeFastCommand(0x00);
}

void DW3000Class::standardTX() {
  DW3000Class::writeFastCommand(0x01);
}

void DW3000Class::standardRX() {
  DW3000Class::writeFastCommand(0x02);
}

void DW3000Class::TXInstantRX() {  // Execute tx, then set receiver to rx instantly
  DW3000Class::writeFastCommand(0x0C);
}

void DW3000Class::clearSystemStatus() {
  write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF, 4);
}

int DW3000Class::checkForDevID() {
  if (read(GEN_CFG_AES_LOW_REG, NO_OFFSET) != 0xDECA0302) {
    //Serial.println("[ERROR] DEV_ID IS WRONG!");
    return 0;
  }
  return 1;
}

void DW3000Class::setMode(int mode) {
  write(0x14, 0x00, mode & 0x7, 4);
}

void DW3000Class::setFrameLength(int frame_len) {  // set Frame length in Bytes
  frame_len = frame_len + FCS_LEN;
  int curr_cfg = read(0x00, 0x24);
  if (frame_len > 1023) {
    //Serial.println("[ERROR] Frame length + FCS_LEN (2) is longer than 1023. Aborting!");
    return;
  }
  int tmp_cfg = (curr_cfg & 0xFC00FC00) | frame_len;

  write(GEN_CFG_AES_LOW_REG, 0x24, tmp_cfg, 4);
}

double DW3000Class::convertToCM(int dw3000_ps_units) {
  return (double)dw3000_ps_units * PS_UNIT * SPEED_OF_LIGHT;
}

int DW3000Class::ds_getStage() {
  return read(0x12, 0x03) & 0b111;
}

bool DW3000Class::ds_isErrorFrame() {
  return ((read(0x12, 0x00) & 0x7) == 7);
}

void DW3000Class::ds_sendErrorFrame() {
  setMode(7);
  setFrameLength(3);
  TXInstantRX();
}

/*
* Set bit in a defined register address
*/
void DW3000Class::setBit(int reg_addr, int sub_addr, int shift, bool b) {
  uint8_t tmpByte = read8bit(reg_addr, sub_addr);
  if (b) {
    bitSet(tmpByte, shift);
  } else {
    bitClear(tmpByte, shift);
  }
  write(reg_addr, sub_addr, tmpByte, 1);
}

void DW3000Class::setBitHigh(int reg_addr, int sub_addr, int shift) {
  setBit(reg_addr, sub_addr, shift, 1);
}
void DW3000Class::setBitLow(int reg_addr, int sub_addr, int shift) {
  setBit(reg_addr, sub_addr, shift, 0);
}

void DW3000Class::setDeviceID(unsigned int addr) {
  device_id = addr;
}


/*
    ==== CONFIGURATION FUNCTIONS ====
*/

void DW3000Class::setChannel(uint8_t data) {
  if (data == CHANNEL_5 || data == CHANNEL_9) config[0] = data;
}

void DW3000Class::setPreambleLength(uint8_t data) {
  if (data == PREAMBLE_32 || data == PREAMBLE_64 || data == PREAMBLE_1024 || data == PREAMBLE_256 || data == PREAMBLE_512 || data == PREAMBLE_1024 || data == PREAMBLE_1536 || data == PREAMBLE_2048 || data == PREAMBLE_4096) config[1] = data;
}

void DW3000Class::setPreambleCode(uint8_t data) {
  if (data <= 12 && data >= 9) config[2] = data;
}

void DW3000Class::setPACSize(uint8_t data) {
  if (data == PAC4 || data == PAC8 || data == PAC16 || data == PAC32) config[3] = data;
}

void DW3000Class::setDatarate(uint8_t data) {
  if (data == DATARATE_6_8MB || data == DATARATE_850KB) config[4] = data;
}

void DW3000Class::setPHRMode(uint8_t data) {
  if (data == PHR_MODE_STANDARD || data == PHR_MODE_LONG) config[5] = data;
}

void DW3000Class::setPHRRate(uint8_t data) {
  if (data == PHR_RATE_6_8MB || data == PHR_RATE_850KB) config[6] = data;
}

void DW3000Class::setTXAntennaDelay(int delay) {
  antenna_delay = delay; // trước đây bị ghi cứng 16385, bỏ qua tham số -> không hiệu chuẩn được
  write(0x01, 0x04, antenna_delay, 2);
  //write(0x0e, 0x00, antenna_delay, 3);
}

int DW3000Class::getTXAntennaDelay() {  //DEPRECATED use antenna_delay variable instead!
  int delay = read(0x01, 0x04) & 0xFFFF;
  return delay;
}

long double DW3000Class::getClockOffset() {
  if (config[0] == CHANNEL_5) {
    return getRawClockOffset() * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
  } else {
    return getRawClockOffset() * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
  }
}

long double DW3000Class::getClockOffset(int32_t sec_clock_offset) {
  if (config[0] == CHANNEL_5) {
    return sec_clock_offset * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
  } else {
    return sec_clock_offset * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
  }
}

int DW3000Class::getRawClockOffset() {
  int raw_offset = read(0x06, 0x29) & 0x1FFFFF;
  if (raw_offset & (1 << 20)) {
    raw_offset |= ~((1 << 21) - 1);
  }
  return raw_offset;
}


float DW3000Class::getTempInC() {
  write(0x07, 0x34, 0x04, 1);  //enable temp sensor readings

  write(0x08, 0x00, 0x01, 1);  //enable poll

  while (!(read(0x08, 0x04) & 0x01)) {};

  int res = read(0x08, 0x08);
  res = (res & 0xFF00) >> 8;
  int otp_temp = readOTP(0x09) & 0xFF;
  float tmp = (float)((res - otp_temp) * 1.05f) + 22.0f;

  write(0x08, 0x00, 0x00, 1);  //Reset poll enable

  return tmp;
}


unsigned int DW3000Class::countBits(unsigned int number) {
  return (int)(number) + 1;
}


void DW3000Class::printDouble(double val, unsigned int precision, bool linebreak) {  //https://forum.arduino.cc/t/printing-a-double-variable/44327/2
  unsigned int frac;
  if (val >= 0) {
    frac = (val - int(val)) * precision;
  } else {
    frac = (int(val) - val) * precision;
  }
  if (linebreak) {
    Serial.println(frac, DEC);
  } else {
    Serial.print(frac, DEC);
  }
}

void DW3000Class::PrepollData(uint8_t* Data) {
  if ((Data[19] == 0x01)) {
    prepoll_data.MsgLen_u8 = 0x2E;
    prepoll_data.Received = 1;
    prepoll_data.FrameCounter_u32 = (((uint32_t)Data[5]) | ((uint32_t)Data[6] << 8)
                                     | ((uint32_t)Data[7] << 16) | ((uint32_t)Data[8] << 24)); /* UWB Data Bytes 5-8  */
    prepoll_data.Ranging_block = ((((uint16_t)Data[prepoll_data.PayloadStart_u8 + 8])
                                   | (uint16_t)Data[prepoll_data.PayloadStart_u8 + 9])
                                  << 8);
    prepoll_data.PayloadStart_u8 = 23;
    prepoll_data.MicLength_u8 = 8;
    prepoll_data.PayloadLength_u8 = prepoll_data.MsgLen_u8 - prepoll_data.PayloadStart_u8 - prepoll_data.MicLength_u8 - 2;
    prepoll_data.StsCounter_u32 = ((uint32_t)Data[prepoll_data.PayloadStart_u8 + 4]) | ((uint32_t)Data[prepoll_data.PayloadStart_u8 + 5] << 8)
                                  | ((uint32_t)Data[prepoll_data.PayloadStart_u8 + 6] << 16) | ((uint32_t)Data[prepoll_data.PayloadStart_u8 + 7] << 24); /* UWB Data Bytes 5-8  */
    prepoll_data.StsCounter_u32 += 72;
  }
}

void DW3000Class::PreparePrepollData(uint8_t* Data) {
    

    
    memset(Data,0xAA,sizeof(uint8_t)*64) ;
    *(Data+19) = 0x01; // Message type: PREPOLL — required for RX to parse
    *(Data+5) = Txprepoll_data.FrameCounter_u32 & 0x00FF;
    *(Data+6) = (Txprepoll_data.FrameCounter_u32 >> 8) & 0x00FF;
    *(Data+7) = (Txprepoll_data.FrameCounter_u32 >> 16) & 0x00FF;
    *(Data+8) = (Txprepoll_data.FrameCounter_u32 >> 24) & 0x00FF;
    Txprepoll_data.FrameCounter_u32 += 1; /* UWB Data Bytes 5-8  */
    *(Data+31) = prepoll_data.PayloadStart_u8 & 0x00FF;
    *(Data+32) = (prepoll_data.PayloadStart_u8 >> 8) & 0x00FF;
    *(Data+33) = 0x00;
    *(Data+27) = Txprepoll_data.StsCounter_u32 & 0x00FF;
    *(Data+28) = (Txprepoll_data.StsCounter_u32 >> 8) & 0x00FF;
    *(Data+29) = (Txprepoll_data.StsCounter_u32 >> 16) & 0x00FF;
    *(Data+30) = (Txprepoll_data.StsCounter_u32 >> 24) & 0x00FF;
    Txprepoll_data.StsCounter_u32 += 72;
  
}


void DW3000Class::PrepareFinalData(uint8_t* Data) {
    
    uint8_t lcounter = 0;
    memset(Data,0xAA,sizeof(uint8_t)*107) ;
    *(Data+19) = 0x02; // Message type: FINALDATA — required for RX to parse
    // Use Txprepoll_data.FrameCounter_u32 (already incremented after prepoll send)
    // so RX sees finaldata_FC - prepoll_FC == 1
    *(Data+5) = Txprepoll_data.FrameCounter_u32 & 0x00FF;
    *(Data+6) = (Txprepoll_data.FrameCounter_u32 >> 8) & 0x00FF;
    *(Data+7) = (Txprepoll_data.FrameCounter_u32 >> 16) & 0x00FF;
    *(Data+8) = (Txprepoll_data.FrameCounter_u32 >> 24) & 0x00FF;
    Txfinaldata_data.StsCounter_u32 = Txprepoll_data.StsCounter_u32 + 9;
    *(Data+32) = Txfinaldata_data.StsCounter_u32 & 0x00FF;
    *(Data+33) = (Txfinaldata_data.StsCounter_u32 >> 8) & 0x00FF;
    *(Data+34) = (Txfinaldata_data.StsCounter_u32 >> 16) & 0x00FF;
    *(Data+35) = (Txfinaldata_data.StsCounter_u32 >> 24) & 0x00FF;

    Txfinaldata_data.Timestamp_FINAL_TX_u32 = timestamp_data.FinalTimestamp_L_u32 - timestamp_data.PollTimestamp_L_u32;
    *(Data+36) = Txfinaldata_data.Timestamp_FINAL_TX_u32 & 0x00FF;
    *(Data+37) = (Txfinaldata_data.Timestamp_FINAL_TX_u32 >> 8) & 0x00FF;
    *(Data+38) = (Txfinaldata_data.Timestamp_FINAL_TX_u32 >> 16) & 0x00FF;
    *(Data+39) = (Txfinaldata_data.Timestamp_FINAL_TX_u32 >> 24) & 0x00FF;
    for (uint8_t i = 0 ; i < 8 ; i++)
    {
      if(timestamp_data.RespondTimestamp_L_u32[i] != 0)
      {
        *(Data+ 41 + 7*lcounter) = i;
        Txfinaldata_data.Timestamp_respond_u32 = timestamp_data.RespondTimestamp_L_u32[i] - timestamp_data.PollTimestamp_L_u32;
        *(Data+ 42 + 7*lcounter) = Txfinaldata_data.Timestamp_respond_u32 & 0x00FF;
        *(Data+ 43 + 7*lcounter) = (Txfinaldata_data.Timestamp_respond_u32 >> 8) & 0x00FF;
        *(Data+ 44 + 7*lcounter) = (Txfinaldata_data.Timestamp_respond_u32 >> 16) & 0x00FF;
        *(Data+ 45 + 7*lcounter) = (Txfinaldata_data.Timestamp_respond_u32 >> 24) & 0x00FF;
        lcounter++;
      }
    }
    
  
}

void DW3000Class::Cleartimestamp() {
  finaldata_data.Timestamp_FINAL_TX_u32 = 0;
  finaldata_data.Timestamp_respond_u32 = 0;
  for (uint8_t i = 0 ; i < 8 ; i++)
  {
    timestamp_data.RespondTimestamp_H_u32[i] = 0;
    timestamp_data.RespondTimestamp_L_u32[i] = 0;
  }

  timestamp_data.PollTimestamp_H_u32 = 0;
  timestamp_data.PollTimestamp_L_u32 = 0;
  timestamp_data.FinalTimestamp_H_u32 = 0;
  timestamp_data.FinalTimestamp_L_u32 = 0;
  finaldata_data.Received = 0;
}

void DW3000Class::FinalData(uint8_t* Data) {
  if ((Data[19] == 0x02) && (finaldata_data.Received == 1)) {
    finaldata_data.Ranging_block = (((uint16_t)Data[27]) | ((uint16_t)Data[28]) << 8);

    finaldata_data.FrameCounter_u32 = (((uint32_t)Data[5]) | ((uint32_t)Data[6] << 8)
                                       | ((uint32_t)Data[7] << 16) | ((uint32_t)Data[8] << 24)); /* UWB Data Bytes 5-8  */
    if ((finaldata_data.FrameCounter_u32 - Txprepoll_data.FrameCounter_u32) == 1) {
      finaldata_data.Timestamp_FINAL_TX_u32 = (((uint32_t)Data[36]) | (((uint32_t)Data[37]) << 8)
                                               | ((uint32_t)Data[38] << 16) | (((uint32_t)Data[39]) << 24));
      for (int i = 0; i < 7; i++) {
        finaldata_data.Responder_index = Data[41 + i * 7];
        if (finaldata_data.Responder_index == 2) {
          finaldata_data.Timestamp_respond_u32 = (((uint32_t)Data[42 + i * 7]) | (((uint32_t)Data[43 + i * 7]) << 8)
                                                  | ((uint32_t)Data[44 + i * 7] << 16) | (((uint32_t)Data[45 + i * 7]) << 24));
        }
      }



      finaldata_data.uncertainty = (((Data[46]) & 0x18) >> 3);
      finaldata_data.factor = (((Data[46]) & 0x60) >> 5);
      DW3000.HandleTimestamp();
    }
  }
}
uint8_t counter_dis = 0;
void DW3000Class::DistanceCal() {
  static uint32_t Timestamp_FINAL_TX_u32 = 0;
  static uint64_t RespondTimestamp_u64 = 0;
  static uint64_t PollTimestamp_u64 = 0;
  static uint64_t FinalTimestamp_u64 = 0;
  static uint64_t lRound1_u32 = 0;
  static uint64_t lReply2_u64 = 0;
  static uint64_t lReply1_u64 = 0;
  static uint64_t lRound2_u64 = 0;
  static uint64_t lRound_u64 = 0;
  static uint64_t lReply_u64 = 0;
  static float lDistanceold = 0;
  static long long tof;
  lRound1_u32 = finaldata_data.Timestamp_respond_u32;
  Timestamp_FINAL_TX_u32 = finaldata_data.Timestamp_FINAL_TX_u32;
  RespondTimestamp_u64 = timestamp_data.RespondTimestamp_L_u32[0];
  PollTimestamp_u64 = timestamp_data.PollTimestamp_L_u32;
  FinalTimestamp_u64 = timestamp_data.FinalTimestamp_L_u32;
  if ((RespondTimestamp_u64 < PollTimestamp_u64)) {
    RespondTimestamp_u64 += 4294967296;
    FinalTimestamp_u64 += 4294967296;
  }

  else if (FinalTimestamp_u64 < RespondTimestamp_u64) {
    FinalTimestamp_u64 += 4294967296;
  }

  lReply2_u64 = Timestamp_FINAL_TX_u32 - lRound1_u32;
  lReply1_u64 = RespondTimestamp_u64 - PollTimestamp_u64 - 65;
  lRound2_u64 = FinalTimestamp_u64 - RespondTimestamp_u64 + 65;

  lRound_u64 = (lRound1_u32 * lRound2_u64);
  lReply_u64 = (lReply2_u64 * lReply1_u64);

  if (lRound_u64 >= lReply_u64) {
    tof = uint16_t((lRound_u64 - lReply_u64) / (lRound1_u32 + lRound2_u64 + lReply2_u64 + lReply1_u64));
  }
  finaldata_data.distance = (tof * (1.0 / 499.2e6 / 128.0) * 299702547);
  counter_dis++;
  lDistanceold = finaldata_data.distance;
  SyncAndSend();
  if (float(finaldata_data.distance) <= float(3)) {
    digitalWrite(5, LOW);
  } else {
    digitalWrite(5, HIGH);
  }
  /* TO DO */
}

void DW3000Class::SyncAndSend() {
  Serial.print("/");
  Serial.print(finaldata_data.distance);
  Serial.print("/");
  Serial.print(finaldata_data.FrameCounter_u32);
  Serial.println("");
}
unsigned long long DW3000Class::ReceivePrepoll() {
  uint32_t ts_low = DW3000.read(0x00, 0x64);                        // Read RX_FRAME buffer0
  unsigned long long ts_high = ((DW3000.read(0x00, 0x68) & 0xFF));  // Read RX_FRAME buffer0
  unsigned long long rx_timestamp = (ts_high << 32) | ts_low;

  return rx_timestamp;
}


void DW3000Class::Receivepoll() {
  // timestamp_data.PollTimestamp_L_u32 = DW3000.read(0x00, 0x64);
  // timestamp_data.PollTimestamp_H_u32 = ((DW3000.read(0x00, 0x68) & 0xFF));  // Read RX_FRAME buffer0
}

void DW3000Class::SendRespond(uint8_t respond_num) {
  // TX is initiator: it RECEIVES respond, so read RX timestamp (0x0C:0x00), not TX timestamp
  timestamp_data.RespondTimestamp_L_u32[respond_num] = DW3000.read(0x0C, 0x00);
  timestamp_data.RespondTimestamp_H_u32[respond_num] = ((DW3000.read(0x0C, 0x04) & 0xFF));
}

void DW3000Class::Receivefinal() {
  timestamp_data.FinalTimestamp_L_u32 = DW3000.read(0x0C, 0x00);             // Read RX_FRAME buffer0
  timestamp_data.FinalTimestamp_H_u32 = ((DW3000.read(0x00, 0x68) & 0xFF));  // Read RX_FRAME buffer0
}

void DW3000Class::Receivefinaldata() {
  finaldata_data.Received = 1;
  timestamp_data.FinalDataTimestamp_L_u32 = DW3000.read(0x0C, 0x00);             // Read RX_FRAME buffer0
  timestamp_data.FinalDataTimestamp_H_u32 = ((DW3000.read(0x00, 0x68) & 0xFF));  // Read RX_FRAME buffer0
}
void DW3000Class::HandleTimestamp() {

  DW3000.DistanceCal();
}

void DW3000Class::SwitchPackage(uint8_t Package) {
  uint32_t usr_cfg;
  switch (Package) {
    case SP0:
      {
        usr_cfg = read(0x00, 0x10);
        usr_cfg &= 0xFFFFFFFFCFFF;
        write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg, 2);
        write(DRX_REG, 0x0C, 0xAF5F584C, 4);
        break;
      }
    case SP3:
      {
        usr_cfg = read(0x00, 0x10);
        usr_cfg |= (0x03 << 12);
        usr_cfg &= (~(1 << 15));
        write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg, 2);
        write(DRX_REG, 0x0C, 0xAF5F35CC, 4);
        break;
      }
  }
}
void DW3000Class::InitCrypto() {
  /*init crypto*/
  cmac.generateMAC(mURSK_low, URSKkey, URSKinput_low, sizeof(URSKinput_low));
  /* HIGH ENCRYPT */
  cmac.generateMAC(mURSK_high, URSKkey, URSKinput_high, sizeof(URSKinput_high));
  /* salt ENCRYPT */
  cmac.generateMAC(Salt, URSKkey, Saltinput, sizeof(Saltinput));
  /* bash padedsalt ENCRYPT */
  memcpy(&PaddedSaltinput[16], Salt, sizeof(Salt));
  /* combine mURSK ENCRYPT */
  memcpy(mURSKinput, mURSK_low, sizeof(mURSK_low));
  memcpy(&mURSKinput[16], mURSK_high, sizeof(mURSK_high));
}

void DW3000Class::Generate_KtURSK() {
  /* bash KT_URSK ENCRYPT */
  for (uint8_t Counter_u8 = 0; Counter_u8 < 32u; Counter_u8++) {
    URSKinputKT_low[12u + Counter_u8] = (uint8_t)((uint8_t)((Txprepoll_data.StsCounter_u32 - 1) >> (uint8_t)(31u - (uint8_t)Counter_u8)) & (uint8_t)1u);
    URSKinputKT_high[12u + Counter_u8] = URSKinputKT_low[12u + Counter_u8];
  }
  cmac.generateMAC(ktURSK_low, mURSKinput, URSKinputKT_low, sizeof(URSKinputKT_low));
  cmac.generateMAC(ktURSK_high, mURSKinput, URSKinputKT_high, sizeof(URSKinputKT_high));
  /* combine KtURSK ENCRYPT */
  memcpy(URSKoutputKT, ktURSK_low, sizeof(ktURSK_low));
  memcpy(&URSKoutputKT[16], ktURSK_high, sizeof(ktURSK_high));
}

void DW3000Class::Generate_dURSK() {
  uint32_t dURSK_temp[4];
  cmac.generateMAC(dURSK, URSKoutputKT, dURSKinput, sizeof(dURSKinput));


  memset(dURSK_respond, 1, sizeof(dURSK_respond));

  memcpy(&dURSK_temp[0], &dURSK[0], 4);
  memcpy(&dURSK_temp[1], &dURSK[4], 4);
  memcpy(&dURSK_temp[2], &dURSK[8], 4);
  memcpy(&dURSK_temp[3], &dURSK[12], 4);

  for (int i = 0; i < 4; i++) {
    dURSK[i] = (dURSK_temp[3] >> (3 - i) * 8) & 0xFF;
    dURSK[i + 4] = (dURSK_temp[2] >> (3 - i) * 8) & 0xFF;
    dURSK[i + 8] = (dURSK_temp[1] >> (3 - i) * 8) & 0xFF;
    dURSK[i + 12] = (dURSK_temp[0] >> (3 - i) * 8) & 0xFF;
  }
  memcpy(&dURSK_respond[0], &dURSK[0], 4);
  memcpy(&dURSK_respond[1], &dURSK[4], 4);
  memcpy(&dURSK_respond[2], &dURSK[8], 4);
  memcpy(&dURSK_respond[3], &dURSK[12], 4);




  for (int i = 0; i < 4; i++) {
    STS_IV[i] = (SaltedHashinput_u32[3] >> (3 - i) * 8) & 0xFF;
    STS_IV[i + 4] = (SaltedHashinput_u32[2] >> (3 - i) * 8) & 0xFF;
    STS_IV[i + 8] = (SaltedHashinput_u32[1] >> (3 - i) * 8) & 0xFF;
    STS_IV[i + 12] = (SaltedHashinput_u32[0] >> (3 - i) * 8) & 0xFF;
  }


  memcpy(&STS_IV_respond[0], &STS_IV[0], 4);
  memcpy(&STS_IV_respond[1], &STS_IV[4], 4);
  memcpy(&STS_IV_respond[2], &STS_IV[8], 4);
  memcpy(&STS_IV_respond[3], &STS_IV[12], 4);

  memcpy(&STS_IV_final[0], &STS_IV[0], 4);
  memcpy(&STS_IV_final[1], &STS_IV[4], 4);
  memcpy(&STS_IV_final[2], &STS_IV[8], 4);
  memcpy(&STS_IV_final[3], &STS_IV[12], 4);

  memcpy(&STS_IV_poll[0], &STS_IV[0], 4);
  memcpy(&STS_IV_poll[1], &STS_IV[4], 4);
  memcpy(&STS_IV_poll[2], &STS_IV[8], 4);
  memcpy(&STS_IV_poll[3], &STS_IV[12], 4);

  

  STS_IV_poll[1] = STS_IV_poll[1] + Txprepoll_data.StsCounter_u32;

  STS_IV_final[1] = STS_IV_final[1] + Txprepoll_data.StsCounter_u32 + 9;
}

void DW3000Class::WriteRespondSTS() {
  DW3000.write(0x02, 0x0C, dURSK_respond[0], 4);
  DW3000.write(0x02, 0x10, dURSK_respond[1], 4);
  DW3000.write(0x02, 0x14, dURSK_respond[2], 4);
  DW3000.write(0x02, 0x18, dURSK_respond[3], 4);
}

void DW3000Class::WriteRespondSTS_IV(uint8_t number_resp) {
  // Compute from base value each time (avoid accumulation bug)
  uint32_t sts_iv_base;
  memcpy(&sts_iv_base, &STS_IV[4], 4);
  STS_IV_respond[1] = sts_iv_base + Txprepoll_data.StsCounter_u32 + number_resp;
  DW3000.write(0x02, 0x1C, STS_IV_respond[0], 4);
  DW3000.write(0x02, 0x20, STS_IV_respond[1], 4);
  DW3000.write(0x02, 0x24, STS_IV_respond[2], 4);
  DW3000.write(0x02, 0x28, STS_IV_respond[3], 4);
}

void DW3000Class::WriteFinalSTS_IV() {
  DW3000.write(0x02, 0x1C, STS_IV_final[0], 4);
  DW3000.write(0x02, 0x20, STS_IV_final[1], 4);
  DW3000.write(0x02, 0x24, STS_IV_final[2], 4);
  DW3000.write(0x02, 0x28, STS_IV_final[3], 4);
}

void DW3000Class::WritePollSTS_IV() {
  DW3000.write(0x02, 0x1C, STS_IV_poll[0], 4);
  DW3000.write(0x02, 0x20, STS_IV_poll[1], 4);
  DW3000.write(0x02, 0x24, STS_IV_poll[2], 4);
  DW3000.write(0x02, 0x28, STS_IV_poll[3], 4);
}

void DW3000Class::ReadFristPath() {
  uint32_t frame_buffer_FP = 0;
  uint64_t frame_buffer_FP_read = 0;
  uint16_t Index_FP = 0;
  uint8_t Index_FP_base = 0;
  //read first path Index
  frame_buffer_FP = DW3000.read(0x0C, 0x48) >> 6;
  //Serial.println("0x0C, 0x28");
  //Serial.println(frame_buffer_FP, HEX);   // Wait until frame was received
  //Serial.println(float(frame_buffer_FP & 0x001FFFFF));   // Wait until frame was received
  //Serial.println((frame_buffer_FP & 0xFFE00000) >> 0x15, HEX);   // Wait until frame was received
  //Serial.println("Index_FP");
  Index_FP = DW3000.read(0x1f, 0x08);
  Index_FP &= 0x8000;
  Index_FP |= uint16_t((frame_buffer_FP));
  Index_FP_base = DW3000.read(0x1f, 0x04);
  Index_FP_base &= 0xE0;
  Index_FP_base |= 0x15;
  //Serial.println(Index_FP, HEX);   // Wait until frame was received
  write(0x1F, 0x04, Index_FP_base, 1);  //set receiver address
  Index_FP -= 10;
  write(0x1F, 0x08, Index_FP, 2);  //set receiver address
  //Serial.println("CIR MAX ");
  //Serial.println(frame_buffer_FP & 0x001FFFFF, HEX);
  // for(int i = 0; i < 10 ; i++)
  uint16_t temp = 500;
  {
    frame_buffer_FP_read = DW3000.read_CIR(0x1D, 0, temp);
  }
}

uint64_t DW3000Class::read_CIR(int base, int sub, uint16_t data_lenght) {
  uint64_t tmp;
  tmp = readCIRFullAddress(base, sub, 0, data_lenght);
  // Ban goc: "return tmp;" bi thut vao than cua if(DEBUG_OUTPUT) -> khi
  // DEBUG_OUTPUT = 0 ham khong return gi ca. Tra ve vo dieu kien.
  return tmp;
}

uint64_t DW3000Class::readCIRFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len) {
  uint32_t header = 0x00;

  if (1) header = header | 0x40;

  header = header | ((base & 0x1F) << 1);

  if (sub > 0) {
    header = header | 0x40;
    header = header << 8;
    header = header | ((sub & 0x7F) << 2);
  }

  uint32_t header_size = header > 0xFF ? 2 : 1;
  uint64_t res = 0;
  {
    int headerArr[header_size];

    if (header_size == 1) {
      headerArr[0] = header;
    } else {
      headerArr[0] = (header & 0xFF00) >> 8;
      headerArr[1] = header & 0xFF;
    }

    res = (uint64_t)sendBytesCIR(headerArr, header_size, data_len);
    return res;
  }
}


uint32_t counter = 0;
uint64_t DW3000Class::sendBytesCIR(int b[], int lenB, uint32_t recLen) {  //WORKING
  counter = 0;
  digitalWrite(CHIP_SELECT_PIN, LOW);
  for (int i = 0; i < lenB; i++) {
    s_uwbSPI.transfer(b[i]);
  }
  int rec;
  uint64_t val, tmp;
  uint8_t read_CIR[500];

  if (recLen > 0) {
    for (counter; counter < recLen; counter++) {
      tmp = s_uwbSPI.transfer(0x00);
      read_CIR[counter] = (uint8_t)tmp;

      if (counter == 0) {
        val = tmp;  //Read first 4 octets
      } else {
        val |= (uint64_t)tmp << 8 * counter;
      }
    }
  } else {
    val = 0;
  }
  digitalWrite(CHIP_SELECT_PIN, HIGH);

  int32_t tempvalue_real = 0;
  int32_t tempvalue_img = 0;
  float tempvalue = 0;
  Serial.println("start-frame");
  for (int i = 0; i < 64; i++) {
    tempvalue_real = bytesdata_process(read_CIR[i * 6 + 4], read_CIR[i * 6 + 3], read_CIR[i * 6 + 2]);
    tempvalue_img = bytesdata_process(read_CIR[i * 6 + 7], read_CIR[i * 6 + 6], read_CIR[i * 6 + 5]);
    tempvalue = sqrt(tempvalue_img * tempvalue_img + tempvalue_real * tempvalue_real);
    Serial.print("real ");
    Serial.println(tempvalue_real);
    Serial.print("img ");
    Serial.println(tempvalue_img);
    Serial.print("amp ");
    Serial.println(tempvalue);
  }
  return val;
}
