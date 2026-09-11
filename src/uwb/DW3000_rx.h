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

#ifndef DW3000_RX_H
#define DW3000_RX_H

#include "Arduino.h"
#include "DW3000Constants_rx.h"
#include <Crypto.h>
#include <AES.h>
#include <string.h>
#include <AES_CMAC.h>
#include <Time.h>

#define SP0 0
#define SP3 3
#define PREPOLL 0
#define POLL 1
#define RESPOND 2
#define FINAL 3
#define FINALDATA 4
#define SENDCIR 5
#define RECEIVECIR 6
#define IDLE 0xFA

// Master switch for verbose ranging debug logs (driver + HAL). 0 = silent.
// Wraps existing Serial.print debug lines; does not remove or alter them.
// MẶC ĐỊNH 0 (giống bản standalone uwb_rx đã chạy): các dòng debug này nằm
// NGAY TRONG cửa sổ POLL→RESPOND→FINAL→FINALDATA, keyfob phát gói kế tiếp chỉ
// vài ms sau gói trước -> Serial.print + SPI read thêm làm car bật RX trễ và
// lỡ gói (FINAL rx_status=3, FINALDATA rx_status=2). Chỉ bật khi debug driver.
// Log lỗi theo từng chu kỳ (ngoài cửa sổ timing) xem UWB_DW3000_DEBUG.
#ifndef UWB_DEBUG
#define UWB_DEBUG 0
#endif


class DW3000Class
{
public:
	static int config[9];

	static void spiSelect(uint8_t cs);

	static void begin();
	static void init();

  static void EnableTimeout();
  static void DisableTimeout();
	static void writeSysConfig();
	static void configureAsTX();
	static void setupGPIO();
  static void setupRFport(uint8_t port);
	static void updateDeviceID();

	// Functions that are used for double-sided ranging
	static void ds_sendFrame(int stage);
	static void ds_sendRTInfo(int t_roundB, int t_replyB);
	static int ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
	static int ds_getStage();
	static bool ds_isErrorFrame();
	static void ds_sendErrorFrame();

	static void setChannel(uint8_t data);
	static void setPreambleLength(uint8_t data);
	static void setPreambleCode(uint8_t data);
	static void setPACSize(uint8_t data);
	static void setDatarate(uint8_t data);
	static void setPHRMode(uint8_t data);
	static void setPHRRate(uint8_t data);
	static void setTXFrame(unsigned long long frame_data);
	static void setFrameLength(int frame_len);
	static void setTXAntennaDelay(int delay);
	static void setMode(int mode);
	static void setDeviceID(unsigned int addr);

	static void setReceiverAddress(unsigned int addr);

	static void setSevenSegmentActivated(bool status);
	static int receivedFrameSucc_test();
	static int receivedFrameSucc();
  static int receivedFrameSucc_noCIA();
  static int receivedFrameSucc_NOTIMEOUT();
  
	static int sentFrameSucc();
	static void detectedIRQ();

	static int getAnchorID();
	static double getSignalStrength();
	static double getFirstPathSignalStrength();
	static int getTXAntennaDelay();
	static long double getClockOffset();
	static long double getClockOffset(int32_t ext_clock_offset);
	static int getRawClockOffset();
	static float getTempInC();
	static void printFullConfig();
	static void printRoundTripInformation();
	static unsigned long long readRXBuffer();
	static bool checkForIDLE();
	static int checkForDevID();
  static void DebugToggle();
	static uint32_t read_payload(int base, int sub, int data_len, uint8_t *Data);
	static uint32_t read(int base, int sub);
	static uint8_t read8bit(int base, int sub);
	static uint64_t read_CIR(int base, int sub,uint16_t data_lenght);
	static uint32_t readOTP(uint8_t addr);
	static unsigned long long readRXTimestamp_Poll();
	static unsigned long long readRXTimestamp_Final();
	static unsigned long long readTXTimestamp();
	static void calculateTXRXdiff();

	static uint32_t write(int base, int sub, uint32_t data, int data_len);
	static uint32_t write(int base, int sub, uint32_t data);

	static void writeTXDelay(uint32_t delay);
	static void writeRXDelay(uint32_t delay);
	static void delayedTXThenRX();
  static void TXThenRX();
	static void prepareDelayedTX();
	static void delayedTX();
	static void delayedRX();

	static void putIdle();
	static void standardTX();
	static void standardRX();
	static void TXInstantRX();
	static void softReset();
	static void hardReset();
	static void clearSystemStatus();
	static void pullLEDHigh(int led);
	static void pullLEDLow(int led);
	// static void interruptDetect();

	static double convertToCM(int dw3000_ps_units);

	static void printDouble(double val, unsigned int precision, bool linebreak);
	unsigned long long ReceivePrepoll();
	static void Receivefinaldata();
	static void Receivefinal();
	static void PrepollData(uint8_t *Data);
	static double convert_pdoa_to_angle_deg(int16_t pdoa_signed);
	static int16_t sign_extend_14bit(uint16_t raw_14);
	static int64_t sign_extend_41bit(uint64_t raw_41);
	static void FinalData(uint8_t *Data);
	static void Cleartimestamp();
	static void DistanceCal();
	// Returns the distance (cm) computed by the most recent DistanceCal() call.
	static float getLastDistance();
	static void SyncAndSend();
	static void Send_Rangingmsg_ethernet();
	static void Send_BLEstatusmsg_ethernet();
	static void SwitchPackage(uint8_t Package);
	static void Generate_KtURSK();
	static void Generate_dURSK();
	static void WriteRespondSTS();
	static void WriteRespondSTS_IV();
		static void WritePollSTS_IV();
	static void WriteFinalSTS_IV();
	static void InitCrypto();
	static void Combo_Switch(uint8_t * pChapsPerSlot_u8_ptr, uint8_t * pSlotsPerRound_u8_ptr, uint8_t * pSessionRanMultiplier_u8_ptr, uint8_t * pNumberOfResponderNodes_u8_ptr);
	static void Generate_SaltedHash(uint8_t * pChapsPerSlot_u8_ptr, uint8_t * pSlotsPerRound_u8_ptr, uint8_t * pSessionRanMultiplier_u8_ptr, uint8_t * pNumberOfResponderNodes_u8_ptr);
	static void Receivepoll();
	static void HandleTimestamp();
	static bool getSevenSegmentStatus();
	static void updateDisplay();
	static void SendRespond();
	static void ReadFristPath();
	static void WriteTimeOutPeriod(uint32_t timeout);

private:
	static void clearAONConfig();

	static void setBit(int reg_addr, int sub_addr, int shift, bool b);
	static void setBitLow(int reg_addr, int sub_addr, int shift);
	static void setBitHigh(int reg_addr, int sub_addr, int shift);

	static void writeFastCommand(int cmd);

	static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit, uint8_t *buffer);
	static uint32_t readOrWriteFullAddress_seprate(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit, uint8_t *Data);
	static uint64_t readCIRFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len );

	static void WritePayload(uint32_t base, uint32_t sub, uint32_t data_len, uint32_t *Data);
	static uint32_t sendBytes(int b[], int lenB, int recLen, uint8_t *Data);
 	static uint64_t sendBytesCIR(int b[], int lenB, uint32_t recLen); 
	int16_t rsl_calculate(uint32_t c, uint32_t n, uint32_t pow2, uint8_t dgc_decision, uint8_t rx_pcode, bool is_sts);
	static unsigned int countBits(unsigned int number);

  int16_t rsl_calculate_signal_power(
    int32_t channel_impulse_response,
    uint8_t quantization_factor,
	uint16_t preamble_accumulation_count,
    uint8_t dgc_decision,
    uint8_t rx_pcode,
    bool is_sts
);

	static bool is_anchor;
	static bool cmd_error;
	static int anchor_id;
};

extern DW3000Class DW3000;

typedef struct
{
	uint8_t MsgLen_u8;		   /* Data array length */
	uint32_t FrameCounter_u32; /* Frame counter */
	uint32_t StsCounter_u32;   /* Frame counter */
	uint8_t SecurityLevel_u8;  /* Security Level */
	uint8_t PayloadStart_u8;   /* Payload start in data array */
	uint8_t PayloadLength_u8;  /* Payload length */
	uint8_t MicStart_u8;	   /* MIC start in data array */
	uint8_t MicLength_u8;	   /* MIC length */
	uint8_t Received;
	uint16_t Ranging_block;
	uint16_t SessionID_payload;
	uint16_t Ranging_round;
  int32_t angle;
} UwbMsgPrepollData_t;

typedef struct
{
	uint8_t MsgLen_u8;		   /* Data array length */
	uint32_t FrameCounter_u32; /* Frame counter */
	uint32_t StsCounter_u32;   /* Frame counter */
	uint8_t PayloadStart_u8;   /* Payload start in data array */
	uint8_t PayloadLength_u8;  /* Payload length */
	uint32_t Timestamp_FINAL_TX_u32;
	uint32_t Timestamp_respond_u32;
	uint8_t Responder_index;
	uint16_t Ranging_block;
	uint8_t Received;
	uint8_t uncertainty;
	uint8_t factor;
	float distance = 0;
  float angle = 0;
} UwbMsgFinalData_t;

typedef struct
{
	uint32_t PrepollTimestamp_H_u32;
	uint32_t PrepollTimestamp_L_u32;
	uint32_t PollTimestamp_H_u32;
	uint32_t PollTimestamp_L_u32;
	uint32_t RespondTimestamp_H_u32;
	uint32_t RespondTimestamp_L_u32;
	uint32_t FinalTimestamp_H_u32;
	uint32_t FinalTimestamp_L_u32;
	uint32_t FinalDataTimestamp_H_u32;
	uint32_t FinalDataTimestamp_L_u32;
} UwbTimestampData_t;

#endif