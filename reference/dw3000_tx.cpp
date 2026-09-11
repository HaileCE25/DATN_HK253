#include "DW3000.h"
#include "Arduino.h"
#include <Time.h>
extern UwbTimestampData_t timestamp_data;
static int frame_buffer; // Variable to store the transmitted message
static int rx_status;    // Variable to store the current status of the receiver operation
static int tx_status;
uint8_t pinToToggle = 5;
void setup()
{

  Serial.begin(115200); // Init Serial
  delay(2000);
  Serial.println(">>> BOOT OK");

  DW3000.begin();       // Init SPI
  DW3000.hardReset();   // hard reset in case that the chip wasn't disconnected from power
  delay(200);           // Wait for DW3000 chip to wake up

  // DEBUG: read DEV_ID as early as possible -- doesn't require IDLE state,
  // only that SPI is physically talking to the chip. Expected: 0xDECA0302.
  uint32_t dbg_dev_id = DW3000.read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
  Serial.print(">>> DEV_ID = 0x");
  Serial.println(dbg_dev_id, HEX);
  Serial.println(dbg_dev_id == 0xDECA0302 ? ">>> DEV_ID OK" : ">>> DEV_ID MISMATCH (expected 0xDECA0302)");

  // DEBUG: bounded retry with periodic register dump instead of the
  // original silent infinite hang (which also never re-checked the
  // condition -- see "while (100);" below in git history).
  uint32_t dbg_tries1 = 0;
  while (!DW3000.checkForIDLE()) // Make sure that chip is in IDLE before continuing
  {
    if ((dbg_tries1 % 10) == 0) {
      Serial.print(">>> waiting IDLE1... try=");
      Serial.print(dbg_tries1);
      Serial.print(" reg0F30=0x");
      Serial.print(DW3000.read(0x0F, 0x30), HEX);
      Serial.print(" reg0044=0x");
      Serial.println(DW3000.read(0x00, 0x44), HEX);
    }
    dbg_tries1++;
    if (dbg_tries1 > 200) { // ~10s timeout at 50ms/try
      Serial.println(">>> IDLE1 TIMEOUT, continuing anyway (debug mode)");
      break;
    }
    delay(50);
  }
  if (dbg_tries1 <= 200) Serial.println(">>> IDLE1 OK");

  DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
  delay(200);         // Wait for DW3000 chip to wake up

  uint32_t dbg_tries2 = 0;
  while (!DW3000.checkForIDLE())
  {
    if ((dbg_tries2 % 10) == 0) {
      Serial.print(">>> waiting IDLE2... try=");
      Serial.print(dbg_tries2);
      Serial.print(" reg0F30=0x");
      Serial.print(DW3000.read(0x0F, 0x30), HEX);
      Serial.print(" reg0044=0x");
      Serial.println(DW3000.read(0x00, 0x44), HEX);
    }
    dbg_tries2++;
    if (dbg_tries2 > 200) {
      Serial.println(">>> IDLE2 TIMEOUT, continuing anyway (debug mode)");
      break;
    }
    delay(50);
  }
  if (dbg_tries2 <= 200) Serial.println(">>> IDLE2 OK");

  Serial.println(">>> calling DW3000.init()...");
  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  Serial.println(">>> DW3000.init() returned");
  uint32_t tx_fctrl_val = DW3000.read(GEN_CFG_AES_LOW_REG, 0x24);
  Serial.print("tx_fctrl_val ");
  tx_fctrl_val = DW3000.read(0x0e, 0x12) & 0xFFFF;
  tx_fctrl_val |= 0x100000;
  DW3000.write(0x0e, 0x12, tx_fctrl_val, 3);
  DW3000.write(0x0e, 0x16, 0x9b, 1);
  DW3000.configureAsTX();          // Configure basic settings for frame transmitting
  DW3000.setTXAntennaDelay(16385); // set default antenna delay

  DW3000.InitCrypto();

  // Serial.println(tx_fctrl_val, HEX);

  delay(2000);
}
uint32_t exact_tx_timestamp;
long long rx_ts;
long long tx_ts;
uint32_t time_start;
uint32_t time_end;
uint32_t time_diff;
static uint8_t counting_index_base = IDLE;
uint8_t buffer_prepoll[127];
uint8_t buffer_finalData[127];
static uint8_t complete_sequence = 0;
static uint8_t counter_resp = 0;
void loop()
{
  uint8_t counting_index = 0;

  DW3000.clearSystemStatus();
  {
    switch (counting_index_base)
    {
    case PREPOLL:
    {
      complete_sequence = 0;
      Serial.println("start");
      
      DW3000.SwitchPackage(SP0);
      Serial.println("Prepoll");
      DW3000.setTXFrame(0, 64, buffer_prepoll); // Set content of frame
      DW3000.setFrameLength(64);                // Set Length of frame in bits
      DW3000.standardTX();

      counting_index_base = POLL;

      while (!(tx_status = DW3000.sentFrameSucc()))
      {
      };
      DW3000.DebugToggle();
      DW3000.SwitchPackage(SP3);
      DW3000.WritePollSTS_IV();
      rx_ts = DW3000.readTXTimestamp();
      exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY_4MS) >> 8;
      DW3000.writeTXDelay(exact_tx_timestamp);
      DW3000.delayedTX();
      break;
    }
    case POLL:
    {
      counter_resp = 1;
      while (!(tx_status = DW3000.sentFrameSucc()))
      {
      };
      DW3000.DebugToggle();
      DW3000.DebugToggle();
      counting_index_base = RESPOND;
      DW3000.SwitchPackage(SP3);
      tx_ts = DW3000.readTXTimestamp();
      timestamp_data.PollTimestamp_L_u32 = (uint32_t)(tx_ts & 0xFFFFFFFF);
      // Write RESPOND STS IV before listening — RX uses fixed offset +3
      DW3000.WriteRespondSTS_IV(3);
      DW3000.write(0x02, 0x04, 1, 1); // Load new STS IV
      // RX sends respond at poll_rx + 12ms, so use standardRX and wait long enough
      DW3000.standardRX();
      
      break;
    }
    case RESPOND:
    {
       time_start = millis();
       time_diff = 0; 
      while ((!(rx_status = DW3000.receivedFrameSucc())) && (time_diff < 15))
      {
        time_end = millis();
        time_diff = time_end - time_start;
      }; // Wait until frame was received (RX respond arrives ~12ms after poll)
      {
        uint32_t sys_lo = DW3000.read(0x00, 0x44);
        uint32_t sys_hi = DW3000.read(0x00, 0x48);
        Serial.print("TX:RESPOND sys_stat=0x");
        Serial.print(sys_hi, HEX);
        Serial.print("_");
        Serial.println(sys_lo, HEX);
      }
      // Read RX timestamp BEFORE putIdle to ensure register is valid
      if (rx_status != 0) {
        DW3000.SendRespond(counter_resp - 1);
      }
      DW3000.putIdle();
      DW3000.DebugToggle();
      DW3000.DebugToggle();
      DW3000.DebugToggle();
      Serial.print("TX:RESPOND rx_status=");
      Serial.print(rx_status);
      Serial.print(" time_diff=");
      Serial.println(time_diff);
      if (rx_status == 0)
      {
        // No frame received at all (timeout) — restart
        Serial.println("TX:RESPOND timeout, restarting");
        counting_index_base = IDLE;
        break;
      }
      // rx_status=1 (OK) or rx_status=2 (CRC error) both acceptable
      // For DS-TWR, only the timestamp matters, not the frame data
      // The respond frame is minimal (FCS-only), CRC error is expected
      {
        complete_sequence = 1;
        DW3000.WriteFinalSTS_IV();
        // SendRespond already called above (before putIdle) to capture RX timestamp
      }
      counter_resp++;
      // Skip unused responder slots (only 1 responder active)
      // Without this, 6 x 5ms dummy timeouts = 30ms wasted, causing FINAL delayed TX to miss its deadline
      if (complete_sequence == 1 && counter_resp < 8) {
        counter_resp = 8;
      }
      DW3000.WriteRespondSTS_IV(counter_resp);
      if (counter_resp < 8)
      {
        
        // exact_tx_timestamp = (long long)(tx_ts + TRANSMIT_DELAY_4MS_LESS*counter_resp) >> 8;
        // DW3000.writeRXDelay(exact_tx_timestamp);
        // DW3000.delayedRX();
        
        counting_index_base = RESPOND;
      }
      else if ((complete_sequence == 1) && (counter_resp == 8))
      {
        // Write FINAL STS IV before sending FINAL
        DW3000.SwitchPackage(SP3);
        DW3000.WriteFinalSTS_IV();
        DW3000.write(0x02, 0x04, 1, 1); // Load STS IV
        // Use standardTX instead of delayedTX to avoid HPDWARN issues
        // TX timestamp is read AFTER sending, so precision is not affected
        DW3000.standardTX();
        counting_index_base = FINAL;
      }

      break;
    }
    case FINAL:
    {
      Serial.println("TX:FINAL waiting for TX done");
      time_start = millis();
      time_diff = 0;
      while (!(tx_status = DW3000.sentFrameSucc()) && (time_diff < 10))
      {
        time_end = millis();
        time_diff = time_end - time_start;
      };
      if (tx_status != 1) {
        Serial.print("TX:FINAL TX failed! tx_status=");
        Serial.print(tx_status);
        uint32_t sys_stat = DW3000.read(0x00, 0x44);
        Serial.print(" sys_stat=0x");
        Serial.println(sys_stat, HEX);
        DW3000.putIdle();
        counting_index_base = IDLE;
        break;
      }
      Serial.println("TX:FINAL sent OK");
      rx_ts = DW3000.readTXTimestamp();
      DW3000.DebugToggle();
      DW3000.DebugToggle();

      // Store FINAL TX timestamp (TX sent FINAL, read TX_TIME register)
      timestamp_data.FinalTimestamp_L_u32 = (uint32_t)(rx_ts & 0xFFFFFFFF);
      DW3000.putIdle();
      DW3000.SwitchPackage(SP0);
      DW3000.PrepareFinalData(buffer_finalData);
      DW3000.setTXFrame(0, 107, buffer_finalData); // Set content of frame
      DW3000.setFrameLength(107);                // Set Length of frame in bits
      DW3000.standardTX();
      counting_index_base = FINALDATA;

      break;
    }
    case FINALDATA:
    {
      time_start = millis();
      time_diff = 0;
      
      while (!(tx_status = DW3000.sentFrameSucc()) && (time_diff < 10))
      {
        time_end = millis();
        time_diff = time_end - time_start;
      };
      if (tx_status != 1) {
        Serial.println("TX:FINALDATA TX failed");
        DW3000.putIdle();
        counting_index_base = IDLE;
        break;
      }

      DW3000.DebugToggle();
      DW3000.DebugToggle();
      DW3000.DebugToggle();
      DW3000.DebugToggle();
      counting_index_base = IDLE;
      break;
    }
    case SENDCIR:
    {

      DW3000.SwitchPackage(SP3);
      DW3000.setTXFrame(0x00, 0x00, NULL);
      DW3000.setFrameLength(0);
      DW3000.standardTX();
      while (!(tx_status = DW3000.sentFrameSucc()))
      {
      };
      time_start = millis();
      counting_index_base = SENDCIR;
      break;
    }
    case RECEIVECIR:
    {
      time_diff = 0;
      time_start = millis();
      DW3000.SwitchPackage(SP0);
      DW3000.standardRX();
      while ((!(rx_status = DW3000.receivedFrameSucc_noCIA())))
      {
        time_end = millis();
        time_diff = time_end - time_start;
      };

      if (1)
      {

        DW3000.ReadFristPath();
      }
      // delay(288);
      counting_index_base = IDLE;

      break;
    }
    case IDLE:
    {
      memset(buffer_prepoll, 0xAA, sizeof(buffer_prepoll));
      DW3000.putIdle();
      DW3000.SwitchPackage(SP0);
      DW3000.PreparePrepollData(buffer_prepoll);
      DW3000.Generate_KtURSK();
      DW3000.Generate_dURSK();
      DW3000.WriteRespondSTS();
      // DW3000.setTXFrame(0x00,0x00,NULL);
      // DW3000.setFrameLength(0);
      DW3000.FinalData(buffer_finalData);
      DW3000.Cleartimestamp();

      counting_index_base = PREPOLL;
      if (1) // digitalRead(5) == HIGH)
      {
        counting_index_base = PREPOLL;
      }
      else
      {
        counting_index_base = RECEIVECIR;
      }
      break;
    }
    default:
    {
      DW3000.putIdle();
      DW3000.Cleartimestamp();
      DW3000.SwitchPackage(SP0);
      DW3000.PrepollData(buffer_prepoll);
      DW3000.Generate_KtURSK();
      DW3000.Generate_dURSK();
      DW3000.WriteRespondSTS();
      DW3000.setTXFrame(0x00, 0x00, NULL);
      DW3000.setFrameLength(0);
      time_start = millis();
      counting_index_base = PREPOLL;
      break;
    }
    }
  }
}

// String inputString = "";      // a String to hold incoming data
// void serialEvent() {
//   while (Serial.available()) {
//     // get the new byte:
//     dont_run_cpd = 1;
//     counter_round = 0;
//     char inChar = (char)Serial.read();
//     // add it to the inputString:
//     if ((inChar == '\n') || (inChar == '\r'))
//     {

//     }
//     else
//     {
//       inputString += inChar;
//     }
//     // if the incoming character is a newline, set a flag so the main loop can
//     // do something about it:
//     if (inChar == '\n') {
//       Serial.print(inputString);
//       DW3000.SyncAndSend();
//       inputString = "";

//     }
//   }
// }