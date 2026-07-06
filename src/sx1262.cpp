#include "sx1262.h"
#include <RadioLib.h>
#include <string.h>

/* ============================================================================
 * OrionTracker - SX1262 LoRa telemetry link implementation.
 * See include/sx1262.h for the public API and packet layout.
 * ========================================================================== */

/* ---------------------------------------------------------------------------
 * PIN MAP - SET THESE TO YOUR OrionTracker PCB WIRING.
 * The SX1262 control lines below are GPIO; SCK/MISO/MOSI use the default SPI
 * bus (STM32F412RE: SPI1 = PA5/PA6/PA7). DIO1 must be an interrupt-capable pin
 * and must not share an EXTI line number with another interrupt pin you use
 * (e.g. PB7 = EXTI7, so avoid PA7/PC7 as interrupts elsewhere).
 * ------------------------------------------------------------------------- */
#ifndef SX1262_NSS
#define SX1262_NSS   PB6   // SPI chip-select (NSS)
#endif
#ifndef SX1262_DIO1
#define SX1262_DIO1  PB7   // IRQ line (TxDone)
#endif
#ifndef SX1262_RST
#define SX1262_RST   PB8   // reset
#endif
#ifndef SX1262_BUSY
#define SX1262_BUSY  PB9   // busy
#endif

/* ---------------------------------------------------------------------------
 * LoRa link parameters - MUST MATCH THE GROUND STATION EXACTLY.
 * SF7 / BW500 / CR4-5 @ 915 MHz gives ~22 kbps: a good range/latency balance
 * for L2 telemetry (~26 ms airtime for this packet, easily 10 Hz).
 *   - Set LORA_FREQUENCY to your legal band (868.0 for EU, watch duty cycle).
 *   - Lower SF / higher BW = faster but shorter range; the reverse for range.
 * ------------------------------------------------------------------------- */
#define LORA_FREQUENCY   915.0f   // MHz
#define LORA_BANDWIDTH   500.0f   // kHz
#define LORA_SPREADING   7        // spreading factor (5..12)
#define LORA_CODINGRATE  5        // coding rate 4/5
#define LORA_TXPOWER     22       // dBm (SX1262 max is +22)
#define LORA_SYNCWORD    0x34     // private-network sync word
#define LORA_PREAMBLE    8        // preamble length (symbols)

// ---------------------------------------------------------------------------
// Module + internal state (file-scoped; the free-function API wraps this)
// ---------------------------------------------------------------------------

// RadioLib driver bound to the default SPI bus.
static SX1262 radio = new Module(SX1262_NSS, SX1262_DIO1, SX1262_RST, SX1262_BUSY);

static TelemetryPacket txPacket;        // the packet currently being assembled
static volatile bool   txDone = true;   // true when the radio is free to send
static bool            initialised = false;
static uint16_t        seqCounter = 0;
static uint32_t        lastGpsMs  = 0;   // millis() of the last radioSetGPS()

// ---------------------------------------------------------------------------
// ISR - fires when the SX1262 finishes sending a packet. Keep it tiny: it only
// flips a flag. No SPI/radio calls, no Serial - those are illegal in an ISR.
// ---------------------------------------------------------------------------
static void onTxDone() {
  txDone = true;
}

// ---------------------------------------------------------------------------
// Private configuration helpers (the "private functions" from the header)
// ---------------------------------------------------------------------------
static bool setFrequency(float mhz) { return radio.setFrequency(mhz)   == RADIOLIB_ERR_NONE; }
static bool setTxPower(int8_t dbm)  { return radio.setOutputPower(dbm) == RADIOLIB_ERR_NONE; }
static bool setSyncWord(uint8_t sw) { return radio.setSyncWord(sw)     == RADIOLIB_ERR_NONE; }

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool radioInit() {
  // begin() resets the chip and sets the carrier frequency.
  if (radio.begin(LORA_FREQUENCY) != RADIOLIB_ERR_NONE) {
    return false;
  }

  bool ok = true;
  ok &= (radio.setBandwidth(LORA_BANDWIDTH)       == RADIOLIB_ERR_NONE);
  ok &= (radio.setSpreadingFactor(LORA_SPREADING) == RADIOLIB_ERR_NONE);
  ok &= (radio.setCodingRate(LORA_CODINGRATE)     == RADIOLIB_ERR_NONE);
  ok &= (radio.setPreambleLength(LORA_PREAMBLE)   == RADIOLIB_ERR_NONE);
  ok &= (radio.setCRC(true)                       == RADIOLIB_ERR_NONE);
  ok &= setFrequency(LORA_FREQUENCY);
  ok &= setTxPower(LORA_TXPOWER);
  ok &= setSyncWord(LORA_SYNCWORD);
  if (!ok) {
    return false;
  }

  // Register the TxDone interrupt so transmits can be non-blocking.
  // (RadioLib v6+ name; older versions call this setDio1Action().)
  radio.setPacketSentAction(onTxDone);

  // Zero the packet and stamp the constant header field.
  memset(&txPacket, 0, sizeof(txPacket));
  txPacket.magic  = ORION_PACKET_MAGIC;
  txPacket.gpsAge = 0xFFFFFFFF;   // "no GPS update yet"

  txDone      = true;
  initialised = true;
  return true;
}

void radioSetGPS(int32_t lat, int32_t lon, float altMSL,
                 uint8_t sats, uint8_t fixType) {
  txPacket.lat     = lat;
  txPacket.lon     = lon;
  txPacket.altGps  = altMSL;
  txPacket.sats    = sats;
  txPacket.fixType = fixType;
  lastGpsMs        = millis();   // age is computed at send time in radioFormat()
}

void radioSetBaro(float altBaro, float pressureHpa, float temperatureC) {
  txPacket.altBaro     = altBaro;
  txPacket.pressure    = pressureHpa;
  txPacket.temperature = temperatureC;
}

void radioSetIMU(float ax, float ay, float az, float gx, float gy, float gz) {
  txPacket.accX  = ax; txPacket.accY  = ay; txPacket.accZ  = az;
  txPacket.gyroX = gx; txPacket.gyroY = gy; txPacket.gyroZ = gz;
}

void radioSetState(uint8_t flightState) {
  txPacket.state = flightState;
}

bool radioFormat() {
  if (!initialised) {
    return false;
  }
  uint32_t now = millis();
  txPacket.magic  = ORION_PACKET_MAGIC;
  txPacket.seq    = seqCounter;
  txPacket.t_ms   = now;
  txPacket.gpsAge = (lastGpsMs == 0) ? 0xFFFFFFFF : (now - lastGpsMs);
  return true;
}

bool radioSend() {
  if (!initialised) {
    return false;
  }
  if (!txDone) {
    return false;   // previous packet is still in the air - try again next loop
  }

  radioFormat();

  txDone = false;
  int state = radio.startTransmit((uint8_t*)&txPacket, sizeof(txPacket));
  if (state != RADIOLIB_ERR_NONE) {
    txDone = true;  // start failed - clear the flag so we retry next cycle
    return false;
  }

  seqCounter++;
  return true;
}

bool radioSendBlocking() {
  if (!initialised) {
    return false;
  }
  radioFormat();
  int state = radio.transmit((uint8_t*)&txPacket, sizeof(txPacket));
  if (state != RADIOLIB_ERR_NONE) {
    return false;
  }
  seqCounter++;
  txDone = true;
  return true;
}

bool radioBusy() {
  return !txDone;
}

uint16_t radioPacketCount() {
  return seqCounter;
}
