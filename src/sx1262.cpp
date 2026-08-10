#include "sx1262.h"
#include <RadioLib.h>
#include <string.h>

/* ============================================================================
 * OrionTracker - SX1262 LoRa telemetry link implementation.
 * See include/sx1262.h for the public API and packet layout.
 * ========================================================================== */

/* ---------------------------------------------------------------------------
 * PIN MAP - OrionTracker PCB wiring, SX1262 on SPI2.
 *   SPI2 bus:  SCK = PB10, MISO = PC2, MOSI = PC3
 *   Control:   NSS/CS = PC1, IRQ (DIO1) = PA2, BUSY = PA3, RST = PA0
 *   RF switch: PA1 (external antenna TX/RX switch, MCU-controlled)
 * DIO1 must be interrupt-capable and not share an EXTI line number with another
 * interrupt pin (PA2 = EXTI2, so avoid PB2/PC2 as *interrupt* sources).
 * NOTE: the schematic labels the IRQ line "DIO0"; the SX1262 IRQ RadioLib uses
 * is DIO1, wired here to PA2.
 * ------------------------------------------------------------------------- */
// SPI2 bus pins
#ifndef SX1262_SCK
#define SX1262_SCK   PB10
#endif
#ifndef SX1262_MISO
#define SX1262_MISO  PC2
#endif
#ifndef SX1262_MOSI
#define SX1262_MOSI  PC3
#endif
// Control lines
#ifndef SX1262_NSS
#define SX1262_NSS   PC1   // SPI chip-select (NSS/CS)
#endif
#ifndef SX1262_DIO1
#define SX1262_DIO1  PA2   // IRQ line (schematic "DIO0")
#endif
#ifndef SX1262_RST
#define SX1262_RST   PA0   // reset
#endif
#ifndef SX1262_BUSY
#define SX1262_BUSY  PA3   // busy
#endif
#ifndef SX1262_RFSW
#define SX1262_RFSW  PA1   // external RF (antenna) switch control
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
#define LORA_SYNCWORD    0x53     // private-network sync word (matches ground station)
#define LORA_PREAMBLE    8        // preamble length (symbols)

// ---------------------------------------------------------------------------
// Module + internal state (file-scoped; the free-function API wraps this)
// ---------------------------------------------------------------------------

// Dedicated SPI2 bus instance (STM32duino: SPIClass(MOSI, MISO, SCK)).
static SPIClass spi2(SX1262_MOSI, SX1262_MISO, SX1262_SCK);

// RadioLib driver bound to SPI2 (NSS, DIO1/IRQ, RST, BUSY, bus).
static SX1262 radio = new Module(SX1262_NSS, SX1262_DIO1, SX1262_RST, SX1262_BUSY, spi2);

// External RF switch (single MCU pin): drive one way for TX, the other for RX.
// RADIOLIB_NC pads the unused pin slots (RadioLib wants RFSWITCH_MAX_PINS = 3).
// !! VERIFY POLARITY against your PCB: this assumes HIGH = TX, LOW = RX. !!
static const uint32_t rfswitch_pins[] = {
  SX1262_RFSW, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  { Module::MODE_IDLE, { LOW,  LOW, LOW, LOW, LOW } },
  { Module::MODE_RX,   { LOW,  LOW, LOW, LOW, LOW } },
  { Module::MODE_TX,   { HIGH, LOW, LOW, LOW, LOW } },
  END_OF_MODE_TABLE,
};

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
  // Bring up the SPI2 bus with our pin mapping before touching the radio.
  spi2.begin();

  // Tell RadioLib how the external antenna switch is wired (before begin()).
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

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
  memcpy(txPacket.id, ORION_ID, ORION_ID_LEN);   // stamp the set/team identifier
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

const TelemetryPacket& radioPacket() {
  return txPacket;
}
