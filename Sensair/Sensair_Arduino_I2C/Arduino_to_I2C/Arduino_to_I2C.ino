// Arduino sketch for reading Sensair K30 CO2 sensor over I2C and sending data via XBee.
// Includes a retry mechanism and bus recovery to handle common I2C issues with the K30
// when used with long wires or in electrically noisy environments.
//
// Error handling and expected behavior is designed according to Sensair data sheet:
// https://rmtplusstoragesenseair.blob.core.windows.net/docs/Dev/publicerat/TDE4700.pdf
//
// Author: Jon Jaroker
// License: MIT

// TODO
// - Check "Error Status" at 0x1E (See section 8 of data sheet) periodically
//   to detect sensor faults (e.g. dirty optics) and report them via XBee.
//

#include <Wire.h>
#include <SoftwareSerial.h>

// XBee configuration
// Use 115200 to minimize interference with actuator servo
// Default is 9600 but that can cause more noise issues on the I2C
#define XBEE_BAUD_RATE 115200  

// K30 configuration
// Changed from 0x68 to 0x69 to avoid conflict with the data logger
#define K30_I2C_ADDR 0x69     // K30 default 7-bit address: 0x68; Any Sensor address: 0x7F
#define MAX_RETRIES 3         // Retries before resetting I2C bus

// I2C hardware pins (Uno/Nano): A4 = SDA, A5 = SCL
// These two pins are also used for the software I2C bus-recovery routine.
#define I2C_SCL_PIN  A5
#define I2C_SDA_PIN  A4

// Lower I2C clock to 50 kHz — more reliable on long wires than the 100 kHz default
// K30 SCL clock frequency is 100kHz
#define I2C_CLOCK_SPEED 50000UL 
#define I2C_TIMEOUT_MS 50UL   // Timeout for I2C reads (prevents infinite loop if sensor stops clocking)

// Datasheet Timing Constants
#define CMD_DELAY_MS 20       // K30 datasheet: ≥20 ms after write
#define RETRY_BACKOFF_MS 50   // extra wait between retries
#define WARMUP_MS 10000UL     // K30 power-on warm-up


// Set up XBee on digital pins 2 and 3
SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

void setup() {
  
  // Initialize XBee Software Serial port. 
  XBee.begin(XBEE_BAUD_RATE); 
  
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED); 
  
  XBee.print(F("K30 I2C CO2 sensor - warming up..."));  
  delay(WARMUP_MS);

  // TODO - Check "Error Status" at 0x1E (See section 8 of data sheet) 
  // to detect sensor faults before continuing

  XBee.println(F("Ready."));
  
}

void loop() {
  int co2 = readK30_CO2_withRetry();
  if (co2 > 0) {
    XBee.print(F("CO2 ppm: "));
    XBee.println(co2);
    
  } else {
    XBee.println(F("CO2 read failed after retries"));
    recoverI2CBus();
  }
  // K30 can lock up if read too frequently, so delay before next read
  delay(2500);
}

// Returns ppm or -1 on failure
int readK30_CO2_withRetry() {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    Wire.beginTransmission(K30_I2C_ADDR);

    //
    // K30 Request Command
    //
    // Read Ram
    //  Byte 1 - Command 0x22 (ReadRam with 2 data bytes)
    //  Byte 2-3 - address 0x0008 ("Space CO2")
    //  Byte 4 - checksum
    Wire.write(0x22); // 7:4 Command bits: 0x2 = read; 3:0 Num of Data bytes to read: 0x2 = 2 bytes (MSB + LSB)
    Wire.write(0x00); // Address MSB 
    Wire.write(0x08); // Address LSB
    // Checksum = sum of all command bytes (0x22 + 0x00 + 0x08 = 0x2A).
    Wire.write(0x2A);  // Checksum

    uint8_t err = Wire.endTransmission();
    if (err != 0) {
      // err codes: 1=data too long, 2=NACK on address, 3=NACK on data, 4=other
      XBee.print(F("endTransmission error: ")); XBee.println(err);
      delay(RETRY_BACKOFF_MS);
      continue;
    }

    // The K30 datasheet specifies ≥20 ms for the sensor to finish writing
    // the result to RAM before you read it back. The original 10 ms caused 
    // intermittent checksum failures.
    delay(CMD_DELAY_MS);  

    //
    // K30 Response
    //
    // Expect 4 Bytes (Section 5.3 Read Ram)
    //  Byte 1 - Status (0x21 "Read Complete" or 0x22 "Read Incomplete")
    //  Byte 2-3 - Data (MSB + LSB)
    //  Byte 4 - Checksum

    // Fill wire buffer with response from K30
    uint8_t received = Wire.requestFrom(K30_I2C_ADDR, (uint8_t)4);  // cast '4' to uint8_t to avoid warning about signed/unsigned mismatch

    // Confirm we got 4 bytes back; if not, something went wrong at the I2C level.
    if (received != 4) { // alternative: Wire.available() != 4
      XBee.println(F("ERROR: Expected 4 bytes, got " + String(received)));
      // Drain any leftover bytes to avoid poisoning the next transaction.
      XBee.print(F("Flushing I2C buffer..."));
      while (Wire.available()) Wire.read();  // flush partial data
      XBee.println(F("done"));
      delay(RETRY_BACKOFF_MS);
      continue;
    }

    // Timeout-guarded read — prevents infinite loop if sensor stops clocking
    // Load the wire buffer into an array for processing
    uint8_t buf[4];
    unsigned long startMs = millis();
    int bytesRead = 0;

    while (bytesRead < 4 && (millis() - startMs) < I2C_TIMEOUT_MS) {
      if (Wire.available() > 0) {
        buf[bytesRead++] = Wire.read();
      } else {
        delayMicroseconds(100);  // Prevent 100% CPU utilization
      }
    }

    // Expect buffer to be loaded within timeout
    // This is a redundant check given the earlier check on Wire.requestFrom(), 
    // but is used here to guard against and troubleshoot I2C lockups
    if (bytesRead != 4) {
      XBee.print(F("ERROR: Timeout Occurred. Loaded "));
      XBee.print(bytesRead);
      XBee.println(F(" bytes instead of 4"));
      XBee.print(F("Flushing I2C buffer..."));
      while (Wire.available()) Wire.read();  // flush partial data
      XBee.println(F("done"));
      delay(RETRY_BACKOFF_MS);
      continue;  // retry
    }

    // Expect status byte 0x21 "Read Complete"
    if (buf[0] != 0x21) {
      XBee.print(F("ERROR: Sensor status indicates read incomplete: 0x")); 
      XBee.println(buf[0], HEX);
      delay(RETRY_BACKOFF_MS);
      continue;
    }

    // Verify checksum
    uint8_t checksum = buf[0] + buf[1] + buf[2];
    if (checksum != buf[3]) {
      XBee.println(F("ERROR: Checksum fail"));
      delay(RETRY_BACKOFF_MS);
      continue;
    }

    // Assemble CO2 value from MSB and LSB
    // The expression (buf[1] << 8) promotes to int but shifts into or past the 
    // sign bit for any value ≥ 128 — undefined behavior in C/C++. 
    // For CO2 readings above ~32767 ppm (unlikely but possible in fault modes) 
    // the result could be spuriously negative. Fixed by casting first
    uint16_t co2 = ((uint16_t)buf[1] << 8) | buf[2];

    // Expect CO2 in a reasonable range for ambient air; if not, something went wrong
    if (co2 == 0 || co2 > 10000) { // 0 ppm is invalid; >10000 is out of K30 range
      XBee.println(F("ERROR: Invalid CO2 reading: " + String(co2)));
      delay(RETRY_BACKOFF_MS);
      continue;
    }

    // Successful read with valid checksum and plausible CO2 value
    return co2;
  }
  return -1;
}

// ══════════════════════════════════════════════════════════════════════════════
// recoverI2CBus()
//   Recovers a locked-up I2C bus by bit-banging up to 9 SCL pulses while SDA
//   is held high, then issuing a START followed by a STOP condition.
//
//   A lockup happens when the master resets (e.g. watchdog) mid-transaction:
//   the slave still holds SDA low waiting for more SCL pulses.  Nine clocks
//   are the worst-case number needed to clock out whatever byte the slave is
//   stuck on, regardless of which bit it was on when the master reset.
//
//   Reference: NXP UM10204 I2C-bus specification §3.1.16
// ══════════════════════════════════════════════════════════════════════════════
void recoverI2CBus() {
  XBee.println(F("Recovering I2C bus..."));

  // Release the Wire library before bit-banging the pins directly.
  Wire.end();

  // SCL driven by us; SDA must be INPUT_PULLUP so we can actually read the bus state
  pinMode(I2C_SCL_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    // If SDA has gone high the slave has released the bus — we can stop early.
    if (digitalRead(I2C_SDA_PIN) == HIGH) break;
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
  }

  // Issue a STOP condition
  digitalWrite(I2C_SCL_PIN, LOW);   // ensure SCL is low before SDA manipulation
  delayMicroseconds(5);
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);   // SDA low (with SCL low — safe)
  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH);  // SCL high
  delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH);  // SDA high while SCL high = STOP

  // Release both pins to INPUT_PULLUP before handing them back to Wire,
  // so Wire.begin() can cleanly take ownership without fighting an OUTPUT driver.
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  // Re-initialise Wire and restore reduced clock speed.
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  XBee.println(F("I2C bus recovery complete."));
}