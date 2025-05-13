#include <SD.h>
#include <SPI.h>
#include <HardwareSerial.h>

#define SD_CS   10
#define SD_MOSI 12
#define SD_MISO 13
#define SD_SCK  11

#define MAX_RETRIES 5


SPIClass mySPI(FSPI);
#define RS485 Serial2  // RS485 on Serial2 (TX=17, RX=16)

// Unique IDs for each node — set accordingly
uint8_t thisID = 0x02;
uint8_t peerID  = 0x01;
uint8_t packetSeq = 0;     // Sender: increments each packet
uint8_t lastSeq   = 0xFF;  // Receiver: tracks last-seen sequence
#define ACK 0x06
#define NAK 0x15
#define REQ_SEND 0xB1
#define OK_READY 0xB2
#define EOF_FLAG 0xEE

// CRC-16-CCITT (XMODEM)
uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  while (len--) {
    crc ^= (*data++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

void setup() {
  Serial.begin(115200);        // USB Serial (for command)
  RS485.begin(115200, SERIAL_8N1, 6, 7); // RX=6, TX=7
  // Initialize SD Card
  mySPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, mySPI)) {
    Serial.println("SD Card initialization failed! Halting.");
    while (1);
  }
  Serial.println("SD card ready.");
}


bool sendDataPacket(uint8_t* payload, uint8_t length) {
  Serial.print("Sending packet of length "); Serial.println(length);
  uint8_t packet[128];
  int i = 0;
  packet[i++] = 0xAB;
  packet[i++] = 0xCD;
  packet[i++] = thisID;
  packet[i++] = peerID;
  packet[i++] = packetSeq++; 
  packet[i++] = length;
  memcpy(&packet[i], payload, length); i += length;
  uint16_t crc = crc16(&packet[2], 4 + length);
  packet[i++] = crc >> 8;
  packet[i++] = crc & 0xFF;
  packet[i++] = 0xDC;
  packet[i++] = 0xBA;

  for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
    RS485.write(packet, i);
    RS485.flush();  // ensure it's all sent
    delay(50);      // allow for settling

  // 🧹 Clear echoed garbage
  while (RS485.available()) {
    uint8_t junk = RS485.read();
    Serial.print("Flushed: 0x"); Serial.println(junk, HEX);
  }

    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (RS485.available()) {
        char resp = RS485.read();
        Serial.print("Got response: 0x"); Serial.println(resp, HEX);

        if (resp == ACK) {
          Serial.println("ACK received");
          return true;
        } else if (resp == NAK) {
          Serial.println("NAK received. Retrying...");
          break;  // retry
        } else {
          Serial.print("Ignoring garbage byte: 0x");
          Serial.println(resp, HEX);
        }
      }
    }

    Serial.println("No valid ACK/NAK. Retrying...");
  }

  Serial.println("Too many NAKs or timeouts. Aborting packet.");
  return false;
}



// bool waitForAck() {
//   uint32_t start = millis();
//   while (millis() - start < 500) {
//     if (RS485.available()) {
//       uint8_t resp = RS485.read();
//       if (resp == ACK) return true;
//       if (resp == NAK) return false;
//     }
//   }
//   return false;
// }

void sendFile(const char* filename) {
  File file = SD.open(filename);
  if (!file) {
    Serial.println("File open failed.");
    return;
  }

  RS485.write(REQ_SEND);
  uint32_t start = millis();
  while (millis() - start < 1000) {
    if (RS485.available() && RS485.read() == OK_READY) break;
  }

  uint8_t buffer[64];
  while (file.available()) {
    int len = file.read(buffer, sizeof(buffer));
    // sendDataPacket(buffer, len);
    // bool success = sendDataPacket(buffer, len);
    // if (!success) {
    //   Serial.println("Packet send failed after retries. Aborting.");
    //   file.close();
    //   return;
    // }
    bool ok = sendDataPacket(buffer, len);
    if (!ok) {
      Serial.println("Packet send failed after retries. Aborting.");
      file.close();
      return;
    }   
  

  }

  RS485.write(EOF_FLAG);
  file.close();
  Serial.println("File sent.");
}

void receiveFile(const char* destFile = "/recv.txt") {
  File outFile = SD.open(destFile, FILE_WRITE);
  if (!outFile) {
    Serial.println("Cannot open output file.");
    return;
  }

  RS485.write(OK_READY);
  bool receiving = true;

  while (receiving) {
    if (RS485.available() >= 8) {  // Wait for at least header + footer
      // Read until we find 0xAB 0xCD
      if (RS485.read() != 0xAB) continue;
      if (RS485.read() != 0xCD) continue;

      uint8_t sender = RS485.read();
      uint8_t target = RS485.read();
      if (target != thisID) continue; // Not for this node
      uint8_t seq = RS485.read();        // ← **NEW** sequence byte

      // Duplicate check
      if (seq == lastSeq) {
        // ACK again, but skip writing
        delay(50);
        RS485.write(ACK);
        RS485.flush();
        delay(10);
        continue;
      }
      lastSeq = seq;

      uint8_t len = RS485.read();
      if (len > 64) { RS485.read(); RS485.read(); continue; }

      // Wait until all bytes are available
      unsigned long start = millis();
      while (RS485.available() < len + 4 && millis() - start < 1000);

      if (RS485.available() < len + 4) {
        Serial.println("Timeout while waiting for packet.");
        RS485.write(NAK);
        continue;
      }

      uint8_t data[64];
      RS485.readBytes(data, len);
      uint8_t crc_hi = RS485.read();
      uint8_t crc_lo = RS485.read();
      uint16_t receivedCRC = (crc_hi << 8) | crc_lo;

      uint8_t stop1 = RS485.read();
      uint8_t stop2 = RS485.read();
      if (stop1 != 0xDC || stop2 != 0xBA) {
        RS485.write(NAK);
        continue;
      }

      // Recompute CRC
      // build buffer: sender, target, seq, length, payload
      uint8_t check[4 + len];
      check[0] = sender;
      check[1] = target;
      check[2] = seq;    // ← include seq here
      check[3] = len;
      memcpy(&check[4], data, len);

      // compute CRC over 4 header bytes + payload
      uint16_t calcCRC = crc16(check, 4 + len);
      Serial.print("Received CRC: 0x"); Serial.println(receivedCRC, HEX);
      Serial.print("Computed CRC: 0x"); Serial.println(calcCRC, HEX);

      if (calcCRC == receivedCRC) {
        outFile.write(data, len);
        Serial.println("Sending ACK...");
        delay(50);         // wait for sender to fully finish sending
        RS485.write(ACK);
        RS485.flush();     // ensure it's fully transmitted
        delay(10); 
      } else {
        delay(50);
        RS485.write(NAK);
        RS485.flush();
        delay(10);
      }
    }

    // Check for EOF flag
    if (RS485.available()) {
      uint8_t b = RS485.read();
      if (b == EOF_FLAG) {
        receiving = false;
        break;
      }
    }
  }

  outFile.close();
  Serial.println("File received.");
}


void processCommand(String cmd) {
  cmd.trim();
  if (cmd == "LIST") {
    File root = SD.open("/");
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      Serial.println(entry.name());
      entry.close();
    }
  } else if (cmd.startsWith("READ ")) {
    String name = cmd.substring(5);
    File f = SD.open("/" + name);
    if (!f) {
      Serial.println("File not found.");
      return;
    }
    while (f.available()) {
      Serial.write(f.read());
    }
    f.close();
  } else if (cmd.startsWith("SEND ")) {
    String name = cmd.substring(5);
    sendFile(("/" + name).c_str());
  } else if (cmd == "RECV") {
    receiveFile();
  } else {
    Serial.println("Commands: LIST, READ <f>, SEND <f>, RECV");
  }
}

void loop() {
  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}


