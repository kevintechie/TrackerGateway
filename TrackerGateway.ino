/*
 * Copyright (c) 2016 Kevin Coleman
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation 
 * files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, 
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS 
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT 
 * OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <RFM69.h>
#include <SPI.h>
#include <Button.h>
#include <ButtonEventCallback.h>
#include <PushButton.h>
#include <Bounce2.h>

#define NODEID      1
#define NETWORKID   100
#define FREQUENCY   RF69_915MHZ
#define KEY         "ChangeMeChangeMe"
#define IS_RFM69HCW true
#define LED         8
#define BEEP_BUTTON 7
#define SERIAL_BAUD 115200
#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9
#define ACK_TIME    30

RFM69 radio;// = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

PushButton beepButton = PushButton(BEEP_BUTTON);

bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

enum PktType {
  GPS,
  MSG,
  BEEP
};

enum BeepType {
  NORMAL,
  EMERGENCY
};

typedef struct {
  PktType       pktType;
  int           nodeId; //store this nodeId
  float         latitude;
  float         longitude;
  unsigned long gpsDate;
  unsigned long gpsTime;
} GpsStruct;
GpsStruct gpsStruct;

typedef struct {
  PktType pktType;
  int     nodeId;
  char  msg[];
} MsgStruct;
MsgStruct msgStruct;

typedef struct {
  PktType  pktType;
  BeepType beepType;
} BeepStruct;

BeepStruct beepStruct;

void setup() {
  SPI.usingInterrupt(RFM69_IRQN);
  beepButton.onPress(onBeepButtonPress);
  while (!Serial); // wait until serial console is open, remove if not tethered to computer
  Serial.begin(SERIAL_BAUD);
  delay(10);

  Serial.println("Arduino RFM69HCW Receiver");

  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  radio.encrypt(KEY);
  radio.promiscuous(promiscuousMode);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}

byte ackCount = 0;
void loop() {
  beepButton.update();
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input == 'p')
    {
      promiscuousMode = !promiscuousMode;
      radio.promiscuous(promiscuousMode);
      Serial.print("Promiscuous mode "); Serial.println(promiscuousMode ? "on" : "off");
    }
  }

  if (radio.receiveDone())
  {
    Serial.print('['); Serial.print(radio.SENDERID, DEC); Serial.print("] ");
    Serial.print(" [RX_RSSI:"); Serial.print(radio.RSSI); Serial.print("]");
    if (promiscuousMode)
    {
      Serial.print("to ["); Serial.print(radio.TARGETID, DEC); Serial.print("] ");
    }

    if (radio.DATALEN != sizeof(GpsStruct))
      Serial.print("Invalid payload received, not matching Payload struct!");
    else
    {
      gpsStruct = *(GpsStruct*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
      Serial.print(" nodeId=");
      Serial.print(gpsStruct.nodeId);
      Serial.print(" Lat=");
      Serial.print(gpsStruct.latitude, 6);
      Serial.print(" Lng=");
      Serial.print(gpsStruct.longitude, 6);
      Serial.print(" ");
      Serial.print(gpsStruct.gpsDate);
      Serial.print(" ");
      Serial.print(gpsStruct.gpsTime);
    }

    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++ % 3 == 0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    Serial.println();
    Blink(LED, 3);
  }
}

void onBeepButtonPress(Button& beepButton) {
  Serial.println("Send Beep Pressed");
  beepStruct.pktType = BEEP;
  beepStruct.beepType = NORMAL;
  if (radio.sendWithRetry(2, (const void*)(&beepStruct), sizeof(beepStruct)))
    Serial.println(" ok!");
  else Serial.println(" nothing...");
  Blink(LED, 3);
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN, LOW);
}
