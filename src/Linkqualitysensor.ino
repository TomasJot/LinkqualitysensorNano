#include "Arduino.h"
#include <SPort.h>

//#define ERSKYTX
#define CH16 //16 or 8 ch mode
#define DISPLAYFS 10// maximum number of failsafes to display
#define LOWCHAN 8 //channel in 8 channel mode and low channel in 16ch mode
#define HIGHCHAN 16 // high channel in 16ch mode

#define SBUS_BAUD 100000
#define SBUS_THRESHOLD 5
#define SBUS_BUFFER 25
#define FRAMELOSS_BUFFER 100

#ifdef ERSKYTX
  #define PHYSICAL_ID 0x1B
  #define SENSOR_ID 0x0900
#else
  #define PHYSICAL_ID 0x12
  #define SENSOR_ID 0x5105
#endif



#define SPORT_PIN 3
#ifdef CH16
 #define STEPMIN 20
#else
 #define STEPMIN 10 //if FL - step is 2x from normal
#endif

SPortHub hub(PHYSICAL_ID, SPORT_PIN);
SimpleSPortSensor qualitySensor(SENSOR_ID);

byte sbusBuffer[SBUS_BUFFER];
long sbusIndex = 0;
byte sbusPreviousValue = 0;
bool sbusInFrame = false;
bool lastFrameLossesBuffer[FRAMELOSS_BUFFER];
int lastFrameLossesIndex = 0;
int previous_channel = 0;
int before_previous_channel = 0;
bool low_channel = true;
unsigned int failsafebuffer[DISPLAYFS];
byte failsafebuffercount = 0;
unsigned long lastmillis = 0;
unsigned long currentmillis = 0;
bool infailsafe = false;
bool failsafe = false;
bool failsafeoutput = false;
bool failsafecountinc = false;
byte failsafeoutputcount = 0;

#ifdef CH16
 int previous_channel_2 = 0;
 int before_previous_channel_2 = 0;
 bool in_sync = false;
#endif

void setup() {
    Serial.begin(SBUS_BAUD, SERIAL_8E2);
    hub.registerSensor(qualitySensor);
    hub.begin();
    for (byte i=0; i < 10; i++) {
      failsafebuffer[i] = 0;
  //    failsafebuffer[i] = i * 100 + 100;

    }
  }

void loop() {
    hub.handle();

    while (Serial.available()) {
        //Read MSB value from S.Bus stream
        byte value = Serial.read();

        //Only start a new frame if everything lines up correctly
        if(!sbusInFrame && value == 0x0F && sbusPreviousValue == 0x00) {
            sbusIndex = 0;
            sbusInFrame = true;
        }

        //When in frame, store the value in the buffer
        if(sbusInFrame) {
            sbusBuffer[sbusIndex] = value;
            sbusIndex++;

            //If all 25 bytes are received in the frame, handle it
            if(sbusIndex == 25) {
                sbusInFrame = false; //Stop capturing
                handleSBusFrame();
            }
        }

        sbusPreviousValue = value;
    }
}

void handleSBusFrame() {
  bool framelost;
  int current_channel;
  switch (LOWCHAN) {
    case 1: current_channel = ((sbusBuffer[1]|sbusBuffer[2]<<8) & 0x07FF); break;
    case 2: current_channel = ((sbusBuffer[2]>>3|sbusBuffer[3]<<5)  & 0x07FF); break;
    case 3: current_channel = ((sbusBuffer[3]>>6 |sbusBuffer[4]<<2 |sbusBuffer[5]<<10)  & 0x07FF); break;
    case 4: current_channel = ((sbusBuffer[5]>>1 |sbusBuffer[6]<<7) & 0x07FF); break;
    case 5: current_channel = ((sbusBuffer[6]>>4 |sbusBuffer[7]<<4) & 0x07FF); break;
    case 6: current_channel = ((sbusBuffer[7]>>7 |sbusBuffer[8]<<1 |sbusBuffer[9]<<9)   & 0x07FF); break;
    case 7: current_channel = ((sbusBuffer[9]>>2 |sbusBuffer[10]<<6) & 0x07FF); break;
    case 8: current_channel = ((sbusBuffer[10]>>5|sbusBuffer[11]<<3) & 0x07FF); break;
  }

  failsafe = sbusBuffer[23] & 0x08;
  if (failsafe) {
    failsafebuffer[failsafebuffercount]++;
    infailsafe = true;
    }
  else if (infailsafe) {
      if (failsafebuffer[failsafebuffercount] > 5) {
          failsafebuffercount++;
        }
      else {
        failsafebuffer[failsafebuffercount] = 0;
      }
      infailsafe = false;

      if (failsafebuffercount == DISPLAYFS) {
        failsafebuffercount = 0;
    }
  }

 #ifdef CH16
  int current_channel_2;
  switch (HIGHCHAN) {
    case 9: current_channel_2 = ((sbusBuffer[12]   |sbusBuffer[13]<<8) & 0x07FF); break;
    case 10: current_channel_2 = ((sbusBuffer[13]>>3|sbusBuffer[14]<<5)  & 0x07FF); break;
    case 11: current_channel_2 = ((sbusBuffer[14]>>6|sbusBuffer[15]<<2|sbusBuffer[16]<<10) & 0x07FF); break;
    case 12: current_channel_2 = ((sbusBuffer[16]>>1|sbusBuffer[17]<<7) & 0x07FF); break;
    case 13: current_channel_2 = ((sbusBuffer[17]>>4|sbusBuffer[18]<<4) & 0x07FF); break;
    case 14: current_channel_2 = ((sbusBuffer[18]>>7|sbusBuffer[19]<<1|sbusBuffer[20]<<9)  & 0x07FF); break;
    case 15: current_channel_2 = ((sbusBuffer[20]>>2|sbusBuffer[21]<<6) & 0x07FF); break;
    case 16: current_channel_2 = ((sbusBuffer[21]>>5|sbusBuffer[22]<<3) & 0x07FF); break;
  }

  if (previous_channel == 0 || previous_channel_2 == 0 ) {
    previous_channel = current_channel;
    previous_channel_2 = current_channel_2;
    return;
  }
 if (!in_sync) {
   if (current_channel != previous_channel || current_channel_2 != before_previous_channel) {
    in_sync = true;

    if (current_channel != previous_channel) {
    low_channel = true; }
  else {
   low_channel = false;
  }
 }
return;
}
 #endif


if (low_channel) {
   if (current_channel == previous_channel && previous_channel == before_previous_channel) {
    framelost = true;
  }
   else if ( (previous_channel == before_previous_channel) && (current_channel - previous_channel > STEPMIN || previous_channel - current_channel > STEPMIN)) {
     framelost = true;
   }
   else {
     framelost = false;
   }
   before_previous_channel = previous_channel;
   previous_channel = current_channel;
#ifdef CH16
   low_channel = false;
#endif
 }

#ifdef CH16
 if (!low_channel) {
   if (current_channel_2 == previous_channel_2 && previous_channel_2 == before_previous_channel_2) {
    framelost = true;
   }
   else if ( (previous_channel_2 == before_previous_channel_2) && (current_channel_2 - previous_channel_2 > STEPMIN || previous_channel_2 - current_channel_2 > STEPMIN)) {
     framelost = true;
   }
   else {
     framelost = false;
   }
   before_previous_channel_2 = previous_channel_2;
   previous_channel_2 = current_channel_2;
   low_channel = true;
}
#endif

  lastFrameLossesBuffer[lastFrameLossesIndex] = framelost;
  lastFrameLossesIndex++;
  if(lastFrameLossesIndex >= FRAMELOSS_BUFFER) {
    lastFrameLossesIndex = 0;
  }

  int lastLost = 0;

  for(int i = 0; i < FRAMELOSS_BUFFER; i++) {
      if(lastFrameLossesBuffer[i]) {
        lastLost++;
      }
  }
  #ifdef CH16
    if (lastLost == 100) {
    in_sync = false;
    }
  #endif
  currentmillis = millis();
  unsigned long timetodisplay = currentmillis - lastmillis;
 if (!failsafe && timetodisplay > 10000 && failsafebuffer[0] > 0) {
  failsafeoutput = true;
  failsafecountinc = true;
  lastmillis = currentmillis;
  timetodisplay = 0;
 }
if (failsafeoutput && timetodisplay < 1000) {
  qualitySensor.value = (double)failsafebuffer[failsafeoutputcount] * (double)0.09;
//  qualitySensor.value = failsafebuffer[failsafeoutputcount];

  }

if (timetodisplay > 1000) {
    failsafeoutput = false;
}

if (!failsafeoutput && failsafecountinc) {
  failsafeoutputcount++;
  if (failsafeoutputcount == DISPLAYFS || failsafebuffer[failsafeoutputcount] == 0) {
    failsafeoutputcount = 0;
  }
  failsafecountinc = false;
}

if (!failsafeoutput) {
  qualitySensor.value = 100 - (((double)lastLost / (double)FRAMELOSS_BUFFER) * (double)100);
}


}
