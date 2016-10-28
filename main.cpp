#include "mbed.h"
#include "rtos.h"

#include <string.h>
#include <string>

#include "Pixy.h"
#include "util.h"

#include "eeprom.h" 

/* the order of the dsmxChannels on a spektrum :
 *
 * Throttle   0
 * Aileron    1
 * Elevator   2
 * Rudder     3
 * Gear       4
 * Flap/Aux1  5
 * Aux2       6
 * ???        7
 */

/* 
 * PAN - Aeleron (Ch1): 1024 neutral, <1024 pan right, >1024 pan left
 * TILT - Elevator (Ch2): 1024 neutral, <1024 tilt up, >1024 tilt down
 */
 
const int kChannelPan   = 1;
const int kChannelTilt  = 2;
const int kChannelEnable  = 4;

const int kDSMRangeMin = 406;
const int kDSMRangeMax = 1642;
const int kDSMRangeNeutral = 1024;

const int kDSMFrameTimeout = 100;   // Frame timeout in millis

const float kPixyInterval = 0.04;   // Pixy polling interval in seconds


/*
 * Global variables
 * 
 */
 
Serial dbg(USBTX, USBRX);
//Serial dbg(PC_10, PC_11);

EEPROM eeprom(PB_9, PB_8, 0x00, EEPROM::T24C08);

SPI spi(PB_15, PB_14, PB_13);
PixySPI pixy(&spi);

RawSerial dsmxUART(PA_0, PA_1);

DigitalOut led1(LED1);

Mutex stdio_mutex; 

Thread dsmxThread;
Thread pixyThread;


PID pidPan(kPixyInterval, -0.8, 0.2);
PID pidTilt(kPixyInterval, 0.8, 0.2);
PID pidCable(kPixyInterval, -0.8, 0.2);



struct Settings {
  uint8_t  valid;
  float kpPan;
  float tdPan;
  
  float kpTilt;
  float tdTilt;
  
  float kpCable;
  float tdCable;
};



uint16_t dsmxChannels[16];

volatile uint8_t dsmxFrame[16];

Timer rxTimer;
Timer rxFrameTimer;

void onDSMXData() 
{  
  static uint8_t frame[16];
  static uint8_t length;
  static uint8_t cnt;
  
  int elapsed_ms = rxTimer.read_ms();
  rxTimer.reset();
  
  if (elapsed_ms > 8) {
    // new frame
    if (++cnt > 10) {
      cnt = 0;
      led1 = !led1;
    }
    
    if (length == 16) {
      // copy old frame to receive buffer and notify DSMX thread
      memcpy((void *)dsmxFrame, frame, 16);
      rxFrameTimer.reset();
      dsmxThread.signal_set(0x1);
    }
    
    length = 0;
  }
  
  while (dsmxUART.readable()) {
    // read serial bytes and decode channels as they come
    uint8_t x = dsmxUART.getc();
    //dsmxUART.putc(x);
    if (length < 16) {
      frame[length++] = x;
    }
  }
}

void dsmxThreadProc() 
{  
  for (uint8_t idx = 0; idx < 16; idx++) {
    dsmxChannels[idx] = 0xFFFF;
  }

  dsmxUART.baud(115200);  
  rxTimer.start();
  rxFrameTimer.start();
  dsmxUART.attach(onDSMXData);

  while (true) {
    // Sync with the receiver
    osEvent evt = Thread::signal_wait(0x1, kDSMFrameTimeout);
    
    if (evt.status == osEventTimeout) {
      // Timeout, reset channels
      for (uint8_t idx = 0; idx < 16; idx++) {
        dsmxChannels[idx] = 0xFFFF;
      }
    }
    
    if (evt.status == osEventSignal) {      
      // Decode frame and modify if necessary
      for (uint8_t idx = 2; idx < 16; idx += 2) {
        uint8_t   hi = dsmxFrame[idx];
        uint8_t   lo = dsmxFrame[idx + 1];
        uint16_t  w = (hi << 8) | lo;
        if (w != 0xFFFF) {
          uint8_t   chID = ((w & 0x7800) >> 11);
          uint16_t  value = (w & 0x07FF);
          if (chID < 16) {
            dsmxChannels[chID] = value;
          }
          if (chID == kChannelPan) {
            float update = (float)value + pidPan.getCV(-1, 1) * (kDSMRangeMax - kDSMRangeNeutral);
            uint16_t newValue = clip(update, kDSMRangeMin, kDSMRangeMax);
            newValue = (chID << 11) | (newValue & 0x07FF);
            dsmxFrame[idx] = (newValue >> 8);
            dsmxFrame[idx + 1] = newValue;
          }
          if (chID == kChannelTilt) {
            float update = (float)value + pidTilt.getCV(-1, 1) * (kDSMRangeMax - kDSMRangeNeutral);
            uint16_t newValue = clip(update, kDSMRangeMin, kDSMRangeMax);
            newValue = (chID << 11) | (newValue & 0x07FF);
            dsmxFrame[idx] = (newValue >> 8);
            dsmxFrame[idx + 1] = newValue;
          }
        }
      }
          
      // Send frame
      for (uint8_t idx = 0; idx < 16; idx++) {
        dsmxUART.putc(dsmxFrame[idx]);
      }

      /*
      for (uint8_t idx = 0; idx < 16; idx++) {
        dbg.printf("%02x ", dsmxFrame[idx]);
      }
      dbg.printf("\n");
      */

      /*
      static uint16_t counter;
      if (++counter > 20) {
        counter = 0;
        stdio_mutex.lock();
        for (uint8_t idx = 0; idx < 16; idx++) {
          if (dsmxChannels[idx] != 0xFFFF) 
            dbg.printf("%d:%.1f%% ", idx, ((int16_t)dsmxChannels[idx] - 406) / 12.36f);
            //dbg.printf("%d:%u ", idx, dsmxChannels[idx]);
        }
        dbg.printf("\n");
        stdio_mutex.unlock();
      }
      */

    }

         
    /*
    */
    
    //Thread::wait(100);
  }
}

PwmOut cableRC(PC_6);
PwmOut auxRC(PC_7);


void pixyThreadProc() 
{    
  cableRC.period_ms(20);
  auxRC.period_ms(20);
    
  cableRC.pulsewidth_us(1500);
  auxRC.pulsewidth_us(1500);
    
  pixy.init();
  
  uint8_t timeout = 0;
  bool wasEnabled = false;
  
  while (true) {  
    // Check CMUCam
    if (dsmxChannels[kChannelEnable] == 0xFFFF || 
        dsmxChannels[kChannelEnable] < kDSMRangeNeutral) 
    {
      if (wasEnabled) {
        dbg.printf("Disabling controls\n");
        wasEnabled = false;
        cableRC.pulsewidth_us(1500);
        auxRC.pulsewidth_us(1500);
        pidPan.cv = 0;
        pidTilt.cv = 0;
        pidCable.cv = 0;
      }
    }
    else {
      if (!wasEnabled) {
        dbg.printf("Enabling controls\n");
        wasEnabled = true;
      }
      uint16_t nBlocks = pixy.getBlocks();
      if (nBlocks > 0) {
        dbg.printf("Block!\n");
        Block &block = pixy.blocks[0];
        
        float cx = (float)block.x / 160.0f - 1;     // X coordinate 0..319
        float cy = (float)block.y / 100.0f - 1;     // Y coordinate 0..199

        pidPan.update(cx);
        pidTilt.update(cy);
        pidCable.update(cx);

        cableRC.pulsewidth_us(1500 + 500 * pidCable.getCV(-1, 1));
        //auxRC.pulsewidth_us(1500 + 500 * pidTilt.getCV(-1, 1));

        timeout = 5;
      }
      else {
        // no pixy block found - stop the movement after a timeout
        if (timeout > 0) {
          timeout--;
        }
        else {
          cableRC.pulsewidth_us(1500);
          auxRC.pulsewidth_us(1500);
          pidPan.cv = 0;
          pidTilt.cv = 0;
          pidCable.cv = 0;
        }        
      }
    }
   
    Thread::wait(40);
  }
}

void readSettings() {
  Settings settings;
  settings.valid = 0xFF;
   
  uint8_t request[] = { 0x00 };
  uint8_t reply[4];

  /*
  i2c.frequency(100000);
  dbg.printf("Scanning I2C devices...\n");
  for (uint8_t addr = 0x08; addr < 0xFF; addr++) {
    if (0 == i2c.write(addr, (const char *)request, 0)) {
      dbg.printf("Found I2C device %02x\n", addr);
    }
    Thread::wait(20);
  }
  dbg.printf("Done!\n");
  */

  /*
  uint8_t addr = 0b1010000;
    
  if (0 != i2c.write(addr, (const char *)request, 1)) {
    dbg.printf("No ACK after write\n");
  }
  if (0 != i2c.read(addr, (char *)reply, 4)) {
    dbg.printf("No ACK after read\n");
  }
  */
  
  eeprom.read(0, (void *)&settings, sizeof(settings));
  
  if (settings.valid != 0xFF) {
    pidPan.kp = settings.kpPan;
    pidPan.td = settings.tdPan;
    pidTilt.kp = settings.kpTilt;
    pidTilt.td = settings.tdTilt;
    pidCable.kp = settings.kpCable;
    pidCable.td = settings.tdCable;    
    dbg.printf("Loaded settings from EEPROM (error code %d)\n", (int)eeprom.getError());
  }
  else {
    dbg.printf("No settings found in EEPROM (error code %d)\n", (int)eeprom.getError());
  }
  
}

void saveSettings() {
  Settings settings;
  settings.valid = 0x00;
  
  settings.kpPan = pidPan.kp;
  settings.tdPan = pidPan.td;
  settings.kpTilt = pidTilt.kp;
  settings.tdTilt = pidTilt.td;
  settings.kpCable = pidCable.kp;
  settings.tdCable = pidCable.td;   
  
  eeprom.write(0, (void *)&settings, sizeof(settings));   
  dbg.printf("Saved settings to EEPROM (error code %d)\n", (int)eeprom.getError());
}


void processLine(const char *line) 
{
  char  tok[4][20];
  int   tokCount = 0;
  
  while (tokCount < 4) {
    int tokLength = 0;
    if (1 != sscanf(line, "%20s %n", tok[tokCount], &tokLength)) {
      break;
    }
    if (tokLength == 0) break;
    line += tokLength;
    //dbg.printf("%s ", tok[tokCount]);
    tokCount++;
  }
  //dbg.printf("\n");
    
  bool isError = true;
  if (strcmp(tok[0], "kp1") == 0) {
    if (tokCount > 1) {
      pidPan.kp = atof(tok[1]);
      saveSettings();
    }
    dbg.printf("kp1 = %.2f\n", pidPan.kp);
    isError = false;
  }
  else if (strcmp(tok[0], "kp2") == 0) {
    if (tokCount > 1) {
      pidTilt.kp = atof(tok[1]);
      saveSettings();
    }
    dbg.printf("kp2 = %.2f\n", pidTilt.kp);
    isError = false;
  }
  else if (strcmp(tok[0], "kp3") == 0) {
    if (tokCount > 1) {
      pidCable.kp = atof(tok[1]);
      saveSettings();
    }
    dbg.printf("kp3 = %.2f\n", pidCable.kp);
    isError = false;
  }
  else if (strcmp(tok[0], "td1") == 0) {
    if (tokCount > 1) {
      pidPan.td = atof(tok[1]);
      saveSettings();
    }
    dbg.printf("td1 = %.2f\n", pidPan.td);
    isError = false;
  }
  else if (strcmp(tok[0], "td2") == 0) {
    if (tokCount > 1) {
      pidTilt.td = atof(tok[1]);
      saveSettings();
    }
    dbg.printf("td2 = %.2f\n", pidTilt.td);
    isError = false;
  }
  else if (strcmp(tok[0], "td3") == 0) {
    if (tokCount > 1) {
      pidCable.td = atof(tok[1]);
      saveSettings();
    }
    dbg.printf("td3 = %.2f\n", pidCable.td);
    isError = false;
  }
  if (strcmp(tok[0], "x") == 0) {
    dbg.printf("PID Pan/Tilt/Cable\n");
    dbg.printf("kp1 = %.2f\n", pidPan.kp);
    dbg.printf("kp2 = %.2f\n", pidTilt.kp);
    dbg.printf("kp3 = %.2f\n", pidCable.kp);
    dbg.printf("td1 = %.2f\n", pidPan.td);
    dbg.printf("td2 = %.2f\n", pidTilt.td);
    dbg.printf("td3 = %.2f\n", pidCable.td);
    isError = false;
  }
  if (isError) {
    dbg.printf("error\n");
  }
  else {
    //dbg.printf("OK\n");
  }
}

FIFO<char, 64> rxBuffer;

void onSerialData() 
{
  while (dbg.readable()) {
    char c = dbg.getc();
    // Local echo
    dbg.putc(c);
    
    rxBuffer.put(c);
  }
}

// main() runs in its own thread in the OS
int main() 
{ 
  dbg.baud(115200);
  dbg.printf("Reset\n");
        
  readSettings();
        
  dsmxThread.start(&dsmxThreadProc);
  pixyThread.start(&pixyThreadProc);
        
  dbg.attach(onSerialData);
    
  char line[80];
  uint8_t lineLength = 0;
  
  while (true) {  
    while (!rxBuffer.empty()) {
      char c;
      if (!rxBuffer.get(c)) break;
      if (c == '\r' || c == '\n') {
        line[lineLength] = '\0';
        if (lineLength > 0) processLine(line);
        lineLength = 0;
      }
      else {
        if (lineLength < 80 - 1) {
          line[lineLength++] = c;
        }
      }
    }
    Thread::wait(100);
    led1 = !led1;    
    //dbg.printf("Waiting...\n");
  }
}
