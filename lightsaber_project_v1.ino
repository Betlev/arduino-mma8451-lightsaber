#include <WiredDevice.h>

#include <RegisterBasedWiredDevice.h>

#include <Wire.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>
#include <pcmConfig.h>
#include <pcmRF.h>
#include <TMRpcm.h>
#include <SPI.h>
#include <SD.h>



/*
Notas: 
configureRegisterBits: esta funcion busca la direccion de configuracion, aplica una mascara AND (como una mascara de red a una IP) y sobre el resutado modifica los bit indicados
param1 : la direccion de configuracion
param2 : mascara de bits a aplicar (AND)
param3 : los bits a activar/desactivar (OR)




*/

TMRpcm audio;


AccelerometerMMA8451 acc(0); // SA0 of MMA8451 is LOW so Address is 0x1C
volatile bool ready = false;
bool ignited = false;
int pin10 =0;
unsigned char buf[6];
byte snd_typ = B0000101;// 0 off, 1 on, 2 clash, 3 swing, 4 hum
void processXYZ(unsigned char* buf) {
    char axis[] = {'x', 'y', 'z'};
    for (int i = 0; i < 6; i++) {
        Serial.print(i, DEC);
        Serial.print(": 0x");
        Serial.println(buf[i], HEX);
    }
    for (int i = 0; i < 6; i += 2) {
        Serial.print(axis[i / 2]);
        Serial.print(": ");
        Serial.println(acc.convertToG(&buf[i], true));// poner a true y probar
    }
}

void isr() {
    ready = true;
}

void setup() {

    Serial.begin(9600);
    Serial.print("Setup...");

    // Step 1: Put the device into Standby Mode: Register 0x2A CTRL_REG1
    acc.standby();
    acc.configureRegisterBits(AccelerometerMMA8451::TRANSIENT_CFG, 0x1E, 0x1E);
    acc.configureRegisterBits(AccelerometerMMA8451::TRANSIENT_THS, 0x40, 0x40); //4g
    acc.writeRegister(AccelerometerMMA8451::TRANSIENT_COUNT, 0x03);

    
    // Step 2: Set Configuration Register for Motion Detection by setting the 
    // "OR" condition OAE = 1, enabling X, Y, and the latch
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_ELE, 0x80); //ELE
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_OAE, 0x40); //OAE

    // Event flag enable on X, Y and Z event.
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_ZEFE, 0x20);
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_YEFE, 0x10);
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_CFG, AccelerometerMMA8451::FF_MT_CFG_XEFE, 0x08);

    // Step 3: Threshold Setting Value for the Motion detection of > 1g
    // Note: The step count is 0.063g/ count
    // 1g/0.063g = 15.8; 
    // Round up to 16
    acc.configureRegisterBits(AccelerometerMMA8451::FF_MT_THS, AccelerometerMMA8451::FF_MT_THS_THS, 0x10);

    // Step 4: Set the debounce counter to eliminate false readings for 100 Hz 
    // sample rate with a requirement of 100 ms timer.
    // Note: 100 ms/10 ms (steps) = 10 counts
    acc.writeRegister(AccelerometerMMA8451::FF_MT_COUNT, 0x0a);

    // Configure the INT pins for Open Drain
    acc.setPushPullOpenDrain(AccelerometerMMA8451::PUSH_PULL);

    // Step 5: Enable Motion/Freefall Interrupt Function in the System
    // disable all interrupt sources we only want Freefall or Motion interrupt
    acc.disableInterrupt(AccelerometerMMA8451::INT_ALL); 
    acc.enableInterrupt(AccelerometerMMA8451::INT_FF_MT, 1);
    acc.enableInterrupt(AccelerometerMMA8451::INT_TRANS, 1);

    // Step 7: Put the device in Active Mode
    acc.activate();


    // interupt 2 > RX pin
    // Step 8: Write a Service Routine to Service the Interrupt
    attachInterrupt(2, isr, FALLING);
  
    if (!SD.begin(4)) {
      return;
    }
    pinMode(A0,OUTPUT);
    pinMode(A1,OUTPUT);
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    pinMode(A4,OUTPUT);
    pinMode(5, OUTPUT);
    audio.speakerPin = 5;
    audio.volume(1);
    audio.setVolume(4);
    pinMode(10,INPUT_PULLUP);
    pinMode(12,OUTPUT);
    snd_typ=1;
}
unsigned long tm = millis(); // timeout counter 

void loop() {
 
  //Serial.println(ignited);
  int rnd = random(1,5);
  pin10 = digitalRead(10);

  AccelerometerMMA8451::INT_SOURCEbits source;
if(!ignited && pin10==LOW){
      snd_typ=1;
      
      audio.play("on.wav");
      digitalWrite(7,HIGH);//LOW
      delay(50);
      digitalWrite(A4,HIGH);//03 - 02
      delay(50);
      digitalWrite(A1,HIGH);//05 - 03
      delay(50);
      digitalWrite(12,HIGH);//TOP - 04
      delay(50);
      digitalWrite(A0,HIGH);//02 -05 
      delay(50);
      digitalWrite(8,HIGH);//04
      ignited = true;
      pin10 = HIGH;
}
 if(ignited){
      digitalWrite(12,HIGH);
      digitalWrite(A0,HIGH);
      digitalWrite(A1,HIGH);
      digitalWrite(7,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(A4,HIGH);

      if( pin10==LOW){
        ignited = false;
        snd_typ=0;
        audio.play("off.wav");

        digitalWrite(8,LOW);
        delay(50);
        digitalWrite(A0,LOW);
        delay(50);
        digitalWrite(12,LOW);
        delay(50);
        digitalWrite(A1,LOW);
        delay(50);
        digitalWrite(A4,LOW);
        delay(50);
        digitalWrite(7,LOW);
      }
  
      if (!audio.isPlaying()) {
          snd_typ=4;
          audio.play("hum.wav");
      }
          


      if((unsigned long) (millis() - tm) > 500){
        source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);

        AccelerometerMMA8451::STATUSbits status;
        status.value = acc.readRegister(AccelerometerMMA8451::STATUS);

        AccelerometerMMA8451::CTRL_REG3bits ctrl_reg3;
        ctrl_reg3.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG3);

        AccelerometerMMA8451::CTRL_REG4bits ctrl_reg4;
        ctrl_reg4.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG4);
        AccelerometerMMA8451::CTRL_REG5bits ctrl_reg5;
        ctrl_reg5.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG5);

        if (source.value) {
            ready = true;
        } 
      }
            
      
          if (ready) {//evento activado
              ready = false;
              //Determine the source of the interrupt by first reading the system interrupt register
      
              source.value = acc.readRegister(AccelerometerMMA8451::INT_SOURCE);
                     
              // Set up Case statement here to service all of the possible interrupts
              
              // Data Ready / Data Overflow Interrupt (bit 0 of source)

              // Freefall or Motion Detection Interrupt (bit 2 of source)
              if (source.SRC_FF_MT) { 
                 AccelerometerMMA8451::STATUSbits status;
                 status.value = acc.readRegister(AccelerometerMMA8451::STATUS);
                 //Read the FF_MT State from the Status Register, clear the interrupt, by reading the FF_MT_SRC Register
                 AccelerometerMMA8451::FF_MT_SRCbits ff_mt_src;
                 ff_mt_src.value = acc.readRegister(AccelerometerMMA8451::FF_MT_SRC);
                 AccelerometerMMA8451::CTRL_REG4bits ctrl_reg4;
                 ctrl_reg4.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG4);
                 AccelerometerMMA8451::CTRL_REG5bits ctrl_reg5;
                 ctrl_reg5.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG5);
     
                 // 10 seconds since last interrupt is something wrong? read most of the MMA8451's status regs
                 if ((unsigned long) (millis() - tm) > 100 ) {
                    tm = (unsigned long) millis(); // restart timeout counter
                    if(snd_typ >3){
                      char hup2[11];
                      sprintf(hup2, "swing%u.wav", rnd);
                       snd_typ = 3;
                       audio.stopPlayback();
                       audio.play(hup2);
                     }
                 }
                  //Read the FF_MT State from the Status Register, clear the interrupt, by reading the FF_MT_SRC Register
                  Serial.print("FreeFall Motion Interrupt: ");
                  Serial.println(ff_mt_src.value, BIN);
              }
         
              // Transient interrupt (bit 5 of status)
              if (source.SRC_TRANS) { 
                
                    if(snd_typ >2){
                        char clash2[9];
                        sprintf(clash2, "hit%u.wav", rnd);
                        snd_typ = 2;
                        audio.stopPlayback();
                        audio.play(clash2);
                  }
                    
                    
                  // Read TRANS_SRC register to clear interrupt
                 
                 AccelerometerMMA8451::TRANSIENT_SRCbits trans_src;
                 trans_src.value = acc.readRegister(AccelerometerMMA8451::TRANSIENT_SRC);

                  AccelerometerMMA8451::STATUSbits status;
                  status.value = acc.readRegister(AccelerometerMMA8451::STATUS);
          
                  AccelerometerMMA8451::CTRL_REG4bits ctrl_reg4;
                  ctrl_reg4.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG4);
          
                  AccelerometerMMA8451::CTRL_REG5bits ctrl_reg5;
                  ctrl_reg5.value = acc.readRegister(AccelerometerMMA8451::CTRL_REG5);
      
                  Serial.print("Transient Detected ");
                  Serial.println(trans_src.value, BIN);

              }
              
        
      
              // init timeout counter
              tm = (unsigned long) millis();
          }
      }
}
