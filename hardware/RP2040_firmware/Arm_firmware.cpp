/*
 * Arm stepper motor firmware for RP2040
 * ------------------------------------------------------------------
 *  – Two TMC2209 drivers (STEP/DIR/EN)
 *  – Two AS5600 magnetic encoders read through a TCA9548A I²C mux
 *  – Per‑axis PID position control (PID ➜ target velocity ➜ step rate)
 *  – Serial CLI (same syntax you used before):
 *        0e 1x 0r 1s 0t90 1p150 0i1.2 0d0
 *  – **Telemetry** every 50 ms only:
 *        M0 tgt=  90.0° pos=  87.4° per=  40 µs | M1 tgt=‑45.0° pos=‑44.7° per=5000 µs
 *    No maxLoop / maxI2C diagnostics anymore.
 *
 *  – I²C read guarded by a 1 ms timeout so a bus glitch skips a cycle but
 *    never freezes the loop.
 */

 #include <Arduino.h>
 #include <Wire.h>
 
 // ------------------ I²C addresses -------------------------
 constexpr uint8_t TCA_ADDR = 0x70;
 constexpr uint8_t AS5600_ADDR = 0x36;
 constexpr uint8_t AS5600_RAW_ANGLE_H = 0x0C;
 
 // ------------------ GPIO map ------------------------------
 constexpr uint8_t M0_STEP_PIN = 2, M0_DIR_PIN = 3, M0_EN_PIN = 6;
 constexpr uint8_t M1_STEP_PIN = 7, M1_DIR_PIN = 8, M1_EN_PIN = 9;
 constexpr uint8_t LIMIT_0_PIN = 21, LIMIT_1_PIN = 22;
 
 // ------------------ Motion constants ----------------------
 constexpr uint32_t MIN_PERIOD_US = 25;      // 40 kµstep/s
 constexpr uint32_t MAX_PERIOD_US = 5000;    // 200 µstep/s
 constexpr uint32_t STEP_HIGH_US  = 4;
 constexpr float    TARGET_TOLERANCE_DEG = 1.0f;
 constexpr uint16_t uStepsPerRev[2] = { 200*8, 200*8 };   // 1/8 µstep both axes
 
 // ------------------ Motor control struct ------------------
 struct MotorCtl {
   volatile uint32_t stepPeriodUs = MAX_PERIOD_US;
   volatile int8_t   dirFlag = 0;
   volatile bool     enableLoop = false;
   volatile float    targetDeg  = 0.0f;
   float Kp=100, Ki=0, Kd=1;
   int32_t revCount=0; uint16_t prevRaw=0; float position=0;
   float integ=0, prevErr=0;
   bool  homing=false; uint32_t nextStep=0;
 } motors[2];
 
 // ------------------ I²C helpers ---------------------------
 void selectMux(uint8_t ch){ Wire.beginTransmission(TCA_ADDR); Wire.write(1<<ch); Wire.endTransmission(); }
 
 bool readAS5600(uint8_t ch,float &deg){
   selectMux(ch);
   Wire.beginTransmission(AS5600_ADDR);
   Wire.write(AS5600_RAW_ANGLE_H);
   if(Wire.endTransmission(false)!=0) return false;
   uint32_t t0=micros();
   Wire.requestFrom(AS5600_ADDR,(uint8_t)2);
   while(Wire.available()<2){ if(micros()-t0>1000){ Wire.endTransmission(true); return false; }}
   uint16_t raw=((Wire.read()<<8)|Wire.read()) & 0x0FFF;
   deg = raw * (360.0f/4096.0f);
   return true;
 }
 
 // ------------------ Step pulse ---------------------------
 template<uint8_t PIN>
 inline void stepPulse(){ digitalWriteFast(PIN,HIGH); delayMicroseconds(STEP_HIGH_US); digitalWriteFast(PIN,LOW); }
 
 // ------------------ Setup --------------------------------
 void setup(){
   Serial.begin(115200);
   Wire.begin(); Wire.setClock(400000);
   pinMode(M0_STEP_PIN,OUTPUT); pinMode(M0_DIR_PIN,OUTPUT); pinMode(M0_EN_PIN,OUTPUT);
   pinMode(M1_STEP_PIN,OUTPUT); pinMode(M1_DIR_PIN,OUTPUT); pinMode(M1_EN_PIN,OUTPUT);
   digitalWriteFast(M0_EN_PIN,LOW); digitalWriteFast(M1_EN_PIN,LOW);
   pinMode(LIMIT_0_PIN,INPUT_PULLUP); pinMode(LIMIT_1_PIN,INPUT_PULLUP);
   motors[0].enableLoop = motors[1].enableLoop = true;
   Serial.println("Closed‑loop dual stepper ready. CLI: 0t90 1x 0e …");
 }
 
 // ------------------ CLI ----------------------------------
 void handleCLI(){
   if(!Serial.available()) return;
   String in=Serial.readStringUntil('\n'); in.trim(); if(in.length()<2) return;
   char idx=in.charAt(0); char cmd=in.charAt(1);
   if(idx!='0'&&idx!='1'){ Serial.println("Bad index"); return; }
   MotorCtl &m=motors[idx-'0'];
   switch(cmd){
     case 'e': m.enableLoop=true;  Serial.printf("Motor %c enabled\n",idx); break;
     case 'x': m.enableLoop=false; Serial.printf("Motor %c disabled\n",idx); break;
     case 'r': m={}; Serial.printf("Motor %c reset\n",idx); break;
     case 's': m.homing=true; m.enableLoop=false; Serial.printf("Motor %c homing\n",idx); break;
     case 't': if(in.length()>2){ m.targetDeg=in.substring(2).toFloat(); Serial.printf("M%c tgt %.1f°\n",idx,m.targetDeg);} break;
     case 'p': if(in.length()>2){ m.Kp=in.substring(2).toFloat(); Serial.printf("M%c Kp %.2f\n",idx,m.Kp);} break;
     case 'i': if(in.length()>2){ m.Ki=in.substring(2).toFloat(); Serial.printf("M%c Ki %.2f\n",idx,m.Ki);} break;
     case 'd': if(in.length()>2){ m.Kd=in.substring(2).toFloat(); Serial.printf("M%c Kd %.2f\n",idx,m.Kd);} break;
     default: Serial.println("Cmds: e x r s t p i d");
   }
 }
 
 // ------------------ Loop ---------------------------------
 void loop(){
   handleCLI();
   static uint32_t lastCtl=micros(); bool doCtl=(micros()-lastCtl)>=1000; if(doCtl) lastCtl+=1000;
 
   for(uint8_t i=0;i<2;++i){
     MotorCtl &m=motors[i];
     float ang; if(!readAS5600(i,ang)) continue;
     float prevDeg=m.prevRaw*360.0f/4096.0f;
     if(prevDeg>270&&ang<90) m.revCount++; else if(prevDeg<90&&ang>270) m.revCount--; 
     m.prevRaw=(uint16_t)((ang/360.0f)*4096.0f)&0x0FFF;
     m.position = m.revCount*360.0f + ang;
 
     if(doCtl){
       if(m.homing){
         uint8_t lp=(i==0)?LIMIT_0_PIN:LIMIT_1_PIN; bool hit=digitalRead(lp);
         if(hit){ m.homing=false; m.enableLoop=true; m.revCount=0; m.position=0; m.targetDeg=0; m.integ=0; m.prevErr=0; }
         else { m.dirFlag=1; m.stepPeriodUs=1000; }
       }else{
         float err=m.targetDeg - m.position; m.integ += err*0.001f; float deriv=(err-m.prevErr)*1000.0f; m.prevErr=err;
         float cmd=m.Kp*err + m.Ki*m.integ + m.Kd*deriv;
         float maxSteps=1e6f/MIN_PERIOD_US; float maxDegPerSec=maxSteps*360.0f/uStepsPerRev[i];
         cmd=constrain(cmd,-maxDegPerSec,maxDegPerSec);
         int dir = (cmd > 0);         // positive velocity => forward direction
         //if (i == 1) dir = !dir;      // invert for motor 1 mechanical orientation
         m.dirFlag=dir;
         float sps=fabsf(cmd)*uStepsPerRev[i]/360.0f;
         uint32_t period=(sps<1)?MAX_PERIOD_US:constrain((uint32_t)(1e6f/sps),MIN_PERIOD_US,MAX_PERIOD_US);
         if(fabsf(err)<=TARGET_TOLERANCE_DEG){ period=MAX_PERIOD_US; m.integ=0; }
         if(!m.enableLoop) period=MAX_PERIOD_US;
         m.stepPeriodUs=period;
       }
     }
 
     if((m.enableLoop||m.homing) && m.stepPeriodUs<MAX_PERIOD_US && (int32_t)(micros()-m.nextStep)>=0){
       if(i==0){ digitalWriteFast(M0_DIR_PIN,m.dirFlag); stepPulse<M0_STEP_PIN>(); }
       else     { digitalWriteFast(M1_DIR_PIN,m.dirFlag); stepPulse<M1_STEP_PIN>(); }
       m.nextStep += m.stepPeriodUs;
     }
   }
 
   // ---------- simple telemetry ----------
   static uint32_t tTele=millis();
   if(millis()-tTele>=50){
     Serial.printf("M0 tgt=%7.1f pos=%7.1f per=%4lu | M1 tgt=%7.1f pos=%7.1f per=%4lu\n",
                   motors[0].targetDeg, motors[0].position, motors[0].stepPeriodUs,
                   motors[1].targetDeg, motors[1].position, motors[1].stepPeriodUs);
     tTele+=50;
   }
 }
 
