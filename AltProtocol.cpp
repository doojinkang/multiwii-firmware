#include "Arduino.h"

#include "config.h"
#include "def.h"
#include "AltSoftSerial.h"
#include "types.h"
#include "EEPROM.h"
#include "Output.h"
#include "MultiWii.h"

#include "MSP.h"

#ifdef USE_ALT_SOFT_SERIAL

#define UART_NUMBER 1

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

static uint8_t CURRENTPORT=0;

static uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}
static uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
static uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}

static void serialize8(uint8_t a) {
  AltSerialSerialize(a);
  checksum[CURRENTPORT] ^= a;
}
static void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}
static void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

static void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

static void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

static void headSerialError() {
  headSerialResponse(1,0);
}

static void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);
  AltUartSendData();     // FIX ME
}

static void serializeNames(PGM_P s) {
  headSerialReply(strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++)
    serialize8(pgm_read_byte(c));
  tailSerialReply();
}


static void __attribute__ ((noinline)) s_struct_w(uint8_t *cb,uint8_t siz) {
  while(siz--) *cb++ = read8();
}

static void s_struct_partial(uint8_t *cb,uint8_t siz) {
  while(siz--) serialize8(*cb++);
}

static void s_struct(uint8_t *cb,uint8_t siz) {
  headSerialReply(siz);
  s_struct_partial(cb,siz);
  tailSerialReply();
}

static void mspAck() {
  headSerialReply(0);tailSerialReply();
}

void altEvaluateOtherData(uint8_t sr);
void altEvaluateCommand(uint8_t c);

void serialAltCom() {
  uint8_t c,cc,port,state,bytesTXBuff;
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static uint8_t c_state[UART_NUMBER];
  uint32_t timeMax; // limit max time in this function in case of GPS

  timeMax = micros();

    port = 0;
    CURRENTPORT=port;

    cc = AltSerialAvailable();  // 0 why?
    while (cc--) {
      bytesTXBuff = AltSerialUsedTXBuff(); // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = AltSerialRead();

        state = c_state[port];
        // regular data handling to detect and handle MSP and other data
        if (state == IDLE) {
          if (c=='$') state = HEADER_START;
          else altEvaluateOtherData(c); // evaluate all other incoming serial data
        } else if (state == HEADER_START) {
          state = (c=='M') ? HEADER_M : IDLE;
        } else if (state == HEADER_M) {
          state = (c=='<') ? HEADER_ARROW : IDLE;
        } else if (state == HEADER_ARROW) {
          if (c > INBUF_SIZE) {  // now we are expecting the payload size
            state = IDLE;
            continue;
          }
          dataSize[port] = c;
          checksum[port] = c;
          offset[port] = 0;
          indRX[port] = 0;
          state = HEADER_SIZE;  // the command is to follow
        } else if (state == HEADER_SIZE) {
          cmdMSP[port] = c;
          checksum[port] ^= c;
          state = HEADER_CMD;
        } else if (state == HEADER_CMD) {
          if (offset[port] < dataSize[port]) {
            checksum[port] ^= c;
            inBuf[offset[port]++][port] = c;
          } else {
            if (checksum[port] == c) // compare calculated and transferred checksum
              altEvaluateCommand(cmdMSP[port]); // we got a valid packet, evaluate it
            state = IDLE;
            cc = 0; // no more than one MSP per port and per cycle
          }
        }
        c_state[port] = state;
    } // while
}

void altEvaluateCommand(uint8_t c) {
  uint32_t tmp=0;

  switch(c) {
    // adding this message as a comment will return an error status for MSP_PRIVATE (end of switch), allowing third party tools to distinguish the implementation of this message
    //case MSP_PRIVATE:
    //  headSerialError();tailSerialReply(); // we don't have any custom msp currently, so tell the gui we do not use that
    //  break;
    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition
      break;
    case MSP_SET_PID:
      mspAck();
      s_struct_w((uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
      break;
    case MSP_SET_BOX:
      mspAck();
      #if EXTAUX
        s_struct_w((uint8_t*)&conf.activate[0],CHECKBOXITEMS*4);
      #else
        s_struct_w((uint8_t*)&conf.activate[0],CHECKBOXITEMS*2);
      #endif
      break;
    case MSP_SET_RC_TUNING:
      mspAck();
      s_struct_w((uint8_t*)&conf.rcRate8,7);
      break;
    #if !defined(DISABLE_SETTINGS_TAB)
    case MSP_SET_MISC:
      struct {
        uint16_t a,b,c,d,e,f;
        uint32_t g;
        uint16_t h;
        uint8_t  i,j,k,l;
      } set_misc;
      mspAck();
      s_struct_w((uint8_t*)&set_misc,22);
      #if defined(POWERMETER)
        conf.powerTrigger1 = set_misc.a / PLEVELSCALE;
      #endif
      conf.minthrottle = set_misc.b;
      #ifdef FAILSAFE
        conf.failsafe_throttle = set_misc.e;
      #endif
      #if MAG
        conf.mag_declination = set_misc.h;
      #endif
      #if defined(VBAT)
        conf.vbatscale        = set_misc.i;
        conf.vbatlevel_warn1  = set_misc.j;
        conf.vbatlevel_warn2  = set_misc.k;
        conf.vbatlevel_crit   = set_misc.l;
      #endif
      break;
    case MSP_MISC:
      struct {
        uint16_t a,b,c,d,e,f;
        uint32_t g;
        uint16_t h;
        uint8_t  i,j,k,l;
      } misc;
      misc.a = intPowerTrigger1;
      misc.b = conf.minthrottle;
      misc.c = MAXTHROTTLE;
      misc.d = MINCOMMAND;
      #ifdef FAILSAFE
        misc.e = conf.failsafe_throttle;
      #else
        misc.e = 0;
      #endif
      #ifdef LOG_PERMANENT
        misc.f = plog.arm;
        misc.g = plog.lifetime + (plog.armed_time / 1000000); // <- computation must be moved outside from serial
      #else
        misc.f = 0; misc.g =0;
      #endif
      #if MAG
        misc.h = conf.mag_declination;
      #else
        misc.h = 0;
      #endif
      #ifdef VBAT
        misc.i = conf.vbatscale;
        misc.j = conf.vbatlevel_warn1;
        misc.k = conf.vbatlevel_warn2;
        misc.l = conf.vbatlevel_crit;
      #else
        misc.i = 0;misc.j = 0;misc.k = 0;misc.l = 0;
      #endif
      s_struct((uint8_t*)&misc,22);
      break;
    #endif
    #if defined (DYNBALANCE)
    case MSP_SET_MOTOR:
      mspAck();
      s_struct_w((uint8_t*)&motor,16);
    break;
    #endif
    #ifdef MULTIPLE_CONFIGURATION_PROFILES
    case MSP_SELECT_SETTING:
      if(!f.ARMED) {
        global_conf.currentSet = read8();
        if(global_conf.currentSet>2) global_conf.currentSet = 0;
        writeGlobalSet(0);
        readEEPROM();
      }
      mspAck();
      break;
    #endif
    case MSP_SET_HEAD:
      mspAck();
      s_struct_w((uint8_t*)&magHold,2);
      break;
    case MSP_IDENT:
      struct {
        uint8_t v,t,msp_v;
        uint32_t cap;
      } id;
      id.v     = VERSION;
      id.t     = MULTITYPE;
      id.msp_v = MSP_VERSION;
      id.cap   = (0+BIND_CAPABLE)|DYNBAL<<2|FLAP<<3|NAVCAP<<4|EXTAUX<<5|((uint32_t)NAVI_VERSION<<28); //Navi version is stored in the upper four bits;
      s_struct((uint8_t*)&id,7);
      break;
    case MSP_STATUS:
      struct {
        uint16_t cycleTime,i2c_errors_count,sensor;
        uint32_t flag;
        uint8_t set;
      } st;
      st.cycleTime        = cycleTime;
      st.i2c_errors_count = i2c_errors_count;
      st.sensor           = ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4;
      #if ACC
        if(f.ANGLE_MODE)   tmp |= 1<<BOXANGLE;
        if(f.HORIZON_MODE) tmp |= 1<<BOXHORIZON;
      #endif
      #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
        if(f.BARO_MODE) tmp |= 1<<BOXBARO;
      #endif
      if(f.MAG_MODE) tmp |= 1<<BOXMAG;
      #if !defined(FIXEDWING)
        #if defined(HEADFREE)
          if(f.HEADFREE_MODE)       tmp |= 1<<BOXHEADFREE;
          if(rcOptions[BOXHEADADJ]) tmp |= 1<<BOXHEADADJ;
        #endif
      #endif
      #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
        if(rcOptions[BOXCAMSTAB]) tmp |= 1<<BOXCAMSTAB;
      #endif
      #if defined(CAMTRIG)
        if(rcOptions[BOXCAMTRIG]) tmp |= 1<<BOXCAMTRIG;
      #endif
      #if GPS
        switch (f.GPS_mode) {
          case GPS_MODE_HOLD:
            tmp |= 1<<BOXGPSHOLD;
            break;
          case GPS_MODE_RTH:
            tmp |= 1<<BOXGPSHOME;
            break;
          case GPS_MODE_NAV:
            tmp |= 1<<BOXGPSNAV;
            break;
        }
      #endif
      #if defined(FIXEDWING) || defined(HELICOPTER)
        if(f.PASSTHRU_MODE) tmp |= 1<<BOXPASSTHRU;
      #endif
      #if defined(BUZZER)
        if(rcOptions[BOXBEEPERON]) tmp |= 1<<BOXBEEPERON;
      #endif
      #if defined(LED_FLASHER)
        if(rcOptions[BOXLEDMAX]) tmp |= 1<<BOXLEDMAX;
        if(rcOptions[BOXLEDLOW]) tmp |= 1<<BOXLEDLOW;
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        if(rcOptions[BOXLLIGHTS]) tmp |= 1<<BOXLLIGHTS;
      #endif
      #if defined(VARIOMETER)
        if(rcOptions[BOXVARIO]) tmp |= 1<<BOXVARIO;
      #endif
      #if defined(INFLIGHT_ACC_CALIBRATION)
        if(rcOptions[BOXCALIB]) tmp |= 1<<BOXCALIB;
      #endif
      #if defined(GOVERNOR_P)
        if(rcOptions[BOXGOV]) tmp |= 1<<BOXGOV;
      #endif
      #if defined(OSD_SWITCH)
        if(rcOptions[BOXOSD]) tmp |= 1<<BOXOSD;
      #endif
      if(f.ARMED) tmp |= 1<<BOXARM;
      st.flag             = tmp;
      st.set              = global_conf.currentSet;
      s_struct((uint8_t*)&st,11);
      break;
    case MSP_RAW_IMU:
      #if defined(DYNBALANCE)
        for(uint8_t axis=0;axis<3;axis++) {imu.gyroData[axis]=imu.gyroADC[axis];imu.accSmooth[axis]= imu.accADC[axis];} // Send the unfiltered Gyro & Acc values to gui.
      #endif
      s_struct((uint8_t*)&imu,18);
      break;
    case MSP_SERVO:
      s_struct((uint8_t*)&servo,16);
      break;
    case MSP_SERVO_CONF:
      s_struct((uint8_t*)&conf.servoConf[0].min,56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
      break;
    case MSP_SET_SERVO_CONF:
      mspAck();
      s_struct_w((uint8_t*)&conf.servoConf[0].min,56);
      break;
    case MSP_MOTOR:
      s_struct((uint8_t*)&motor,16);
      break;
    case MSP_ACC_TRIM:
      headSerialReply(4);
      s_struct_partial((uint8_t*)&conf.angleTrim[PITCH],2);
      s_struct_partial((uint8_t*)&conf.angleTrim[ROLL],2);
      tailSerialReply();
      break;
    case MSP_SET_ACC_TRIM:
      mspAck();
      s_struct_w((uint8_t*)&conf.angleTrim[PITCH],2);
      s_struct_w((uint8_t*)&conf.angleTrim[ROLL],2);
      break;
    case MSP_RC:
      s_struct((uint8_t*)&rcData,RC_CHANS*2);
      break;

    case MSP_ATTITUDE:
      s_struct((uint8_t*)&att,6);
      break;
    case MSP_ALTITUDE:
      s_struct((uint8_t*)&alt,6);
      break;
    case MSP_ANALOG:
      s_struct((uint8_t*)&analog,7);
      break;
    case MSP_RC_TUNING:
      s_struct((uint8_t*)&conf.rcRate8,7);
      break;
    case MSP_PID:
      s_struct((uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
      break;
    case MSP_PIDNAMES:
      serializeNames(pidnames);
      break;
    case MSP_BOX:
      #if EXTAUX
        s_struct((uint8_t*)&conf.activate[0],4*CHECKBOXITEMS);
      #else
        s_struct((uint8_t*)&conf.activate[0],2*CHECKBOXITEMS);
      #endif
      break;
    case MSP_BOXNAMES:
      serializeNames(boxnames);
      break;
    case MSP_BOXIDS:
      headSerialReply(CHECKBOXITEMS);
      for(uint8_t i=0;i<CHECKBOXITEMS;i++)
        serialize8(pgm_read_byte(&(boxids[i])));
      tailSerialReply();
      break;
    case MSP_MOTOR_PINS:
      s_struct((uint8_t*)&PWM_PIN,8);
      break;
    case MSP_RESET_CONF:
      if(!f.ARMED) LoadDefaults();
      mspAck();
      break;
    case MSP_ACC_CALIBRATION:
      if(!f.ARMED) calibratingA=512;
      mspAck();
      break;
    case MSP_EEPROM_WRITE:
      writeParams(0);
      mspAck();
      break;
    case MSP_DEBUG:
      s_struct((uint8_t*)&debug,8);
      break;
    default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
      headSerialError();tailSerialReply();
      break;
  }
}

// evaluate all other incoming serial data
void altEvaluateOtherData(uint8_t sr) {
  #ifndef SUPPRESS_OTHER_SERIAL_COMMANDS
    switch (sr) {
    // Note: we may receive weird characters here which could trigger unwanted features during flight.
    //       this could lead to a crash easily.
    //       Please use if (!f.ARMED) where neccessary
      #ifdef LCD_CONF
        case 's':
        case 'S':
          if (!f.ARMED) configurationLoop();
          break;
      #endif
    }
  #endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

#endif