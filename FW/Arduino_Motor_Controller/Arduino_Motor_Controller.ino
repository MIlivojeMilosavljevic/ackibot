
///////////////////////////////////////////////////////////////////////////////

#define M0_PWM 3
#define M0_DIR 4
//#define M0_DIR_N 11
//#define M1_DIR_P 12
//#define M1_DIR_N 13
#define M1_PWM 8

#define ENC0_A A0
#define ENC0_B A1
// #define ENC1_A A2
// #define ENC1_B A3

// Ramp rate selection.
//#define CFG_0 4
//#define CFG_1 5
//
//#define CFG_2 6
//#define CFG_3 7


// L298 settings.
#define SLOW_RAMP_RATE 250

#define MID_RAMP_RATE 80

// Cytron MDD20A settings.
#define FAST_RAMP_RATE 15

#define WATCHDOG_TIMEOUT_MS 200

///////////////////////////////////////////////////////////////////////////////
// Debug stuff.

#define TOP_OF_PWM A6

#define SW_UART_TX 11
#define SW_UART_RX 10

///////////////////////////////////////////////////////////////////////////////
// Cfg.


// Sabertooth settings.
#define DEFAULT_RAMP_RATE 2047 // Maximum.
#define FULL_RAMP_SEC 8
#define FULL_STEERING_RAMP_SEC 1.5

//TODO Not defines but from settings.
#if 0
// L289
//#define PRESCALER_CFG 0b011
#define PRESCALER 64
#else
// Cytron
//#define PRESCALER_CFG 0b010
#define PRESCALER 8
#endif

#define MODULUS 2048

#define SENSOR_HZ 25

///////////////////////////////////////////////////////////////////////////////
#define BLDC 0
#define SERVO 1

///////////////////////////////////////////////////////////////////////////////

#include "SoftwareSerial2.h"

#include "avr_io_bitfields.h"

#include "sabertooth.h"

#include "fw_pkgs.hpp"

#include <Servo.h>

//bool console_mode = true;  // start u konzolnom režimu radi testiranja


///////////////////////////////////////////////////////////////////////////////

SoftwareSerial sw_ser(SW_UART_RX, SW_UART_TX);
#define DEBUG(x) \
 do{ \
   sw_ser.print(#x" = "); sw_ser.println(x); \
 }while(0)
#define DEBUG_HEX(x) \
 do{ \
   sw_ser.print(#x" = 0x"); sw_ser.println(x, HEX); \
 }while(0)

///////////////////////////////////////////////////////////////////////////////


enum dir_t {
  CW = +1,
  CCW = -1
};

// Tipovi (koriste se u ostatku koda)

typedef int16_t speed_t;
typedef int16_t steering_angle_t;

// TODO: VIDJETI LOGIKU OKO PINOVA
void set_dir(int motor, dir_t dir){
  // switch(motor_idx){
  //  case 0:
  //    if(dir == CW){
  //      digitalWrite(M0_DIR_P, 1);
  //      digitalWrite(M0_DIR_N, 0);
  //    }else{
  //      digitalWrite(M0_DIR_P, 0);
  //      digitalWrite(M0_DIR_N, 1);
  //    }
  //    break;
  //  case 1:
  //    if(dir == CW){
  //      digitalWrite(M1_DIR_P, 1);
  //      digitalWrite(M1_DIR_N, 0);
  //    }else{
  //      digitalWrite(M1_DIR_P, 0);
  //      digitalWrite(M1_DIR_N, 1);
  //    }
  //    break;
  //  default:
  //    // Out of range.
  //    return;
  // }

  // TODO: Realizovati razlicite pinove za BLDC i SERVO
  switch (motor) {
    case BLDC:
      if(dir == CW){
        digitalWrite(M0_DIR, 1); //KOD BLDC 1 (HIGH) IDE U CW SMJERU
        //digitalWrite(M0_DIR_, 0);
      }else{
        digitalWrite(M0_DIR, 0); //KOD BLDC 0 (LOW) IDE CCW, a prestace da se krece kada je PWM = 0
        // digitalWrite(M0_DIR_N, 1);
      }
      break;

    // case SERVO:
    //  if(dir == CW){
    //    digitalWrite(M0_DIR_P, 1);
    //    digitalWrite(M0_DIR_N, 0);
    //  }else{
    //    digitalWrite(M0_DIR_P, 0);
    //    digitalWrite(M0_DIR_N, 1);
    //  }
      //break;
  
    default:
      return;
  }
  
}
// TODO: MOZDA IMPLEMENTIRATI LOGIKU SWITCH CASE ZA RAZLICITE PINOVE ZA SERVO I BLDC MOTOR
//void set_pwm(u16 s){
//
////  tc2.ocrb = s; // hardverski tajmer, postavka pwm za bldc 
//  //OCR2B = s;
//  
//}



Servo servo;



void set_servo_angle(int angle) {
  angle = constrain(angle, 0, 180);  // Uveri se da je u opsegu
  servo.write(angle); // Pošalji ugao servu

// --- TEST PROGRAM ---
//  servo.write(90);
//  delay(500);
//  servo.write(45);
//  delay(500);
//  servo.write(10);
//  delay(500);
//  servo.write(70);
}


template<typename T>
T clamp(T val, T min_lim, T max_lim) {
  if(val < min_lim){
    return min_lim;
  }else if(val > max_lim){
    return max_lim;
  }else{
    return val;
  }
}

template<typename T>
T sym_clamp(T val, T lim) {
  return clamp(val, T(-lim), T(+lim));
}

template<typename T>
T sign(T val) {
  return val >= 0 ? T(+1) : T(-1);
}


// 2 because duty correct PWM.
#define PWM_HZ ((F_CPU)/(PRESCALER)/(MODULUS)/2)


//u8 read_cfg() {
//  return
//    digitalRead(CFG_0)<<0 |
//    digitalRead(CFG_1)<<1 |
//    digitalRead(CFG_2)<<2 |
//    digitalRead(CFG_3)<<3;
//}

///////////////////////////////////////////////////////////////////////////////


#define WATCHDOG_TIMEOUT_PERIODS ((WATCHDOG_TIMEOUT_MS)*PWM_HZ/1000)
#if WATCHDOG_TIMEOUT_PERIODS > 255
#error "WATCHDOG_TIMEOUT_MS too big"
#endif

// u8 to be atomic variable.
volatile u8 watchdog_cnt;

void watchdog_rst() {
  watchdog_cnt = WATCHDOG_TIMEOUT_PERIODS;
}

void watchdog_dec() {
  if(watchdog_cnt != 0){
    watchdog_cnt--;
//    if(watchdog_cnt == 0){
//      sw_ser.println("WARN: Watchdog stop motors!");
//    }
  }
}

void watchdog_apply() {
  if(watchdog_cnt == 0){
    
    //set_target_speed(0);

    set_target_steering_angle(90);
    
  }
}


///////////////////////////////////////////////////////////////////////////////


typedef i16 speed_t;
typedef i16 steering_angle_t;

//TODO Change that to be in ms
/**
 * With max rate 2047, need 8 sec to max speed.
 * that means that pwm_curr goes from 0 to 2047 in 8 sec.
 * With 61Hz PWM it need 480 periods of PWM to do that.
 * Which means it step 4.27 per period.
 * @param ramp_rate [0, 2047]
 * @return Ramp per PWM period.
 */
//u16 ramp_rate_2_speed_ramp_delta(u16 ramp_rate){
//  return speed_t(
//    (
//      float(MODULUS-1)*(MODULUS-1) 
//    )/(
//      (float)ramp_rate*FULL_RAMP_SEC*PWM_HZ
//    )
//  );
//}
//
//u16 ramp_rate_2_steering_angle_ramp_delta(u16 ramp_rate){
//  return steering_angle_t(
//    (
//      float(180)*(180) 
//    )/(
//      (float)ramp_rate*FULL_STEERING_RAMP_SEC*PWM_HZ
//    )
//  );
//}
//
//speed_t speed_ramp_delta;
//speed_t steering_angle_ramp_delta;



/**
 * Target speed. Input speed for ramp algo.
 * read:
 *     - ISR(TIMER1_OVF_vect)
 *     - print_status_if_changed()
 * write:
 *     - poll_pkg()
 *         - set_target_speed()
 *         - watchdog_apply()
*/
volatile speed_t speed_i;
volatile steering_angle_t steering_angle_i;

/**
 * Actual speed. Output speed from ramp algo.
 * read/write:
 *     - ISR(TIMER1_OVF_vect)
 *         - ramp_motor()
*/
volatile speed_t speed_o;
volatile steering_angle_t steering_angle_o;

// TODO: MOZDA CEMO KAO PARAMTAR PROSLJEDJIVATI BLDC ILI SERVO MOTOR OZNAKU
//void set_bldc() {
//  set_dir(
//    BLDC,
//    speed_o >= 0 ? CW : CCW
//  );
//  set_pwm(abs(speed_o));
//}

// mapiranje brzine iz opsega projekta u 8-bit PWM [-255..255]
int map_speed_to_pwm(int speed_val) {
  // očekujemo speed_val u opsegu [-(MODULUS-1), +(MODULUS-1)]
  const int in_min = -(MODULUS-1);
  const int in_max =  (MODULUS-1);
  const int out_min = -255;
  const int out_max = 255;
  // clamp pre mapiranja
  if(speed_val < in_min) speed_val = in_min;
  if(speed_val > in_max) speed_val = in_max;
  //skaliranje brzine iz velikog opseg
  long mapped = (long)(speed_val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return (int)mapped;
}

///////////////////////////////////////////////////////////////////////////////
// Kontrola BLDC (jednostavno: DIR pin i analogWrite za PWM)

void set_bldc_from_speed(int16_t speed_val) {
  //pwm_signed odredjuje smer
  int pwm_signed = map_speed_to_pwm(speed_val); // -255 .. +255
  //odredjuje brzinu
  int pwm = abs(pwm_signed);
  if(pwm > 255) pwm = 255;

  if(pwm == 0) {
    // stop motor
    digitalWrite(M0_DIR, LOW); // definicija: LOW = stop/direction prema tvom hardveru
    analogWrite(M0_PWM, 255);
    return;
  }

  if(pwm_signed >= 0) {
    // napred / napomena: proveri da li HIGH odgovara CW ili CCW na tvom driveru
    digitalWrite(M0_DIR, HIGH);
  } else {
    digitalWrite(M0_DIR, LOW);
  }

  analogWrite(M0_PWM, 255 - pwm); // 0-255
}
//
//void set_servo() {
//  set_dir(
//    SERVO,
//    steering_angle_o >= 90 ? CW : CCW
//  );
//  set_pwm_servo(abs(steering_angle_o));
//}

template<typename T>
T voting_read(const T& i) {
  T o;
  do{
    o = i;
  }while(o != i);
  return o;
}
//
//void ramp_speed() {
//  speed_t target = voting_read(speed_i);
//#if 0
//  speed_o = target;
//#else
//  speed_o = clamp(
//    target,
//    speed_o - speed_ramp_delta,
//    speed_o + speed_ramp_delta
//  );
//#endif
//  set_bldc();
//}
//
//void ramp_steering_angle() {
//  steering_angle_t target = voting_read(steering_angle_i);
//#if 0
//  steering_angle_i = target;
//#else
//  steering_angle_i = clamp(
//    target,
//    steering_angle_i - steering_angle_ramp_delta,
//    steering_angle_i + steering_angle_ramp_delta
//  );
//#endif
//  set_servo();
//}
//
//ISR(TIMER1_OVF_vect) {
//  digitalWrite(TOP_OF_PWM, !digitalRead(TOP_OF_PWM));
//
//  watchdog_dec();
//  
//  //for(u8 i = 0; i < 2; i++){
//  //  ramp_speed(i);
//  //}
//  ramp_speed();
//  ramp_steering_angle();
//}
//
//ISR(TIMER1_COMPA_vect) {
//  /*
//  if(down_cnt[0]){  
//    down_cnt[0] = 0;
//    digitalWrite(M0_PWM, 0);
//  }else{
//    digitalWrite(M0_PWM, 1);
//  }
//  */
//}
//
//ISR(TIMER1_COMPB_vect) {
//}
//
//// Setovanje brzine
//void set_target_speed(i16 val) {
//  i16 new_pwm = sym_clamp(val, MODULUS-1);
//  speed_i = new_pwm;
//}
void set_target_speed(i16 val) {
  // spremi cilj - bez rampe
  speed_i = val;
  // primeni odmah (možeš promeniti logiku ako želiš da se rampuje)
  set_bldc_from_speed(speed_i);
  
}
void set_target_steering_angle(i16 val) {
  steering_angle_i = val;
  set_servo_angle(steering_angle_i);
}

//
//// Setovanje ugla
//void set_target_steering_angle(i16 val) {
//  i16 new_pwm = sym_clamp(val, 180);
//  steering_angle_i = new_pwm;
//}

///////////////////////////////////////////////////////////////////////////////


//volatile i32 enc[2];
volatile i32 enc;

/**
 * Incrementing sequence is 00, 01, 11, 10
 * -0 means error, in case of 2 bits changed.
 * Usage: enc_ba_to_pos_inc[prev ba][curr ba]
 */
const i8 enc_ba_to_pos_inc[4][4] = {
// curr ba   00  01  10  11
            {+0, +1, -1, -0}, // prev ba = 00
            {-1, +0, -0, +1}, // prev ba = 01
            {+1, -0, +0, -1}, // prev ba = 10
            {-0, -1, +1, +0}, // prev ba = 11
};

union bf_enc {
  struct {
    u8 ba0 : 2;
    //u8 ba1 : 2;
  };
  u8 r : 8;
};

ISR(PCINT1_vect) {
  bf_enc curr;
  static bf_enc prev;
  
  curr.r = PINC;
  
  i8 inc = enc_ba_to_pos_inc[prev.ba0][curr.ba0];
  //i8 inc1 = enc_ba_to_pos_inc[prev.ba1][curr.ba1];

  enc += inc;
  //enc[1] += inc1;
  
  prev = curr;
}


///////////////////////////////////////////////////////////////////////////////

void setup() {

 
  servo.attach(M1_PWM); // pin 8 za servo

  Serial.begin(115200);
  
  sw_ser.begin(115200);

  pinMode(TOP_OF_PWM, OUTPUT);
  
  pinMode(M0_PWM, OUTPUT);
  analogWrite(M0_PWM, 0);

  pinMode(M0_DIR, OUTPUT);
  digitalWrite(M0_DIR, LOW);  // low = CW
  //pinMode(M0_DIR_N, OUTPUT);
  //pinMode(M1_DIR_P, OUTPUT);
  // pinMode(M1_DIR_N, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  //pinMode(CFG_0, INPUT_PULLUP);
//  pinMode(CFG_1, INPUT_PULLUP);
//  pinMode(CFG_2, INPUT_PULLUP);
//  pinMode(CFG_3, INPUT_PULLUP);

  //digitalWrite(M0_PWM, HIGH);
  //digitalWrite(M0_DIR, HIGH);



  // Enc cfg - jedan enkoder na A0 (PC0/PCINT8) i A1 (PC1/PCINT9)
  pinMode(ENC0_A, INPUT_PULLUP);
  pinMode(ENC0_B, INPUT_PULLUP);
  //pinMode(ENC1_A, INPUT_PULLUP);
  //pinMode(ENC1_B, INPUT_PULLUP);
  irq.pcie1 = 1; // Port C, ukljuci Pin Change Interrupt grupu za PORT C
  // PC0-3
  // WARNING: This too need to be changed with ENC pins.
  //TODO Figure out which pcint to set from ENC0_A...
  irq.pcint08 = 1; // omoguci prekid za A0 (PCIN8)
  irq.pcint09 = 1; // omoguci prekid za A1 (PCIN9)
  // irq.pcint10 = 1;
  // irq.pcint11 = 1;

  
  // for(u8 i = 0; i < 2; i++){
  //  set_target_speed(i, 0);
  // }

  // Inicijalne vrednosti
  speed_i = 0;
  steering_angle_i = 90; 
  speed_o = 0;
  steering_angle_o = 90;
  enc = 0;
  watchdog_cnt = 255;
  
  set_target_speed(0);
  set_target_steering_angle(90);

  // if (console_mode) {
  //   Serial.println(F("\n[Arduino] Console mode ON. Kucaj H za help."));
  // }


/**
 * Ramp Rate selection.
 * oo - 11 -> Cytron MDD20A
 * jo - 10 -> L298
 * jj - 01 -> Old School Motor Driver
 */
 // u8 cfg = read_cfg();
//  u16 ramp_rate;
//  switch(cfg & 0b0011){
//    case 0b11:
//      ramp_rate = FAST_RAMP_RATE;
//      break;
//    case 0b01:
//      ramp_rate = MID_RAMP_RATE;
//      break;
//    case 0b10:
//    default:
//      ramp_rate = SLOW_RAMP_RATE;
//      break;
//  }
//  speed_ramp_delta = ramp_rate_2_speed_ramp_delta(ramp_rate);
//  steering_angle_ramp_delta = ramp_rate_2_steering_angle_ramp_delta(ramp_rate);


//  
//  tc1.tccra = 0;
//  tc1.tccrb = 0;
//  tc1.tccrc = 0;
//  irq.timsk[1] = 0;
//  tc1.coma = 0b10; // Normal output.
//  tc1.comb = 0b10; // Normal output.
//  tc1.icr = MODULUS;
//  tc1.ocra = 0;
//  tc1.ocrb = 0;
//  // Phase correct PWM, ICR1 is top.
//  tc1.wgm3 = 1;
//  tc1.wgm2 = 0;
//  tc1.wgm1 = 0;
//  tc1.wgm0 = 0;
//  irq.timsk1.toie = 1;
//  //irq.timsk1.ociea = 1;
//  //irq.timsk1.ocieb = 1;
//  tc1.cs = PRESCALER_CFG;
//  
  


  sw_ser.println("Arduino Motor Controller (1 BLDC + 1 Servo) to ROS2");
  //DEBUG(DEFUALT_BAUDRATE);
  DEBUG(sizeof(pkg_m2s_t));
  DEBUG(sizeof(pkg_s2m_t));
}

///////////////////////////////////////////////////////////////////////////////

// Puluje povremeno da vidi da li ima paket
void poll_pkg() {
  watchdog_apply();

  if(Serial.available() < 1){
    return;
  }

  int len;

  pkg_magic_t exp_magic = PKG_MAGIC;
  pkg_magic_t obs_magic = 0;
  
  for(u8 i = 0; i < sizeof(pkg_magic_t); i++){
    u8 b;
    len = Serial.readBytes(
      &b,
      1
    );
   if(len != 1){
//      sw_ser.println("ERROR: Lost start of pkg!");
      return;
   }

    reinterpret_cast<u8*>(&obs_magic)[i] = b;

    if(
      reinterpret_cast<u8*>(&exp_magic)[i] !=
      reinterpret_cast<u8*>(&obs_magic)[i]
    ){
      // Lost magic.
      //sw_ser.println("ERROR: Lost magic!");
      return;
    }
  }

  pkg_m2s_t p;
  p.magic = obs_magic;

  len = Serial.readBytes(
    reinterpret_cast<u8*>(&p) + sizeof(pkg_magic_t),
    sizeof(p) - sizeof(pkg_magic_t)
  );
  
  pkg_crc_t obs_crc = CRC16().add(p.payload).get_crc();
  if(obs_crc != p.crc){
//    sw_ser.println("ERROR: Wrong CRC!");
   return;
  }
  
  // Setovanje brzine i ugla
  set_target_speed(p.payload.speed);
  set_target_steering_angle(p.payload.steering_angle);

  // TODO call set_speed(), set_steering_angle()
  // KOD IZ PRETHODNOG PROJEKTA
  // for(u8 i = 0; i < 2; i++){
  //  // Postavlja se za 2 motora
  //  set_target_speed(
  //    i,
  //    p.payload.speed[i]
  //  );
  // }

  watchdog_rst();
}

void print_status() {
  static u8 cnt;
  cnt++;
  if(cnt == 10){
   cnt = 0;
    sw_ser.println("  speed_i\t   enc\t steering");
  }

  const int L = 4*6 + 2*10 + 2;
  char buf[L];
  snprintf(
    buf,
    L,
    "% 5d\t% 10ld\t% 5d\t",
    speed_i,
    (long)enc,//TODO Swap these two
    steering_angle_i
  );
  
  sw_ser.println(buf);
}

void print_status_if_changed() {
  static i16 prev_speed_i = 0;
  static i32 prev_enc = 0;
  static i16 prev_steering_angle_i = 90;

  bool print_it = false;

  if(
      speed_i != prev_speed_i || 
      enc != prev_enc ||
      steering_angle_i != prev_steering_angle_i
    ){
      print_it = true;
    }
    
  // Update previous values
  prev_speed_i = speed_i;
  prev_enc = enc;
  prev_steering_angle_i = steering_angle_i;

  if(print_it){
    print_status();
  }
}


// Metoda za popunjavanje paketa koji se salje sa mikrokontrolera na pc
void send_pkg() {
  pkg_s2m_t p;
  p.magic = PKG_MAGIC;

  // TODO: Zakomentarisati enkoder jer nam ne treba, realizovati slanje rezultata ultrazvucnog senzora
  // Voting read.
  // for(u8 i = 0; i < 2; i++){
  do{
    p.payload.enc = enc;
  }while(p.payload.enc != enc);

  do{
    p.payload.speed_i = speed_i;
  }while(p.payload.speed_i != speed_i);
  do{
    p.payload.speed_o = speed_o;
  }while(p.payload.speed_o != speed_o);

  
  do{
    p.payload.steering_angle_i= steering_angle_i;
  }while(p.payload.steering_angle_i != steering_angle_i);
  do{
    p.payload.steering_angle_o = steering_angle_o;
  }while(p.payload.steering_angle_o != steering_angle_o);

  //p.payload.enc = enc;
  // treba nam niz za rezultata ultrazvucnog senzora
 // p.payload.cfg = read_cfg();

  p.crc = CRC16().add(p.payload).get_crc();

  // Upisuje paket
  Serial.write(
    reinterpret_cast<u8*>(&p),
    sizeof(p)
  );
}


typedef unsigned long ms_t;

void loop() {
  // if (console_mode) {
  //   // Tekstualni režim za picocom
  //   //handle_console();
  //   print_status_if_changed();  // opciono: tiho stanje bez spama
  //   // ne zovi send_pkg() u konzolnom rezimu da ne izlazi binarni „šum”
  //   delay(1);
  //   return;
  // }

  
  poll_pkg();


  print_status_if_changed();
  
  // Send slower than reading.
  static ms_t t_prev;
  ms_t t_curr = millis();
  
  if((t_curr - t_prev) > 1000/SENSOR_HZ) {
    t_prev = t_curr;
    send_pkg();

    static u16 cnt = 0;
    cnt++;
    if(cnt == SENSOR_HZ*5){
      cnt = 0;
      print_status();
    }
  }
  //handle_console();

}
// ===== Console mode toggle =====

// void print_help() {
//   Serial.println(F("=== Console Mode ==="));
//   Serial.println(F("Komande:"));
//   Serial.println(F("  S<val>   -> Set speed, npr: S200 ili S-150"));
//   Serial.println(F("  A<deg>   -> Set servo angle [0..180], npr: A120"));
//   Serial.println(F("  P        -> Print status (enc, speed, angle)"));
//   Serial.println(F("  BIN      -> Predji u binarni rezim (ROS/packet)"));
//   Serial.println(F("  CON      -> Vrati se u konzolni rezim"));
//   Serial.println(F("  H        -> Help"));
// }

// void handle_console() {
//   if (!Serial.available()) return;

//   String cmd = Serial.readStringUntil('\n');
//   cmd.trim();
//   cmd.toUpperCase();

//   if (cmd == "H" || cmd == "HELP") {
//     print_help();
//     return;
//   }
//   if (cmd == "CON") {
//     console_mode = true;
//     Serial.println(F("Console mode ON"));
//     return;
//   }
//   if (cmd == "BIN") {
//     console_mode = false;
//     Serial.println(F("Binary (packet) mode ON"));
//     return;
//   }
//   if (cmd == "P") {
//     Serial.print(F("enc=")); Serial.print((long)enc);
//     Serial.print(F(" speed_i=")); Serial.print((int)speed_i);
//     Serial.print(F(" angle=")); Serial.println((int)steering_angle_i);
//     return;
//   }
//   if (cmd.startsWith("S")) {
//     int val = cmd.substring(1).toInt();
//     set_target_speed(val);
//     Serial.print(F("Speed set to ")); Serial.println(val);
//     return;
//   }
//   if (cmd.startsWith("A")) {
//     int val = cmd.substring(1).toInt();
//     set_target_steering_angle(val);
//     Serial.print(F("Angle set to ")); Serial.println(val);
//     return;
//   }

//   Serial.println(F("Nepoznata komanda. Kucaj H za help."));
// }
