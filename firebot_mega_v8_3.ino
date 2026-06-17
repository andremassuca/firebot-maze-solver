/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║      FIREBOT + MAZE SOLVER — Arduino Mega 2560               ║
 * ║      Código v8.3 — scheduler servos + mapa maze + melhorias  ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * NOVIDADES v8.3:
 *  + Velocidade adaptativa — reduz em curvas, máxima em linha recta
 *  + Confidence meter de chama — confirma 5× antes de LOCALIZAR
 *  + LEDs em IDLE mostram bateria: VERDE=OK AMARELO=médio VERMELHO=baixo
 *  + RETORNAR a 70% da velocidade — mais preciso e seguro
 *  + Scheduler de servos por estado — sem conflito entre Elecbee e sonar
 *  + Scan de intersecção no MAZE (sonar servo) — L/F/R medidos no ponto
 *  + Mapa básico do labirinto — imprime no Serial no fim de cada MAZE
 *
 * MONTAGEM FÍSICA DOS SERVOS (evita bloqueio mútuo):
 *  servoFlame (Elecbee, D9):  topo do piso 2, inclinado ~15° para baixo
 *  servoUS    (sonar,  D10):  meio do piso 2, horizontal
 *  → Elecbee olha para a altura da vela, sonar olha para as paredes
 *
 * SCHEDULER DE SERVOS:
 *  BUSCA/APROXIMAR → Elecbee varre, sonar fica a 90°
 *  MAZE            → sonar varre intersecções, Elecbee para a 90°
 *  LOCALIZAR       → Elecbee faz sweep 180°, sonar para a 90°
 *  Outros          → ambos a 90°
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <MPU6050.h>
#include <NewPing.h>
#define DECODE_NEC
#define IR_RECEIVE_PIN 2
#define IR_USE_AVR_TIMER3
#include <IRremote.hpp>
#include <avr/wdt.h>

// ============================================================
// PINOS
// ============================================================
#define ENA              3
#define IN1              4
#define IN2              5
#define IN3              6
#define IN4              7
#define ENB              8
#define SERVO_FLAME_PIN  9
#define SERVO_US_PIN    10
#define HC_FL_TRIG      22
#define HC_FL_ECHO      23
#define HC_FR_TRIG      24
#define HC_FR_ECHO      25
#define HC_SC_TRIG      26
#define HC_SC_ECHO      27
#define FLAME_A0        A0
#define FLAME_A1        A1
#define FLAME_A2        A2
#define FLAME_A3        A3
#define FLAME_A4        A4
#define KY026_PIN       28
#define IR_BOUND_L      29
#define IR_BOUND_R      30
#define FAN_PIN         44
#define BUZZER_PIN      45
#define LED_R           46
#define LED_Y           47
#define LED_G           48
#define BTN_START       49
#define LINE_S1         A6
#define LINE_S2         A7
#define LINE_S3         A8
#define LINE_S4         A9
#define LINE_S5         A10
#define BAT_PIN         A11

// ============================================================
// CÓDIGOS IR
// ============================================================
#define IR_CMD_POWER    0x45
#define IR_CMD_VOLUP    0x46
#define IR_CMD_VOLDOWN  0x15
#define IR_CMD_STOP     0x47
#define IR_CMD_LEFT     0x44
#define IR_CMD_PLAY     0x40
#define IR_CMD_RIGHT    0x43
#define IR_CMD_DOWN     0x07
#define IR_CMD_UP       0x09
#define IR_CMD_EQ       0x19
#define IR_CMD_STREPT   0x0D
#define IR_CMD_0        0x16
#define IR_CMD_1        0x0C
#define IR_CMD_2        0x18
#define IR_CMD_3        0x5E
#define IR_CMD_4        0x08
#define IR_CMD_5        0x1C
#define IR_CMD_6        0x5A
#define IR_CMD_7        0x42
#define IR_CMD_8        0x52
#define IR_CMD_9        0x4A

// ============================================================
// CONSTANTES
// ============================================================
#define ROBOT_WIDTH_CM      18
#define ROBOT_LENGTH_CM     27
#define MAX_SPEED           220
#define TURN_SPEED          160
#define FLAME_ANALOG_THRESHOLD  350
#define OBSTACLE_STOP        15
#define OBSTACLE_AVOID       28
#define MAZE_WALL_THRESHOLD  (ROBOT_WIDTH_CM + 8)
#define MAZE_COOLDOWN_MS     900
#define MAZE_SPEED           130
#define FOLLOW_TARGET_CM     20
#define FOLLOW_DEAD_ZONE      3
#define INCLINE_THRESHOLD  2000
#define INCLINE_BOOST_MAX   1.4f
#define EXTINGUIR_TIMEOUT_MS 12000
#define SQUARE_SIDE_MS       2500
#define SPEED_STEP           10
#define SPEED_MIN            80
#define SPEED_MAX            220
#define SPEED_TEST_SPEED     200
#define CALIBRAR_DURATION_MS 5000
#define SOFT_START_MS        200
#define LOW_BATTERY_V        7.0f   // vermelho
#define MED_BATTERY_V        8.0f   // amarelo
#define BTN_LONG_PRESS_MS    800
#define MANUAL_TIMEOUT_MS    500

// Velocidade adaptativa — factores por nível de curvatura
#define ADAPT_ERR_HIGH      150.0f  // curva apertada
#define ADAPT_ERR_MED        80.0f  // curva média
#define ADAPT_FACTOR_HIGH    0.65f
#define ADAPT_FACTOR_MED     0.82f

// Confidence meter de chama
#define FLAME_CONF_THRESH     5    // detecções consecutivas para confirmar
#define FLAME_CONF_MAX        8

// Velocidade de retorno
#define RETORNAR_FACTOR      0.70f

// Mapa do labirinto
#define MAX_MAZE_NODES       20

// EEPROM
#define EEPROM_MAGIC     0x42
#define EEPROM_ADDR_MAG  0
#define EEPROM_ADDR_THR  1
#define EEPROM_ADDR_GYR  3
#define EEPROM_ADDR_SPD  5
#define EEPROM_ADDR_KP   7
#define EEPROM_ADDR_KD   11
#define EEPROM_ADDR_KI   15

// ============================================================
// OBJECTOS
// ============================================================
Servo servoFlame;
Servo servoUS;
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;
NewPing sonarFL(HC_FL_TRIG, HC_FL_ECHO, 200);
NewPing sonarFR(HC_FR_TRIG, HC_FR_ECHO, 200);
NewPing sonarSC(HC_SC_TRIG, HC_SC_ECHO, 300);

double kpLine=22.0, kiLine=0.0, kdLine=12.0;
double kpHead=2.5,  kiHead=0.05, kdHead=0.8;
double linePID_In=0, linePID_Out=0, linePID_SP=0;
PID linePID(&linePID_In,&linePID_Out,&linePID_SP, kpLine,kiLine,kdLine, DIRECT);
double headPID_In=0, headPID_Out=0, headPID_SP=0;
PID headPID(&headPID_In,&headPID_Out,&headPID_SP, kpHead,kiHead,kdHead, DIRECT);

// ============================================================
// ESTADOS
// ============================================================
enum Estado {
  IDLE, BUSCA, LOCALIZAR, APROXIMAR, EXTINGUIR, RETORNAR,
  MAZE, LINE_ONLY, MODO_MANUAL, SENSOR_TEST,
  CALIBRAR, SPEED_TEST, GYRO_DISP, MOTOR_TEST,
  FOLLOW, COMPETITION
};
Estado estadoActual = IDLE;
unsigned long estadoInicio = 0;

enum CompSub { COMP_MAZE, COMP_LOCALIZAR, COMP_APROXIMAR, COMP_EXTINGUIR, COMP_RECUPERAR };
CompSub compSub = COMP_MAZE;

// ============================================================
// MAPA DO LABIRINTO
// ============================================================
struct MazeNode {
  int16_t heading;   // heading no nó (graus)
  uint8_t dF;        // distância frente (cm, max 200)
  uint8_t dL;        // distância esquerda
  uint8_t dR;        // distância direita
  uint8_t decision;  // 0=ESQ 1=FRENTE 2=DIR 3=TRÁS
};
MazeNode mazeMap[MAX_MAZE_NODES];
uint8_t mazeNodeCount = 0;

// ============================================================
// VARIÁVEIS GLOBAIS
// ============================================================
int BASE_SPEED    = 150;
int lineThreshold = 500;
float lastLineError  = 0;
int flameConfidence  = 0;
unsigned long runStartTime  = 0;
unsigned long lastManualCmd = 0;

float heading = 0.0, targetHeading = 0.0;
unsigned long lastGyroMicros = 0;
int16_t gz_offset = 0;

int sirenFreq = 700; bool sirenUp = true;
unsigned long lastSiren = 0, lastLCD = 0;

bool btnAnterior = HIGH;
unsigned long btnPressStart = 0;
bool btnLongHandled = false;
bool lowBatWarning = false;

// ============================================================
// EEPROM
// ============================================================
void saveToEEPROM() {
  EEPROM.write(EEPROM_ADDR_MAG,EEPROM_MAGIC);
  EEPROM.put(EEPROM_ADDR_THR,lineThreshold);
  EEPROM.put(EEPROM_ADDR_GYR,gz_offset);
  EEPROM.put(EEPROM_ADDR_SPD,BASE_SPEED);
  EEPROM.put(EEPROM_ADDR_KP,(float)kpLine);
  EEPROM.put(EEPROM_ADDR_KD,(float)kdLine);
  EEPROM.put(EEPROM_ADDR_KI,(float)kiLine);
  Serial.println(F("EEPROM guardada."));
}
void loadFromEEPROM() {
  if(EEPROM.read(EEPROM_ADDR_MAG)!=EEPROM_MAGIC){Serial.println(F("EEPROM: sem dados."));return;}
  EEPROM.get(EEPROM_ADDR_THR,lineThreshold);
  EEPROM.get(EEPROM_ADDR_GYR,gz_offset);
  EEPROM.get(EEPROM_ADDR_SPD,BASE_SPEED);
  float kp,kd,ki;
  EEPROM.get(EEPROM_ADDR_KP,kp);EEPROM.get(EEPROM_ADDR_KD,kd);EEPROM.get(EEPROM_ADDR_KI,ki);
  kpLine=kp;kdLine=kd;kiLine=ki;
  Serial.print(F("EEPROM: thr="));Serial.print(lineThreshold);Serial.print(F(" sp="));Serial.print(BASE_SPEED);Serial.print(F(" kp="));Serial.println(kpLine);
}

// ============================================================
// BATERIA
// ============================================================
float lerBateria(){return(analogRead(BAT_PIN)*5.0f/1023.0f)*2.0f;}
void verificarBateria(){
  static unsigned long lc=0; if(millis()-lc<5000)return; lc=millis();
  float v=lerBateria(); lowBatWarning=(v<LOW_BATTERY_V&&v>1.0f);
  if(lowBatWarning){Serial.print(F("BAT BAIXA: "));Serial.print(v);Serial.println(F("V"));}
}

// ============================================================
// VELOCIDADE ADAPTATIVA
// ============================================================
int getAdaptiveSpeed() {
  float absErr=abs(lastLineError);
  if(absErr>ADAPT_ERR_HIGH) return max((int)(BASE_SPEED*ADAPT_FACTOR_HIGH),SPEED_MIN);
  if(absErr>ADAPT_ERR_MED)  return (int)(BASE_SPEED*ADAPT_FACTOR_MED);
  return getSoftSpeed();
}
int getSoftSpeed(){
  unsigned long el=millis()-estadoInicio;
  if(el>=SOFT_START_MS)return BASE_SPEED;
  return map(constrain((long)el,0L,(long)SOFT_START_MS),0L,(long)SOFT_START_MS,(long)SPEED_MIN,(long)BASE_SPEED);
}

// ============================================================
// INCLINAÇÃO
// ============================================================
float getInclineBoost(){
  int16_t ax,ay,az,gx,gy,gz;mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  if(ax<-INCLINE_THRESHOLD){float b=1.0f+(float)(-ax-INCLINE_THRESHOLD)/16384.0f*1.5f;return min(b,INCLINE_BOOST_MAX);}
  return 1.0f;
}

// ============================================================
// CONFIDENCE METER DE CHAMA
// ============================================================
void updateFlameConf(){
  if(chamaDetetada()) flameConfidence=min(flameConfidence+1,FLAME_CONF_MAX);
  else                flameConfidence=max(flameConfidence-1,0);
}
bool flameConfirmed(){return flameConfidence>=FLAME_CONF_THRESH;}

// ============================================================
// MAPA DO LABIRINTO
// ============================================================
void addMazeNode(int h,uint8_t dF,uint8_t dL,uint8_t dR,uint8_t dec){
  if(mazeNodeCount>=MAX_MAZE_NODES)return;
  mazeMap[mazeNodeCount]={h,dF,dL,dR,dec};
  mazeNodeCount++;
}
void printMazeMap(){
  Serial.print(F("=== MAZE MAP (")); Serial.print(mazeNodeCount); Serial.println(F(" nós) ==="));
  const char* dirs[]={"ESQ","FRENTE","DIR","TRAS"};
  for(int i=0;i<mazeNodeCount;i++){
    Serial.print(F("N"));Serial.print(i);
    Serial.print(F(" H:"));Serial.print(mazeMap[i].heading);
    Serial.print(F(" F:"));Serial.print(mazeMap[i].dF);
    Serial.print(F("cm L:"));Serial.print(mazeMap[i].dL);
    Serial.print(F("cm R:"));Serial.print(mazeMap[i].dR);
    Serial.print(F("cm → "));Serial.println(dirs[mazeMap[i].decision]);
  }
  mazeNodeCount=0;
}

// ============================================================
// SCAN DE INTERSECÇÃO (sonar servo)
// Usa o servo do sonar para medir L/F/R no ponto de decisão
// ============================================================
void scanIntersecao(unsigned int& dF,unsigned int& dL,unsigned int& dR){
  servoFlame.write(90); // Elecbee para ao centro durante o scan

  servoUS.write(10);  delay(300); wdt_reset();
  dL=sonarSC.ping_cm(); if(dL==0)dL=200;

  servoUS.write(90);  delay(200); wdt_reset();
  dF=sonarSC.ping_cm(); if(dF==0)dF=200;

  servoUS.write(170); delay(300); wdt_reset();
  dR=sonarSC.ping_cm(); if(dR==0)dR=200;

  servoUS.write(90);  delay(150); wdt_reset();
}

// ============================================================
// TUNING VIA SERIAL
// ============================================================
void lerSerial(){
  if(!Serial.available())return;
  String s=Serial.readStringUntil('\n');s.trim();
  if(s=="save"){saveToEEPROM();return;}
  if(s=="load"){loadFromEEPROM();linePID.SetTunings(kpLine,kiLine,kdLine);return;}
  if(s=="bat"){Serial.print(F("Bat:"));Serial.print(lerBateria());Serial.println(F("V"));return;}
  if(s=="map"){printMazeMap();return;}
  if(s=="status"){Serial.print(F("thr="));Serial.print(lineThreshold);Serial.print(F(" sp="));Serial.print(BASE_SPEED);Serial.print(F(" kp="));Serial.print(kpLine);Serial.print(F(" kd="));Serial.print(kdLine);Serial.print(F(" bat="));Serial.print(lerBateria());Serial.println(F("V"));return;}
  int eq=s.indexOf('=');if(eq<0)return;
  String key=s.substring(0,eq);float val=s.substring(eq+1).toFloat();
  if(key=="kp"){kpLine=val;linePID.SetTunings(kpLine,kiLine,kdLine);Serial.print(F("kp="));Serial.println(kpLine);}
  else if(key=="kd"){kdLine=val;linePID.SetTunings(kpLine,kiLine,kdLine);Serial.print(F("kd="));Serial.println(kdLine);}
  else if(key=="ki"){kiLine=val;linePID.SetTunings(kpLine,kiLine,kdLine);}
  else if(key=="hkp"){kpHead=val;headPID.SetTunings(kpHead,kiHead,kdHead);}
  else if(key=="hkd"){kdHead=val;headPID.SetTunings(kpHead,kiHead,kdHead);}
  else if(key=="sp"){BASE_SPEED=constrain((int)val,SPEED_MIN,SPEED_MAX);linePID.SetOutputLimits(-BASE_SPEED,BASE_SPEED);}
  else if(key=="th"){lineThreshold=(int)val;Serial.print(F("th="));Serial.println(lineThreshold);}
}

// ============================================================
// SETUP
// ============================================================
void setup(){
  Serial.begin(115200);
  Serial.println(F("=== FIREBOT + MAZE v8.3 ==="));

  pinMode(ENA,OUTPUT);pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);pinMode(IN4,OUTPUT);
  pinMode(FAN_PIN,OUTPUT);pinMode(BUZZER_PIN,OUTPUT);
  pinMode(LED_R,OUTPUT);pinMode(LED_Y,OUTPUT);pinMode(LED_G,OUTPUT);
  pinMode(BTN_START,INPUT_PULLUP);
  pinMode(IR_BOUND_L,INPUT_PULLUP);pinMode(IR_BOUND_R,INPUT_PULLUP);
  pinMode(KY026_PIN,INPUT_PULLUP);
  pinMode(BAT_PIN,INPUT);

  motorsStop();analogWrite(FAN_PIN,0);
  digitalWrite(LED_R,HIGH);digitalWrite(LED_Y,LOW);digitalWrite(LED_G,LOW);

  servoFlame.attach(SERVO_FLAME_PIN);servoUS.attach(SERVO_US_PIN);
  servoFlame.write(90);servoUS.write(90);
  delay(500);

  Wire.begin();lcd.init();lcd.backlight();
  lcdPrint("FIREBOT+MAZE v8.3","A iniciar...");

  loadFromEEPROM();

  mpu.initialize();
  if(!mpu.testConnection()){lcdPrint("ERRO: MPU-6050","Verificar I2C!");while(true){digitalWrite(LED_R,!digitalRead(LED_R));delay(200);}}
  Serial.println(F("MPU: OK"));

  calibrarGyro();

  linePID.SetMode(AUTOMATIC);linePID.SetOutputLimits(-MAX_SPEED,MAX_SPEED);linePID.SetSampleTime(10);
  headPID.SetMode(AUTOMATIC);headPID.SetOutputLimits(-90,90);headPID.SetSampleTime(10);

  IrReceiver.begin(IR_RECEIVE_PIN,ENABLE_LED_FEEDBACK);
  Serial.println(F("IR pronto (Timer3, NEC)"));
  Serial.println(F("Serial: kp= kd= ki= hkp= hkd= sp= th= save load status bat map"));

  Serial.println(F("Elecbee em repouso:"));
  for(int i=0;i<5;i++){Serial.print(F("  C"));Serial.print(i+1);Serial.print(F(":"));Serial.println(analogRead(FLAME_A0+i));}

  float bat=lerBateria();
  if(bat>1.0f){Serial.print(F("Bat:"));Serial.print(bat);Serial.println(F("V"));}

  lcdPrint("PRONTO v8.3","1=fire 5=man 2=maze");
  wdt_enable(WDTO_8S);
  beep(2,250);wdt_reset();
}

// ============================================================
// LOOP
// ============================================================
void loop(){
  wdt_reset();
  lerSerial();
  verificarBateria();

  bool bordaE=(digitalRead(IR_BOUND_L)==LOW);
  bool bordaD=(digitalRead(IR_BOUND_R)==LOW);
  if(estadoActual!=IDLE&&estadoActual!=MODO_MANUAL&&
     estadoActual!=SENSOR_TEST&&estadoActual!=GYRO_DISP&&
     estadoActual!=MOTOR_TEST&&(bordaE||bordaD)){
    recuperarBordaDir(bordaE,bordaD);return;
  }

  verificarInput();

  switch(estadoActual){
    case IDLE:        doIDLE();        break;
    case BUSCA:       doBUSCA();       break;
    case LOCALIZAR:   doLOCALIZAR();  break;
    case APROXIMAR:   doAPROXIMAR();  break;
    case EXTINGUIR:   doEXTINGUIR();  break;
    case RETORNAR:    doRETORNAR();   break;
    case MAZE:        doMAZE();        break;
    case LINE_ONLY:   doLINE();        break;
    case MODO_MANUAL: doMODO_MANUAL();break;
    case SENSOR_TEST: doSensorTest(); break;
    case CALIBRAR:    doCalibracao(); break;
    case SPEED_TEST:  doSpeedTest();  break;
    case GYRO_DISP:   doGyroDisp();   break;
    case MOTOR_TEST:  doMotorTest();  break;
    case FOLLOW:      doFOLLOW();     break;
    case COMPETITION: doCOMPETITION();break;
  }

  if(millis()-lastLCD>400){atualizarLCD();atualizarLEDs();lastLCD=millis();}
}

// ============================================================
// MODO_MANUAL
// ============================================================
void doMODO_MANUAL(){
  if(millis()-lastManualCmd>MANUAL_TIMEOUT_MS)motorsStop();
}

// ============================================================
// ESTADOS FIREBOT
// ============================================================
void doIDLE(){
  motorsStop();analogWrite(FAN_PIN,0);noTone(BUZZER_PIN);
  servoFlame.write(90);servoUS.write(90);
  flameConfidence=0;
}

void doBUSCA(){
  // SCHEDULER: Elecbee varre, sonar fica a 90°
  static int fp=90,fd=3;static unsigned long lm=0;
  if(millis()-lm>35){fp+=fd;if(fp>=180||fp<=0)fd=-fd;servoFlame.write(fp);lm=millis();}
  servoUS.write(90); // sonar fixo ao centro — fixos cobrem obstáculos

  updateFlameConf();
  if(flameConfirmed()){motorsStop();mudarEstado(LOCALIZAR);return;}

  unsigned int dF=sonarSC.ping_cm();
  if(dF>0&&dF<OBSTACLE_AVOID){evitarObstaculo();return;}

  float e=lerErroLinha();
  int spd=getAdaptiveSpeed();float boost=getInclineBoost();
  if(e>900.0f){motorDef(55,-55);return;}
  linePID_In=e;linePID.Compute();
  motorDef(constrain((int)((spd+(int)linePID_Out)*boost),0,MAX_SPEED),
           constrain((int)((spd-(int)linePID_Out)*boost),0,MAX_SPEED));
}

void doLOCALIZAR(){
  // SCHEDULER: Elecbee faz sweep, sonar para a 90°
  servoUS.write(90);
  motorsStop();delay(100);
  int mp=90,ms=1023;
  for(int p=0;p<=180;p+=5){servoFlame.write(p);delay(20);wdt_reset();int s=lerIntensidadeChama();if(s<ms){ms=s;mp=p;}}
  for(int p=max(0,mp-20);p<=min(180,mp+20);p+=2){servoFlame.write(p);delay(15);wdt_reset();int s=lerIntensidadeChama();if(s<ms){ms=s;mp=p;}}
  servoFlame.write(90);
  if(ms>=FLAME_ANALOG_THRESHOLD){flameConfidence=0;mudarEstado(BUSCA);return;}
  targetHeading=heading+(mp-90.0f)*0.8f;
  girarParaAngulo(targetHeading);
  mudarEstado(APROXIMAR);
}

void doAPROXIMAR(){
  // SCHEDULER: Elecbee rastreia chama, sonar a 90° (fixos cobrem lados)
  servoUS.write(90);
  atualizarHeading();
  unsigned int dC=sonarSC.ping_cm(),dE=sonarFL.ping_cm(),dD=sonarFR.ping_cm();
  if(dC>0&&dC<=OBSTACLE_STOP){motorsStop();mudarEstado(EXTINGUIR);return;}
  if((dE>0&&dE<OBSTACLE_AVOID)||(dD>0&&dD<OBSTACLE_AVOID)){evitarObstaculoAproximando(dE,dD);return;}
  if(!chamaDetetada()){flameConfidence=0;motorsStop();mudarEstado(LOCALIZAR);return;}
  float dir=lerDirecaoChama(),eh=heading-targetHeading;
  while(eh>180.0f)eh-=360.0f;while(eh<-180.0f)eh+=360.0f;
  headPID_In=(dir<900.0f)?(dir*15.0f)*0.7f+eh*0.3f:eh;
  headPID.Compute();float boost=getInclineBoost();
  motorDef(constrain((int)((BASE_SPEED-(int)headPID_Out)*boost),0,MAX_SPEED),
           constrain((int)((BASE_SPEED+(int)headPID_Out)*boost),0,MAX_SPEED));
}

void doEXTINGUIR(){
  motorsStop();analogWrite(FAN_PIN,255);
  if(millis()-lastSiren>12){
    lastSiren=millis();tone(BUZZER_PIN,sirenFreq);
    sirenFreq+=sirenUp?10:-10;
    if(sirenFreq>=960)sirenUp=false;if(sirenFreq<=700)sirenUp=true;
  }
  if(!chamaDetetada()&&digitalRead(KY026_PIN)==HIGH){
    unsigned long t=millis()-runStartTime;
    Serial.print(F("TEMPO: "));Serial.print(t/1000);Serial.print(F("."));Serial.print((t%1000)/100);Serial.println(F("s"));
    desligarFanEBuzzer();beep(5,100);mudarEstado(RETORNAR);return;
  }
  if(millis()-estadoInicio>EXTINGUIR_TIMEOUT_MS){desligarFanEBuzzer();mudarEstado(RETORNAR);}
}

void doRETORNAR(){
  // Velocidade reduzida para mais precisão na linha
  desligarFanEBuzzer();
  int retSpd=(int)(BASE_SPEED*RETORNAR_FACTOR);
  float e=lerErroLinha();
  if(e<900.0f){
    linePID_In=e;linePID.Compute();
    motorDef(constrain(retSpd+(int)linePID_Out,0,MAX_SPEED),
             constrain(retSpd-(int)linePID_Out,0,MAX_SPEED));
  } else {
    if(lastLineError>0)motorDef(TURN_SPEED/2,-TURN_SPEED/2);
    else               motorDef(-TURN_SPEED/2,TURN_SPEED/2);
  }
}

void doLINE(){
  analogWrite(FAN_PIN,0);servoFlame.write(90);servoUS.write(90);
  unsigned int dF=sonarSC.ping_cm();
  if(dF>0&&dF<OBSTACLE_AVOID){evitarObstaculo();return;}
  float e=lerErroLinha();
  int spd=getAdaptiveSpeed();float boost=getInclineBoost();
  if(e>900.0f){if(lastLineError>0)motorDef(60,-60);else motorDef(-60,60);return;}
  linePID_In=e;linePID.Compute();
  motorDef(constrain((int)((spd+(int)linePID_Out)*boost),0,MAX_SPEED),
           constrain((int)((spd-(int)linePID_Out)*boost),0,MAX_SPEED));
}

// ============================================================
// MAZE com scan de intersecção + mapa
// ============================================================
void doMAZE(){
  // SCHEDULER: sonar servo faz scan nas intersecções, Elecbee a 90°
  servoFlame.write(90);
  analogWrite(FAN_PIN,0);noTone(BUZZER_PIN);

  static unsigned long lastDec=0;static bool jt=false;
  atualizarHeading();

  if(jt&&millis()-lastDec<MAZE_COOLDOWN_MS){
    headPID_In=heading-targetHeading;
    while(headPID_In>180.0f)headPID_In-=360.0f;while(headPID_In<-180.0f)headPID_In+=360.0f;
    headPID.Compute();float boost=getInclineBoost();
    motorDef(constrain((int)((MAZE_SPEED-(int)headPID_Out)*boost),0,MAX_SPEED),
             constrain((int)((MAZE_SPEED+(int)headPID_Out)*boost),0,MAX_SPEED));
    return;
  }
  jt=false;

  // Para na intersecção e faz scan com sonar servo
  motorsStop();
  unsigned int dF,dL,dR;
  scanIntersecao(dF,dL,dR);

  Serial.print(F("MAZE: F="));Serial.print(dF);
  Serial.print(F(" L="));Serial.print(dL);
  Serial.print(F(" R="));Serial.println(dR);

  // Left-hand rule
  uint8_t decision;
  if(dL>MAZE_WALL_THRESHOLD){
    targetHeading=heading-90.0f;
    girarParaAngulo(targetHeading);
    decision=0;
  } else if(dF>MAZE_WALL_THRESHOLD){
    // Segue em frente — usa heading PID
    decision=1;
  } else if(dR>MAZE_WALL_THRESHOLD){
    targetHeading=heading+90.0f;
    girarParaAngulo(targetHeading);
    decision=2;
  } else {
    targetHeading=heading+180.0f;
    girarParaAngulo(targetHeading);
    decision=3;
  }

  // Guarda no mapa
  addMazeNode((int)heading,(uint8_t)min(dF,200U),(uint8_t)min(dL,200U),(uint8_t)min(dR,200U),decision);

  lastDec=millis();jt=true;
}

// ============================================================
// FOLLOW
// ============================================================
void doFOLLOW(){
  atualizarHeading();
  unsigned int d=sonarSC.ping_cm();if(d==0)d=100;
  int erro=(int)d-FOLLOW_TARGET_CM;
  if(abs(erro)<=FOLLOW_DEAD_ZONE){motorsStop();return;}
  headPID_In=heading-targetHeading;
  while(headPID_In>180.0f)headPID_In-=360.0f;while(headPID_In<-180.0f)headPID_In+=360.0f;
  headPID.Compute();
  int vel=constrain(BASE_SPEED+abs(erro)*4,80,MAX_SPEED);
  if(erro>0)motorDef(constrain(vel-(int)headPID_Out,0,MAX_SPEED),constrain(vel+(int)headPID_Out,0,MAX_SPEED));
  else       motorDef(-vel,-vel);
}

// ============================================================
// COMPETITION
// ============================================================
void doCOMPETITION(){
  static unsigned long compExtStart=0;
  servoFlame.write(90); // default — será sobreposto quando necessário
  switch(compSub){
    case COMP_MAZE:{
      analogWrite(FAN_PIN,0);
      updateFlameConf();
      if(flameConfirmed()){motorsStop();compSub=COMP_LOCALIZAR;return;}
      static unsigned long lastDec=0;static bool jt=false;
      atualizarHeading();
      if(jt&&millis()-lastDec<MAZE_COOLDOWN_MS){
        headPID_In=heading-targetHeading;while(headPID_In>180.0f)headPID_In-=360.0f;while(headPID_In<-180.0f)headPID_In+=360.0f;
        headPID.Compute();motorDef(constrain(MAZE_SPEED-(int)headPID_Out,0,MAX_SPEED),constrain(MAZE_SPEED+(int)headPID_Out,0,MAX_SPEED));return;
      }
      jt=false;
      unsigned int dF,dL,dR;scanIntersecao(dF,dL,dR);
      if(dL>MAZE_WALL_THRESHOLD){motorsStop();targetHeading=heading-90.0f;girarParaAngulo(targetHeading);}
      else if(dF>MAZE_WALL_THRESHOLD){headPID_In=heading-targetHeading;while(headPID_In>180.0f)headPID_In-=360.0f;while(headPID_In<-180.0f)headPID_In+=360.0f;headPID.Compute();motorDef(constrain(MAZE_SPEED-(int)headPID_Out,0,MAX_SPEED),constrain(MAZE_SPEED+(int)headPID_Out,0,MAX_SPEED));}
      else if(dR>MAZE_WALL_THRESHOLD){motorsStop();targetHeading=heading+90.0f;girarParaAngulo(targetHeading);}
      else{motorsStop();targetHeading=heading+180.0f;girarParaAngulo(targetHeading);}
      lastDec=millis();jt=true;break;}
    case COMP_LOCALIZAR:{
      servoUS.write(90);motorsStop();delay(100);
      int mp=90,ms=1023;
      for(int p=0;p<=180;p+=5){servoFlame.write(p);delay(20);wdt_reset();int s=lerIntensidadeChama();if(s<ms){ms=s;mp=p;}}
      for(int p=max(0,mp-20);p<=min(180,mp+20);p+=2){servoFlame.write(p);delay(15);wdt_reset();int s=lerIntensidadeChama();if(s<ms){ms=s;mp=p;}}
      servoFlame.write(90);
      if(ms>=FLAME_ANALOG_THRESHOLD){flameConfidence=0;compSub=COMP_MAZE;return;}
      targetHeading=heading+(mp-90.0f)*0.8f;girarParaAngulo(targetHeading);
      compSub=COMP_APROXIMAR;break;}
    case COMP_APROXIMAR:{
      servoUS.write(90);atualizarHeading();
      unsigned int dC=sonarSC.ping_cm();
      if(dC>0&&dC<=OBSTACLE_STOP){motorsStop();compSub=COMP_EXTINGUIR;compExtStart=millis();return;}
      if(!chamaDetetada()){flameConfidence=0;compSub=COMP_MAZE;return;}
      float dir=lerDirecaoChama(),eh=heading-targetHeading;
      while(eh>180.0f)eh-=360.0f;while(eh<-180.0f)eh+=360.0f;
      headPID_In=(dir<900.0f)?(dir*15.0f)*0.7f+eh*0.3f:eh;headPID.Compute();
      motorDef(constrain(BASE_SPEED-(int)headPID_Out,0,MAX_SPEED),constrain(BASE_SPEED+(int)headPID_Out,0,MAX_SPEED));break;}
    case COMP_EXTINGUIR:{
      motorsStop();analogWrite(FAN_PIN,255);
      if(millis()-lastSiren>12){lastSiren=millis();tone(BUZZER_PIN,sirenFreq);sirenFreq+=sirenUp?10:-10;if(sirenFreq>=960)sirenUp=false;if(sirenFreq<=700)sirenUp=true;}
      if(!chamaDetetada()&&digitalRead(KY026_PIN)==HIGH){desligarFanEBuzzer();beep(3,100);compSub=COMP_RECUPERAR;return;}
      if(millis()-compExtStart>EXTINGUIR_TIMEOUT_MS){desligarFanEBuzzer();compSub=COMP_RECUPERAR;}break;}
    case COMP_RECUPERAR:{
      desligarFanEBuzzer();flameConfidence=0;
      motorDef(-BASE_SPEED,-BASE_SPEED);delay(500);wdt_reset();
      motorsStop();calibrarGyro();compSub=COMP_MAZE;break;}
  }
}

// ============================================================
// SENSOR TEST
// ============================================================
void doSensorTest(){
  static int fase=0;static unsigned long lf=0;
  motorsStop();analogWrite(FAN_PIN,0);
  if(millis()-lf<2500)return;
  lf=millis();wdt_reset();
  lcd.clear();
  switch(fase){
    case 0: lcd.setCursor(0,0);lcd.print("ELECBEE A0-A4:"); lcd.setCursor(0,1); for(int i=0;i<5;i++){lcd.print(analogRead(FLAME_A0+i)/10);if(i<4)lcd.print(" ");} break;
    case 1:{int p[5]={LINE_S1,LINE_S2,LINE_S3,LINE_S4,LINE_S5}; lcd.setCursor(0,0);lcd.print("LINE T:");lcd.print(lineThreshold); lcd.setCursor(0,1); for(int i=0;i<5;i++){lcd.print(analogRead(p[i])/10);if(i<4)lcd.print(" ");} break;}
    case 2:{unsigned int dF=sonarSC.ping_cm(),dE=sonarFL.ping_cm(),dD=sonarFR.ping_cm(); lcd.setCursor(0,0);lcd.print("HC-SR04 F/E/D"); lcd.setCursor(0,1);lcd.print("F:");lcd.print(dF);lcd.print(" E:");lcd.print(dE);lcd.print(" D:");lcd.print(dD); break;}
    case 3: atualizarHeading(); lcd.setCursor(0,0);lcd.print("GYRO:"); lcd.setCursor(0,1);lcd.print(heading);lcd.print((char)223);lcd.print(" KY:");lcd.print(digitalRead(KY026_PIN)); break;
    case 4:{float bat=lerBateria(); lcd.setCursor(0,0);lcd.print("BAT:");lcd.print(bat,1);lcd.print("V ");lcd.print(lowBatWarning?"LOW!":"OK"); lcd.setCursor(0,1);lcd.print("B:");lcd.print(digitalRead(IR_BOUND_L));lcd.print("/");lcd.print(digitalRead(IR_BOUND_R));lcd.print(" S:");lcd.print(BASE_SPEED); break;}
  }
  fase=(fase+1)%5;
}

void calibrarLinha(){
  static int etapa=0;static int vSobre=0;
  if(etapa==0){
    lcdPrint("CALIB LINHA","Sobre a linha...");delay(2000);
    int pn[5]={LINE_S1,LINE_S2,LINE_S3,LINE_S4,LINE_S5};long s=0;
    for(int i=0;i<5;i++)s+=analogRead(pn[i]);
    vSobre=s/5;
    lcd.clear();lcd.setCursor(0,0);lcd.print("Linha:");lcd.print(vSobre);
    lcd.setCursor(0,1);lcd.print("EQ=fora da linha");
    etapa=1;
  } else {
    lcdPrint("CALIB LINHA","Fora da linha...");delay(2000);
    int pn[5]={LINE_S1,LINE_S2,LINE_S3,LINE_S4,LINE_S5};long s=0;
    for(int i=0;i<5;i++)s+=analogRead(pn[i]);
    lineThreshold=(vSobre+s/5)/2;
    saveToEEPROM();
    lcd.clear();lcd.setCursor(0,0);lcd.print("Thr:");lcd.print(lineThreshold);
    lcd.setCursor(0,1);lcd.print("Guardado OK!");
    beep(2,200);delay(2000);etapa=0;mudarEstado(IDLE);
  }
}

void doCalibracao(){
  static int fase=0;static unsigned long tI=0;static float h0=0;
  switch(fase){
    case 0: lcdPrint("CALIBRAR","Imobil 2s..."); calibrarGyro(); h0=heading; tI=millis(); fase=1; break;
    case 1:
      lcdPrint("CALIBRAR","A medir desvio..");
      atualizarHeading();
      headPID_In=heading-h0;while(headPID_In>180.0f)headPID_In-=360.0f;while(headPID_In<-180.0f)headPID_In+=360.0f;
      headPID.Compute();
      motorDef(constrain(BASE_SPEED-(int)headPID_Out,0,MAX_SPEED),constrain(BASE_SPEED+(int)headPID_Out,0,MAX_SPEED));
      if(millis()-tI>CALIBRAR_DURATION_MS){
        motorsStop();float dev=heading-h0;
        gz_offset+=(int)(dev*131.0f/(CALIBRAR_DURATION_MS/1000.0f));
        saveToEEPROM();
        lcd.clear();lcd.setCursor(0,0);lcd.print("Dev:");lcd.print(dev,1);lcd.print((char)223);
        lcd.setCursor(0,1);lcd.print("Guardado! OK");
        beep(3,150);delay(2000);fase=0;mudarEstado(IDLE);
      }
      break;
  }
}

void doSpeedTest(){
  static unsigned long tI=0;static bool ini=false;
  if(!ini){tI=millis();ini=true;lcdPrint("SPEED TEST","A cronometrar...");}
  unsigned long el=millis()-tI;
  float e=lerErroLinha();float boost=getInclineBoost();
  if(e>900.0f){if(lastLineError>0)motorDef(60,-60);else motorDef(-60,60);}
  else{linePID_In=e;linePID.Compute();motorDef(constrain((int)((SPEED_TEST_SPEED+(int)linePID_Out)*boost),0,MAX_SPEED),constrain((int)((SPEED_TEST_SPEED-(int)linePID_Out)*boost),0,MAX_SPEED));}
  if(millis()-lastLCD>500){lcd.clear();lcd.setCursor(0,0);lcd.print("SPEED TEST");lcd.setCursor(0,1);lcd.print(el/1000);lcd.print(".");lcd.print((el%1000)/100);lcd.print("s");lastLCD=millis();}
  if(el>30000){motorsStop();lcd.clear();lcd.setCursor(0,0);lcd.print("Fim:");lcd.print(el/1000);lcd.print("s");beep(2,300);delay(3000);ini=false;mudarEstado(IDLE);}
}

void doGyroDisp(){
  atualizarHeading();motorsStop();
  if(millis()-lastLCD>200){
    float h=heading;while(h<0)h+=360.0f;while(h>=360)h-=360.0f;
    lcd.clear();lcd.setCursor(0,0);lcd.print("GYRO:");lcd.print(h,1);lcd.print((char)223);
    lcd.setCursor(0,1);lcd.print("raw:");lcd.print((int)heading);lcd.print(" off:");lcd.print(gz_offset);
    lastLCD=millis();
  }
}

void doMotorTest(){
  static int fase=0;static unsigned long tF=0;
  if(tF==0)tF=millis();
  if(millis()-tF<1000){
    switch(fase){
      case 0: lcdPrint("MOTOR TEST","Esq frente..."); digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);analogWrite(ENA,180);analogWrite(ENB,0);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW); break;
      case 1: lcdPrint("MOTOR TEST","Esq tras...");   digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);analogWrite(ENA,180);analogWrite(ENB,0);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW); break;
      case 2: lcdPrint("MOTOR TEST","Dir frente..."); analogWrite(ENA,0);digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);analogWrite(ENB,180); break;
      case 3: lcdPrint("MOTOR TEST","Dir tras...");   analogWrite(ENA,0);digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);analogWrite(ENB,180); break;
    }
    return;
  }
  motorsStop();beep(1,100);delay(300);tF=millis();fase++;
  if(fase>3){fase=0;tF=0;lcdPrint("MOTOR TEST","Concluido!");beep(3,150);delay(1000);mudarEstado(IDLE);}
  wdt_reset();
}

// ============================================================
// MOTORES
// ============================================================
void motorDef(int e,int d){
  if(e>=0){digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);}else{digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);}
  analogWrite(ENA,constrain(abs(e),0,255));
  if(d>=0){digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}else{digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}
  analogWrite(ENB,constrain(abs(d),0,255));
}
void motorsStop(){analogWrite(ENA,0);analogWrite(ENB,0);digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);}

// ============================================================
// SENSORES
// ============================================================
float lerErroLinha(){
  static const int p[5]={-200,-100,0,100,200};
  int pn[5]={LINE_S1,LINE_S2,LINE_S3,LINE_S4,LINE_S5};
  long s=0;int a=0;
  for(int i=0;i<5;i++)if(analogRead(pn[i])>lineThreshold){s+=p[i];a++;}
  if(a==0)return 999.0f;
  float e=(float)s/a;lastLineError=e;return e;
}
int lerIntensidadeChama(){int m=1023;for(int i=0;i<5;i++){int v=analogRead(FLAME_A0+i);if(v<m)m=v;}return m;}
float lerDirecaoChama(){static const int p[5]={-2,-1,0,1,2};long s=0,ps=0;for(int i=0;i<5;i++){int inv=1023-analogRead(FLAME_A0+i);if(inv>(1023-FLAME_ANALOG_THRESHOLD)){s+=(long)p[i]*inv;ps+=inv;}}return(ps==0)?999.0f:(float)s/ps;}
bool chamaDetetada(){return(lerIntensidadeChama()<FLAME_ANALOG_THRESHOLD||digitalRead(KY026_PIN)==LOW);}
bool modoChama(){return(estadoActual==BUSCA||estadoActual==LOCALIZAR||estadoActual==APROXIMAR||estadoActual==EXTINGUIR||(estadoActual==COMPETITION&&(compSub==COMP_APROXIMAR||compSub==COMP_EXTINGUIR)));}

// ============================================================
// GYRO
// ============================================================
void calibrarGyro(){
  lcdPrint("Calibrar gyro","Imobil 2s...");
  long s=0;
  for(int i=0;i<400;i++){int16_t ax,ay,az,gx,gy,gz;mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);s+=gz;if(i%80==0)wdt_reset();delay(5);}
  gz_offset=s/400;heading=0.0f;targetHeading=0.0f;lastGyroMicros=micros();
}
void atualizarHeading(){
  int16_t ax,ay,az,gx,gy,gz;mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  unsigned long n=micros();float dt=(n-lastGyroMicros)/1000000.0f;lastGyroMicros=n;
  heading+=((gz-gz_offset)/131.0f)*dt;
}
void girarParaAngulo(float a){
  unsigned long t=millis();
  while(millis()-t<4000){wdt_reset();atualizarHeading();float e=a-heading;while(e>180.0f)e-=360.0f;while(e<-180.0f)e+=360.0f;if(abs(e)<2.0f)break;int v=constrain((int)(abs(e)*2.8f),65,TURN_SPEED);if(e>0)motorDef(v,-v);else motorDef(-v,v);}
  motorsStop();delay(100);
}
void doQuadrado(){
  lcdPrint("Modo Quadrado","4 lados gyro");calibrarGyro();
  for(int l=0;l<4;l++){float h0=heading;unsigned long t=millis();while(millis()-t<SQUARE_SIDE_MS){wdt_reset();atualizarHeading();headPID_In=heading-h0;while(headPID_In>180.0f)headPID_In-=360.0f;while(headPID_In<-180.0f)headPID_In+=360.0f;headPID.Compute();motorDef(BASE_SPEED-(int)headPID_Out,BASE_SPEED+(int)headPID_Out);}motorsStop();delay(300);girarParaAngulo(heading+90.0f);delay(300);}
  motorsStop();beep(3,200);
}

// ============================================================
// OBSTÁCULOS
// ============================================================
void evitarObstaculo(){
  motorsStop();delay(100);
  unsigned int dE=sonarFL.ping_cm(),dD=sonarFR.ping_cm();
  if(dE==0)dE=200;if(dD==0)dD=200;int s=(dE>=dD)?-1:1;
  motorDef(s*TURN_SPEED,-s*TURN_SPEED);delay(420);wdt_reset();
  motorDef(BASE_SPEED,BASE_SPEED);delay(700);wdt_reset();
  motorDef(-s*TURN_SPEED,s*TURN_SPEED);delay(420);wdt_reset();
  motorDef(BASE_SPEED,BASE_SPEED);delay(720);wdt_reset();
  motorDef(-s*TURN_SPEED,s*TURN_SPEED);delay(400);wdt_reset();
  motorDef(BASE_SPEED,BASE_SPEED);delay(620);wdt_reset();
  motorDef(s*TURN_SPEED,-s*TURN_SPEED);delay(400);wdt_reset();
  motorsStop();
}
void evitarObstaculoAproximando(unsigned int dE,unsigned int dD){
  if(dE==0)dE=200;if(dD==0)dD=200;
  if(dE<OBSTACLE_AVOID&&dE<dD)motorDef(TURN_SPEED,BASE_SPEED/2);else motorDef(BASE_SPEED/2,TURN_SPEED);
  delay(150);wdt_reset();
}
void recuperarBordaDir(bool bE,bool bD){
  tone(BUZZER_PIN,1500,300);
  motorDef(-BASE_SPEED,-BASE_SPEED);delay(350);wdt_reset();
  if(bE&&!bD)motorDef(TURN_SPEED,-TURN_SPEED);
  else if(bD&&!bE)motorDef(-TURN_SPEED,TURN_SPEED);
  else motorDef(TURN_SPEED,-TURN_SPEED);
  delay(400);wdt_reset();motorsStop();
}

// ============================================================
// INTERFACE
// ============================================================
void desligarFanEBuzzer(){analogWrite(FAN_PIN,0);noTone(BUZZER_PIN);sirenFreq=700;sirenUp=true;}
void beep(int n,int ms){for(int i=0;i<n;i++){tone(BUZZER_PIN,1000);delay(ms);noTone(BUZZER_PIN);delay(ms);wdt_reset();}}
void lcdPrint(const char* l1,const char* l2){lcd.clear();lcd.setCursor(0,0);lcd.print(l1);lcd.setCursor(0,1);lcd.print(l2);}

void atualizarLCD(){
  lcd.clear();lcd.setCursor(0,0);
  switch(estadoActual){
    case IDLE:        lcd.print("IDLE");break;
    case BUSCA:       lcd.print("BUSCA");break;
    case LOCALIZAR:   lcd.print("LOCALIZAR");break;
    case APROXIMAR:   lcd.print("APROX");break;
    case EXTINGUIR:   lcd.print("EXTINGUIR");break;
    case RETORNAR:    lcd.print("RETORNAR");break;
    case MAZE:        lcd.print("MAZE");lcd.print(" N:");lcd.print(mazeNodeCount);break;
    case LINE_ONLY:   lcd.print("LINE");break;
    case MODO_MANUAL: lcd.print("MANUAL");break;
    case SENSOR_TEST: lcd.print("SENS");break;
    case CALIBRAR:    lcd.print("CALIBRAR");break;
    case SPEED_TEST:  lcd.print("SPEED");break;
    case GYRO_DISP:   lcd.print("GYRO");break;
    case MOTOR_TEST:  lcd.print("MOTOR");break;
    case FOLLOW:      lcd.print("FOLLOW");break;
    case COMPETITION: lcd.print("COMP");break;
  }
  if(lowBatWarning)lcd.print(" BAT!");
  else if(modoChama()&&chamaDetetada())lcd.print(" F!");
  lcd.setCursor(0,1);
  if(estadoActual==BUSCA||estadoActual==APROXIMAR){
    lcd.print("F:");lcd.print(lerIntensidadeChama());
    lcd.print(" C:");lcd.print(flameConfidence);
    lcd.print(" T:");lcd.print((millis()-runStartTime)/1000);lcd.print("s");
  } else if(estadoActual==IDLE){
    float bat=lerBateria();lcd.print("B:");lcd.print(bat,1);lcd.print("V S:");lcd.print(BASE_SPEED);
  } else {
    unsigned int d=sonarSC.ping_cm();
    lcd.print("D:");lcd.print(d>0?d:0);lcd.print("cm S:");lcd.print(BASE_SPEED);
  }
}

void atualizarLEDs(){
  if(estadoActual==IDLE){
    float bat=lerBateria();
    if(bat>1.0f){
      // VERDE=OK  AMARELO=médio  VERMELHO=baixo
      digitalWrite(LED_G, bat>=MED_BATTERY_V);
      digitalWrite(LED_Y, bat>=LOW_BATTERY_V && bat<MED_BATTERY_V);
      digitalWrite(LED_R, bat<LOW_BATTERY_V);
    } else {
      digitalWrite(LED_R,HIGH);digitalWrite(LED_Y,LOW);digitalWrite(LED_G,LOW);
    }
    return;
  }
  digitalWrite(LED_R,false);
  digitalWrite(LED_Y, estadoActual==BUSCA||estadoActual==LOCALIZAR||estadoActual==APROXIMAR||
                      estadoActual==MAZE||estadoActual==LINE_ONLY||estadoActual==MODO_MANUAL||
                      estadoActual==SENSOR_TEST||estadoActual==CALIBRAR||estadoActual==SPEED_TEST||
                      estadoActual==GYRO_DISP||estadoActual==MOTOR_TEST||estadoActual==FOLLOW||
                      (estadoActual==COMPETITION&&compSub==COMP_MAZE));
  digitalWrite(LED_G, estadoActual==EXTINGUIR||estadoActual==RETORNAR||
                      (estadoActual==COMPETITION&&(compSub==COMP_EXTINGUIR||compSub==COMP_RECUPERAR)));
}

// ============================================================
// INPUT
// ============================================================
void verificarInput(){
  bool ba=digitalRead(BTN_START);
  if(ba==LOW&&btnAnterior==HIGH){btnPressStart=millis();btnLongHandled=false;delay(40);}
  if(ba==LOW&&!btnLongHandled&&millis()-btnPressStart>BTN_LONG_PRESS_MS){
    btnLongHandled=true;
    if(estadoActual==IDLE){targetHeading=heading;lastManualCmd=millis();mudarEstado(MODO_MANUAL);lcdPrint("MODO MANUAL","Setas=mover STOP=sair");}
    else{mudarEstado(IDLE);desligarFanEBuzzer();motorsStop();}
  }
  if(ba==HIGH&&btnAnterior==LOW&&!btnLongHandled){
    if(estadoActual==IDLE){runStartTime=millis();mudarEstado(BUSCA);}
    else{mudarEstado(IDLE);desligarFanEBuzzer();motorsStop();}
  }
  btnAnterior=ba;

  if(!IrReceiver.decode())return;
  uint8_t cmd=IrReceiver.decodedIRData.command;
  bool repeat=(IrReceiver.decodedIRData.flags&IRDATA_FLAGS_IS_REPEAT);
  IrReceiver.resume();
  if(IrReceiver.decodedIRData.protocol==UNKNOWN)return;
  if(repeat&&cmd!=IR_CMD_UP&&cmd!=IR_CMD_DOWN&&cmd!=IR_CMD_LEFT&&
             cmd!=IR_CMD_RIGHT&&cmd!=IR_CMD_VOLUP&&cmd!=IR_CMD_VOLDOWN)return;

  if(cmd==IR_CMD_STOP){mudarEstado(IDLE);desligarFanEBuzzer();motorsStop();return;}
  if(cmd==IR_CMD_POWER){
    if(estadoActual==IDLE){runStartTime=millis();mudarEstado(BUSCA);}
    else{mudarEstado(IDLE);desligarFanEBuzzer();motorsStop();}
    return;
  }
  if(cmd==IR_CMD_EQ){
    if(estadoActual==SENSOR_TEST){calibrarLinha();return;}
    if(estadoActual==MAZE){printMazeMap();return;}
    motorsStop();calibrarGyro();saveToEEPROM();lcdPrint("Gyro+EEPROM","OK");beep(1,300);return;
  }

  if(estadoActual==IDLE){
    if     (cmd==IR_CMD_1){runStartTime=millis();mudarEstado(BUSCA);}
    else if(cmd==IR_CMD_2){calibrarGyro();runStartTime=millis();mazeNodeCount=0;mudarEstado(MAZE);}
    else if(cmd==IR_CMD_3){runStartTime=millis();mudarEstado(LINE_ONLY);}
    else if(cmd==IR_CMD_4)  mudarEstado(CALIBRAR);
    else if(cmd==IR_CMD_5){targetHeading=heading;lastManualCmd=millis();mudarEstado(MODO_MANUAL);lcdPrint("MODO MANUAL","Setas=mover STOP=sair");}
    else if(cmd==IR_CMD_6){runStartTime=millis();mudarEstado(SPEED_TEST);}
    else if(cmd==IR_CMD_7)  mudarEstado(GYRO_DISP);
    else if(cmd==IR_CMD_8)  mudarEstado(MOTOR_TEST);
    else if(cmd==IR_CMD_9){targetHeading=heading;mudarEstado(FOLLOW);}
    else if(cmd==IR_CMD_0)  mudarEstado(SENSOR_TEST);
    else if(cmd==IR_CMD_PLAY){doQuadrado();mudarEstado(IDLE);}
    else if(cmd==IR_CMD_LEFT){calibrarGyro();compSub=COMP_MAZE;flameConfidence=0;runStartTime=millis();mazeNodeCount=0;mudarEstado(COMPETITION);}
    return;
  }

  // EQ em MAZE → imprime mapa
  bool isManualCmd=false;
  if(cmd==IR_CMD_UP){if(estadoActual!=MODO_MANUAL)mudarEstado(MODO_MANUAL);atualizarHeading();headPID_In=heading-targetHeading;headPID.Compute();motorDef(constrain(BASE_SPEED-(int)headPID_Out,0,MAX_SPEED),constrain(BASE_SPEED+(int)headPID_Out,0,MAX_SPEED));isManualCmd=true;}
  else if(cmd==IR_CMD_DOWN) {if(estadoActual!=MODO_MANUAL)mudarEstado(MODO_MANUAL);motorDef(-BASE_SPEED,-BASE_SPEED);isManualCmd=true;}
  else if(cmd==IR_CMD_RIGHT){if(estadoActual!=MODO_MANUAL)mudarEstado(MODO_MANUAL);motorDef(TURN_SPEED,-TURN_SPEED);isManualCmd=true;}
  else if(cmd==IR_CMD_LEFT) {if(estadoActual!=MODO_MANUAL)mudarEstado(MODO_MANUAL);motorDef(-TURN_SPEED,TURN_SPEED);isManualCmd=true;}
  if(isManualCmd){lastManualCmd=millis();return;}

  if(cmd==IR_CMD_VOLUP){BASE_SPEED=constrain(BASE_SPEED+SPEED_STEP,SPEED_MIN,SPEED_MAX);linePID.SetOutputLimits(-BASE_SPEED,BASE_SPEED);lcd.clear();lcd.setCursor(0,0);lcd.print("Velocidade:");lcd.setCursor(0,1);lcd.print(BASE_SPEED);}
  else if(cmd==IR_CMD_VOLDOWN){BASE_SPEED=constrain(BASE_SPEED-SPEED_STEP,SPEED_MIN,SPEED_MAX);linePID.SetOutputLimits(-BASE_SPEED,BASE_SPEED);lcd.clear();lcd.setCursor(0,0);lcd.print("Velocidade:");lcd.setCursor(0,1);lcd.print(BASE_SPEED);}
  else if(cmd==IR_CMD_STREPT){motorDef(-BASE_SPEED,-BASE_SPEED);delay(500);wdt_reset();motorsStop();mudarEstado(IDLE);}
}

void mudarEstado(Estado n){
  const char* nm[]={"IDLE","BUSCA","LOCALIZAR","APROXIMAR","EXTINGUIR","RETORNAR","MAZE","LINE_ONLY","MANUAL","SENSOR_TEST","CALIBRAR","SPEED_TEST","GYRO_DISP","MOTOR_TEST","FOLLOW","COMPETITION"};
  Serial.print(F("→ "));Serial.println(nm[n]);
  // Ao sair do MAZE imprime o mapa
  if(estadoActual==MAZE&&n!=MAZE&&mazeNodeCount>0)printMazeMap();
  estadoActual=n;estadoInicio=millis();
  if(n==BUSCA||n==LINE_ONLY||n==MAZE||n==SPEED_TEST||n==COMPETITION){sirenFreq=700;sirenUp=true;}
  if(n==IDLE)flameConfidence=0;
}
