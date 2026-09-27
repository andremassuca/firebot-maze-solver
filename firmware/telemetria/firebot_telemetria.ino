// FIREBOT: firmware de telemetria para a app do PC (Arduino Mega 2560)
// Lê tudo o que está montado e aceita comandos da app pela porta série (115200).
// Se uma peça ainda não estiver ligada, o resto continua a funcionar.
//
// Telemetria (10x por segundo), uma linha:
//   FL:<cm> FR:<cm> FC:<cm> FCOK:<0/1> FA:<0-1023> FD:<0/1> FI:<intensidade> FB:<base> FT:<limiar> BAT:<V> BR:<fator>
//   ST:<estado> MPU:<0/1> WHO:<hex> HDG:<graus> PIT:<graus> ROL:<graus> GZ:<graus/s> GB:<bias>
//   ML:<-255..255> MR:<-255..255> FAN:<0-255>
//   BOK/FOK: 1 se o divisor da bateria / o KY-026 estiverem ligados (0 = pino no ar)
//   BATA:<V> BRA:<fator> BAOK:<0/1>  segunda bateria (Fonte A) no A12
//   LCD:<0/1>  ecrã 16x2 I2C (PCF8574, endereço 0x20-0x27 ou 0x38-0x3F)
//   IR:<hex>   último botão do comando (0 = nenhum)   I2CE:<n> erros no I2C
// Comando IR (Elegoo, NEC) no D2: setas = conduzir, POWER/STOP = parar tudo, >|| = ventoinha, VOL+/- = velocidade
// LEDs: verde D48 (tudo bem), amarelo D47 (manual/calibrar/bateria a meio), vermelho D46 (chama/obstáculo/bateria baixa)
// Buzzer D45
// Tudo sai pela USB (Serial) e pelo HC-05 (Serial1, D18/D19) ao mesmo tempo, a 115200.
// Os comandos também são aceites pelos dois.
// Eventos: linhas começadas por "EV:"
//
// Comandos (terminar com Enter):
//   M <esq> <dir>   motores -255..255 (param sozinhos ao fim de 500 ms sem novo comando)
//   STOP            pára motores e ventoinha
//   FAN <0-255>     ventoinha (MOSFET)
//   CAL GYRO        calibra o giroscópio (robô parado)
//   CAL IR          mede o IR ambiente do sensor de chama (sem chama à frente)
//   FTHR <n>        limiar da chama (acima/abaixo da base)
//   BATR <x>        fator do divisor da Fonte B (20k + 10k = 3.0)
//   BATRA <x>       fator do divisor da Fonte A
//   HDG0            põe a direção atual a 0 graus
//   PING            responde PONG
//   TESTE           acende os LEDs um a um e apita
//   SIMCHAMA        chama simulada durante 20 s (plano B sem vela): dispara sirene e ventoinha
//   SIRENE <0|1>    desliga/liga a sequência automática sirene + ventoinha ao detetar chama
//   DIAG FC         diagnóstico do sonar central: onde está o Echo, se Trig/Echo estão trocados
//   SOM <0|1>       buzzer sem som / com som (fica guardado na EEPROM, sobrevive a reinícios; por defeito sem som)
// Chama detetada (0,3 s): sirene de bombeiros 3 s, depois ventoinha até a chama desaparecer (máx. 8 s).
// Tecla 0 do comando: chama simulada. Tecla EQ: liga/desliga o som.
// Com o robô a andar, aviso de distância como nos carros: LED verde > 50 cm, amarelo 25-50, vermelho < 25,
// vermelho a piscar < 12; bips cada vez mais rápidos abaixo de 60 cm (só com SOM 1).
// Farol IR (plano C): a vela envia de 2 em 2 s um código NEC próprio (endereço 0xF1, comando 0xB7)
// pelo mesmo LED infravermelho. O robô apanha-o pelo recetor do comando e trata-o como "vela detetada",
// sempre identificado como FAROL IR (não como chama).
//   BTAT            configura o HC-05 (tem de estar em modo AT: botão carregado ao ligar)

#include <Wire.h>
#include <EEPROM.h>
#include <stdarg.h>

// Motores: sentidos confirmados em teste
#define ENA 3   // roda DIREITA (Motor A)
#define IN1 4   // direita: trás
#define IN2 5   // direita: frente
#define IN3 6   // esquerda: trás
#define IN4 7   // esquerda: frente
#define ENB 8   // roda ESQUERDA (Motor B)

// Sonares: pinos confirmados com o teste da mão
#define FR_TRIG 22
#define FR_ECHO 23
#define FL_TRIG 24
#define FL_ECHO 25
#define FC_TRIG 26   // sonar central, por baixo, a olhar em frente
#define FC_ECHO 27

#define FLAME_AO A0   // KY-026 saída analógica
#define FLAME_DO 28   // KY-026 saída digital
#define FAN_PIN  44   // gate do MOSFET da ventoinha (PWM)
#define BAT_PIN  A12  // meio do divisor da bateria dos MOTORES (fios ficaram assim na montagem)
#define BATA_PIN A11  // meio do divisor da bateria do ARDUINO
#define BT_BAUD  9600   // HC-05 na velocidade de fábrica: não precisa de modo AT
#define BT_TELEM_MS 500 // pelo Bluetooth só 2 leituras por segundo (9600 não dá para mais)
#define IR_PIN   2    // recetor de IR do comando (saída OUT)
#define BUZZER   45
#define LED_R    46
#define LED_Y    47
#define LED_G    48

#define DEADMAN_MS 500
#define TELEM_MS   100
#define SONAR_MS   30

// Escreve ao mesmo tempo na USB e no HC-05
class SaidaDupla : public Print {
public:
  bool bt = true;
  size_t write(uint8_t c) { Serial.write(c); if (bt) Serial1.write(c); return 1; }
};
SaidaDupla out;

enum Estado { IDLE, MANUAL, CALIBRAR };
const char* const nomeEstado[] = {"IDLE", "MANUAL", "CALIBRAR"};
Estado estado = IDLE;

int cmFL = 0, cmFR = 0, cmFC = 0;
uint8_t vez = 0; // 0 = FL, 1 = FR, 2 = FC (um de cada vez, para não se ouvirem uns aos outros)
unsigned long ultimoSonar = 0;

int motorEsq = 0, motorDir = 0;
unsigned long ultimoCmdMotor = 0;
int fan = 0;            // potência pedida à ventoinha (0-255)
int fanAtual = 0;       // potência aplicada agora no pino: sobe devagar (arranque suave), desce logo
unsigned long ultimoPassoFan = 0;

// Arranque suave: de 0 a 255 em cerca de 1 s, para o pico de corrente não fazer cair a bateria.
// Para desligar ou baixar é imediato (segurança).
void atualizarVentoinha() {
  if (fan < fanAtual) { fanAtual = fan; analogWrite(FAN_PIN, fanAtual); return; }
  if (fan > fanAtual && millis() - ultimoPassoFan >= 12) {
    ultimoPassoFan = millis();
    fanAtual = min(fan, fanAtual < 40 ? 40 : fanAtual + 3);  // começa logo em ~15%, depois sobe
    analogWrite(FAN_PIN, fanAtual);
  }
}

int flameBase = 0, flameThr = 100;

// Sirene de bombeiros ao detetar chama: 3 s de alarme, depois ventoinha (no máximo 8 s)
#define CONFIRMA_MS 300    // a chama tem de durar isto para disparar (evita falsos alarmes)
#define ALARME_MS   3000   // sirene antes de ligar a ventoinha
#define FAN_MAX_MS  8000   // ventoinha no máximo este tempo por ciclo
#define SIM_MS      20000  // duração máxima da chama simulada
enum FaseSirene { S_DESLIGADA, S_ALARME, S_APAGAR, S_ESPERA };
FaseSirene faseSirene = S_DESLIGADA;
bool sireneAtiva = true, fanAuto = false, chamaLigada = false;
#define EEPROM_SOM 0             // 1 = com som; qualquer outro valor (inclui EEPROM nova, 0xFF) = sem som
bool somLigado = false;
unsigned long chamaDesde = 0, semChamaDesde = 0, inicioFase = 0;
unsigned long simInicio = 0, simChamaAte = 0;
#define FAROL_ENDERECO 0x0EF1  // 0xF1 e o seu inverso 0x0E, como o NEC manda
#define FAROL_COMANDO  0xB7
unsigned long farolAte = 0;     // o farol conta como visto até este instante
bool farolVisto() { return millis() < farolAte; }

bool simulando() { return millis() < simChamaAte; }

// Intensidade da chama = diferença para o infravermelho do ambiente.
// Em simulação: sobe em 1,5 s, tremeluz, e desce quando a ventoinha sopra.
int intensidadeChama() {
  if (simulando()) {
    unsigned long t = millis() - simInicio;
    long base = t < 1500 ? (long)t * 250 / 1500 : 250;
    if (faseSirene == S_APAGAR) {
      unsigned long d = millis() - inicioFase;
      if (d > 2500) base = max(0L, 250 - (long)(d - 2500) * 250 / 2000);
    }
    if (base <= 0) return 0;
    return (int)constrain(base + (long)(35 * sin(t / 90.0)) + random(-15, 16), 0, 1023);
  }
  int d = analogRead(FLAME_AO) - flameBase;   // uma só leitura: o abs() do Arduino avaliava duas vezes
  return d < 0 ? -d : d;
}
float batRatio = 3.0, batRatioA = 3.0;
float batV = 0, batVA = 0;

bool mpuOK = false, giroCalibrado = false;
unsigned int errosI2C = 0;
uint8_t mpuAddr = 0x68, mpuWho = 0;
float gzBias = 0, heading = 0, pitch = 0, roll = 0, gzDps = 0;
unsigned long ultimoMpu = 0, ultimaProcuraMpu = 0;

unsigned long ultimaTelem = 0;
char linha[48], linhaBT[48];
uint8_t nLinha = 0, nLinhaBT = 0;

// ---------------- Motores ----------------
void rodaDireita(int v) {
  digitalWrite(IN2, v > 0 ? HIGH : LOW);
  digitalWrite(IN1, v < 0 ? HIGH : LOW);
  analogWrite(ENA, abs(v));
}

void rodaEsquerda(int v) {
  digitalWrite(IN4, v > 0 ? HIGH : LOW);
  digitalWrite(IN3, v < 0 ? HIGH : LOW);
  analogWrite(ENB, abs(v));
}

void motores(int esq, int dir) {
  motorEsq = constrain(esq, -255, 255);
  motorDir = constrain(dir, -255, 255);
  rodaEsquerda(motorEsq);
  rodaDireita(motorDir);
}

// ---------------- Sonares ----------------
int medir(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(4);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long d = pulseIn(echo, HIGH, 25000UL); // ~4 m
  return d / 58;                                  // 0 = sem eco
}


// ---------------- Diagnóstico do sonar central (comando DIAG FC) ----------------
// Diz onde está realmente ligado o Echo e se Trig/Echo estão trocados.
const uint8_t pinosLivres[] = {26, 27, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 49, 50, 51, 52, 53};

// dispara o pino "trig" e devolve o primeiro pino livre que sobe a HIGH em 3 ms (o Echo), ou 0
uint8_t procurarEcho(uint8_t trig) {
  for (uint8_t i = 0; i < sizeof(pinosLivres); i++) if (pinosLivres[i] != trig) pinMode(pinosLivres[i], INPUT);
  pinMode(trig, OUTPUT);
  bool antes[sizeof(pinosLivres)];
  for (uint8_t i = 0; i < sizeof(pinosLivres); i++) antes[i] = digitalRead(pinosLivres[i]);
  digitalWrite(trig, LOW); delayMicroseconds(4);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long t0 = micros();
  while (micros() - t0 < 3000)
    for (uint8_t i = 0; i < sizeof(pinosLivres); i++)
      if (pinosLivres[i] != trig && !antes[i] && digitalRead(pinosLivres[i])) return pinosLivres[i];
  return 0;
}

void diagnosticoFC() {
  out.println(F("EV:DIAG FC: inicio (robo parado)"));
  pinMode(FC_ECHO, INPUT_PULLUP); delay(5);
  bool comPull = digitalRead(FC_ECHO);
  pinMode(FC_ECHO, INPUT); delay(5);
  bool semPull = digitalRead(FC_ECHO);
  out.print(F("EV:DIAG FC: D27 em repouso, com pull-up=")); out.print(comPull);
  out.print(F(" sem pull-up=")); out.println(semPull);
  pinMode(FC_TRIG, OUTPUT);
  int cm = medir(FC_TRIG, FC_ECHO);
  out.print(F("EV:DIAG FC: normal (Trig D26, Echo D27) -> ")); out.print(cm); out.println(F(" cm"));
  pinMode(FC_ECHO, OUTPUT); pinMode(FC_TRIG, INPUT);
  int cmT = medir(FC_ECHO, FC_TRIG);
  out.print(F("EV:DIAG FC: trocados (Trig D27, Echo D26) -> ")); out.print(cmT); out.println(F(" cm"));
  for (uint8_t i = 0; i < sizeof(pinosLivres); i++) {
    uint8_t trig = pinosLivres[i];
    uint8_t eco = procurarEcho(trig);
    if (eco) { out.print(F("EV:DIAG FC: disparar D")); out.print(trig); out.print(F(" faz responder o Echo em D")); out.println(eco); }
    delay(30);
  }
  // repor
  for (uint8_t i = 0; i < sizeof(pinosLivres); i++) pinMode(pinosLivres[i], INPUT);
  pinMode(FC_TRIG, OUTPUT); digitalWrite(FC_TRIG, LOW);
  pinMode(FC_ECHO, INPUT_PULLUP);
  out.println(F("EV:DIAG FC: fim"));
}

// ---------------- MPU (6050 / 6500 / 9250, mesmo mapa de registos) ----------------
bool mpuEscrever(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuProcurar() {
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    Wire.beginTransmission(a);
    Wire.write(0x75); // WHO_AM_I
    if (Wire.endTransmission(false) != 0) { Wire.clearWireTimeoutFlag(); continue; }
    if (Wire.requestFrom(a, (uint8_t)1) != 1) continue;
    uint8_t w = Wire.read();
    if (w == 0x00 || w == 0xFF) continue;
    mpuAddr = a; mpuWho = w;
    mpuEscrever(0x6B, 0x00); delay(50); // acordar
    mpuEscrever(0x1A, 0x03);            // filtro ~44 Hz
    mpuEscrever(0x1B, 0x00);            // giroscópio ±250 graus/s (131 LSB por grau/s)
    mpuEscrever(0x1C, 0x00);            // acelerómetro ±2 g
    return true;
  }
  return false;
}

bool mpuLer(int16_t v[7]) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) return false;
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  return true; // ax, ay, az, temp, gx, gy, gz
}

void calibrarGyro() {
  if (!mpuOK) { out.println(F("EV:CAL GYRO falhou, MPU nao encontrado")); return; }
  Estado antes = estado;
  estado = CALIBRAR;
  motores(0, 0);
  out.println(F("EV:A calibrar giroscopio, nao mexer no robo..."));
  long soma = 0; int n = 0; int16_t v[7];
  for (int i = 0; i < 200; i++) {
    if (mpuLer(v)) { soma += v[6]; n++; }
    delay(5);
  }
  if (n > 0) { gzBias = soma / (float)n / 131.0; giroCalibrado = true; }
  heading = 0;
  ultimoMpu = micros();
  estado = (antes == CALIBRAR) ? IDLE : antes;
  out.print(F("EV:CAL GYRO ok, bias=")); out.print(gzBias, 3); out.println(F(" graus/s"));
}

void atualizarMpu() {
  if (!mpuOK) {
    if (millis() - ultimaProcuraMpu > 2000) {
      ultimaProcuraMpu = millis();
      mpuOK = mpuProcurar();
      if (mpuOK) {
        out.print(F("EV:MPU encontrado, WHO_AM_I=0x")); out.println(mpuWho, HEX);
        if (!giroCalibrado) calibrarGyro(); else ultimoMpu = micros();
      }
    }
    return;
  }
  int16_t v[7];
  if (!mpuLer(v)) {
    mpuOK = false;
    errosI2C++;
    Wire.clearWireTimeoutFlag();
    out.println(F("EV:MPU deixou de responder"));
    return;
  }
  unsigned long t = micros();
  float dt = (t - ultimoMpu) / 1e6;
  ultimoMpu = t;
  gzDps = v[6] / 131.0 - gzBias;
  if (fabs(gzDps) > 0.3) heading -= gzDps * dt; // positivo = rodar para a direita
  while (heading >= 360) heading -= 360;
  while (heading < 0) heading += 360;
  float ax = v[0], ay = v[1], az = v[2];
  pitch = atan2(ax, sqrt(ay * ay + az * az)) * 57.2958;
  roll  = atan2(ay, az) * 57.2958;
}

// ---------------- Ecrã LCD 16x2 I2C (PCF8574), sem biblioteca ----------------
// Ligação da placa I2C: P0=RS P1=RW P2=EN P3=luz P4..P7=D4..D7 (a mais comum)
uint8_t lcdAddr = 0;
bool lcdOK = false, lcdErro = false;
unsigned long ultimoLcd = 0, ultimaProcuraLcd = 0;

bool lcdByte(uint8_t v) {
  if (lcdErro) return false; // desiste logo à primeira falha, para não prender o programa
  Wire.beginTransmission(lcdAddr);
  Wire.write(v | 0x08); // luz de fundo ligada
  if (Wire.endTransmission() != 0) { lcdErro = true; errosI2C++; Wire.clearWireTimeoutFlag(); return false; }
  return true;
}
void lcdNibble(uint8_t v) {
  lcdByte(v | 0x04); delayMicroseconds(1);
  lcdByte(v & ~0x04); delayMicroseconds(40);
}
void lcdEnviar(uint8_t v, uint8_t rs) {
  lcdNibble((v & 0xF0) | rs);
  lcdNibble(((v << 4) & 0xF0) | rs);
}
void lcdCmd(uint8_t c) { lcdEnviar(c, 0); if (c <= 3) delay(2); }

void lcdLinha(uint8_t l, const char* txt) {
  lcdCmd(l ? 0xC0 : 0x80);
  for (uint8_t i = 0; i < 16; i++) lcdEnviar(*txt ? *txt++ : ' ', 1);
}

bool lcdProcurar() {
  const uint8_t ends[] = {0x27, 0x3F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E};
  for (uint8_t i = 0; i < sizeof(ends); i++) {
    Wire.beginTransmission(ends[i]);
    if (Wire.endTransmission() != 0) { Wire.clearWireTimeoutFlag(); continue; }
    lcdAddr = ends[i];
    lcdErro = false;
    delay(50);
    lcdByte(0); delay(20);
    lcdNibble(0x30); delay(5); lcdNibble(0x30); delay(5); lcdNibble(0x30); delayMicroseconds(150);
    lcdNibble(0x20);          // modo de 4 bits
    lcdCmd(0x28);             // 2 linhas, 5x8
    lcdCmd(0x0C);             // ecrã ligado, sem cursor
    lcdCmd(0x01);             // limpar
    lcdCmd(0x06);             // escrever da esquerda para a direita
    lcdLinha(0, "FIREBOT");
    lcdLinha(1, "a arrancar...");
    return !lcdErro;
  }
  return false;
}

void procurarI2C() {
  out.print(F("EV:I2C encontrados:"));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { out.print(F(" 0x")); out.print(a, HEX); n++; }
    else Wire.clearWireTimeoutFlag();
  }
  if (!n) out.print(F(" nenhum"));
  out.println();
}

// Páginas do ecrã: mudam a cada 3 s. Chama ou obstáculo perto passam à frente de tudo.
// Só se escreve uma linha de cada vez e só quando o texto muda (sem piscar, sem prender o programa).
#define LCD_PAGINA_MS 3000
#define LCD_PAGINAS   5
char lcdAtual[2][17] = {"", ""};
char lcdNovo[2][17];

void lcdTexto(uint8_t l, const char* fmt, ...) {
  va_list ap; va_start(ap, fmt);
  vsnprintf(lcdNovo[l], 17, fmt, ap);
  va_end(ap);
  for (uint8_t i = strlen(lcdNovo[l]); i < 16; i++) lcdNovo[l][i] = ' ';
  lcdNovo[l][16] = 0;
}

const char* estadoPT() {
  switch (estado) {
    case MANUAL:   return "MANUAL";
    case CALIBRAR: return "A CALIBRAR";
    default:       return "PARADO";
  }
}

void montarPagina() {
  char a[8], b[8];
  int fi = intensidadeChama();
  int perto = 999;
  if (cmFL > 0) perto = min(perto, cmFL);
  if (cmFR > 0) perto = min(perto, cmFR);
  if (cmFC > 0) perto = min(perto, cmFC);

  if (fi > flameThr || farolVisto()) {       // alerta: chama (ou vela vista pelo farol IR)
    bool soFarol = fi <= flameThr && !simulando();
    lcdTexto(0, simulando() ? "CHAMA SIMULADA" : (soFarol ? "VELA: FAROL IR" : "!!! CHAMA !!!"));
    if (faseSirene == S_APAGAR) lcdTexto(1, "A apagar...");
    else lcdTexto(1, "Intensidade %d", fi);
    return;
  }
  if (perto < 15 && (motorEsq || motorDir)) { // alerta: obstáculo, só com o robô a andar
    lcdTexto(0, "!! OBSTACULO !!");
    lcdTexto(1, "a %d cm", perto);
    return;
  }
  switch ((millis() / LCD_PAGINA_MS) % LCD_PAGINAS) {
    case 0:
      lcdTexto(0, "FIREBOT %s", estadoPT());
      lcdTexto(1, fan > 0 ? "Ventoinha %d%%" : "Ventoinha parada", (int)(fan / 2.55 + 0.5));
      break;
    case 1:
      lcdTexto(0, "Distancias (cm)");
      if (cmFC > 0) snprintf(a, sizeof(a), "%d", cmFC); else strcpy(a, "--");
      lcdTexto(1, "E%-4d C%-4s D%d", cmFL, a, cmFR);
      break;
    case 2:
      lcdTexto(0, "Chama: nao");
      lcdTexto(1, "Nivel %d de %d", fi, flameThr);
      break;
    case 3:
      if (mpuOK) {
        float h = heading >= 359.5 ? 0 : heading;
        lcdTexto(0, "Direcao %d graus", (int)(h + 0.5));
        lcdTexto(1, "Incl F%+d L%+d", (int)pitch, (int)roll);
      } else {
        lcdTexto(0, "Giroscopio");
        lcdTexto(1, "nao ligado");
      }
      break;
    default:
      dtostrf(batV, 3, 1, a);
      dtostrf(batVA, 3, 1, b);
      lcdTexto(0, "Bat. motor %sV", a);
      lcdTexto(1, "Bat. Ardu. %sV", b);
      break;
  }
}

void atualizarLcd() {
  if (!lcdOK) {
    if (millis() - ultimaProcuraLcd > 3000) {
      ultimaProcuraLcd = millis();
      lcdOK = lcdProcurar();
      if (lcdOK) { out.print(F("EV:Ecra LCD encontrado em 0x")); out.println(lcdAddr, HEX); lcdAtual[0][0] = lcdAtual[1][0] = 0; }
    }
    return;
  }
  if (millis() - ultimoLcd < 150) return;
  ultimoLcd = millis();
  montarPagina();
  for (uint8_t l = 0; l < 2; l++) {
    if (strcmp(lcdAtual[l], lcdNovo[l]) != 0) {
      lcdLinha(l, lcdNovo[l]);
      strcpy(lcdAtual[l], lcdNovo[l]);
      break; // uma linha por vez
    }
  }
  if (lcdErro || Wire.getWireTimeoutFlag()) { Wire.clearWireTimeoutFlag(); lcdOK = false; out.println(F("EV:Ecra LCD deixou de responder")); }
}

// ---------------- Buzzer (sem bloquear) ----------------
uint8_t bipsFalta = 0;
unsigned long proximoBip = 0;
void bip(uint8_t n) { bipsFalta = n; proximoBip = millis(); }
void atualizarBuzzer() {
  if (faseSirene == S_ALARME || faseSirene == S_APAGAR) return; // a sirene tem prioridade
  if (!somLigado) { bipsFalta = 0; return; }
  if (bipsFalta && millis() >= proximoBip) {
    tone(BUZZER, 2700, 250);
    bipsFalta--;
    proximoBip = millis() + 400;
  }
}

// ---------------- Aviso de distância por bips (robô a andar) ----------------
unsigned long proximoBipDist = 0;
void atualizarAvisoDistancia() {
  if (!somLigado || bipsFalta || faseSirene == S_ALARME || faseSirene == S_APAGAR) return;
  if (!(motorEsq || motorDir)) return;
  int perto = 999;
  if (cmFL > 0) perto = min(perto, cmFL);
  if (cmFR > 0) perto = min(perto, cmFR);
  if (cmFC > 0) perto = min(perto, cmFC);
  if (perto >= 60) return;
  unsigned long agora = millis();
  if ((long)(agora - proximoBipDist) < 0) return;
  int periodo = perto < 10 ? 70 : map(perto, 10, 60, 90, 700);   // mais perto = mais rápido
  tone(BUZZER, 2000, perto < 10 ? 60 : 40);
  proximoBipDist = agora + periodo;
}

// ---------------- Sirene de bombeiros e ventoinha automática ----------------
int ultimoTom = -1;
void tocarSirene() {                      // dois tons alternados, como os carros de bombeiros
  if (!somLigado) return;
  int tom = ((millis() / 450) % 2) ? 960 : 640;
  if (tom != ultimoTom) { tone(BUZZER, tom); ultimoTom = tom; }
}

void acabarSirene(const __FlashStringHelper* msg) {
  noTone(BUZZER); digitalWrite(BUZZER, LOW); ultimoTom = -1;
  if (fanAuto) { fan = 0; fanAuto = false; }
  if (faseSirene != S_DESLIGADA) faseSirene = S_ESPERA;
  simChamaAte = 0;
  if (msg) out.println(msg);
}

void definirSom(bool ligado) {   // SOM 0/1 pela app, ou tecla EQ do comando; fica guardado na EEPROM
  somLigado = ligado;
  EEPROM.update(EEPROM_SOM, somLigado ? 1 : 0);
  if (!somLigado) { noTone(BUZZER); digitalWrite(BUZZER, LOW); ultimoTom = -1; }
  out.println(somLigado ? F("EV:Som ligado") : F("EV:Som desligado (buzzer em silencio)"));
  if (somLigado) bip(1);
}

void iniciarSimulacao() {
  simInicio = millis(); simChamaAte = simInicio + SIM_MS;
  faseSirene = S_DESLIGADA; chamaDesde = 0; semChamaDesde = 0;
  out.println(F("EV:Chama SIMULADA (teste da sequencia, sem vela)"));
}

void atualizarSirene() {
  unsigned long agora = millis();
  bool chama = ((chamaLigada || simulando()) && intensidadeChama() > flameThr) || farolVisto();
  if (chama) semChamaDesde = 0; else if (!semChamaDesde) semChamaDesde = agora;
  bool apagada = !chama && semChamaDesde && agora - semChamaDesde > 1500;

  switch (faseSirene) {
    case S_DESLIGADA:
      if (!sireneAtiva || !chama) { chamaDesde = 0; break; }
      if (!chamaDesde) chamaDesde = agora;
      if (agora - chamaDesde >= CONFIRMA_MS) {
        faseSirene = S_ALARME; inicioFase = agora;
        if (simulando()) out.println(F("EV:Chama simulada: sirene"));
        else if (farolVisto() && intensidadeChama() <= flameThr) out.println(F("EV:Vela detetada pelo farol IR: sirene"));
        else out.println(F("EV:Chama detetada: sirene"));
      }
      break;
    case S_ALARME:
      tocarSirene();
      if (apagada) { acabarSirene(F("EV:A chama desapareceu antes de ligar a ventoinha")); break; }
      if (agora - inicioFase >= ALARME_MS) {
        if (fan == 0) { fan = 255; fanAuto = true; }
        faseSirene = S_APAGAR; inicioFase = agora;
        out.println(F("EV:Ventoinha ligada para apagar a chama"));
      }
      break;
    case S_APAGAR:
      tocarSirene();
      if (apagada) acabarSirene(F("EV:Chama apagada"));
      else if (agora - inicioFase >= FAN_MAX_MS) acabarSirene(F("EV:Ventoinha parada (tempo maximo de 8 s)"));
      break;
    case S_ESPERA: // só volta a armar depois de deixar de ver chama durante 2 s
      if (!chama && semChamaDesde && agora - semChamaDesde > 2000) { faseSirene = S_DESLIGADA; chamaDesde = 0; }
      break;
  }
}

// ---------------- LEDs de estado ----------------
bool testeLuzes = false;
unsigned long inicioTeste = 0;
void atualizarLeds() {
  if (testeLuzes) { // vermelho, amarelo, verde, 0,5 s cada
    unsigned long t = millis() - inicioTeste;
    digitalWrite(LED_R, t < 1000);
    digitalWrite(LED_Y, t >= 1000 && t < 2000);
    digitalWrite(LED_G, t >= 2000 && t < 3000);
    if (t >= 3000) testeLuzes = false;
    return;
  }
  if (faseSirene == S_ALARME || faseSirene == S_APAGAR) { // luzes de emergência
    bool p = (millis() / 250) % 2;
    digitalWrite(LED_R, p); digitalWrite(LED_Y, !p); digitalWrite(LED_G, LOW);
    return;
  }
  bool pisca = (millis() / 150) % 2;
  int fi = intensidadeChama();
  int perto = 999;
  if (cmFL > 0) perto = min(perto, cmFL);
  if (cmFR > 0) perto = min(perto, cmFR);
  if (cmFC > 0) perto = min(perto, cmFC);
  bool batBaixa = (batV > 1 && batV < 6.6) || (batVA > 1 && batVA < 6.6);
  bool batMeio  = (batV > 1 && batV < 7.5) || (batVA > 1 && batVA < 7.5);
  bool vista = fi > flameThr || farolVisto();
  bool andar = motorEsq || motorDir;
  bool batFraca = (batV > 1 && batV < 6.6) || (batVA > 1 && batVA < 6.6);
  if (andar && perto < 999 && !vista && !batFraca) {   // aviso de distância, como nos carros
    bool r = perto < 25, y = perto >= 25 && perto <= 50, g = perto > 50;
    if (perto < 12) r = (millis() / 100) % 2;
    digitalWrite(LED_R, r); digitalWrite(LED_Y, y); digitalWrite(LED_G, g);
    return;
  }
  bool alerta = vista || (perto < 15 && (motorEsq || motorDir)) || batBaixa;
  digitalWrite(LED_R, alerta ? (vista ? pisca : HIGH) : LOW);
  digitalWrite(LED_Y, !alerta && (estado != IDLE || batMeio));
  digitalWrite(LED_G, !alerta && estado == IDLE && !batMeio);
}

// ---------------- Comando IR (NEC), sem biblioteca ----------------
// O recetor põe a saída a LOW enquanto recebe; mede-se o tempo entre descidas:
// 13,5 ms início, 11,25 ms repetição (botão mantido), 1,125 ms bit 0, 2,25 ms bit 1.
volatile uint32_t irBits = 0;
volatile uint8_t irN = 0, irCmd = 0;
volatile uint16_t irEnd = 0;
volatile unsigned long irUlt = 0;
volatile bool irInicio = false, irNovo = false, irRepete = false;
uint8_t ultimoIR = 0;
unsigned long ultimoIRms = 0;
int velIR = 160;

void irIsr() {
  unsigned long agora = micros(), dt = agora - irUlt;
  irUlt = agora;
  if (dt > 12500 && dt < 14500) { irInicio = true; irN = 0; irBits = 0; return; }
  if (dt > 10500 && dt < 12500) { irRepete = true; irInicio = false; return; }
  if (!irInicio) return;
  if (dt > 800 && dt < 1500) irBits >>= 1;
  else if (dt > 1800 && dt < 2700) irBits = (irBits >> 1) | 0x80000000UL;
  else { irInicio = false; return; }
  if (++irN == 32) {
    irInicio = false;
    uint8_t cmd = (irBits >> 16) & 0xFF, inv = (irBits >> 24) & 0xFF;
    if ((uint8_t)~cmd == inv) { irCmd = cmd; irEnd = irBits & 0xFFFF; irNovo = true; }
  }
}

void acaoIR(uint8_t c, bool primeiro) {
  int v = velIR, r = velIR * 7 / 10;
  switch (c) {
    case 0x45: case 0x47: case 0x1C:                 // POWER, FUNC/STOP, 5: parar tudo
      motores(0, 0); fan = 0; estado = IDLE;
      if (faseSirene == S_ALARME || faseSirene == S_APAGAR || simulando()) acabarSirene(NULL);
      if (primeiro) { out.println(F("EV:Comando: parar tudo")); bip(2); }
      return;
    case 0x19:                                       // EQ: liga/desliga o som
      if (primeiro) definirSom(!somLigado);
      return;
    case 0x16:                                       // 0: chama simulada (plano B)
      if (primeiro) iniciarSimulacao();
      return;
    case 0x40:                                       // >|| : ventoinha liga/desliga
      if (primeiro) { fan = fan ? 0 : 255; out.print(F("EV:Comando: ventoinha ")); out.println(fan ? F("ligada") : F("desligada")); bip(1); }
      return;
    case 0x46: if (primeiro) { velIR = min(255, velIR + 20); out.print(F("EV:Comando: velocidade ")); out.println(velIR); bip(1); } return; // VOL+
    case 0x15: if (primeiro) { velIR = max(60, velIR - 20);  out.print(F("EV:Comando: velocidade ")); out.println(velIR); bip(1); } return; // VOL-
    case 0x09: case 0x18: motores(v, v);   break;    // ▲ ou 2: frente
    case 0x07: case 0x52: motores(-v, -v); break;    // ▼ ou 8: trás
    case 0x44: case 0x08: motores(-r, r);  break;    // |<< ou 4: rodar à esquerda
    case 0x43: case 0x5A: motores(r, -r);  break;    // >>| ou 6: rodar à direita
    default: return;
  }
  ultimoCmdMotor = millis();
  estado = MANUAL;
}

void tratarIR() {
  bool novo, repete; uint8_t c; uint16_t end;
  noInterrupts(); novo = irNovo; repete = irRepete; c = irCmd; end = irEnd; irNovo = false; irRepete = false; interrupts();
  if (novo && end == FAROL_ENDERECO && c == FAROL_COMANDO) {   // farol da vela: não é uma tecla do comando
    if (!farolVisto()) out.println(F("EV:Farol IR da vela detetado"));
    farolAte = millis() + 3500;
    return;
  }
  if (novo) {
    ultimoIR = c; ultimoIRms = millis();
    out.print(F("EV:Comando IR botao 0x")); out.println(c, HEX);
    acaoIR(c, true);
  } else if (repete && ultimoIR && millis() - ultimoIRms < 300) {
    ultimoIRms = millis();
    acaoIR(ultimoIR, false); // botão mantido: continua a andar
  }
}

// ---------------- HC-05: configuração por comandos AT ----------------
// Só funciona com o HC-05 em modo AT (botão carregado enquanto se liga o VCC; LED a piscar devagar).
// Aqui escreve-se só na USB: a saída dupla mandaria lixo ao HC-05.
// Pergunta "AT" a uma velocidade; devolve true se o HC-05 responder OK
bool btResponde(long baud) {
  Serial1.end();
  Serial1.begin(baud);
  delay(200);
  for (uint8_t t = 0; t < 2; t++) {
    while (Serial1.available()) Serial1.read();
    Serial1.print("AT\r\n");
    unsigned long t0 = millis();
    char r[12]; uint8_t n = 0;
    while (millis() - t0 < 500) while (Serial1.available() && n < 11) r[n++] = Serial1.read();
    r[n] = 0;
    if (strstr(r, "OK")) return true;
  }
  return false;
}

void configurarBT() {
  const long vel[] = {38400, 9600, 115200};
  long encontrada = 0;
  for (uint8_t i = 0; i < 3 && !encontrada; i++) {
    Serial.print(F("EV:HC-05: a tentar AT a ")); Serial.println(vel[i]);
    if (btResponde(vel[i])) encontrada = vel[i];
  }
  if (!encontrada) {
    Serial1.end(); Serial1.begin(BT_BAUD); pinMode(19, INPUT_PULLUP);
    Serial.println(F("EV:HC-05 nao respondeu a nenhuma velocidade. Tem energia (LED aceso)? TXD no D19 e RXD no D18 (com divisor)? Esta em modo AT?"));
    return;
  }
  Serial.print(F("EV:HC-05 respondeu a ")); Serial.println(encontrada);
  const char* cmds[] = {"AT", "AT+VERSION?", "AT+NAME=FIREBOT", "AT+UART=115200,0,0", "AT+UART?"};
  bool algum = false;
  for (uint8_t i = 0; i < 5; i++) {
    while (Serial1.available()) Serial1.read();
    Serial1.print(cmds[i]); Serial1.print("\r\n");
    unsigned long t0 = millis();
    char resp[40]; uint8_t n = 0;
    while (millis() - t0 < 700) {
      while (Serial1.available() && n < sizeof(resp) - 1) { char c = Serial1.read(); if (c >= 32) resp[n++] = c; else if (n && resp[n - 1] != ' ') resp[n++] = ' '; }
    }
    resp[n] = 0;
    if (n) algum = true;
    Serial.print(F("EV:HC-05 ")); Serial.print(cmds[i]); Serial.print(F(" -> ")); Serial.println(n ? resp : "(sem resposta)");
  }
  Serial1.end();
  Serial1.begin(BT_BAUD);
  pinMode(19, INPUT_PULLUP);
  if (algum) Serial.println(F("EV:HC-05 configurado. Desliga e volta a ligar o VCC do HC-05 (sem o botao) para sair do modo AT."));
  else Serial.println(F("EV:HC-05 nao respondeu. Esta em modo AT (LED a piscar devagar, ~2 s)? TXD no D19 e RXD no D18?"));
}

// ---------------- Chama (KY-026) ----------------
void calibrarIR() {
  long soma = 0;
  for (int i = 0; i < 40; i++) { soma += analogRead(FLAME_AO); delay(10); }
  flameBase = soma / 40;
  out.print(F("EV:CAL IR ok, base=")); out.println(flameBase);
}

// ---------------- Comandos ----------------
void processarComando(char* s, bool doBT) {
  // as nossas próprias linhas (telemetria/eventos) nunca são comandos: evita ecos
  if (strchr(s, ':') != NULL) return;
  for (char* p = s; *p; p++) *p = toupper(*p);
  int a, b;
  if (sscanf(s, "M %d %d", &a, &b) == 2) {
    motores(a, b);
    ultimoCmdMotor = millis();
    estado = (motorEsq || motorDir) ? MANUAL : IDLE;
    return;
  }
  if (!strcmp(s, "SIMCHAMA")) { iniciarSimulacao(); return; }
  if (!strcmp(s, "DIAG FC")) { diagnosticoFC(); return; }
  if (sscanf(s, "SOM %d", &a) == 1) { definirSom(a != 0); return; }
  if (sscanf(s, "SIRENE %d", &a) == 1) {
    sireneAtiva = a != 0;
    if (!sireneAtiva) { acabarSirene(NULL); faseSirene = S_DESLIGADA; }
    out.println(sireneAtiva ? F("EV:Sirene automatica ligada") : F("EV:Sirene automatica desligada"));
    return;
  }
  if (!strcmp(s, "STOP")) {
    if (faseSirene == S_ALARME || faseSirene == S_APAGAR || simulando()) acabarSirene(NULL);
    motores(0, 0); fan = 0; estado = IDLE;
    out.println(F("EV:STOP, motores e ventoinha parados"));
    return;
  }
  if (sscanf(s, "FAN %d", &a) == 1) {
    fan = constrain(a, 0, 255);
    out.print(F("EV:Ventoinha ")); out.println(fan);
    return;
  }
  if (!strcmp(s, "CAL GYRO")) { calibrarGyro(); return; }
  if (!strcmp(s, "CAL IR"))   { calibrarIR(); return; }
  if (sscanf(s, "FTHR %d", &a) == 1) {
    flameThr = constrain(a, 1, 1023);
    out.print(F("EV:Limiar chama ")); out.println(flameThr);
    return;
  }
  if (!strncmp(s, "BATR ", 5)) {
    float r = atof(s + 5);
    if (r > 0.5 && r < 20) { batRatio = r; batV = 0; }
    out.print(F("EV:Fator bateria ")); out.println(batRatio, 2);
    return;
  }
  if (!strncmp(s, "BATRA ", 6)) {
    float r = atof(s + 6);
    if (r > 0.5 && r < 20) { batRatioA = r; batVA = 0; }
    out.print(F("EV:Fator bateria A ")); out.println(batRatioA, 2);
    return;
  }
  if (!strcmp(s, "HDG0")) { heading = 0; out.println(F("EV:Direcao a 0")); return; }
  if (!strcmp(s, "PING")) { out.println(F("PONG")); return; }
  if (!strcmp(s, "TESTE")) { testeLuzes = true; inicioTeste = millis(); bip(3); out.println(F("EV:Teste de LEDs e buzzer")); return; }
  if (!strcmp(s, "BTAT")) { configurarBT(); return; }
  if (!strcmp(s, "BUZTESTE")) {
    out.println(F("EV:Buzzer: 1) som a 2700 Hz durante 1 s"));
    tone(BUZZER, 2700, 1000); delay(1500);
    out.println(F("EV:Buzzer: 2) tensao continua durante 1 s (buzzer ativo)"));
    noTone(BUZZER); digitalWrite(BUZZER, HIGH); delay(1000); digitalWrite(BUZZER, LOW);
    return;
  }
  if (!strncmp(s, "LED ", 4)) { // LED R / LED Y / LED G: acende só esse durante 3 s
    testeLuzes = false;
    uint8_t pino = s[4] == 'R' ? LED_R : (s[4] == 'Y' ? LED_Y : LED_G);
    digitalWrite(LED_R, LOW); digitalWrite(LED_Y, LOW); digitalWrite(LED_G, LOW);
    digitalWrite(pino, HIGH); delay(3000); digitalWrite(pino, LOW);
    return;
  }
  if (doBT) return; // ruído no Bluetooth: não responder
  out.print(F("EV:Comando desconhecido: ")); out.println(s);
}

void lerCanal(Stream& st, char* buf, uint8_t& n, bool doBT) {
  while (st.available()) {
    char c = st.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; processarComando(buf, doBT); n = 0; }
    } else if (c >= 32 && c <= 126 && n < 47) { // ignora ruído da ligação
      buf[n++] = c;
    }
  }
}

void lerSerial() {
  lerCanal(Serial, linha, nLinha, false);
  lerCanal(Serial1, linhaBT, nLinhaBT, true);
}

// ---------------- Telemetria ----------------
// Um pino analógico "no ar" (nada ligado) guarda a carga que lhe damos:
//  1) descarrega-se por instantes a 0 V -> no ar fica perto de 0, ligado volta logo ao valor do módulo
//  2) puxa-se para 5 V com o pull-up   -> no ar sobe a ~1023, ligado fica perto do seu valor
// Só está no ar se as duas coisas acontecerem. Descarregar dura 10 us e passa por resistências
// (divisor de 20k/10k ou saída do KY-026), por isso não faz mal a nada.
bool pinoLigado(uint8_t pino) {
  pinMode(pino, OUTPUT); digitalWrite(pino, LOW);
  delayMicroseconds(10);
  pinMode(pino, INPUT);
  int baixo = analogRead(pino);
  pinMode(pino, INPUT_PULLUP);
  delayMicroseconds(100);
  analogRead(pino);
  int alto = analogRead(pino);
  pinMode(pino, INPUT);
  delayMicroseconds(100);
  analogRead(pino);
  return !(baixo < 150 && alto > 1000);
}

unsigned long ultimaTelemBT = 0;

void enviarTelemetria() {
  out.bt = (millis() - ultimaTelemBT >= BT_TELEM_MS);
  if (out.bt) ultimaTelemBT = millis();
  bool batOk = pinoLigado(BAT_PIN);
  bool batAOk = pinoLigado(BATA_PIN);
  bool chamaOk = pinoLigado(FLAME_AO);
  chamaLigada = chamaOk;
  int fa = analogRead(FLAME_AO);
  float v = analogRead(BAT_PIN) * 5.0 / 1023.0 * batRatio;
  batV = (batV == 0) ? v : batV * 0.8 + v * 0.2;
  float va = analogRead(BATA_PIN) * 5.0 / 1023.0 * batRatioA;
  batVA = (batVA == 0) ? va : batVA * 0.8 + va * 0.2;

  out.print(F("FL:"));    out.print(cmFL);
  out.print(F(" FR:"));   out.print(cmFR);
  out.print(F(" FC:"));   out.print(cmFC);
  out.print(F(" FCOK:")); out.print(digitalRead(FC_ECHO) == LOW ? 1 : 0); // ECHO em repouso fica LOW se o sonar estiver ligado
  out.print(F(" FA:"));   out.print(fa);
  out.print(F(" FD:"));   out.print(digitalRead(FLAME_DO));
  out.print(F(" FI:"));   out.print(intensidadeChama());
  out.print(F(" FB:"));   out.print(flameBase);
  out.print(F(" FT:"));   out.print(flameThr);
  out.print(F(" BAT:"));  out.print(batV, 2);
  out.print(F(" BR:"));   out.print(batRatio, 2);
  out.print(F(" BOK:"));  out.print(batOk ? 1 : 0);
  out.print(F(" FOK:"));  out.print((chamaOk || simulando()) ? 1 : 0);
  out.print(F(" SIM:"));  out.print(simulando() ? 1 : 0);
  out.print(F(" FAR:"));  out.print(farolVisto() ? 1 : 0);
  out.print(F(" SOM:"));  out.print(somLigado ? 1 : 0);
  out.print(F(" SIR:"));  out.print((int)faseSirene);
  out.print(F(" BATA:")); out.print(batVA, 2);
  out.print(F(" BRA:"));  out.print(batRatioA, 2);
  out.print(F(" BAOK:")); out.print(batAOk ? 1 : 0);
  out.print(F(" ST:"));   out.print(nomeEstado[estado]);
  out.print(F(" MPU:"));  out.print(mpuOK ? 1 : 0);
  out.print(F(" WHO:"));  out.print(mpuWho, HEX);
  out.print(F(" HDG:"));  out.print(heading >= 359.95 ? 0.0 : heading, 1);
  out.print(F(" PIT:"));  out.print(pitch, 1);
  out.print(F(" ROL:"));  out.print(roll, 1);
  out.print(F(" GZ:"));   out.print(gzDps, 1);
  out.print(F(" GB:"));   out.print(gzBias, 2);
  out.print(F(" ML:"));   out.print(motorEsq);
  out.print(F(" MR:"));   out.print(motorDir);
  out.print(F(" FAN:"));  out.print(fan);
  out.print(F(" LCD:"));  out.print(lcdOK ? 1 : 0);
  out.print(F(" IR:"));   out.print(ultimoIR, HEX);
  out.print(F(" I2CE:")); out.println(errosI2C);
  out.bt = true; // eventos saem sempre pelos dois
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BT_BAUD);
  pinMode(19, INPUT_PULLUP); // RX1: sem HC-05 ligado não fica no ar
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motores(0, 0);
  pinMode(FAN_PIN, OUTPUT); analogWrite(FAN_PIN, 0);
  pinMode(FL_TRIG, OUTPUT); pinMode(FR_TRIG, OUTPUT); pinMode(FC_TRIG, OUTPUT);
  pinMode(FC_ECHO, INPUT_PULLUP); // sem sonar ligado fica HIGH: lê 0 e FCOK:0
  pinMode(FL_ECHO, INPUT);  pinMode(FR_ECHO, INPUT);
  pinMode(FLAME_DO, INPUT);
  pinMode(LED_R, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_G, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), irIsr, FALLING);

  Wire.begin();
  Wire.setClock(50000);            // I2C mais lento: mais tolerante a fios compridos
  Wire.setWireTimeout(10000, true); // não bloqueia se o MPU não estiver ligado

  somLigado = EEPROM.read(EEPROM_SOM) == 1;
  out.println(F("EV:FIREBOT telemetria v1 pronto"));
  out.println(somLigado ? F("EV:Som ligado") : F("EV:Som desligado (SOM 1 para ligar)"));
  testeLuzes = true; inicioTeste = millis(); bip(1);
  procurarI2C();
  lcdOK = lcdProcurar();
  ultimaProcuraLcd = millis();
  if (lcdOK) { out.print(F("EV:Ecra LCD encontrado em 0x")); out.println(lcdAddr, HEX); }
  calibrarIR();
  mpuOK = mpuProcurar();
  ultimaProcuraMpu = millis();
  if (mpuOK) {
    out.print(F("EV:MPU encontrado, WHO_AM_I=0x")); out.println(mpuWho, HEX);
    calibrarGyro();
  } else {
    out.println(F("EV:MPU nao encontrado (tenta outra vez a cada 2 s)"));
  }
}

void loop() {
  lerSerial();

  if ((motorEsq || motorDir) && millis() - ultimoCmdMotor > DEADMAN_MS) {
    motores(0, 0);
    if (estado == MANUAL) estado = IDLE;
  }

  atualizarMpu();
  atualizarLcd();
  tratarIR();
  atualizarSirene();
  atualizarVentoinha();
  atualizarBuzzer();
  atualizarAvisoDistancia();
  atualizarLeds();

  if (millis() - ultimoSonar >= SONAR_MS) {
    ultimoSonar = millis();
    if (vez == 0)      cmFL = medir(FL_TRIG, FL_ECHO);
    else if (vez == 1) cmFR = medir(FR_TRIG, FR_ECHO);
    else               cmFC = medir(FC_TRIG, FC_ECHO);
    vez = (vez + 1) % 3;
  }

  if (millis() - ultimaTelem >= TELEM_MS) {
    ultimaTelem = millis();
    enviarTelemetria();
  }
}
