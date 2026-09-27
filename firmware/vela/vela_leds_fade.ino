// Efeito de chama para 5 LEDs normais (sem chip de flicker embutido)
// + emissor de infravermelhos para o sensor de chama do FIREBOT
//
// LEDs: brilho sempre alto, cada LED faz um fade suave ao seu ritmo,
//       e de vez em quando há uma "rajada de vento" que baixa todos um pouco ao mesmo tempo
// IR:   sempre ligado a 100%, sem PWM, para o sensor ver um sinal forte e estável
// Farol: de 2 em 2 s o mesmo LED IR pisca um código NEC a 38 kHz (endereço 0xF1, comando 0xB7).
//        O FIREBOT apanha-o pelo recetor do comando e mostra "VELA: FAROL IR" (plano C, se o sensor de chama falhar).
//        Cada envio dura ~70 ms; durante esse tempo os LEDs ficam parados, o que não se nota.

const int NUM_LEDS = 5;
const int ledPins[NUM_LEDS] = {3, 5, 6, 9, 10};

// Emissor IR: se estiver ligado a um pino do Arduino, põe aqui o número desse pino.
// Se estiver ligado direto aos 5V, fica sempre aceso de qualquer forma.
const int PINO_IR = 2;
const bool FAROL_LIGADO = true;           // false: o LED IR fica só sempre aceso, como antes
const unsigned long FAROL_MS = 1000;      // intervalo entre envios
const bool IR_SEMPRE_ACESO = false;       // true: LED IR aceso entre envios (para um sensor de chama); pode encandear o recetor do robô
const uint8_t FAROL_END = 0xF1, FAROL_CMD = 0xB7;
unsigned long proximoFarol = 1000;

// brilho percebido (0-255), antes da correção gama
const float BRILHO_MIN = 190;
const float BRILHO_MAX = 255;

const unsigned long PASSO_MS = 10; // atualiza a 100 Hz, para o movimento ser contínuo
const float SUAVIDADE = 0.04;      // fração do caminho até ao alvo feita em cada passo (menor = mais suave)

float brilhoAtual[NUM_LEDS];
float brilhoAlvo[NUM_LEDS];
unsigned long proximoAlvo[NUM_LEDS];

float rajada = 1.0;     // fator comum a todos os LEDs (1 = sem rajada)
float rajadaAlvo = 1.0;
unsigned long fimRajada = 0;
unsigned long ultimoPasso = 0;

// ---------- farol: NEC a 38 kHz feito "à mão" no pino do LED IR ----------
volatile uint8_t *regIR; uint8_t mascaraIR;

void marca(unsigned int us) {             // 38 kHz: ~13 us aceso, ~13 us apagado
  for (unsigned int n = us / 26; n > 0; n--) {
    *regIR |= mascaraIR;  delayMicroseconds(12);
    *regIR &= ~mascaraIR; delayMicroseconds(12);
  }
}
void espaco(unsigned int us) { *regIR &= ~mascaraIR; delayMicroseconds(us); }

void enviarByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++, b >>= 1) { marca(562); espaco((b & 1) ? 1687 : 562); }
}

void enviarFarol() {
  *regIR &= ~mascaraIR; delay(20);        // apagado um instante antes do código
  marca(9000); espaco(4500);              // início
  enviarByte(FAROL_END); enviarByte((uint8_t)~FAROL_END);
  enviarByte(FAROL_CMD); enviarByte((uint8_t)~FAROL_CMD);
  marca(562);                             // fim
  if (IR_SEMPRE_ACESO) *regIR |= mascaraIR; else *regIR &= ~mascaraIR;
  Serial.println(F("farol enviado"));
}

// Correção gama aproximada: o olho não vê o PWM de forma linear,
// sem isto o fade parece "saltar" nos brilhos baixos e ficar parado nos altos
int gama(float v) {
  float n = v / 255.0;
  return (int)(n * n * 255.0 + 0.5);
}

void setup() {
  pinMode(PINO_IR, OUTPUT);
  digitalWrite(PINO_IR, IR_SEMPRE_ACESO ? HIGH : LOW);
  Serial.begin(9600);
  Serial.println(F("vela pronta"));
  regIR = portOutputRegister(digitalPinToPort(PINO_IR));
  mascaraIR = digitalPinToBitMask(PINO_IR);

  randomSeed(analogRead(A0)); // pino analógico livre, só para variar a semente a cada arranque

  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);
    brilhoAtual[i] = BRILHO_MAX;
    brilhoAlvo[i] = BRILHO_MAX;
    proximoAlvo[i] = random(100, 400);
  }
}

void loop() {
  unsigned long agora = millis();
  if (FAROL_LIGADO && (long)(agora - proximoFarol) >= 0) {
    enviarFarol();
    proximoFarol = millis() + FAROL_MS + random(0, 300); // um pouco irregular, para não coincidir sempre com o comando
  }
  if (agora - ultimoPasso < PASSO_MS) return;
  ultimoPasso = agora;

  // rajada de vento: cerca de uma vez a cada 6 s, todos baixam um pouco durante um instante
  if (rajadaAlvo < 1.0) {
    if ((long)(agora - fimRajada) >= 0) rajadaAlvo = 1.0;
  } else if (random(0, 600) == 0) {
    rajadaAlvo = random(80, 93) / 100.0;
    fimRajada = agora + random(150, 500);
  }
  rajada += (rajadaAlvo - rajada) * 0.08;

  for (int i = 0; i < NUM_LEDS; i++) {
    // cada LED escolhe um novo alvo em intervalos irregulares, sem sincronismo com os outros
    if ((long)(agora - proximoAlvo[i]) >= 0) {
      brilhoAlvo[i] = random((int)BRILHO_MIN, (int)BRILHO_MAX + 1);
      proximoAlvo[i] = agora + random(120, 600);
    }

    // aproxima-se do alvo de forma exponencial: começa mais rápido e abranda no fim, sem degraus
    brilhoAtual[i] += (brilhoAlvo[i] - brilhoAtual[i]) * SUAVIDADE;

    analogWrite(ledPins[i], gama(brilhoAtual[i] * rajada));
  }
}
