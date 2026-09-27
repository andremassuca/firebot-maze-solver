# LIGAÇÕES — Esquema Completo de Pinos

## Tabela rápida de pinos (v2.0 — blueprint.am + correcções)

| Pino Mega | Componente | Terminal | Notas |
|---|---|---|---|
| D2 | IR Receiver | Signal | INT4 — hardware interrupt obrigatório |
| D3 | L298N | ENA | PWM velocidade motor esquerdo |
| D4 | L298N | IN1 | Direção motor esq A |
| D5 | L298N | IN2 | Direção motor esq B |
| D6 | L298N | IN3 | Direção motor dir A |
| D7 | L298N | IN4 | Direção motor dir B |
| D8 | L298N | ENB | PWM velocidade motor direito |
| D9 | Servo Elecbee | Signal | PWM — servo sensor chama |
| D10 | Servo HC-SR04 | Signal | PWM — servo sensor ultrassónico |
| D20 | MPU-6050 + LCD | SDA | I2C partilhado (endereços diferentes) |
| D21 | MPU-6050 + LCD | SCL | I2C partilhado |
| D22 | HC-SR04 fixo esq | TRIG | Fixo 30° esquerda |
| D23 | HC-SR04 fixo esq | ECHO | |
| D24 | HC-SR04 fixo dir | TRIG | Fixo 30° direita |
| D25 | HC-SR04 fixo dir | ECHO | |
| D26 | HC-SR04 servo | TRIG | Em servo, varrimento |
| D27 | HC-SR04 servo | ECHO | |
| D28 | KY-026 | DO | Activo LOW — backup chama |
| D29 | MH IR borda esq | DO | Fundo chassis canto frente esq |
| D30 | MH IR borda dir | DO | Fundo chassis canto frente dir |
| D44 | MOSFET D4184 | J1 IN+ | PWM fan — timer 5 |
| D45 | Buzzer passivo | + | PWM — PASSIVO obrigatório |
| D46 | LED vermelho | + (c/330Ω) | IDLE |
| D47 | LED amarelo | + (c/330Ω) | Em missão |
| D48 | LED verde | + (c/330Ω) | Extinguir/Retornar |
| D49 | Botão start | Terminal A | Pull-up interno, B→GND |
| A0 | Elecbee ch1 | AO (analógico) | Canal extremo esquerdo |
| A1 | Elecbee ch2 | AO (analógico) | Canal esq-centro |
| A2 | Elecbee ch3 | AO (analógico) | Canal centro |
| A3 | Elecbee ch4 | AO (analógico) | Canal dir-centro |
| A4 | Elecbee ch5 | AO (analógico) | Canal extremo direito |
| A6 | Line sensor 1 | AO (analógico) | Extremo esquerdo, peso -200 |
| A7 | Line sensor 2 | AO (analógico) | Esq-centro, peso -100 |
| A8 | Line sensor 3 | AO (analógico) | Centro, peso 0 |
| A9 | Line sensor 4 | AO (analógico) | Dir-centro, peso +100 |
| A10 | Line sensor 5 | AO (analógico) | Extremo direito, peso +200 |
| Vin | Fonte A | Switch A (+) | 9V alkaline |
| GND | GND comum | — | Estrela — todos os GND aqui |
| 5V | Sensores / LCD / IR | VCC | Máx 500mA total |

---

## Ligações detalhadas por componente

### L298N Motor Driver
```
ENA  → Mega D3  (PWM)
IN1  → Mega D4
IN2  → Mega D5
IN3  → Mega D6
IN4  → Mega D7
ENB  → Mega D8  (PWM)
VIN  → LM2596 OUT+ (6V)
GND  → GND comum
OUT1 + OUT2 → Motor TT esquerdo  [+ 100nF entre terminais]
OUT3 + OUT4 → Motor TT direito   [+ 100nF entre terminais]
```

### Servo SG90 — Elecbee (chama)
```
Signal → Mega D9  (PWM)
VCC    → LM2596 OUT+ (6V)  ← NÃO usar pino 5V do Mega
GND    → GND comum
Posição: frente do robô, piso 2. 90°=frente, varre 0-180°.
```

### Servo SG90 — HC-SR04 (obstáculos)
```
Signal → Mega D10 (PWM)
VCC    → LM2596 OUT+ (6V)
GND    → GND comum
Posição: frente do robô, piso 2, centro. 90°=frente.
```

### HC-SR04 — servo (varrimento, centro)
```
TRIG → Mega D26
ECHO → Mega D27
VCC  → Mega 5V
GND  → GND comum
```

### HC-SR04 — fixo esquerdo (30° para esq)
```
TRIG → Mega D22
ECHO → Mega D23
VCC  → Mega 5V
GND  → GND comum
```

### HC-SR04 — fixo direito (30° para dir)
```
TRIG → Mega D24
ECHO → Mega D25
VCC  → Mega 5V
GND  → GND comum
```

### Elecbee 5-ch Flame Sensor (em servo, ANALÓGICO)
```
AO canal 1 (extremo esq) → Mega A0
AO canal 2 (esq-centro)  → Mega A1
AO canal 3 (centro)      → Mega A2
AO canal 4 (dir-centro)  → Mega A3
AO canal 5 (extremo dir) → Mega A4
VCC → Mega 5V
GND → GND comum
NOTA: Usar saídas ANALÓGICAS (AO), não digitais (DO)
      Valor baixo = chama forte. Calibrar FLAME_ANALOG_THRESHOLD.
```

### KY-026 Sensor Chama Backup
```
DO  → Mega D28
VCC → Mega 5V
GND → GND comum
Activo LOW: LOW = chama detectada
```

### Sensores de Linha TCRT5000 (×5, ANALÓGICO)
```
Sensor 1 (extremo esq) → Mega A6
Sensor 2 (esq-centro)  → Mega A7
Sensor 3 (centro)      → Mega A8
Sensor 4 (dir-centro)  → Mega A9
Sensor 5 (extremo dir) → Mega A10
VCC → Mega 5V (todos)
GND → GND comum (todos)
Montagem: fundo piso 1, 3-5mm acima do chão, frente do robô.
Espaçamento entre sensores: 8-12mm.
Linha preta em fundo branco: sobre linha = valor analógico alto.
```

### MPU-6050 GY-521
```
VCC → Mega 5V  (módulo GY-521 tem LDO interno)
GND → GND comum
SCL → Mega D21
SDA → Mega D20
INT → Mega D2  (opcional — não usado no código base)
AD0 → GND  (define endereço I2C 0x68)
```

### LCD1602 com I2C Adapter PCF8574
```
VCC → Mega 5V
GND → GND comum
SCL → Mega D21  (partilha com MPU-6050, sem conflito)
SDA → Mega D20  (partilha com MPU-6050, sem conflito)
Endereço: 0x27 (verificar com I2C scanner sketch)
Potenciómetro azul no adapter: ajustar contraste
```

### MOSFET D4184 HW-517 — Arctic P12
```
J1 IN+  → Mega D44 (PWM)
J1 GND  → GND comum
Arctic P12 vermelho (+) → 12V direto (Switch B)
Arctic P12 preto (−)   → MOSFET J5 output terminal
MOSFET J5 GND          → GND comum
Arctic P12 azul (PWM)  → 12V (fan sempre a 100% quando ligada)
Arctic P12 amarelo (tach) → não ligar
NOTA: MOSFET N-channel comuta o negativo da fan (low-side switch)
```

### MH Flying Fish IR — Boundary Sensors
```
Esquerdo DO → Mega D29
Direito  DO → Mega D30
VCC → Mega 5V (ambos)
GND → GND (ambos)
Montagem: fundo do chassis, cantos frontais. Apontados para o chão.
Testar na arena antes: ajustar HIGH/LOW em verificarBorda() no código.
```

### Buzzer Passivo
```
(+) → Mega D45
(−) → GND
SEM resistência em série.
PASSIVO obrigatório (tem buraco no topo). O buzzer activo não permite
sweep de frequência com tone() — a sirene não funcionaria.
```

### LEDs Status (×3)
```
LED vermelho: Mega D46 → 330Ω → LED(+) → LED(−) → GND
LED amarelo:  Mega D47 → 330Ω → LED(+) → LED(−) → GND
LED verde:    Mega D48 → 330Ω → LED(+) → LED(−) → GND
Significado: Vermelho=IDLE, Amarelo=em missão, Verde=extinguir/retornar
```

### Botão Start
```
Terminal A → Mega D49
Terminal B → GND
Pull-up interno activo no código (INPUT_PULLUP).
Activo LOW: pressionar = LOW em D49.
```

### IR Receiver
```
Signal → Mega D2  (INT4 — hardware interrupt, obrigatório)
VCC    → Mega 5V
GND    → GND
Remote ELEGOO: botão 1=start, #=stop, OK=modo quadrado
Confirmar códigos hex com sketch IRrecvDemo e atualizar no código.
```

---

## Alimentação — Ligações físicas

### Fonte A (9V — lógica e movimento)
```
6×AA(+) → Switch A positivo
Switch A → Split em dois:
  │
  ├─► Mega Vin  (9V directamente)
  │
  └─► LM2596 IN+
        │
        └─► LM2596 OUT+ (ajustado a 6.0V) → Split:
              ├─► L298N VIN
              ├─► Servo Elecbee VCC
              └─► Servo HC-SR04 VCC

6×AA(−) → GND comum (ponto estrela)
```

### Fonte B (12V — ventoinha)
```
4×AA_holder_1(+) → 4×AA_holder_2(−) [em série]
4×AA_holder_2(+) → Switch B positivo
Switch B → Split:
  ├─► Arctic P12 fio vermelho (+)
  └─► MOSFET D4184 linha de alimentação

4×AA_holder_1(−) → GND comum (ponto estrela)
```

### GND Comum (ponto estrela — OBRIGATÓRIO)
```
Ligar num único ponto físico:
  - Mega GND
  - L298N GND
  - LM2596 OUT−
  - MOSFET J1 GND
  - 6×AA (−)
  - 4×AA_holder_1 (−)
  - Todos os sensores GND (via Mega GND)

Não criar loops de terra — um único ponto de referência.
```

---

## Condensadores 100nF (EMI Motors)

```
Motor TT esquerdo: soldar 100nF cerâmico directamente
  entre terminal M+ e terminal M− (nos bornes L298N OUT1/OUT2)

Motor TT direito: soldar 100nF cerâmico directamente
  entre terminal M+ e terminal M− (nos bornes L298N OUT3/OUT4)

Total: 2× condensadores cerâmicos 104 (100nF).
Quanto mais perto do motor, mais eficaz.
Reduz ruído electromagnético que interfere com sensores IR.
```
