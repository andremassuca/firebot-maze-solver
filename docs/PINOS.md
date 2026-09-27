# PINOS — Tabela de Referência Rápida

## Arduino Mega 2560 — Mapa completo de pinos utilizados

```
PINO    COMPONENTE                  TIPO        NOTAS
────────────────────────────────────────────────────────────────
D2      IR Receiver signal          INPUT       INT4 interrupt
D3      L298N ENA                   OUTPUT PWM  Velocidade motor E
D4      L298N IN1                   OUTPUT      Direcção motor E_A
D5      L298N IN2                   OUTPUT      Direcção motor E_B
D6      L298N IN3                   OUTPUT      Direcção motor D_A
D7      L298N IN4                   OUTPUT      Direcção motor D_B
D8      L298N ENB                   OUTPUT PWM  Velocidade motor D
D9      Servo Elecbee signal        OUTPUT PWM  Servo sensor chama
D10     Servo HC-SR04 signal        OUTPUT PWM  Servo sensor obstáculo
D20     MPU-6050 + LCD SDA          I2C         Hardware I2C Mega
D21     MPU-6050 + LCD SCL          I2C         Hardware I2C Mega
D22     HC-SR04 fixo esq TRIG       OUTPUT      30° esquerda
D23     HC-SR04 fixo esq ECHO       INPUT
D24     HC-SR04 fixo dir TRIG       OUTPUT      30° direita
D25     HC-SR04 fixo dir ECHO       INPUT
D26     HC-SR04 servo TRIG          OUTPUT      Centro, scanning
D27     HC-SR04 servo ECHO          INPUT
D28     KY-026 DO                   INPUT       Activo LOW
D29     MH IR borda esq DO          INPUT       Fundo chassis
D30     MH IR borda dir DO          INPUT       Fundo chassis
D44     MOSFET D4184 J1 IN+         OUTPUT PWM  Fan Arctic P12
D45     Buzzer passivo (+)          OUTPUT PWM  tone() sirene
D46     LED vermelho (+)            OUTPUT      330Ω em série
D47     LED amarelo (+)             OUTPUT      330Ω em série
D48     LED verde (+)               OUTPUT      330Ω em série
D49     Botão START terminal A      INPUT       INPUT_PULLUP
A0      Elecbee canal 1 (ext esq)   ANALOG IN   Valor baixo = chama
A1      Elecbee canal 2 (esq-ctr)   ANALOG IN
A2      Elecbee canal 3 (centro)    ANALOG IN
A3      Elecbee canal 4 (dir-ctr)   ANALOG IN
A4      Elecbee canal 5 (ext dir)   ANALOG IN
A6      Line sensor 1 (ext esq)     ANALOG IN   Peso PID -200
A7      Line sensor 2 (esq-ctr)     ANALOG IN   Peso PID -100
A8      Line sensor 3 (centro)      ANALOG IN   Peso PID    0
A9      Line sensor 4 (dir-ctr)     ANALOG IN   Peso PID +100
A10     Line sensor 5 (ext dir)     ANALOG IN   Peso PID +200
Vin     Fonte A (+)                 POWER IN    9V alkaline (6×AA)
GND     GND comum estrela           POWER GND   Todos os GND aqui
5V      Sensores e periféricos      POWER OUT   Máx 500mA total
────────────────────────────────────────────────────────────────
PINOS LIVRES (para expansões futuras):
D11-D19, D31-D43, D46-D53
A5, A11-A15
```

## Pinos que NÃO usar (conflitos Mega)

```
D0, D1   → Serial0 (USB/Debug) — reservados
D14/D15  → Serial3 RX3/TX3 — evitar
D16/D17  → Serial2 RX2/TX2 — evitar
D18/D19  → Serial1 RX1/TX1 — evitar (também INT5/INT4 mas 
             conflitam com Serial)
D13      → LED onboard — funciona mas pisca com algumas operações
```

## Barramento I2C — Dois dispositivos em D20/D21

```
Dispositivo        Endereço    Notas
──────────────────────────────────────────
MPU-6050 GY-521    0x68        AD0 → GND
LCD1602 PCF8574    0x27        (ou 0x3F — verificar)
```
Sem conflito — endereços diferentes no mesmo barramento.

## Alimentação — Resumo visual

```
6×AA alkaline (9V)
  │
  Switch A
  ├─► Mega Vin ──► regulador interno ──► pino 5V ──► sensores/LCD/IR
  └─► LM2596 IN+ ──► OUT+ 6V ──► L298N VIN + Servo×2 VCC

2×4×AA alkaline série (12V)
  │
  Switch B
  ├─► Arctic P12 fio vermelho (+)
  └─► MOSFET J5 supply
        │ (quando D44=HIGH)
        └─► Arctic P12 fio preto (−) → fecha circuito

GND ESTRELA ◄── Mega GND, L298N GND, LM2596 OUT−,
              MOSFET J1 GND, 6×AA(−), 4×AA(−)
```
