# COMPONENTES — Lista Completa

## Electrónica

| Qtd | Componente | Pinos/Interface | Origem | Notas |
|---|---|---|---|---|
| 1× | Arduino Mega 2560 (ELEGOO clone) | 54 dig, 16 anal, I2C | Kit 1 | Controlador principal |
| 1× | L298N Motor Driver Module | ENA/ENB/IN1-IN4/OUT1-4 | Tens | VIN=6V (LM2596) |
| 2× | Motor TT DC amarelo (caixa redução) | 2 terminais | Tens | 3-6V, 100nF EMI nos terminais |
| 2× | Servo SG90 | Signal/VCC/GND | Kit 1 + Kit 5 | VCC=6V (LM2596), não 5V Mega |
| 1× | HC-SR04 — servo (varrimento centro) | TRIG D26, ECHO D27 | Kit 1 | Distância 0-3m |
| 1× | HC-SR04 — fixo esquerdo (30°) | TRIG D22, ECHO D23 | Tens | Ângulo 30° para esq |
| 1× | HC-SR04 — fixo direito (30°) | TRIG D24, ECHO D25 | Kit 5 | Ângulo 30° para dir |
| 1× | Elecbee 5-ch flame sensor | A0-A4 (analógico) | Tens | Em servo, activo LOW, valor baixo=chama |
| 1× | KY-026 sensor chama (backup) | DO → D28 | Tens | Confirma extinção, activo LOW |
| 5× | TCRT5000 Tracker sensor linha | A6-A10 (analógico) | Tens | Fundo chassis, 3-5mm chão |
| 1× | MPU-6050 GY-521 | I2C D20/D21, addr 0x68 | Comprar | AD0→GND para endereço 0x68 |
| 1× | LCD1602 + I2C adapter PCF8574 | I2C D20/D21, addr 0x27 | Tens | Partilha I2C com MPU-6050 |
| 1× | Arctic P12 120mm fan | 12V/GND/PWM/Tach | Tens | 12V direto, PWM→12V, Tach não ligar |
| 1× | MOSFET D4184 HW-517 | J1 IN+=D44, J5=fan GND | Comprar | Low-side switch, logic-level |
| 1× | LM2596 Buck Converter | IN=9V, OUT=6V ajustado | Kit 4 | Calibrar com multímetro antes de ligar |
| 2× | MH Flying Fish IR sensor | DO → D29, D30 | Tens | Fundo chassis, cantos frente |
| 1× | Buzzer passivo | D45 (PWM) | Kit 1 | PASSIVO obrigatório para sirene |
| 3× | LED 5mm (R/Y/G) + 330Ω | D46, D47, D48 | Kit 1 | 330Ω em série cada |
| 1× | Push button start | D49, pull-up interno | Kit 1 | Activo LOW |
| 1× | IR Receiver + remote ELEGOO | D2 (INT4) | Kit 1 + Kit 5 | Hardware interrupt obrigatório |
| 2× | Condensadores cerâmicos 100nF (104) | Nos terminais motores | Tens | EMI suppression, soldar nos bornes |
| 1× | Suporte 6×AA (6-cell) | Switch A → 9V | Kit 5 | Alkaline APENAS (não NiMH) |
| 2× | Suporte 4×AA em série | Switch B → 12V | Tens | Alkaline recomendado |
| 2× | Interruptor DC rocker/slide | Linha + de cada fonte | Tens | Um por fonte (Switch A e B) |

## Mecânica e Hardware

| Qtd | Peça | Material | Notas |
|---|---|---|---|
| 1× | Chassis kevr102 (Arduino+Chassis.3mf) | PLA | Piso 1 — motores + L298N |
| 1× | Perfil A1 (Arduino+car+A1+profile.3mf) | PLA | Estrutura piso 2 |
| 1× | Mount servo chama | PLA | Impressão própria |
| 1× | Mount Elecbee no servo | PLA | Impressão própria |
| 1× | Mount KY-026 | PLA | Impressão própria |
| 1× | Mount servo HC-SR04 | PLA | Impressão própria |
| 1× | Mount HC-SR04 em servo | PLA | Impressão própria |
| 2× | Mount HC-SR04 fixo (30°) | PLA | Impressão própria |
| 1× | Mount array sensores linha (5 slots) | PLA | Impressão própria |
| 5× | Holder sensor linha individual | PLA | Slot-in ou M2 |
| 1× | Mount MPU-6050 | PLA | Impressão própria |
| 1× | Shroud/mount Arctic P12 | PLA | Impressão própria — direciona ar |
| 1× | Bezel mount LCD1602 | PLA | Impressão própria |
| 2× | Mount boundary sensor (fundo) | PLA | Impressão própria |
| 1× | Mount Arduino Mega | PLA | Impressão própria ou standoffs |
| 1× | Mount L298N | PLA | Impressão própria |
| 1× | Mount D4184 MOSFET | PLA | Impressão própria |
| 1× | Mount LM2596 | PLA | Impressão própria |
| 1× | Panel LEDs status | PLA | 3 furos press-fit |
| 1× | Mount buzzer | PLA | Impressão própria |
| 1× | Mount botão start | PLA | Impressão própria |
| 1× | Mount IR receiver | PLA | Impressão própria |
| 1× | Mount suporte bateria lógica | PLA | Para 6-cell holder |
| 2× | Mount suportes bateria fan | PLA | Para 4-cell holders |
| 2× | Mount interruptores | PLA | Acessíveis de fora |
| 6-8× | Standoff M3×35mm (ou 40mm) | Metal/Nylon | Liga pisos 1 e 2 |
| 20+ | Parafusos M3×6-10mm | Metal | Montagem geral |
| 20+ | Porcas M3 | Metal | |
| 2× | Rodas TT compatíveis | Borracha | Dos kits |
| 1× | Roda caster (mini ball caster) | Plástico | Traseira |
| — | Jumper wires M-M, M-F, F-F | — | Dos kits |
| — | Fio eléctrico 20-22 AWG | — | Alimentação (vermelho/preto) |
| — | Abraçadeiras de cabo | — | Organização cabos |

## Pilhas

| Qtd | Tipo | Tensão | Fonte |
|---|---|---|---|
| 6× | AA alkaline | 1.5V (9V total) | Comprar |
| 8× | AA alkaline | 1.5V (12V total série) | Comprar |

**TOTAL: 14× AA alkaline.** Não misturar alkaline com NiMH no mesmo suporte.
Custo: ~€5-7 por conjunto completo.

## Software (gratuito)

- Arduino IDE (arduino.cc)
- Bibliotecas: PID_v1, MPU6050, LiquidCrystal_I2C, NewPing, IRremote
