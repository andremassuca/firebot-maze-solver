# FIREBOT LINE FOLLOWER & EXTINTOR — Documentação Completa

## Visão Geral

Robô autónomo de dois pisos capaz de seguir uma linha preta em fundo branco,
localizar uma chama de vela e extingui-la com uma ventoinha Arctic P12 120mm.
Controlado por Arduino Mega 2560. Alimentação dupla: 9V para lógica/movimento,
12V dedicado à ventoinha.

---

## Ficheiros neste pacote

| Ficheiro | Conteúdo |
|---|---|
| `firebot_mega_v2.ino` | Código Arduino completo (872 linhas) |
| `COMPONENTES.md` | Lista completa de peças com origem |
| `LIGACOES.md` | Esquema de ligações — todos os pinos |
| `ALIMENTACAO.md` | Sistema de alimentação — tensões, correntes, autonomia |
| `MONTAGEM.md` | Guia de montagem passo a passo |
| `CALIBRACAO.md` | Calibração de todos os sensores e PIDs |
| `MAQUINA_ESTADOS.md` | Máquina de estados e comportamentos |
| `PINOS.md` | Tabela de pinos rápida |

---

## Funcionalidades

- **Seguimento de linha** com PID — 5× TCRT5000 analógico, erro por média ponderada
- **Localização de chama 360°** — Elecbee 5-ch em servo SG90, varredura 0-180° analógica
- **Evitar obstáculos** — 3× HC-SR04 (2 fixos 30°, 1 em servo), contorno automático
- **Linha recta exacta** — MPU-6050 giroscópio com PID de heading
- **Modo quadrado** — 4 lados com viragens 90° controladas por gyro
- **Extinção** — Arctic P12 120mm a 12V via MOSFET D4184
- **Sirene bombeiros** — buzzer passivo, sweep 700–960 Hz não-bloqueante
- **Deteção de borda** — 2× MH Flying Fish IR no fundo do chassis
- **Arranque remoto** — IR receiver + remote ELEGOO (botão 1 = start, # = stop, OK = quadrado)
- **Display** — LCD1602 I2C: estado + distância + heading em tempo real
- **LEDs** — R=IDLE, Y=em missão, G=extinguir/retornar
- **Watchdog timer** — reset automático em 4s se código travar

---

## Máquina de Estados

```
IDLE ──(start)──► BUSCA ──(chama)──► LOCALIZAR ──(dir)──► APROXIMAR
                    ▲                                           │
                    │                                    (dist<15cm)
                    │                                           │
                 RETORNAR ◄──(extinta)── EXTINGUIR ◄────────────
```

Qualquer estado → borda detectada → para + recua → retoma estado
Qualquer estado → botão longo → IDLE

---

## Chassis

- **Base:** kevr102 (Arduino+Chassis.3mf) — impresso em PLA
- **Perfil:** Arduino+car+A1+profile.3mf — impresso em PLA
- **Estrutura:** 2 pisos ligados por 6-8× standoffs M3×35-40mm
- **Piso 1 (base):** motores, L298N, sensores de linha, boundary IR, cabagem potência
- **Piso 2 (topo):** Mega, LM2596, MOSFET, LCD, fan, servos, baterias, switches

---

## Bibliotecas necessárias (Arduino Library Manager)

1. **PID_v1** — Brett Beauregard
2. **MPU6050** — Electronic Cats
3. **LiquidCrystal_I2C** — Frank de Brabander
4. **NewPing** — Tim Eckel
5. **IRremote** — shirriff / Arduino-IRremote
6. Wire, Servo, avr/wdt — built-in no Arduino IDE

---

## Alimentação — Resumo

| Fonte | Composição | Tensão | Alimenta |
|---|---|---|---|
| A | 6×AA alkaline (suporte 6-cell) | 9V | Mega Vin + LM2596→6V (motores+servos) |
| B | 2×4×AA alkaline em série | 12V | Arctic P12 via MOSFET |
| — | GND comum | 0V | Ligar os dois circuitos |

**IMPORTANTE:** Nunca usar NiMH no suporte de 6 células (7.2V é insuficiente para o LM2596 manter 6V).

---

## Ordem de arranque recomendada

1. Colocar robô imóvel na posição de início
2. Ligar Switch A (9V — lógica)
3. Ligar Switch B (12V — fan)
4. LCD mostra "PRONTO — Prima START"
5. Aguardar bips de confirmação (2 bips)
6. Premir botão ou botão 1 do remote IR → arranca
