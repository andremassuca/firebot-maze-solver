# MÁQUINA DE ESTADOS — Comportamentos do Firebot

## Diagrama de estados

```
                    ┌─────────────────────────────────────┐
                    │         Botão longo / # remote       │
                    ▼                                       │
              ┌──────────┐                                 │
              │   IDLE   │ ◄── arranque inicial            │
              └────┬─────┘                                 │
                   │ botão START / botão 1 remote          │
                   ▼                                       │
              ┌──────────┐   chama detectada               │
              │  BUSCA   │ ─────────────────►──────────┐  │
              └──────────┘                             │  │
                   ▲                                   ▼  │
                   │ chama        ┌───────────────────┐   │
                   │  perdida     │    LOCALIZAR       │   │
                   │              └─────────┬──────────┘   │
                   │                        │ dir. confirmada│
                   │                        ▼               │
                   │              ┌───────────────────┐     │
                   │ chama        │    APROXIMAR       │     │
                   └──── perdida ─┤                   │     │
                                  └─────────┬──────────┘     │
                                            │ dist<15cm      │
                                            ▼               │
                                  ┌───────────────────┐     │
                                  │    EXTINGUIR       │     │
                                  │  🔥 fan + sirene   │     │
                                  └─────────┬──────────┘     │
                                            │ chama extinta  │
                                            ▼               │
                                  ┌───────────────────┐     │
                                  │    RETORNAR        │─────┘
                                  │  procura linha     │ (retoma BUSCA)
                                  └───────────────────┘

Qualquer estado (excepto IDLE):
  Borda detectada (IR fundo) → para + recua + retoma estado
```

---

## IDLE

**Condição de entrada:** arranque, botão longo, # remote
**O que faz:**
- Motores parados
- Fan desligada (MOSFET OFF)
- Servos centrados a 90°
- Buzzer silencioso
- LED vermelho ligado
- Aguarda input

**Saída para BUSCA:** botão START pressionado OU botão 1 remote

---

## BUSCA

**Condição de entrada:** transição de IDLE
**O que faz:**
1. Varre servo Elecbee lentamente 0°→180°→0° (não-bloqueante, 35ms/passo)
2. Verifica chama em cada ciclo → se detectada: LOCALIZAR
3. Segue linha com PID:
   - 5 sensores analógicos → erro por média ponderada
   - PID calcula output → ajusta velocidade esq/dir
   - Se linha perdida: roda devagar à procura
4. Verifica obstáculo à frente (sonarSC): se < 28cm → evitar
5. LED amarelo

**Saída:**
- Chama detectada → LOCALIZAR
- (nunca termina sozinho — aguarda chama)

**Algoritmo evitar obstáculo (na linha):**
1. Para
2. Mede distância esq e dir
3. Contorna pelo lado com mais espaço (S-curve de 7 movimentos)
4. Loop detecta linha e realinha automaticamente

---

## LOCALIZAR

**Condição de entrada:** chama detectada em BUSCA
**O que faz:**
1. Para imediatamente
2. Varredura grossa: servo Elecbee 0°→180° em passos de 5°, 20ms/passo
3. Regista posição com sinal analógico mais forte (valor mais baixo)
4. Varredura fina: ±20° em torno do melhor ponto, passos de 2°, 15ms
5. Centra servo a 90°
6. Calcula ângulo de viragem: (melhorPos - 90°) × 0.8
7. Executa viragem com gyro para o heading calculado
8. LED amarelo

**Saída:**
- Direcção confirmada → APROXIMAR
- Chama perdida durante varredura → BUSCA

**Porquê analógico é melhor:**
Valor mais baixo = sinal IR mais intenso = chama mais próxima nessa direcção.
A varredura em duas passagens (grossa + fina) dá precisão de 2° no melhor ponto.

---

## APROXIMAR

**Condição de entrada:** direcção calculada em LOCALIZAR
**O que faz:**
1. Navega em direcção ao target heading
2. **Blend de heading:**
   - Se Elecbee vê chama (servo a 90°): 70% Elecbee + 30% gyro
   - Se Elecbee não vê: 100% gyro
3. Verifica HC-SR04 fixos (E/D) a cada ciclo — desvio suave se obstáculo
4. Verifica HC-SR04 servo (centro) — se < 15cm → EXTINGUIR
5. Verifica Elecbee continuamente — se perde chama → LOCALIZAR
6. LED amarelo

**Porquê o blend:**
O gyro mantém o heading geral mas acumula drift.
O Elecbee corrige em tempo real enquanto vê a chama.
O blend 70/30 quando Elecbee activo dá apontamento mais preciso
à medida que o robô se aproxima.

**Saída:**
- HC-SR04 < 15cm → EXTINGUIR
- Chama perdida → LOCALIZAR

---

## EXTINGUIR

**Condição de entrada:** distância < 15cm (OBSTACLE_STOP)
**O que faz:**
1. Motores param
2. Fan liga a 100% (analogWrite(FAN_PIN, 255))
3. Sirene bombeiros não-bloqueante:
   - Sweep 700Hz → 960Hz → 700Hz → ... (12ms por passo, ±10Hz)
   - tone(BUZZER_PIN, sirenFreq) actualizado sem delay()
4. Verifica extinção: Elecbee SEM sinal E KY-026 SEM sinal
5. Timeout 12s de segurança
6. LED verde

**Saída:**
- Chama extinta (ambos sensores confirmam) → 5 bips + RETORNAR
- Timeout 12s → RETORNAR (sem confirmação)

**Porquê dois sensores para confirmar extinção:**
Elecbee: sensível à radiation IR da chama (visão ampla)
KY-026: também sensível a IR + luz visível (backup próximo)
Ambos a zero = extinção confirmada com alta confiança.

---

## RETORNAR

**Condição de entrada:** extinção confirmada
**O que faz:**
1. Fan desliga
2. Sirene para
3. Procura linha com os 5 sensores:
   - Se linha detectada: segue com PID de volta
   - Se linha perdida: roda lentamente à procura
4. LED verde
5. Segue linha até... (definir ponto de paragem — pode ser IDLE manual)

**Saída:**
- Botão/# remote → IDLE
- (actualmente não pára sozinho — aguarda input do operador)

*Possível melhoria: detectar marca de início/fim na linha para parar automaticamente.*

---

## Interrupções de prioridade

### Borda da arena (QUALQUER estado excepto IDLE)
```
Detecta: IR_BOUND_L=LOW ou IR_BOUND_R=LOW
Acção:
  1. Para imediatamente
  2. Bip 1500Hz 300ms
  3. Recua (BASE_SPEED inverso) durante 350ms
  4. Vira (TURN_SPEED) durante 400ms
  5. Retoma loop principal (estado não muda)
```

### Watchdog Timer (4 segundos)
```
Se código travar por > 4s sem wdt_reset():
  → Arduino faz reset automático
  → Robot reinicia em IDLE
```

---

## LEDs — Código de cores

| Estado | LED R | LED Y | LED G | Significado |
|---|---|---|---|---|
| IDLE | ON | off | off | Aguarda arranque |
| BUSCA | off | ON | off | A seguir linha + procurar |
| LOCALIZAR | off | ON | off | Parado, a varrer |
| APROXIMAR | off | ON | off | A navegar para chama |
| EXTINGUIR | off | off | ON | Fan ligada, sirene |
| RETORNAR | off | off | ON | A voltar à base |

---

## Remote IR — Comandos

| Botão | Acção | Estado necessário |
|---|---|---|
| 1 | START → BUSCA | IDLE |
| # | STOP → IDLE | Qualquer |
| OK | Modo Quadrado | IDLE |

**Modo Quadrado (botão OK):**
- Calibra gyro (2s imóvel)
- Percorre 4 lados (SQUARE_SIDE_MS ms cada)
- Cada lado: heading PID mantém linha recta
- Cada canto: girarParaAngulo(heading + 90°)
- Após 4 lados: 3 bips + IDLE
