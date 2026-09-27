# CALIBRAÇÃO — Guia Completo

## Ordem de calibração recomendada

1. LM2596 (antes de tudo)
2. I2C Scanner (verificar endereços)
3. IR Remote (confirmar códigos)
4. Elecbee analógico (threshold chama)
5. Sensores de linha (threshold linha)
6. Boundary sensors (testar na arena)
7. KY-026 (ajustar sensibilidade)
8. PID linha (Kp, Ki, Kd)
9. PID heading (Kp, Ki, Kd)
10. Modo quadrado (tempo por lado)

---

## 1. LM2596 — Tensão de saída 6V

**Fazer antes de qualquer outra ligação.**

```
1. Ligar APENAS pilhas ao LM2596 (IN+ e IN−)
2. Ligar multímetro em OUT+ e OUT−
3. Rodar potenciómetro azul (anti-horário = baixa tensão):
   - Rodar devagar até marcar exactamente 6.00V
4. Desligar pilhas
5. Só agora ligar L298N e servos ao OUT+
```

**ATENÇÃO:** Se não calibrar e a tensão for 9V, os servos SG90 queimam imediatamente (rated max 6V). Os motores TT também (rated max 6V).

---

## 2. I2C Scanner — Verificar endereços

Carregar este sketch para confirmar endereços do MPU-6050 e LCD:

```cpp
#include <Wire.h>
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("I2C Scanner");
}
void loop() {
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Encontrado: 0x");
      Serial.println(addr, HEX);
    }
  }
  delay(5000);
}
```

**Resultado esperado:**
- 0x27 = LCD1602 I2C adapter PCF8574
- 0x68 = MPU-6050

Se LCD aparecer em 0x3F em vez de 0x27: alterar no código:
```cpp
LiquidCrystal_I2C lcd(0x3F, 16, 2);  // em vez de 0x27
```

---

## 3. IR Remote — Confirmar códigos

Carregar sketch IRrecvDemo (exemplos da biblioteca IRremote).
Apontar remote ELEGOO para o IR receiver e premir cada botão.
Anotar os códigos hex que aparecem no Serial Monitor.

Actualizar no código (firebot_mega_v2.ino):
```cpp
#define IR_BTN_1    0xXXXXXX  // Botão 1 → START
#define IR_BTN_HASH 0xXXXXXX  // Botão # → STOP
#define IR_BTN_OK   0xXXXXXX  // Botão OK → quadrado
```

Guardar os códigos aqui para referência:
- Botão 1:  ___________
- Botão #:  ___________
- Botão OK: ___________

---

## 4. Elecbee 5-ch — FLAME_ANALOG_THRESHOLD

**Método:**

No arranque do robô, o Serial Monitor (115200 baud) imprime automaticamente
os valores dos 5 canais Elecbee em repouso (sem chama). Exemplo típico:
```
Canal 1: 750
Canal 2: 780
Canal 3: 760
Canal 4: 770
Canal 5: 755
```

Aproximar uma vela a ~30cm do sensor e anotar os valores mínimos observados:
```
Canal 1: 650
Canal 2: 200   ← mais próximo, sinal mais forte
Canal 3: 350
Canal 4: 500
Canal 5: 720
```

**Definir threshold a meio caminho entre repouso e chama:**
- Repouso mínimo: ~750
- Com chama mínimo: ~200
- Threshold: (750 + 200) / 2 = **475** (arredondar para 400-500)

Actualizar no código:
```cpp
#define FLAME_ANALOG_THRESHOLD 400  // ajustar conforme testes
```

**Teste de confirmação:**
- Sem chama → Serial deve mostrar lerIntensidadeChama() > threshold
- Com chama → lerIntensidadeChama() < threshold e chamaDetetada() = true

---

## 5. Sensores de Linha — LINE_THRESHOLD

**Método:**

Abrir Serial Monitor (115200 baud).
Adicionar temporariamente ao loop() para ver valores:
```cpp
Serial.print(analogRead(A8)); Serial.print(" ");  // sensor central
Serial.println(analogRead(A6));  // sensor esquerdo
```

Posição 1: Robô sobre a linha → anotar valor (tipicamente 600-900)
Posição 2: Robô fora da linha → anotar valor (tipicamente 100-300)

**Threshold = média dos dois valores:**
```
Sobre linha: 750
Fora linha:  200
Threshold:   (750+200)/2 = 475
```

Actualizar no código:
```cpp
#define LINE_THRESHOLD 475
```

**Testar com 5 sensores em simultâneo:**
Sensor central deve dar valor ALTO quando sobre a linha.
Sensor esq e dir devem dar valor BAIXO quando não há linha nessa posição.

---

## 6. Boundary Sensors — Borda da Arena

**Método:**
Testar na arena real antes de qualquer corrida de competição.

Colocar o robô dentro da arena, deslizar devagar para a borda.
Verificar com Serial Monitor o que acontece em D29 e D30:

```cpp
// Adicionar temporariamente ao loop():
Serial.print("IR_L: "); Serial.print(digitalRead(29));
Serial.print(" IR_R: "); Serial.println(digitalRead(30));
```

**Se HIGH = borda (sensor detects arena edge):**
→ Código actual está correcto (LOW = dentro da arena)

**Se LOW = borda (sensor não detecta arena):**
→ Inverter lógica em verificarBorda():
```cpp
bool verificarBorda() {
  return (digitalRead(IR_BOUND_L) == HIGH || digitalRead(IR_BOUND_R) == HIGH);
}
```

Ajustar a sensibilidade com o potenciómetro em cada sensor MH Flying Fish.

---

## 7. KY-026 — Sensibilidade backup chama

Girar o potenciómetro do sensor com uma vela a ~15cm.
LED verde do módulo deve acender quando há chama.
DO deve ficar LOW (o código usa digitalRead(KY026_PIN) == LOW).
Ajustar distância de detecção com o potenciómetro.

---

## 8. PID Linha — Kp, Ki, Kd

**Valores iniciais no código:**
```cpp
#define KP_LINE   22.0
#define KI_LINE    0.0
#define KD_LINE   12.0
```

**Procedimento de ajuste:**

**Passo 1 — Ajustar Kp (proporcional):**
- Definir Ki=0, Kd=0, Kp=5
- Testar na linha. Se o robô oscila: Kp demasiado alto.
- Se reage devagar: Kp demasiado baixo.
- Aumentar Kp gradualmente até o robô seguir razoavelmente mas com alguma oscilação.
- Típico para TT motors: Kp=15-25

**Passo 2 — Ajustar Kd (derivativo — amortece):**
- Com Kp ajustado e Ki=0, aumentar Kd até oscilação desaparecer.
- Típico: Kd=10-15
- Kd demasiado alto: robô reage a ruído dos sensores.

**Passo 3 — Ajustar Ki (integrativo — corrige erro acumulado):**
- Normalmente Ki=0 é suficiente.
- Se o robô consistentemente desvia para um lado: Ki=0.1-0.5
- Usar com cuidado — Ki pode criar oscilação lenta ("windup").

---

## 9. PID Heading (Gyro) — Kp, Ki, Kd

**Valores iniciais:**
```cpp
#define KP_HEAD    2.5
#define KI_HEAD    0.05
#define KD_HEAD    0.8
```

**Procedimento:**

**Testar linha recta:**
- Colocar robô numa superfície plana
- Arrancar (BUSCA com linha reta)
- Observar se vai em linha recta ou curva
- Se curva: Kp demasiado baixo → aumentar
- Se oscila: Kp demasiado alto → diminuir + aumentar Kd

**Testar viragens (modo quadrado):**
- Premir OK no remote → doQuadrado()
- Deve fazer 4 lados iguais
- Se os lados não são iguais: Kp do heading precisa ajuste

---

## 10. Modo Quadrado — Tempo por lado

**Valor inicial:**
```cpp
#define SQUARE_SIDE_MS  2500  // 2.5 segundos por lado
```

**Calibrar:**
1. Premir OK no remote → executa quadrado
2. Medir o lado resultante com régua
3. Calcular: novo_tempo = 2500 × (lado_desejado_cm / lado_medido_cm)
4. Exemplo: quero 40cm, mediu 35cm → novo_tempo = 2500 × (40/35) = 2857ms

Actualizar e testar novamente.

---

## Tabela de constantes — resumo final

Preencher depois de calibrar:

| Constante | Valor default | Valor calibrado |
|---|---|---|
| LINE_THRESHOLD | 500 | ___ |
| FLAME_ANALOG_THRESHOLD | 400 | ___ |
| FLAME_STRONG_SIGNAL | 200 | ___ |
| KP_LINE | 22.0 | ___ |
| KI_LINE | 0.0 | ___ |
| KD_LINE | 12.0 | ___ |
| KP_HEAD | 2.5 | ___ |
| KI_HEAD | 0.05 | ___ |
| KD_HEAD | 0.8 | ___ |
| SQUARE_SIDE_MS | 2500 | ___ |
| IR_BTN_1 | 0xFF6897 | ___ |
| IR_BTN_HASH | 0xFF4AB5 | ___ |
| IR_BTN_OK | 0xFF38C7 | ___ |
| LCD addr | 0x27 | ___ |
