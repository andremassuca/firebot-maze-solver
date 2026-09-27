# MONTAGEM — Guia Passo a Passo

## Ferramentas necessárias
- Chave de parafusos M3 (Phillips e hexagonal)
- Multímetro (obrigatório para calibrar LM2596)
- Ferro de soldar + estanho (condensadores e conexões permanentes)
- Alicate de bico fino
- Pistola de cola quente (fixações temporárias)
- Cable ties / abraçadeiras
- Fita isolante ou heat shrink tubing

---

## FASE 1 — Piso 1 (Base) — Tracção e Sensores de Fundo

### 1.1 Motores TT
1. Encaixar os dois motores TT nos suportes do chassis kevr102
2. Apertar com parafusos M3×6
3. Montar rodas nos eixos dos motores
4. Montar roda caster traseira com M3×6
5. **ANTES de ligar ao L298N:** soldar condensador cerâmico 100nF directamente
   entre os dois terminais de cada motor (M+ e M−). Sem polaridade.

### 1.2 L298N Motor Driver
1. Fixar L298N no mount do piso 1 com M3×6
2. Ligar OUT1+OUT2 ao motor esquerdo (marcações no L298N)
3. Ligar OUT3+OUT4 ao motor direito
4. Verificar: rodar os motores à mão → não deve haver resistência anormal

### 1.3 Array de sensores de linha
1. Montar o array holder no fundo do piso 1, frente do robô
2. Encaixar os 5 sensores TCRT5000 nos slots (espaçamento 8-12mm)
3. Sensor central = A8 (centro do array)
4. Sensores ordenados esq→dir: A6, A7, A8, A9, A10
5. Altura: 3-5mm acima do chão. Verificar com folha de papel.
6. Ligar VCC→5V e GND→GND de todos os sensores

### 1.4 Boundary sensors (MH Flying Fish IR)
1. Montar nos cantos frontais do piso 1, apontados para o chão
2. Esquerdo → D29, Direito → D30
3. VCC→5V, GND→GND
4. Testar ajuste de sensibilidade (potenciómetro no sensor) na arena real

---

## FASE 2 — Standoffs entre pisos

1. Montar 6-8× standoffs M3×35mm (ou 40mm) nos pontos de fixação
   - 4 nos cantos
   - 2-4 a meio dos lados mais longos
2. Verificar que o L298N (27mm de altura) não bate na placa de cima
   com M3×35mm: espaço = 35mm − altura L298N (27mm) = 8mm de folga ✓
   com M3×25mm: 25-27mm = NEGATIVO — não funciona, L298N bate!

---

## FASE 3 — Piso 2 (Topo) — Electrónica Principal

### 3.1 LM2596 Buck Converter — CALIBRAR PRIMEIRO
1. Fixar LM2596 no mount (piso 2)
2. **Antes de ligar qualquer coisa:**
   - Ligar apenas Fonte A (6×AA, 9V) ao LM2596 IN+/IN−
   - Medir com multímetro em OUT+/OUT−
   - Rodar potenciómetro azul até marcar exactamente 6.00V
   - Desligar pilhas

### 3.2 Arduino Mega
1. Fixar Mega no mount do piso 2 com standoffs M3 pequenos (4×)
2. USB port deve ficar acessível (frente ou lateral do robô)
3. Não ligar ainda — montar primeiro

### 3.3 LCD1602
1. Soldar o adaptador I2C PCF8574 ao LCD1602 (se ainda não soldado)
2. Fixar no bezel mount na frente do piso 2
3. Ligar: VCC→5V, GND, SCL→D21, SDA→D20
4. Ajustar potenciómetro de contraste até texto ser legível

### 3.4 MOSFET D4184 HW-517
1. Fixar no mount do piso 2
2. J1 IN+ → D44, J1 GND → GND comum
3. Não ligar ainda à fan (esperar Fase 5)

### 3.5 MPU-6050
1. Fixar rigidamente no piso 2 — movimento relativo = erros de heading
2. Orientar com eixos alinhados com o eixo do robô
3. Ligar: VCC→5V, GND, SCL→D21, SDA→D20, AD0→GND

### 3.6 Breadboard (buzzer, LEDs, botão)
Alternativa: usar os mounts dedicados impressos em vez de breadboard.

**Com breadboard:**
1. Buzzer passivo: (+)→D45 (sem resistência), (−)→GND
2. LED vermelho: D46→330Ω→LED(+), LED(−)→GND
3. LED amarelo: D47→330Ω→LED(+), LED(−)→GND
4. LED verde: D48→330Ω→LED(+), LED(−)→GND
5. Botão start: terminal A→D49, terminal B→GND

### 3.7 IR Receiver
1. Fixar com visibilidade directa para o operador
2. Signal→D2, VCC→5V, GND

---

## FASE 4 — Sensores frontais (frente do piso 2)

### 4.1 HC-SR04 fixos (×2)
1. Montar nos suportes angulados 30°: esquerdo e direito
2. Esquerdo: TRIG→D22, ECHO→D23
3. Direito: TRIG→D24, ECHO→D25
4. VCC→5V, GND para ambos

### 4.2 Servo HC-SR04 (varrimento)
1. Fixar servo no mount central (piso 2, frente)
2. Montar HC-SR04 no braço do servo
3. Servo signal→D10, VCC→6V(LM2596), GND
4. HC-SR04: TRIG→D26, ECHO→D27, VCC→5V, GND
5. Verificar que servo centrado a 90° aponta directamente para a frente

### 4.3 Servo Elecbee (chama)
1. Fixar servo no mount (piso 2, frente — pode ser elevado acima do HC-SR04)
2. Montar Elecbee no braço do servo
3. Servo signal→D9, VCC→6V(LM2596), GND
4. Elecbee: AO canais 1-5 → A0-A4, VCC→5V, GND
5. Verificar que a 90° o Elecbee aponta para a frente

### 4.4 KY-026 (backup chama)
1. Fixar no piso 2, apontado para a frente e ligeiramente para baixo
2. DO→D28, VCC→5V, GND
3. Ajustar sensibilidade com potenciómetro no sensor

---

## FASE 5 — Arctic P12 e baterias

### 5.1 Arctic P12
1. Fixar no shroud mount do piso 2 (topo), apontada para a FRENTE
2. A fan deve soprar ar para onde o robô se dirige (para a chama)
3. Verificar direcção de fluxo: há seta no frame da fan
4. Fio vermelho (+)→Switch B (12V), fio preto (−)→MOSFET J5 output
5. Fio azul (PWM)→12V (fan sempre a 100% quando alimentada)
6. Fio amarelo (tach)→não ligar

### 5.2 Baterias
1. Fixar suporte 6×AA (Fonte A) no piso 2, traseira
2. Fixar suportes 4×AA×2 (Fonte B) no piso 2, traseira
3. Ligar switches: Switch A na Fonte A, Switch B na Fonte B
4. GND estrela: um ponto físico com todos os GND

---

## FASE 6 — Cabagem e organização

1. Usar cable ties para prender fios ao chassis
2. Separar fisicamente fios de potência (grossos, vermelho/preto) dos sinais
3. Verificar que nenhum fio toca nas rodas ou nos motores
4. Deixar comprimento suficiente nos fios dos servos para o movimento
5. Etiquetar os fios se possível

---

## FASE 7 — Verificação antes de ligar

Checklist obrigatória:

- [ ] LM2596 calibrado para 6.00V (verificado com multímetro)
- [ ] GND estrela conectado (Mega + L298N + LM2596 + MOSFET + baterias)
- [ ] Condensadores 100nF nos dois motores
- [ ] Switch A e B em OFF
- [ ] Arctic P12 orientada para a frente (não para trás)
- [ ] Servo Elecbee a 90° aponta para a frente
- [ ] Servo HC-SR04 a 90° aponta para a frente
- [ ] Array de linha a 3-5mm do chão
- [ ] Boundary sensors apontados para o chão
- [ ] USB do Mega acessível para programar
- [ ] Código carregado no Mega (firebot_mega_v2.ino)
- [ ] Bibliotecas instaladas no Arduino IDE

---

## FASE 8 — Primeiro arranque

1. Ligar USB ao Mega (para monitorizar Serial a 115200 baud)
2. Ligar Switch A (9V)
3. LCD deve mostrar "FIREBOT v2.0" → "A iniciar..."
4. Se LCD não acender: verificar I2C adapter (endereço 0x27)
5. Se "ERRO: MPU-6050": verificar ligações SCL/SDA/AD0
6. Calibração gyro automática (2s — robô imóvel)
7. Serial Monitor mostra valores Elecbee em repouso → anotar para calibrar threshold
8. LCD mostra "PRONTO — Prima START"
9. Ligar Switch B (12V)
10. Verificar que fan NÃO arranca (MOSFET deve estar OFF em IDLE)

---

## Standoffs — Nota Importante

Usar M3×35mm ou M3×40mm — NÃO M3×25mm.

O L298N tem 27mm de altura. Com M3×25mm entre os pisos, o L298N ficaria
a bater na placa superior. Com M3×35mm há 8mm de folga.

Com carga pesada (fan + baterias no topo), usar 6-8 standoffs:
- 4 nos cantos do chassis
- 2-4 no meio dos lados longos
