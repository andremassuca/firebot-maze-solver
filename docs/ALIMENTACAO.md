# ALIMENTAÇÃO — Sistema de Potência Completo

## Dois circuitos independentes

O Firebot usa dois circuitos de alimentação separados com GND comum.
Esta separação protege a electrónica sensível dos picos de corrente da ventoinha.

---

## Fonte A — Lógica e Movimento (9V)

**Composição:** 6×AA alkaline em série, no suporte 6-cell do Kit 5

| Componente | Tensão recebida | Corrente típica | Notas |
|---|---|---|---|
| Arduino Mega (Vin) | 9V | ~50mA | Regula internamente para 5V |
| Sensores + LCD + LEDs | 5V (pino Mega) | ~280mA | Máx 500mA total no pino 5V |
| LM2596 (entrada) | 9V | ~495mA | Eficiência 85%, alimenta abaixo |
| L298N VIN | 6V (LM2596) | ~400mA | Motores ficam com ~4V |
| Servo SG90 ×2 VCC | 6V (LM2596) | ~200mA pico | Rated 4.8-6V |
| **TOTAL Fonte A** | **9V** | **~725mA médios** | **em movimento normal** |

**Autonomia estimada:**
- 6×AA alkaline: ~1800mAh efectivos a 725mA → **≈ 2.5h contínuas**
- Em competição (ciclos 2-5min): muitas horas reais

**Curva de descarga 6×AA alkaline:**
- Novas: 9.0–9.6V → LM2596 OK (headroom 3-3.6V)
- Meia carga: 8.4V → LM2596 OK (headroom 2.4V)
- Baixas: 7.2V → LM2596 marginal mas ainda funciona
- Mega Vin: funciona de 7V a 12V — toda a gama coberta ✓

---

## Fonte B — Ventoinha (12V)

**Composição:** 2× suportes 4×AA alkaline em SÉRIE

```
Suporte 1: 4×AA = 6V
Suporte 2: 4×AA = 6V
Em série:  8×AA = 12V
```

| Componente | Tensão | Corrente | Quando activo |
|---|---|---|---|
| Arctic P12 | 12V | 0.35A (4.2W) | Só no estado EXTINGUIR |
| MOSFET D4184 (perdas) | ~0V | negligível | — |
| **TOTAL Fonte B** | **12V** | **0.35A** | **segundos por run** |

**Autonomia:** A fan só actua 5-15 segundos por run de competição.
8×AA alkaline (~2500mAh) / 0.35A = **~7 horas de fan contínua**.
Na prática: centenas de runs de competição antes de trocar.

---

## Porquê NiMH 6×AA NÃO funciona na Fonte A

O LM2596 precisa de input ≥ (output + 1.5V dropout mínimo) = 6 + 1.5 = **7.5V mínimo**.

| Tipo pilha | 6× células | Vc meia descarga | LM2596 mantém 6V? |
|---|---|---|---|
| Alkaline 1.5V | 9.0V | 8.4V | ✓ Sim, com folga |
| NiMH 1.2V | 7.2V | 6.8V | ✗ Não (6.8-6=0.8V < 1.5V dropout) |

**Conclusão:** Sempre alkaline no suporte de 6 células. Nunca NiMH.

Para a Fonte B (fan), NiMH funciona (9.6V = 80% potência da fan) mas alkaline é preferido para competição.

---

## Se quiseres recarregáveis

**Opção A — NiMH 8×AA para Fonte A:**
Usar os dois suportes de 4×AA em série com NiMH = 9.6V.
Funciona bem para a lógica e LM2596 (headroom 3.6V).
Mas então precisas de um terceiro par de suportes para a fan (12V alkaline).
Total de suportes: 3 pares de 4×AA.

**Opção B — Pack 3S 18650 Li-ion (recomendado se queres simplificar):**
- 3×18650 em série = 11.1V nominal (12.6V cheio, 9.0V mínimo)
- Alimenta Mega Vin ✓ (7-12V range)
- LM2596: headroom 5V ✓ (excelente)
- Fan via MOSFET a 11.1V = 92% potência ✓
- **Uma única fonte para tudo** → um único switch → máxima simplicidade
- Custo: ~€15-20 com BMS e suporte de 3 células
- O suporte de 18650 do Kit 5 só tem 2 células (2S=7.4V) — insuficiente para esta opção

---

## LM2596 — Calibração

**FAZER ANTES de ligar ao L298N e servos:**

1. Ligar apenas a Fonte A (6×AA, 9V)
2. Ligar LM2596 IN+ ao 9V, LM2596 IN− ao GND
3. Ligar multímetro nos terminais OUT+ e OUT−
4. Rodar o potenciómetro azul (sentido anti-horário = tensão desce)
5. Ajustar até o multímetro marcar **6.00V exacto**
6. Desligar — só agora ligar L298N e servos

**NUNCA ligar L298N ou servos sem calibrar o LM2596 primeiro.**

---

## Interruptores

| Switch | Corta | Localização |
|---|---|---|
| Switch A | Fonte A (9V) — toda a lógica e movimento | Acessível lateralmente, piso 1 ou 2 |
| Switch B | Fonte B (12V) — ventoinha | Junto a Switch A |

**Sequência de arranque:** Switch A primeiro → Switch B → aguardar LCD "PRONTO" → START.
**Sequência de paragem:** Premir botão (ou # no remote) → Switch B → Switch A.

**DPDT opcional:** Um único switch de duplo polo (DPDT) pode cortar ambas as fontes simultaneamente com um movimento. Cada polo liga a um circuito.

---

## GND Estrela — Porquê é Crítico

Sem GND comum, os dois circuitos não têm referência partilhada e o MOSFET não consegue comutar correctamente (o gate usa referência do Mega, o source usa referência da Fonte B — têm de ser o mesmo ponto).

Ligar fisicamente num único ponto (parafuso ou bloco de terminais):
- Mega GND
- L298N GND  
- LM2596 OUT−
- MOSFET J1 GND
- Suporte 6×AA (−)
- Suporte 4×AA pair 1 (−) [o negativo da Fonte B]
