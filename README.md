# FIREBOT

Robô autónomo de combate a incêndios. Numa sala com obstáculos, procura uma
chama, aproxima-se dela sem bater em nada, apaga-a com uma ventoinha e regressa
ao ponto de partida orientado pela direcção do giroscópio.

Construído no âmbito de Elementos de Robótica, na Licenciatura em Engenharia
Biomédica da Universidade Lusófona, Faculdade de Engenharia. Apresentado
publicamente na Noite dos Investigadores 2026, a 25 de Setembro, com poster A3
e demonstração ao vivo.

![O FIREBOT visto de lado](media/firebot-lado.jpg)

## A missão

1. **Procura.** Roda devagar até o sensor de chama detectar o infravermelho da
   vela.
2. **Aproxima-se.** Avança e usa os três sonares para evitar obstáculos.
3. **Apaga.** Pára perto da vela e liga a ventoinha.
4. **Volta.** Regressa orientado pela direcção do giroscópio.

O alvo é uma vela simulada, construída de propósito: chama impressa em 3D,
LEDs a tremeluzir e um emissor de infravermelhos. O sensor de chama responde
entre cerca de 760 e 1100 nm, pelo que o alvo de teste tem de emitir nessa
gama para ser detectável. O programa da vela está em `firmware/vela/`.

## Estrutura

```
firmware/
  missao-autonoma/   missão completa e resolução de labirinto (v8.3)
  telemetria/        build de demonstração, telemetria e comandos
  vela/              programa da vela simulada
app-pc/              aplicação de ambiente de trabalho em C#
docs/                componentes, ligações, pinos, estados, montagem,
                     calibração e alimentação
media/               poster A3 e fotografias
```

### firmware/missao-autonoma

Build de 1341 linhas com a missão autónoma. Máquina de dezasseis estados,
onde `BUSCA`, `LOCALIZAR`, `APROXIMAR`, `EXTINGUIR` e `RETORNAR` conduzem a
missão e os restantes cobrem o modo labirinto, condução manual, calibração e
os testes por módulo. Inclui escalonamento dos servos por estado, velocidade
adaptativa, um contador de confiança que exige cinco confirmações antes de
assumir a chama, e um mapa básico do labirinto impresso na porta série no fim
de cada percurso.

### firmware/telemetria

Build separado, de 996 linhas, escrito para a demonstração pública. Reporta
todos os sensores dez vezes por segundo numa linha por trama, com as três
distâncias dos sonares, intensidade da chama contra a base calibrada,
direcção, inclinação, as duas tensões de alimentação e o esforço dos motores
e da ventoinha. Sai pela USB e pelo rádio ao mesmo tempo, e aceita comandos
por qualquer um dos dois.

Duas decisões de comportamento em falha que vale a pena conhecer antes de
mexer no código: os motores param sozinhos 500 ms após o último comando, para
que uma quebra de ligação pare o robô em vez de o deixar em andamento; e um
módulo que não esteja ligado é reportado como ausente em vez de interromper o
ciclo, para que um robô a que falte um sensor continue a demonstrar tudo o
resto.

Manter os dois builds separados foi deliberado: permitiu tornar a versão que
enfrentou o público mais conservadora sem tocar no código que carrega a missão
autónoma.

### app-pc

Aplicação em C# que desenha os retornos dos sonares em radar, a direcção, o
canal da chama e os controlos dos motores. Liga-se por BLE a um módulo BT24
através do canal de dados FFE1. O firmware trata esse rádio como porta série
comum, e é por isso que o mesmo conjunto de comandos funciona por cabo e por
ar. Compila com o `csc` da .NET Framework contra os `.winmd` do Windows, sem
SDK instalado, o que permite corrê-la a partir de uma pasta em qualquer
máquina.

![A aplicação em funcionamento durante a demonstração](media/firebot-stand.jpg)

## Hardware

Arduino Mega 2560, três sonares HC-SR04, giroscópio, ecrã LCD 16×2 por I2C,
sensor de chama por infravermelhos, ventoinha accionada por MOSFET, receptor
de infravermelhos para comando, LEDs e buzzer de aviso, e duas fontes
independentes, uma para os motores e outra para a lógica.

A lista completa, com quantidades, pinos e notas de montagem, está em
[`docs/COMPONENTES.md`](docs/COMPONENTES.md) e
[`docs/LIGACOES.md`](docs/LIGACOES.md).

**Nota sobre a documentação.** Os ficheiros em `docs/` foram escritos para uma
fase anterior do robô, a do seguidor de linha com extintor, e em alguns
pontos não coincidem com o build apresentado em Setembro: descrevem um
MPU-6050 onde o poster indica um MPU-6500, e sonares laterais a 30° onde o
poster indica 28°. Continuam a ser a referência mais completa de montagem e
calibração que existe, mas confirme contra o hardware antes de seguir um valor
à letra.

## Estado

A missão autónoma foi concluída e testada no primeiro build. O que correu
perante o público foi o firmware de telemetria, de demonstração.

Da tabela de testes do poster: motores e sentido das rodas validados, os três
sonares validados, giroscópio e ecrã por I2C validados, dados para o PC a dez
por segundo por cabo e dois por segundo por Bluetooth, paragem sem ordens da
aplicação em 0,5 s, e ventoinha com arranque suave validada. **O sensor de
chama está marcado para substituição**: a sensibilidade e o campo de visão
foram o factor limitante no alcance de detecção fiável, e é a primeira coisa a
mudar.

## O que aprendi

Ligar cada módulo sem alimentação e confirmar a polaridade evita
curto-circuitos nos 5 V. Com duas fontes, o GND tem de ser comum para os
sinais partilharem referência. Testar módulo a módulo contra telemetria em
tempo real isola uma falha em segundos, ao passo que testar a missão montada
apenas informa que algo, algures, correu mal; o firmware de telemetria foi
escrito para a demonstração e acabou por ser a melhor ferramenta de
diagnóstico do projecto.

## Autoria

André Oliveira Massuça. Licenciatura em Engenharia Biomédica, Universidade
Lusófona, Faculdade de Engenharia.

[andremassuca.com](https://andremassuca.com) ·
[ORCID 0009-0005-1527-843X](https://orcid.org/0009-0005-1527-843X)

## Licença

MIT. Ver [LICENSE](LICENSE).
