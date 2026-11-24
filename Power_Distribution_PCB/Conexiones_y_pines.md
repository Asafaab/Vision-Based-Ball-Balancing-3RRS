# Conexiones PCB v13 (una capa, L414)

Placa 66.1 x 57.9 mm.

| Ref | Función | Pines |
|---|---|---|
| J1 | ENTRADA 12V | 1=GND, 2=V12_RAW |
| TB1 | SALIDA 12V (F1) | 1=GND, 2=XL_IN |
| TB2 | SALIDA 12V (F2) | 1=GND, 2=CAB_IN12 |
| TB3 | SALIDA 12V (F3) | 1=GND, 2=MYRIO_12V |
| TB5 | ENTRADA 6V SERVOS 1-3 | 1=GND, 2=V6_3RRS |
| F1 | 6A 5x20 | 1=V12, 2=XL_IN |
| F2 | 3A 5x20 | 1=V12, 2=CAB_IN12 |
| F3 | 3A 5x20 | 1=V12, 2=MYRIO_12V |
| J10 | SERVO1 GND V+ SIG | 1=GND, 2=V6_3RRS, 3=SERVO1_SIG |
| J11 | SERVO2 GND V+ SIG | 1=GND, 2=V6_3RRS, 3=SERVO2_SIG |
| J12 | SERVO3 GND V+ SIG | 1=GND, 2=V6_3RRS, 3=SERVO3_SIG |
| J20 | GND PWM1 PWM2 PWM3 | 1=GND, 2=PI_PWM1, 3=PI_PWM2, 4=PI_PWM3 |
| TB6 | ENTRADA 6V CABECEO | 1=GND, 2=V6_CAB |
| J13 | SERVO 4 (CABECEO) GND V+ SIG | 1=GND, 2=V6_CAB, 3=SERVO4_SIG |
| J26 | PWM CABECEO, GND | 1=CAB_IN, 2=GND |
| TB8 | SALIDAS GND x4 | 1=GND, 2=GND, 3=GND, 4=GND |
| TB7 | SALIDAS + x4 (2x2) | 1=V12, 2=V12, 3=V12, 4=V12 |

- F1–F3: portafusible de 2 clips con patas en línea (cada clip: 2 patas a 5 mm; clips a 15 mm; patas exteriores a 20 mm).
- TB7: 4 salidas + (después de Q1, sin fusible). TB8: 4 GND.
- J20: pin 1 = GND, luego PWM1-3. J10-J13: GND, V+, señal.
