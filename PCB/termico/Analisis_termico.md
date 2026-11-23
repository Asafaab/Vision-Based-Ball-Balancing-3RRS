# Análisis térmico PCB v13

Método: IPC-2221 (cobre externo 35 µm), ΔT despejado por tramo; componentes con I²·R. Ambiente supuesto: 30 °C.

## Corrientes supuestas (A)

| Rama | Normal | Pico (segundos) |
|---|---|---|
| XL_IN | 1.7 | 4.2 |
| CAB_IN12 | 0.6 | 1.4 |
| MYRIO_12V | 0.5 | 1.2 |
| OUT_BAT | 1.5 | 3.0 |
| V6_3RRS | 3.0 | 7.5 |
| V6_CAB | 1.0 | 2.5 |
| V12_RAW | 4.3 | 9.8 |
| V12 | 4.3 | 9.8 |

## Pistas, escenario normal

| Red | I máx (A) | Ancho mín (mm) | ΔT máx (°C) | Caída (mV) | Pérdida (mW) |
|---|---|---|---|---|---|
| V12_RAW | 4.3 | 2.4 | 9.0 | 10 | 42 |
| V12 | 4.3 | 1.6 | 9.0 | 67 | 272 |
| V6_3RRS | 3.0 | 1.7 | 7.0 | 22 | 66 |
| XL_IN | 1.7 | 3.0 | 0.8 | 3 | 6 |
| V6_CAB | 1.0 | 1.6 | 0.6 | 8 | 8 |
| CAB_IN12 | 0.6 | 1.6 | 0.2 | 2 | 1 |
| MYRIO_12V | 0.5 | 1.6 | 0.1 | 2 | 1 |

## Pistas, escenario pico

| Red | I máx (A) | Ancho mín (mm) | ΔT máx (°C) | Caída (mV) | Pérdida (mW) |
|---|---|---|---|---|---|
| V12_RAW | 9.8 | 2.4 | 58.3 | 28 | 273 |
| V12 | 9.8 | 1.6 | 58.3 | 173 | 1614 |
| V6_3RRS | 7.5 | 1.7 | 56.0 | 63 | 472 |
| XL_IN | 4.2 | 3.0 | 5.9 | 8 | 35 |
| V6_CAB | 2.5 | 1.6 | 5.1 | 21 | 52 |
| CAB_IN12 | 1.4 | 1.6 | 1.4 | 5 | 7 |
| MYRIO_12V | 1.2 | 1.6 | 1.0 | 4 | 5 |

Los cuellos de 1.5–1.7 mm entre patas de paso 2.54 mm miden 1–2 mm; ahí IPC-2221 no aplica (el calor se va a los pads y a las patas).

## Componentes

| Componente | Escenario | I (A) | P (W) | Tj (°C) |
|---|---|---|---|---|
| Q1 IRF4905 (sin disipador, RthJA 62 °C/W, regimen permanente) | normal | 4.30 | 0.428 | 57 |
| F1 6A (28 % de su corriente nominal) | normal | 1.70 | 0.036 | - |
| F2 3A (20 % de su corriente nominal) | normal | 0.60 | 0.011 | - |
| F3 3A (17 % de su corriente nominal) | normal | 0.50 | 0.007 | - |
| R1 10k (compuerta) | normal | 0.00 | 0.014 | - |
| Bornera J1 (contacto ~2 mOhm x2) | normal | 4.30 | 0.074 | - |
| Q1 IRF4905 (sin disipador, RthJA 62 °C/W, regimen permanente) | pico | 9.80 | 4.646 | 318 |
| F1 6A (70 % de su corriente nominal) | pico | 4.20 | 0.221 | - |
| F2 3A (47 % de su corriente nominal) | pico | 1.40 | 0.059 | - |
| F3 3A (40 % de su corriente nominal) | pico | 1.20 | 0.043 | - |
| R1 10k (compuerta) | pico | 0.00 | 0.014 | - |
| Bornera J1 (contacto ~2 mOhm x2) | pico | 9.80 | 0.384 | - |

## Conclusiones

- Uso normal (≈4.3 A): pistas suben ≤ 9 °C; Q1 ≈ 57 °C sin disipador.
- Corriente continua total máxima recomendada: **6 A** (suma de ramas + TB7). Picos de ~10 A solo por segundos; si hay más de 6 A sostenidos, disipador clip TO-220 en Q1.
- Fusibles 6 A / 3 A / 3 A: 17–28 % de su nominal en uso normal y ≤ 70 % en pico.

Estimación teórica, no medida.
