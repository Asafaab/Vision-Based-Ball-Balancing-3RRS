# Verificación PCB v13 contra la guía L414 e IPC-2221

| Requisito | Guía / norma | v13 |
|---|---|---|
| Capas | Una cara (bottom) | Solo B.Cu, 0 vías |
| Área de corte | <= 8 x 7 cm (placa del lab 7x10) | 66.1 x 57.9 mm |
| Ancho de pista | >= 0.6 mm | mínimo 0.6 mm |
| Separación | >= 0.8 mm | DRC a 0.8 mm: 0 violaciones, 0 pads sin conectar |
| Taladros | 0.3–1.2 mm | 0.8 (10), 1.0 (24), 1.1 (15), 1.2 (22) |
| Cuadrante / archivos | Primer cuadrante; .GBL + .DRL + .GKO en zip con nombre | Origen abajo-izquierda; Carranza_Mefy_PCB_v13.zip |
| Ángulos | Pistas a 45° | 0 segmentos fuera de 0/45/90; 0 quiebres ni uniones en T a 90° (1 bifurcación en Y a 135°) |
| Cobre al borde | IPC-2221 | >= 1.0 mm |
| Anillo anular | IPC-2221 >= 0.15 mm (sin metalizar) | mínimo 0.30 mm |
| Alivios térmicos | IPC-2221 | Rayos 1.2 mm en pads pequeños; conexión directa en borneras, condensadores, TVS, J13 y J26 |
| Esquemático vs PCB | — | Netlist del esquemático KiCad = netlist de la PCB (17 redes, 0 diferencias) |

Corriente: ver termico/Analisis_termico.md (6 A continuos totales; picos de ~10 A por segundos).
