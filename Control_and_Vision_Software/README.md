# Ball Balancer 3RRR con compensación de pendiente

La Raspberry Pi 5 lee el MPU6050 por I²C y compensa el pitch del chasis con los tres servos del mecanismo. El PID V1 sigue usando la cámara para centrar la pelota. Su salida define una normal de la plataforma respecto de la vertical y se transforma al sistema de coordenadas del chasis antes de calcular la cinemática inversa. Los comandos del myRIO y las tramas `POS`, `ESTADO` y `ACK` siguen usando el UART. En el programa del myRIO/LabVIEW, desactiva la salida del servo de la plataforma auxiliar y cualquier lógica que espere tramas `IMU`. El programa del myRIO no viene en este ZIP.

## Montaje y puesta en marcha

1. Fija la base del 3RRR al chasis o a un soporte rígido solidario con el chasis. Fija también el MPU6050 al **chasis**. La placa superior del 3RRR se mueve y su pitch daría una medición distinta de la pendiente. Orienta el MPU para que el cabeceo longitudinal se mida como pitch y para que el eje Z dé aproximadamente +1 g en horizontal.
2. Mantén el carro inmóvil al ejecutar el programa. Se calibra el giroscopio al inicio y el acelerómetro conserva la referencia gravitatoria, incluso si el arranque se hace sobre la rampa. `CMD:RESET` reinicia el PID y el setpoint sin redefinir el pitch cero.
3. Para comprobar el montaje, ejecuta primero con el carro inmóvil y sin pelota. El control se puede iniciar desde el myRIO o con `--auto-start`. Eleva el frente unos grados. La superficie debe girar en sentido contrario al chasis. Si gira en el mismo sentido, repite con `--pitch-azimut 180`. El valor predeterminado de 0° supone que un pitch positivo del IMU inclina la normal del chasis hacia +X de la cinemática. Si los ejes están girados en planta, ajusta el azimut a esa dirección.
4. Con el chasis horizontal, lee `imu_pitch_deg` en el CSV. Si tiene un sesgo estable, pasa ese mismo valor a `--pitch-cero`. Por ejemplo, para una lectura de −0.8° usa `--pitch-cero -0.8`.
5. Una vez confirmado el sentido y verificado el recorrido con el mecanismo real, prueba con la pelota en horizontal y después a 5°, 10° y 15°. Compara el error de posición y el recorte de PID con el carro quieto y durante la subida.

```sh
sudo python3 main.py --pid v1 --auto-start --dur 20 --log prueba_pitch.csv
```

Para usar el arranque normal desde el myRIO se omite `--auto-start`. `--uart-sim` permite comprobar el control sin conectar el myRIO. `--no-imu` permite repetir la prueba usando solo la cámara. `--imu-sim` simula un MPU6050 horizontal. La cámara y el PWM requieren el hardware y la configuración original en la Pi. Para el MPU6050 real se necesita `smbus2` o `smbus`.

## Límites y registro

- El filtro complementario usa 0.7 Hz para la corrección del acelerómetro. El pitch de compensación está acotado a ±15° y cambia como máximo a 45°/s. La aceleración longitudinal del carro puede sesgar la estimación durante el arranque y el frenado.
- La inclinación total respecto al chasis se limita a 20° a la altura original de 8 cm. Si se llega al límite, se conserva el pitch de compensación y se recorta la corrección del PID en su misma dirección. A 15° de pendiente quedan 5° de corrección en la dirección más exigente. El rango se comprobó con el modelo y los límites angulares del código. El recorrido físico, el par y el balance durante la subida requieren ensayo.
- Si se pierde la pelota, la corrección de cámara decae en 0.12 s y se sigue nivelando con el pitch. Si la IMU deja de actualizar durante 0.25 s, su aporte vuelve gradualmente a cero y el control de cámara puede seguir. Con el control en espera, el mecanismo vuelve a su postura neutra respecto al chasis.
- En el CSV, `theta_deg` y `phi_deg` son la corrección de cámara antes de combinarla o recortarla. `pitch_comp_deg` es el pitch del chasis usado en la transformación inversa. `theta_rel_deg` y `phi_rel_deg` son la orientación de la normal enviada a cinemática, relativa al chasis. `pid_recortado`, `imu_valida` y `pelota_detectada` ayudan a interpretar la prueba. Cuando no hay pelota, la posición y el error quedan vacíos.
