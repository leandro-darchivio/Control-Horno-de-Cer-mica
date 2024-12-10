# Horno de Cerámica con Control PID

Este proyecto consiste en la implementación de un controlador para un horno de cerámica, utilizando un Arduino. Incluye control de temperatura mediante termopares, un display LCD para la interfaz de usuario, y almacenamiento de datos en una tarjeta microSD.

## Características

- Control de temperatura mediante PID.
- Compatible con hasta 3 zonas de calentamiento.
- Pantalla LCD para visualización y configuración.
- Almacenamiento de datos en tarjeta microSD.
- Botones de navegación para la selección de programas y ajustes en tiempo real.
- Seguridad integrada para apagado en caso de sobretemperatura.

## Requisitos de Hardware

- Arduino (compatible con la biblioteca `PID` y la librería para LCD).
- Termopares con placa MAX6675.
- Pantalla LCD I2C (20x4).
- Tarjeta microSD y módulo lector.
- Relés para controlar los elementos calefactores.
- Botones físicos para la interfaz de usuario.

### Pines Conectados

| Componente          | Pines en Arduino       |
|----------------------|------------------------|
| Termopares (MAX6675)| 5 (SCK), 6 (CS), 7 (SO)|
| Botón Subir         | 2                      |
| Botón Bajar         | 3                      |
| Botón Seleccionar   | 4                      |
| Relé Calefactor     | 9                      |
| Módulo SD           | 10 (CS), 11 (MOSI), 12 (MISO), 13 (SCK) |
| LCD I2C             | I2C (A4: SDA, A5: SCL) |


## Uso

1. **Selección de Programa:**
   - Usa los botones "Subir" y "Bajar" para elegir el programa de cocción desde la memoria SD.
   - Presiona "Seleccionar" para iniciar.

2. **Monitoreo:**
   - La pantalla LCD muestra la temperatura actual, el programa seleccionado y el estado del horno.

3. **Apagado de Emergencia:**
   - Si se detecta una temperatura superior a 1600 °C, el horno se apagará automáticamente para proteger los componentes.

## Notas Importantes

- **Seguridad:** El sistema apagará los calefactores si se alcanza la temperatura máxima configurada (`maxTemp`).
- **Memoria SD:** Utiliza una fuente de alimentación externa para garantizar que la tarjeta SD sea reconocida correctamente.

