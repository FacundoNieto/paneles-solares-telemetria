# Proyecto de Telemetría y Control de Paneles Solares - ESP32

Este repositorio contiene el firmware desarrollado para un sistema de telemetría y control de paneles solares fotovoltaicos, utilizando un microcontrolador ESP32 y el sistema operativo en tiempo real FreeRTOS. Este nodo embebido forma parte de un sistema distribuido de adquisición de datos y actuación remota, comunicado mediante MQTT con una Raspberry Pi que actúa como servidor central.

## Características Principales

### 1. Conectividad

* Conexión Wi-Fi persistente.
* Cliente MQTT con reconexión automática y re-suscripción a tópicos.
* Publicación con retención de datos.
* Mecanismo Last Will para detección de caídas.

### 2. Adquisición y Publicación de Datos

* **Corriente continua (CC):** Sensor ACS712, lectura analógica en GPIO 36.
* **Tensión continua (CC):** Divisor resistivo en GPIO 39.
* **Corriente alterna (CA):** Sensor PZEM-004T por UART (GPIO 16/17).
* **Brújula digital:** Magnetómetro QMC5883L por I2C (GPIO 21/22).
* Cálculo de potencia CC: V × I.

### 3. Control de Posicionamiento

* Servomotor MG996R controlado mediante PWM (LEDC canal 0, 50Hz, 10 bits).
* **Modo automático:** Control proporcional (P) basado en el error de azimut.
* **Modo manual:** Control por valor recibido MQTT, escalado al rango de duty.
* Movimiento progresivo por pasos de ±1 para mayor estabilidad mecánica.

## Arquitectura del Sistema

El firmware se organiza mediante tareas FreeRTOS:

| Tarea               | Funciones principales                                            | Periodicidad |
| ------------------- | ---------------------------------------------------------------- | ------------ |
| `tareaMqttLoop`     | Mantenimiento de conexión MQTT y ejecución de loop MQTT          | 10 ms        |
| `tareaLeerSensores` | Adquisición de sensores CC, CA, brújula y cálculo de potencia    | 1 s          |
| `tareaRevisarPos`   | Evaluación de error angular y ajuste de servo (modo auto/manual) | 150 ms       |

Las variables compartidas entre tareas se protegen mediante semáforos tipo mutex para evitar condiciones de carrera.

## Requisitos de Hardware

* ESP32 DevKit v1 (o similar)
* Sensor ACS712 (corriente CC)
* Divisor resistivo (tensión CC)
* Sensor PZEM-004T (parámetros CA)
* Magnetómetro QMC5883L
* Servomotor MG996R
* Conversores de nivel lógico (5V a 3.3V y viceversa)

## Dependencias (Arduino IDE)

* `PubSubClient` (MQTT)
* `Wire` (I2C)
* `MechaQMC5883` (brújula digital)
* `WiFi` (conexión a red local)
* FreeRTOS incluido por defecto en ESP32

## Licencia

Este proyecto se desarrolla con fines educativos en el marco de un proyecto final de Ingeniería Electrónica - UTN Facultad Regional La Rioja.

---

Para más detalles técnicos, consultar el \[informe final] o el código fuente en `prototipo_esp32.ino`.
