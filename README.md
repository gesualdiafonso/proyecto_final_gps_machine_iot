# 🛰️ ESP32 GPS Tracker con Supabase y Detección de Movimiento

Este proyecto implementa un sistema de rastreo para dispositivos IoT utilizando un módulo ESP32 y un receptor GPS Neo-6M. El dispositivo es capaz de detectar si está estático o en movimiento y sincronizar su ubicación, estado y metadatos con un servicio de backend en la nube (Supabase) a través de HTTPS.

## 🛠️ Componentes Principales

| Componente                 | Descripción                                                                 |
|---------------------------|-----------------------------------------------------------------------------|
| **Microcontrolador ESP32** | Gestiona Wi-Fi, comunicación serial, GPIO y el protocolo HTTP.               |
| **Módulo GPS Neo-6M**       | Obtiene coordenadas, velocidad y tiempo.                                     |
| **Servicio Cloud (Supabase)** | Función Edge de Supabase que actúa como API para recibir y almacenar datos. |
| **Indicadores LED**         | LEDs de estado (Rojo, Amarillo, Verde) indican el estado del dispositivo.    |

## ⚙️ Configuración y Dependencias

Este proyecto requiere las siguientes librerías:

- **TinyGPSPlus.h**: Para procesar y decodificar mensajes NMEA del módulo GPS.
- **WiFi.h**: Para gestionar la conexión Wi-Fi del ESP32.
- **WiFiClientSecure.h**: Necesaria para conexiones HTTPS seguras.
- **HTTPClient.h**: Para realizar solicitudes HTTP/HTTPS (GET, POST, PUT) hacia la API de Supabase.

## 📝 Parámetros Críticos (Archivo `.ino`)

Antes de compilar, modifica las siguientes constantes en la sección `// ================== CONFIG ==================` del código:

| Constante               | Descripción                                                       | Ejemplo                        |
|-------------------------|-------------------------------------------------------------------|--------------------------------|
| **WIFI_SSID**            | Nombre de tu red Wi-Fi                                            | `"Mi_Casa_WiFi"`               |
| **WIFI_PASS**            | Contraseña de tu red Wi-Fi                                        | `"password123"`                |
| **SUPABASE_URL**         | URL del endpoint de la Edge Function (API)                        | `https://[ref].supabase.co/functions/v1/tracker` |
| **SUPABASE_API_KEY**     | Llave API anónima de Supabase para autenticación.                | `eyJhbGciOi...`                |
| **DEVICE_ID**            | Identificador único para este dispositivo rastreador.             | `"CUTETAG_004"`                |

## 🧭 Lógica de Funcionamiento

El sistema opera en un ciclo continuo de detección, control de estado y sincronización:

### 1. Detección de Estado (`updateStateMachine`)

El dispositivo utiliza los datos del GPS para determinar uno de los cuatro estados posibles:

| Estado                    | Condición                                                                                               | LED |
|---------------------------|-----------------------------------------------------------------------------------------------------------|-----|
| **STATE_ALL_FAIL_BLINK**   | No hay fix GPS válido y nunca se ha obtenido uno previamente.                                            | Rojo parpadeando |
| **STATE_GPS_FAIL_MORSE**   | El fix GPS es obsoleto o inválido, pero anteriormente sí había uno.                                      | Rojo fijo, Amarillo en SOS (Morse) |
| **STATE_GPS_OK_STATIC**    | GPS válido, velocidad < 1.5 km/h y distancia recorrida < 3 metros.                                       | Verde fijo, Amarillo fijo |
| **STATE_GPS_OK_MOVING**    | GPS válido y el dispositivo está en movimiento (velocidad o distancia por encima del umbral).            | Verde fijo, Amarillo parpadeando |

### 2. Detección de Movimiento (`detectMovement`)

El dispositivo se considera en movimiento si se cumple alguno de los siguientes criterios dentro del intervalo (`MOVEMENT_WINDOW_MS`):

- La velocidad reportada por el GPS es ≥ 1.5 km/h.
- La distancia calculada entre la posición previa (`prevFix`) y la actual (`lastFix`) es ≥ 3 metros usando la fórmula Haversine.

### 3. Sincronización con la Nube (`sendToCloudIfNeeded`)

La comunicación con Supabase se maneja de la siguiente forma:

- **POST (Creación)**: Si es la primera vez que se envían datos (`recordCreated = false`), se crea el registro inicial.
- **PUT (Actualización)**: Si el registro ya existe (`recordCreated = true`), se actualizan la latitud, longitud y el estado actual (STATIC o MOVING).

#### Intervalos de Envío:

- **Intervalo Mínimo**: El envío se bloquea si no han pasado 2.5 segundos desde el último envío (`SEND_MIN_INTERVAL`).
- **Intervalos según Estado**:
  - **MOVING_INTERVAL_MS**: 10 segundos (rastreo continuo).
  - **STATIC_INTERVAL_MS**: 15 segundos (ahorro de energía cuando está estático).

## 🔌 Diagrama de Conexiones (Neo-6M → ESP32)

Conecta el módulo GPS al puerto serial secundario del ESP32 (`SerialGPS(2)`):

| Módulo GPS (Neo-6M) | ESP32                  | Función                                      |
|---------------------|------------------------|----------------------------------------------|
| **VCC**             | **3.3V / 5V**          | Alimentación                                 |
| **GND**             | **GND**                | Tierra                                       |
| **TX**              | **PIN 16 (GPS_RX_PIN)**| Recepción de datos NMEA                      |
| **RX**              | **PIN 17 (GPS_TX_PIN)**| Transmisión de datos hacia el GPS            |

---

## 🛠️ Cómo Ejecutar el Proyecto

1. Clona este repositorio.  
2. Abre el archivo `.ino` en Arduino IDE o en tu IDE preferida.  
3. Configura las constantes según lo indicado anteriormente.  
4. Conecta el ESP32 a tu computadora y sube el código.  
5. El dispositivo comenzará a rastrear su ubicación y enviar los datos a Supabase.

---

## 📄 Licencia

Este proyecto está licenciado bajo la [Licencia MIT](LICENSE).
