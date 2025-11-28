|/* gps_state_machine.ino — CORREÇÃO SSL E PARSER JSON
  ==================================================
  - El sistema está encargado de criar apenas 1 registro por device_id (POST) y despues actualizar (PUT)
  - Envio de datos a BackEnd por medio de supabase, por lo tanto al HTTP enviamos el señar de wifi
  - Se crea un anti-flood: evitando PUTs desnecesarios (check de cambios + interlavos mínimo)
  - Mantiene toda la lógica de GPS por librería de la TinyGPS++, detección de movimineto en el GPS
  - Utilización de LEDs para envio de comunicación FAIL, MOVING, STATIC y creación de Morses.
  - Implementación de GET inicial (cloudCheckExisting) antes del primero POST para sincronización por recordCreated y obtener con cloudRecord.
  - Delay interno de 2000ms en el POST/PUT (EdgeRutime y EarlyDrop del Supabase)
  - Cambio del intervalo: estático --> 15s | movimiento 10s
*/

#include <TinyGPSPlus.h>       // Incluye la biblioteca para proceso de datos de un módulo GPS (como el Neo-6M).
#include <WiFi.h>              // Incluye la biblioteca para gerenciar la conexión Wi-Fi del ESP32.
#include <WiFiClientSecure.h>  // Importante para HTTPS (Incluye la biblioteca para conexión segura, esenciales para HTTPS y Supabase).
#include <HTTPClient.h>        // Incluye la biblioteca para hacer requisiciones HTTP e HTTPS.

// ================== CONFIG ==================

#define DEBUG 1 // Define la constante DEBUG como 1 (habilitado).
#if DEBUG
  #define DBG Serial // Se DEBUG for verdadero, define DBG como 'Serial' para usar la puerta serial para depuración.
#endif

// WiFi config
const char* WIFI_SSID = "SU_WIFI"; // Define o nombre de la rede Wi-Fi (SSID).
const char* WIFI_PASS = "SU_CONTRASENA";

// Supabase config
// Configuración de Supabase (Servicio de Backend como Servicio)
// URL del endpoint de la función 'tracker' de Supabase (servidor/API al que se enviarán los datos).
const char* SUPABASE_URL = "https://vifurugyrmprstcmceol.supabase.co/functions/v1/tracker";
// Clave API 'Anon' de Supabase (se utiliza para la autenticación y el acceso a la API).
const char* SUPABASE_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InZpZnVydWd5cm1wcnN0Y21jZW9sIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NjIwMjUzODAsImV4cCI6MjA3NzYwMTM4MH0.fizbcG4NWTCWkVZIbPV0OWpJffEyr_7Z4IAmTbBHJTw"; 
const char* DEVICE_ID = "CUTETAG_004"; // Identificador único para este dispositivo de rastreo.

// Pines de Hardware
// Pines para comunicación con el módulo GPS Neo-6M
const int GPS_RX_PIN = 16; // Pin del ESP32 conectado al TX del GPS (Recepción de datos del GPS).
const int GPS_TX_PIN = 17; // Pin del ESP32 conectado al RX del GPS (Transmisión al GPS, aunque generalmente no se usa).
// Pines de los LEDs de estado
const uint8_t PIN_RED = 22;    // Pin conectado al LED rojo (generalmente indica fallo o estado crítico).
const uint8_t PIN_GREEN = 23;  // Pin conectado al LED verde (generalmente indica estado OK o en movimiento).
const uint8_t PIN_YELLOW = 21; // Pin conectado al LED amarillo (generalmente indica estado estático o esperando).

// Tiempos (Timings)
// Umbral de velocidad en km/h: por encima de esto, se considera que el dispositivo está 'MOVING' (en movimiento).
const float SPEED_THRESHOLD_KMPH = 1.5;
// Umbral de distancia en metros: si la última posición enviada está a menos de esta distancia de la actual, no se envía.
const float DIST_THRESHOLD_METERS = 3.0;
// Ventana de tiempo en milisegundos para evaluar si el dispositivo está 'MOVING' o 'STATIC'.
const unsigned long MOVEMENT_WINDOW_MS = 5000;
// Intervalo mínimo en milisegundos entre dos intentos de envío de datos a la nube.
const unsigned long SEND_MIN_INTERVAL = 2500;
// Intervalo de envío en milisegundos cuando el dispositivo se considera 'MOVING' (en movimiento).
const unsigned long MOVING_INTERVAL_MS = 10000;
// Intervalo de envío en milisegundos cuando el dispositivo se considera 'STATIC' (estático).
const unsigned long STATIC_INTERVAL_MS = 15000;
// Periodo de parpadeo del LED amarillo en milisegundos (tiempo total de encendido + apagado para un ciclo).
const unsigned long YELLOW_BLINK_PERIOD_MS = 333;
// Periodo de parpadeo de los LEDs en estado de fallo total ('ALL_FAIL_BLINK') en milisegundos.
const unsigned long ALL_FAIL_BLINK_MS = 700;
// Tiempo en milisegundos que los datos del GPS se consideran válidos antes de ser 'stale' (obsoletos/no actualizados).
const unsigned long GPS_STALE_MS = 4000;

// Configuración de Código Morse
const unsigned int DOT_MS = 200; // Duración en milisegundos de un punto (símbolo corto) en Morse.
const unsigned int DASH_MS = DOT_MS * 3; // Duración en milisegundos de una raya (símbolo largo) en Morse (3 veces el punto).
const unsigned int SYMBOL_GAP_MS = DOT_MS; // Pausa en milisegundos entre puntos y rayas dentro de la misma letra.
const unsigned int WORD_GAP_MS = DOT_MS * 7; // Pausa en milisegundos entre palabras (o el equivalente para la señal SOS).
const char *MORSE_SOS = "...---..."; // Secuencia de puntos y rayas para la señal de auxilio "SOS".

// Objetos
TinyGPSPlus gps; // Objeto de la clase TinyGPSPlus para procesar los datos NMEA del GPS.
HardwareSerial SerialGPS(2); // Objeto de la clase HardwareSerial (puerto serial 2 del ESP32) para la comunicación con el GPS.

// Estructura para almacenar una posición GPS (Fix)
struct Fix {
  double lat, lng; // Latitud y longitud.
  unsigned long t; // Marca de tiempo (timestamp) de la lectura.
  bool valid; // Indica si la lectura del GPS es válida.
};
Fix lastFix = {0,0,0,false}; // Almacena la última posición GPS válida recibida. Inicializada a valores nulos/no válidos.
Fix prevFix = {0,0,0,false}; // Almacena la posición GPS anterior para el cálculo de movimiento/velocidad. Inicializada a valores nulos/no válidos.

// Enumeración de los posibles estados de funcionamiento del sistema
enum SystemState {
  STATE_ALL_FAIL_BLINK, // Fallo total (sin WiFi y GPS)
  STATE_GPS_FAIL_MORSE, // Estado de fallo GPS (conexión wifi puede exisitir)
  STATE_GPS_OK_STATIC, // GPS OK y dispositivo está estatico
  STATE_GPS_OK_MOVING // Estado GPS OK y Dispositivo en movimiento
};
SystemState currentState = STATE_ALL_FAIL_BLINK; // Inicializa el sistema en el estado de fallo total.

// Controle
/* 
    Variables de controll de timepo
    - Almacenamiento del último momento en que se precesaron los datos del GPS
    - Almacenamiento del ultimo mommento en que se cambió el estaod del Led Amarillo
    - Almacenamiento del ultimo momento que se envió un paquete de datos a la nube
    - indicardor booleano para el estado del LED Amarillo (true = encedido)
*/

unsigned long lastGPSProcess = 0;
unsigned long lastYellowToggle = 0;
unsigned long lastSend = 0;
bool yellowOn = false;

// Cloud State (Estado de la Nube)
/* 
    Variables de controll de la nube
    - Indica si ya se creo un registrio inicial a la nube
    - Almacenamiento del id registrado del rastreo
    - Almacenamiento de la Latitude del útimo Fic enviado y Longitud del ultimo fix enviado a la nube
    - Almacena el último estado de movimiento ('STATIC' O 'MOVING') enviado a la nube
*/
bool recordCreated = false;
String cloudRecordId = "";
double lastSentLat = 0;
double lastSentLng = 0;
String lastSentState = "";

// ================== FUNCOES AUXILIARES ==================
/*
 * Calcular la distancia de gran círculo (la ruta más corta sobre la superficie de la esfera) entre
 * dos coordenadas GPS (latitud y longitud) utilizando la **Fórmula del Haversine**.
 * El resultado debe estar en **metros**, basándose en el radio terrestre (R = 6371000.0 metros).
 *
 * Parámetros: Recibe la latitud y longitud del punto 1 (lat1, lon1) y del punto 2 (lat2, lon2).
 * Retorna: La distancia en metros (double). Si los puntos son idénticos, retorna 0.0.
 */
double haversine(double lat1, double lon1, double lat2, double lon2) {
  if (lat1 == lat2 && lon1 == lon2) return 0.0;
  const double R = 6371000.0;
  double dLat = (lat2-lat1) * (M_PI/180.0);
  double dLon = (lon2-lon1) * (M_PI/180.0);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(lat1*(M_PI/180.0))*cos(lat2*(M_PI/180.0)) *
             sin(dLon/2)*sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R*c;
}

/*
 * Verificar si los datos GPS actuales son **válidos y recientes**.
 * Se considera OK si:
 * 1. La librería `gps` reporta que la ubicación es válida (`gps.location.isValid()` es true).
 * 2. La antigüedad del Fix de la ubicación (`gps.location.age()`) no supera el umbral de obsolescencia
 * definido por `GPS_STALE_MS`.
 * Retorna: `true` si el GPS tiene un Fix válido y reciente; `false` en caso contrario.
 */
bool gpsOk() {
  if (!gps.location.isValid()) return false;
  if (gps.location.age() > GPS_STALE_MS) return false;
  return true;
}

/*
 * Leer todos los caracteres disponibles en el puerto serial del GPS (`SerialGPS`)
 * y pasarlos a la librería `TinyGPSPlus` (`gps.encode()`) para su procesamiento
 * y la extracción de datos de navegación (latitud, longitud, velocidad, etc.).
 * Esta función debe ser llamada repetidamente en el bucle principal (`loop`) para asegurar
 * la continua recepción y actualización de los datos del GPS.
 * Retorna: Nada (void).
 */
void feedGPS() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }
}
/*
 * Actualizar las estructuras de datos que almacenan la posición GPS actual (`lastFix`)
 * y la posición GPS anterior (`prevFix`), lo cual es crucial para el cálculo de movimiento.
 *
 * 1. Si no hay un Fix GPS válido y reciente (ver `gpsOk`), la función termina.
 * 2. Si es el primer Fix válido, inicializa `lastFix`.
 * 3. Si ha transcurrido un tiempo igual o mayor a `MOVEMENT_WINDOW_MS` desde la última
 * actualización de `lastFix`, entonces:
 * a. Mueve el valor actual de `lastFix` a `prevFix`.
 * b. Actualiza `lastFix` con las nuevas coordenadas y la marca de tiempo actual (`millis()`).
 * Retorna: Nada (void).
 */
void updateFix() {
  if (!gpsOk()) return;
  double lat = gps.location.lat();
  double lng = gps.location.lng();
  unsigned long now = millis();

  if (!lastFix.valid) {
    lastFix = {lat, lng, now, true};
    #if DEBUG
      DBG.println("[GPS] Primeiro fix");
    #endif
    return;
  }
  if (now - lastFix.t >= MOVEMENT_WINDOW_MS) {
    prevFix = lastFix;
    lastFix = {lat, lng, now, true};
    #if DEBUG
      DBG.println("[GPS] Fix atualizado");
    #endif
  }
}

/*
 * Determinar si el dispositivo está en movimiento ('MOVING') basándose en dos criterios:
 * 1. **Criterio de velocidad del GPS**: Si la velocidad reportada por el GPS (`gps.speed.kmph()`)
 * es válida y es igual o superior a `SPEED_THRESHOLD_KMPH`.
 * 2. **Criterio de distancia Haversine**: Si tanto `prevFix` como `lastFix` son válidos, calcula la
 * distancia entre ellos. Si esta distancia es igual o superior a `DIST_THRESHOLD_METERS`,
 * indica movimiento.
 * Retorna: `true` si se detecta movimiento por cualquiera de los dos criterios; `false` en caso contrario.
 */
bool detectMovement() {
  if (gps.speed.isValid() && gps.speed.kmph() >= SPEED_THRESHOLD_KMPH) return true;
  if (prevFix.valid && lastFix.valid) {
    double d = haversine(prevFix.lat, prevFix.lng, lastFix.lat, lastFix.lng);
    if (d >= DIST_THRESHOLD_METERS) return true;
  }
  return false;
}

// ================ MORSE & LEDS ================
/*
 * Implementar la secuencia de **Código Morse SOS (`...---...`)** de manera no bloqueante
 * (utilizando `millis()` y variables estáticas para controlar el tiempo y el estado).
 *
 * 1. **Control de tiempo:** Utiliza `until` para saber cuándo debe ocurrir la próxima acción (cambio de estado del LED).
 * 2. **Reproducción de símbolos:**
 * - Cuando no está `active`, verifica el siguiente símbolo (`. ` o `-`) en `MORSE_SOS`.
 * - Enciende el LED amarillo (`PIN_YELLOW`) y establece `until` para la duración del punto (`DOT_MS`) o la raya (`DASH_MS`). Establece `active = true`.
 * 3. **Pausas:**
 * - Cuando está `active`, apaga el LED amarillo y establece `until` para la duración de la pausa entre símbolos (`SYMBOL_GAP_MS`). Establece `active = false` y avanza al siguiente símbolo (`idx++`).
 * 4. **Fin de la secuencia:** Si se llega al final de `MORSE_SOS`, reinicia el índice (`idx = 0`), apaga el LED amarillo y establece la pausa larga entre palabras (`WORD_GAP_MS`).
 *
 * El LED amarillo se utiliza para emitir la señal SOS.
 * Retorna: Nada (void).
 */
void runMorse() {
  static size_t idx = 0;
  static bool active = false;
  static unsigned long until = 0;
  unsigned long now = millis();
  if (until == 0) { until = now; return; }
  if (now < until) return;

  if (!active) {
    if (idx >= strlen(MORSE_SOS)) {
      idx = 0; until = now + WORD_GAP_MS; digitalWrite(PIN_YELLOW, LOW); return;
    }
    char s = MORSE_SOS[idx];
    digitalWrite(PIN_YELLOW, HIGH);
    until = now + ((s=='.') ? DOT_MS : DASH_MS);
    active = true;
  } else {
    digitalWrite(PIN_YELLOW, LOW);
    until = now + SYMBOL_GAP_MS;
    active = false;
    idx++;
  }
}

/*
 * Lógica: Controlar el estado de los LEDs (Rojo, Verde, Amarillo) basándose en el
 * `currentState` del sistema.
 *
 * Utiliza un `switch` para aplicar la configuración de LEDs específica para cada estado:
 *
 * 1. STATE_GPS_OK_STATIC (GPS OK, Estático): Verde FIJO (HIGH), Amarillo FIJO (HIGH), Rojo APAGADO (LOW).
 * 2. STATE_GPS_OK_MOVING (GPS OK, Movimiento): Verde FIJO (HIGH), Rojo APAGADO (LOW). El Amarillo
 * debe parpadear de forma no bloqueante con el periodo `YELLOW_BLINK_PERIOD_MS`.
 * 3. STATE_GPS_FAIL_MORSE (Fallo de GPS): Verde APAGADO (LOW), Rojo FIJO (HIGH). Llama a `runMorse()`
 * para que el Amarillo emita la secuencia SOS.
 * 4. STATE_ALL_FAIL_BLINK (Fallo Total): Verde APAGADO (LOW), Amarillo APAGADO (LOW). El Rojo debe
 * parpadear de forma no bloqueante con el periodo `ALL_FAIL_BLINK_MS`.
 *
 * Retorna: Nada (void).
 */
void updateLEDs() {
  unsigned long now = millis();
  switch (currentState) {
    case STATE_GPS_OK_STATIC:
      digitalWrite(PIN_GREEN, HIGH); digitalWrite(PIN_YELLOW, HIGH); digitalWrite(PIN_RED, LOW);
      break;
    case STATE_GPS_OK_MOVING:
      digitalWrite(PIN_GREEN, HIGH); digitalWrite(PIN_RED, LOW);
      if (now - lastYellowToggle >= YELLOW_BLINK_PERIOD_MS) {
        yellowOn = !yellowOn; digitalWrite(PIN_YELLOW, yellowOn); lastYellowToggle = now;
      }
      break;
    case STATE_GPS_FAIL_MORSE:
      digitalWrite(PIN_GREEN, LOW); digitalWrite(PIN_RED, HIGH); runMorse();
      break;
    case STATE_ALL_FAIL_BLINK:
      digitalWrite(PIN_GREEN, LOW); digitalWrite(PIN_YELLOW, LOW);
      if (now - lastYellowToggle >= ALL_FAIL_BLINK_MS) {
        yellowOn = !yellowOn; digitalWrite(PIN_RED, yellowOn); lastYellowToggle = now;
      }
      break;
  }
}

// ================== WIFI ==================
/*
 * Inicializar y establecer la conexión Wi-Fi del ESP32 a la red definida.
 *
 * 1. Llama a `WiFi.begin()` usando el SSID (`WIFI_SSID`) y la contraseña (`WIFI_PASS`) configurados.
 * 2. Entra en un bucle que espera hasta que el estado de la conexión sea `WL_CONNECTED` (Conectado).
 * 3. Mientras espera, imprime puntos (`.`) en la consola de depuración (si DEBUG está activado) para
 * indicar que el proceso está en curso.
 * 4. Incluye un mecanismo de **tiempo de espera** (timeout) que reinicia la cuenta cada 30 segundos
 * (aunque el bucle sigue siendo infinito si la conexión nunca se establece).
 * 5. Una vez conectado, imprime la dirección IP local del ESP32 en la consola de depuración.
 *
 * Retorna: Nada (void).
 */
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  #if DEBUG
    DBG.print("[WIFI] Conectando");
  #endif
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    #if DEBUG
      DBG.print(".");
    #endif
    if (millis() - start > 30000) start = millis();
  }
  #if DEBUG
    DBG.println("\n[WIFI] Conectado: " + WiFi.localIP().toString());
  #endif
}

// ===================== CLOUD SYNC =====================
// Encargado de Gestionar las comunicaciones HTTP/HTTPS (GET, POST, PUT) con el servicio de backend Supabase
// (Edge Function) para mantener actualizado el registro de seguimiento del dispositivo.

/*
 * Vamos Extraer el valor del campo "id" de una cadena JSON.
 * Esta función debe ser capaz de manejar dos formatos comunes de respuesta JSON:
 * 1. Un objeto simple: `{"id": "valor_del_id"}`.
 * 2. Un array con un objeto: `[{"id": "valor_del_id", ...}]`.
 *
 * 1. Busca la cadena `"id"`.
 * 2. Busca los caracteres de inicio del valor, ignorando espacios y comillas (`:` y `"`).
 * 3. Extrae la subcadena que representa el ID (que consiste en caracteres alfanuméricos y guiones).
 *
 * Parámetros: `json` (La cadena JSON de la que se extraerá el ID).
 * Retorna: El ID extraído como una `String`. Retorna una cadena vacía (`""`) si no se encuentra el ID.
 */
// Extrai ID de JSON (aceita tanto objeto {"id":"..."} quanto array [{"id":"..."}])
String extractIdFromJson(const String& json) {
  int idPos = json.indexOf("\"id\"");
  if (idPos < 0) return "";
  int colonPos = json.indexOf(":", idPos);
  if (colonPos < 0) return "";
  
  int i = colonPos + 1;
  // Avanza más allá de ':' y cualquier espacio o comilla de apertura
  while (i < json.length() && (json[i] == ' ' || json[i] == '\"')) i++;
  int start = i;
  // Avanza mientras encuentra caracteres alfanuméricos o guiones (formato común de IDs de Supabase/UUID)
  while (i < json.length() && (isalnum(json[i]) || json[i] == '-')) i++;

  String id = json.substring(start, i);
  return id;
}

// GET inicial
/*
 * Realizar una solicitud GET inicial a la Edge Function de Supabase para verificar si
 * ya existe un registro de seguimiento para el `DEVICE_ID` actual.
 *
 * 1. Configura un cliente seguro (`WiFiClientSecure`) e ignora la validación del certificado (`client.setInsecure()`)
 * para evitar problemas comunes de SSL/TLS en el ESP32.
 * 2. Construye la URL de la consulta, incluyendo el `device_id` como parámetro de consulta.
 * 3. Agrega los *headers* necesarios (`apikey` y `Content-Type`).
 * 4. Envía la solicitud GET.
 * 5. Si el código de respuesta es `200 (OK)` y el contenido tiene un ID válido:
 * a. Extrae el ID usando `extractIdFromJson`.
 * b. Actualiza las variables de control globales `cloudRecordId` y `recordCreated`.
 *
 * Retorna: `true` si se encontró un registro existente y se extrajo el ID; `false` en caso contrario.
 */
bool cloudCheckExisting() {
  // CLIENTE SEGURO para HTTPS
  WiFiClientSecure client;
  client.setInsecure(); // Ignora validación de certificado (Importante para evitar errores en ESP32 con HTTPS)
  
  HTTPClient http;
  String url = String(SUPABASE_URL) + "?device_id=" + DEVICE_ID; // URL con parámetro de consulta para filtrar

  if (!http.begin(client, url)) {
    DBG.println("[GET] Falha ao iniciar conexao HTTPS");
    return false;
  }

  http.addHeader("apikey", SUPABASE_API_KEY);
  http.addHeader("Content-Type", "application/json");

  DBG.println("[GET] Buscando registro...");
  int code = http.GET();
  
  if (code == 200) {
    String payload = http.getString();
    // Verifica si no está vacío y si contiene un campo "id"    if (payload.length() > 5 && payload.indexOf("\"id\"") > 0) {
      cloudRecordId = extractIdFromJson(payload);
      recordCreated = true;
      DBG.println("[GET] Registro encontrado ID: " + cloudRecordId);
      http.end();
      return true;
    } else {
       DBG.println("[GET] Resposta vazia/sem ID (sera criado novo).");
    }
  } else {
    DBG.println("[GET] Nao encontrado ou erro: " + String(code));
  }
  
  recordCreated = false;
  http.end();
  return false;
}

// POST
/*
 * Realizar una solicitud POST a la Edge Function de Supabase para crear un nuevo registro
 * de seguimiento en la base de datos.
 *
 * 1. Configura el cliente HTTPS.
 * 2. Configura los *headers* de autenticación (`apikey` y `Authorization: Bearer`).
 * 3. Construye el cuerpo JSON con los datos actuales del Fix (`lat`, `lng`, `ts`, `state`) y el `mode: "POST"`.
 * 4. Envía la solicitud POST.
 * 5. Si la respuesta es exitosa (`200` o `201`), extrae el ID del nuevo registro y actualiza
 * `cloudRecordId` y `recordCreated`.
 *
 * Parámetros: `lat`, `lng` (coordenadas), `ts` (timestamp), `state` ('STATIC' o 'MOVING').
 * Retorna: `true` si el POST fue exitoso y el registro fue creado/aceptado; `false` en caso contrario.
 */
bool cloudPOST(double lat, double lng, unsigned long ts, const char* state) {
  WiFiClientSecure client;
  client.setInsecure(); // HTTPS fix
  
  HTTPClient http;
  if (!http.begin(client, SUPABASE_URL)) return false;

  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_API_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_API_KEY); // Requerido por algunas Edge Functions

  // Construcción del cuerpo JSON para la función Edge
  String body = "{";
  body += "\"mode\":\"POST\","; // Indica a la Edge Function que debe crear un registro
  body += "\"device_id\":\"" + String(DEVICE_ID) + "\","; 
  body += "\"lat\":" + String(lat, 7) + ",";
  body += "\"lng\":" + String(lng, 7) + ",";
  body += "\"timestamp\":" + String(ts) + ",";
  body += "\"state\":\"" + String(state) + "\"";
  body += "}";

  DBG.println("[POST] Enviando...");
  int code = http.POST(body);
  String payload = http.getString();
  http.end(); // Cierra conexión para liberar recursos y evitar overhead

  delay(2000); // Pausa de cortesía/desaceleración

  if (code == 200 || code == 201) {
    String id = extractIdFromJson(payload);
    if (id.length() > 0) cloudRecordId = id;
    recordCreated = true;
    DBG.println("[POST] Sucesso (" + String(code) + ")");
    return true;
  }

  DBG.println("[POST] Falhou (" + String(code) + "): " + payload);
  return false;
}

// PUT
/*
 * Realizar una solicitud PUT a la Edge Function de Supabase para actualizar un registro existente
 * (identificado por el `DEVICE_ID`).
 *
 * 1. Similar a `cloudPOST`, configura el cliente HTTPS y los *headers* de autenticación.
 * 2. Construye el cuerpo JSON con los datos actualizados (`lat`, `lng`, `ts`, `state`) y el `mode: "PUT"`.
 * 3. Envía la solicitud PUT (método nativo HTTP).
 * 4. Si la respuesta está en el rango de éxito (`200` a `299`), considera la actualización como exitosa.
 *
 * Parámetros: `lat`, `lng` (coordenadas), `ts` (timestamp), `state` ('STATIC' o 'MOVING').
 * Retorna: `true` si el PUT fue exitoso y el registro fue actualizado; `false` en caso contrario.
 */
bool cloudPUT(double lat, double lng, unsigned long ts, const char* state) {
  WiFiClientSecure client;
  client.setInsecure(); // HTTPS fix
  
  HTTPClient http;
  if (!http.begin(client, SUPABASE_URL)) return false;

  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_API_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_API_KEY);
  
  // NAO usamos header Override, usamos metodo PUT direto para alinhar com Edge Function
  // http.addHeader("X-HTTP-Method-Override", "PUT"); 

  String body = "{";
  body += "\"mode\":\"PUT\","; // Indica a la Edge Function que debe actualizar un registro
  body += "\"device_id\":\"" + String(DEVICE_ID) + "\",";
  body += "\"lat\":" + String(lat, 7) + ",";
  body += "\"lng\":" + String(lng, 7) + ",";
  body += "\"timestamp\":" + String(ts) + ",";
  body += "\"state\":\"" + String(state) + "\"";
  body += "}";

  DBG.println("[PUT] Atualizando...");
  int code = http.PUT(body); // Método PUT nativo
  String payload = http.getString();
  http.end();

  delay(2000);

  if (code >= 200 && code < 300) {
    DBG.println("[PUT] Sucesso (" + String(code) + ")");
    return true;
  }

  DBG.println("[PUT] Falhou (" + String(code) + "): " + payload);
  return false;
}

/*
 * Sincronizar el estado inicial de la nube.
 * Llama a `cloudCheckExisting()` para determinar si ya existe un registro de seguimiento para este
 * dispositivo en la base de datos de Supabase. Esto solo se hace si el Wi-Fi está conectado.
 * Esta función generalmente se ejecuta una vez al inicio.
 * Retorna: Nada (void).
 */
void syncCloudState() {
  if (WiFi.status() == WL_CONNECTED) {
    cloudCheckExisting();
  }
}

/*
 * Función principal de control que decide si es necesario enviar datos a la nube
 * basándose en el estado actual y los umbrales de tiempo/cambio.
 *
 * Pasos de verificación:
 * 1. Wi-Fi Conectado Debe haber conexión Wi-Fi.
 * 2. Intervalo Mínimo Respeta el intervalo mínimo entre envíos (`SEND_MIN_INTERVAL`).
 * 3. Umbral de Cambio/Tiempo (Si el registro ya existe):
 *  a. Si las coordenadas y el estado no han cambiado significativamente, verifica si el intervalo
 *  específico del estado (`MOVING_INTERVAL_MS` o `STATIC_INTERVAL_MS`) ha transcurrido. Si no ha
 *  transcurrido, no envía.
 * 4. Acción
 *  a. Si `recordCreated` es `false`, llama a `cloudPOST` (crear nuevo registro).
 *  b. Si `recordCreated` es `true`, llama a `cloudPUT` (actualizar registro existente).
 * 5. **Actualización del Estado Local:** Si el envío (`POST` o `PUT`) es exitoso, actualiza las
 * variables `lastSentLat`, `lastSentLng`, `lastSentState` y `lastSend`.
 *
 * Parámetros: `lat`, `lng` (coordenadas), `ts` (timestamp), `state` ('STATIC' o 'MOVING').
 * Retorna: `true` si los datos fueron enviados exitosamente; `false` en caso contrario.
 */
bool sendToCloudIfNeeded(double lat, double lng, unsigned long ts, const char* state) {
  unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED) return false;
  
  // 1. Respetar el intervalo mínimo entre envíos
  if (now - lastSend < SEND_MIN_INTERVAL) return false;

  // 2. Verificar si el cambio es significativo (sólo si el registro ya existe)
  if (recordCreated) {
    double dlat = fabs(lat - lastSentLat);
    double dlng = fabs(lng - lastSentLng);
    // Verificar si la posición y el estado son prácticamente los mismos que el último enviado
    if (dlat < 0.000005 && dlng < 0.000005 && String(state) == lastSentState) {
      unsigned long interval = (currentState == STATE_GPS_OK_MOVING) ? MOVING_INTERVAL_MS : STATIC_INTERVAL_MS;
      // Si no hubo cambio significativo, respetar el intervalo largo (MOVING o STATIC)
      if (now - lastSend < interval) return false;
    }
  }

  // 3. Determinar si se necesita POST o PUT
  bool ok = false;
  if (!recordCreated) {
    ok = cloudPOST(lat, lng, ts, state); // Crear nuevo registro
  } else {
    ok = cloudPUT(lat, lng, ts, state); // Actualizar registro existente
  }

  // 4. Actualizar el estado local si el envío fue exitoso
  if (ok) {
    lastSentLat = lat;
    lastSentLng = lng;
    lastSentState = String(state);
    lastSend = now;
  }
  return ok;
}

// ================== STATE & LOOP ==================
/*
 * Actualizar la variable de estado global `currentState` basándose en el estado actual del GPS.
 * Esta función define el comportamiento general del dispositivo (qué LED encender, qué intervalo de envío usar).
 *
 * 1. **Fallo de GPS:** Si `gpsOk()` es falso:
 * a. Si nunca se ha obtenido un Fix válido (`!lastFix.valid`), el estado es `STATE_ALL_FAIL_BLINK` (Fallo Total).
 * b. Si se tenía un Fix pero ya no es reciente (obsoleto), el estado es `STATE_GPS_FAIL_MORSE` (Fallo de GPS).
 * 2. **GPS OK:** Si `gpsOk()` es verdadero:
 * a. Si `detectMovement()` retorna `true`, el estado es `STATE_GPS_OK_MOVING` (En Movimiento).
 * b. Si `detectMovement()` retorna `false`, el estado es `STATE_GPS_OK_STATIC` (Estático).
 *
 * Retorna: Nada (void). Actualiza la variable global `currentState`.
 */
void updateStateMachine() {
  if (!gpsOk()) {
    // Si no hay datos GPS recientes
    currentState = (!lastFix.valid) ? STATE_ALL_FAIL_BLINK : STATE_GPS_FAIL_MORSE;
    return;
  }
  // Si los datos GPS son OK, determina el movimiento
  currentState = detectMovement() ? STATE_GPS_OK_MOVING : STATE_GPS_OK_STATIC;
}

/*
 * Lógica: Gestionar el envío de datos a Supabase, asegurando que se respeten los intervalos de tiempo
 * y que se envíe el estado y la posición correctos.
 *
 * 1. **Pre-verificación:** Si no hay un Fix GPS válido guardado (`!lastFix.valid`), no hace nada.
 * 2. **Intervalo:** Calcula el intervalo de envío (`MOVING_INTERVAL_MS` o `STATIC_INTERVAL_MS`)
 * según el `currentState` y verifica si ha transcurrido el tiempo necesario desde el último envío (`lastSend`). Si no ha pasado, no envía.
 * 3. **Etiqueta de Estado:** Determina la etiqueta de estado (`"STATIC"`, `"MOVING"`, o `"FAIL"`) que se enviará a la nube.
 * 4. **Envío:** Llama a `sendToCloudIfNeeded()` con los datos de `lastFix` y la etiqueta de estado.
 *
 * Retorna: Nada (void).
 */
void handleCloudSend() {
  if (!lastFix.valid) return; // No enviar nada si no hay posición válida
  unsigned long now = millis();
  // Determina el intervalo de envío (largo si es estático, más corto si se mueve)
  unsigned long interval = (currentState == STATE_GPS_OK_MOVING) ? MOVING_INTERVAL_MS : STATIC_INTERVAL_MS;
  if (now - lastSend < interval) return; // Espera el tiempo necesario

  // Determina la etiqueta de estado a enviar a la nube
  const char* stateLabel = (currentState == STATE_GPS_OK_STATIC) ? "STATIC" :
                           (currentState == STATE_GPS_OK_MOVING) ? "MOVING" : "FAIL";
                           
  // Envía los datos a la nube (la función se encarga de decidir si hace POST o PUT)
  sendToCloudIfNeeded(lastFix.lat, lastFix.lng, lastFix.t, stateLabel);
}

// ----------------------------------------------------

/*
 * Función de inicialización que se ejecuta una sola vez al encender el ESP32.
 *
 * 1. Configuración de Pines: Establece los pines de los LEDs como salidas (`OUTPUT`) y los apaga (`LOW`).
 * 2. Inicialización Serial: Inicia la comunicación serial para depuración (si `DEBUG` está activado).
 * 3. Inicialización GPS: Inicia el puerto serial secundario (`SerialGPS`) con la configuración estándar
 * de un módulo Neo-6M (9600 baudios, 8 bits de datos, sin paridad, 1 bit de parada) y asigna los pines RX/TX.
 * 4. Conexión Wi-Fi: Llama a `initWiFi()` para establecer la conexión a la red.
 * 5. Sincronización de Nube: Llama a `syncCloudState()` para verificar si ya existe un registro de seguimiento en Supabase.
 *
 * Retorna: Nada (void).
 */

void setup() {
  // Configuración y apagado inicial de los pines de los LEDs
  pinMode(PIN_RED, OUTPUT); pinMode(PIN_GREEN, OUTPUT); pinMode(PIN_YELLOW, OUTPUT);
  digitalWrite(PIN_RED, LOW); digitalWrite(PIN_GREEN, LOW); digitalWrite(PIN_YELLOW, LOW);

  #if DEBUG
    Serial.begin(115200);
    DBG.println("[BOOT] Iniciando...");
  #endif

  // Inicialización del puerto serial 2 para la comunicación con el GPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  initWiFi(); // Conecta a la red Wi-Fi
  syncCloudState(); // Verifica si existe un registro en la nube
}

/*
 * Función principal que se ejecuta continuamente después de `setup()`.
 * Está diseñada para ser no bloqueante y manejar todas las tareas de forma periódica.
 *
 * 1. Procesar GPS:** Llama a `feedGPS()` para leer todos los datos NMEA disponibles del módulo GPS.
 * 2. Temporización (200 ms): Las tareas intensivas (actualizar Fix, actualizar Estado y enviar a la nube)
 * solo se ejecutan si han pasado 200 ms desde la última vez (`lastGPSProcess`).
 * a. Si el GPS está OK, actualiza las estructuras de posición (`updateFix`).
 * b. Actualiza el estado del sistema (`updateStateMachine`).
 * c. Controla el envío de datos a la nube (`handleCloudSend`).
 * 3. Actualizar LEDs: Llama a `updateLEDs()` para reflejar el `currentState` mediante los LEDs (se ejecuta en cada iteración del loop).
 * 4. Rendimiento: Llama a `yield()` (permite que el sistema operativo del ESP32 ejecute tareas en segundo plano) y tiene un pequeño `delay(5)` para control del ciclo.
 *
 * Retorna: Nada (void).
 */
void loop() {
  feedGPS(); // Leer datos del GPS continuamente
  unsigned long now = millis();
  // Bloque de ejecución temporal (cada 200ms)
  if (now - lastGPSProcess >= 200) {
    lastGPSProcess = now; 
    if (gpsOk()) updateFix(); // Solo actualiza el Fix si el dato actual es válido
    updateStateMachine(); // Transición de estados (FAIL, STATIC, MOVING)
    handleCloudSend(); // Intenta enviar datos a la nube si es necesario
  }
  updateLEDs(); // Actualiza el estado visual de los LEDs (siempre activo)
  yield(); // Permite que el ESP32 ejecute tareas internas (no bloqueante)
  delay(5); // Pequeña pausa
}