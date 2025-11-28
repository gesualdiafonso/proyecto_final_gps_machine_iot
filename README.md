# 🛰️ ESP32 GPS Tracker com Supabase e Detecção de Movimento

Este projeto implementa um sistema de rastreamento para dispositivos IoT utilizando um módulo ESP32 e um receptor GPS Neo-6M. O dispositivo é capaz de detectar se está estático ou em movimento e sincronizar sua localização, estado e metadados com um serviço de backend em nuvem (Supabase) por meio de HTTPS.

## 🛠️ Componentes Principais

| Componente       | Descrição                                                                 |
|------------------|---------------------------------------------------------------------------|
| **Microcontrolador ESP32** | Gerencia Wi-Fi, comunicação serial, GPIO e protocolo HTTP.             |
| **Módulo GPS Neo-6M**     | Obtém coordenadas, velocidade e tempo.                                  |
| **Serviço Cloud (Supabase)** | Função Edge do Supabase para atuar como API de recepção e armazenamento de dados. |
| **Indicadores LED**       | LEDs de estado (Vermelho, Amarelo, Verde) indicam o status do dispositivo. |

## ⚙️ Configuração e Dependências

Este projeto requer as seguintes bibliotecas:

- **TinyGPSPlus.h**: Para processamento e decodificação das mensagens NMEA do módulo GPS.
- **WiFi.h**: Para gerenciar a conexão Wi-Fi do ESP32.
- **WiFiClientSecure.h**: Necessário para estabelecer conexões HTTPS seguras.
- **HTTPClient.h**: Para realizar requisições HTTP/HTTPS (GET, POST, PUT) para a API do Supabase.

## 📝 Parâmetros Críticos (Arquivo `.ino`)

Antes de compilar, modifique as seguintes constantes na seção `// ================== CONFIG ==================` do código:

| Constante               | Descrição                                                      | Exemplo                        |
|-------------------------|----------------------------------------------------------------|--------------------------------|
| **WIFI_SSID**            | Nome da sua rede Wi-Fi                                         | `"Mi_Casa_WiFi"`               |
| **WIFI_PASS**            | Senha da sua rede Wi-Fi                                        | `"password123"`                |
| **SUPABASE_URL**         | URL do endpoint da Função Edge (API)                           | `https://[ref].supabase.co/functions/v1/tracker` |
| **SUPABASE_API_KEY**     | Chave API anônima do Supabase para autenticação.               | `eyJhbGciOi...`                |
| **DEVICE_ID**            | Identificador único para este dispositivo rastreador.          | `"CUTETAG_004"`                |

## 🧭 Lógica de Funcionamento

O sistema opera em um loop contínuo de detecção, controle de estado e sincronização:

### 1. Deteção de Estado (`updateStateMachine`)

O dispositivo usa os dados GPS para determinar um dos quatro estados possíveis:

| Estado                      | Condição                                                                                          | LED |
|-----------------------------|---------------------------------------------------------------------------------------------------|-----|
| **STATE_ALL_FAIL_BLINK**     | Sem fix GPS válido e nunca teve um fix.                                                           | Vermelho piscando |
| **STATE_GPS_FAIL_MORSE**     | Fix GPS obsoleto ou inválido, mas já teve um fix antes.                                           | Vermelho fixo, Amarelo em SOS (Morse) |
| **STATE_GPS_OK_STATIC**      | GPS OK e a velocidade é menor que 1,5 km/h e a distância percorrida é menor que 3 metros.        | Verde fixo, Amarelo fixo |
| **STATE_GPS_OK_MOVING**      | GPS OK e o dispositivo está em movimento, com velocidade ou distância acima do limite.           | Verde fixo, Amarelo piscando |

### 2. Detecção de Movimento (`detectMovement`)

O dispositivo é considerado em movimento se um dos seguintes critérios for atendido no intervalo de tempo (`MOVEMENT_WINDOW_MS`):

- A velocidade reportada pelo GPS é maior ou igual a 1,5 km/h.
- A distância calculada entre a posição anterior (`prevFix`) e a atual (`lastFix`) é maior ou igual a 3 metros, usando a fórmula Haversine.

### 3. Sincronização com a Nuvem (`sendToCloudIfNeeded`)

A comunicação com o Supabase é feita da seguinte maneira:

- **POST (Criação)**: Se for a primeira vez que os dados são enviados (variável `recordCreated = false`), um POST é realizado para criar o registro inicial do dispositivo.
- **PUT (Atualização)**: Se o registro já existir (variável `recordCreated = true`), um PUT é feito para atualizar a latitude, longitude e o estado atual (estático ou em movimento).

#### Intervalos de Envio:

- **Intervalo Mínimo**: O envio é bloqueado se não se passaram 2,5 segundos desde o último envio (`SEND_MIN_INTERVAL`).
- **Intervalo de Estado**: Se não houver mudanças significativas de posição/estado, o envio será feito com intervalos mais longos:
  - **MOVING_INTERVAL_MS**: 10 segundos (para rastreamento contínuo).
  - **STATIC_INTERVAL_MS**: 15 segundos (para economizar recursos quando está parado).

## 🔌 Diagrama de Conexões (Neo-6M para ESP32)

Certifique-se de conectar o módulo GPS ao porto serial secundário do ESP32 (`SerialGPS(2)`):

| Módulo GPS (Neo-6M) | ESP32                 | Função                                      |
|---------------------|-----------------------|---------------------------------------------|
| **VCC**             | **3.3V / 5V**          | Alimentação                                 |
| **GND**             | **GND**                | Terra                                       |
| **TX**              | **PIN 16 (GPS_RX_PIN)**| Recepção de dados NMEA do GPS               |
| **RX**              | **PIN 17 (GPS_TX_PIN)**| Transmissão de dados ao GPS                 |

---

## 🛠️ Como Rodar o Projeto

1. Clone este repositório.
2. Abra o arquivo `.ino` no Arduino IDE ou qualquer outra IDE de sua preferência.
3. Altere as constantes de configuração conforme mencionado acima.
4. Conecte o ESP32 à sua máquina e faça o upload do código.
5. O dispositivo começará a rastrear sua localização e a enviar os dados para o Supabase.

---

## 📄 Licença

Este projeto está licenciado sob a [Licença MIT](LICENSE).
