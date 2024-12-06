ahi estudian esto: #include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ArduinoJson.h>

// Configuración de WiFi
const char* ssid = "NEXTF1384";
const char* password = "next108d";

// Configuración del servidor MQTT
const char* mqtt_server = "34.194.171.140";
const int mqtt_port = 1883;
const char* mqtt_user = "guest";
const char* mqtt_password = "guest";
const char* mqtt_topic = "mqtt";

// Definiciones para los sensores
#define DHTPIN 13
#define DHTTYPE DHT22
#define MQ135_AMONIACO_PIN 36
#define MQ135_CO2_PIN 39
#define PHPIN 34
#define RELAY_PIN 12

// Definición de los límites según la NOM
#define LIMITE_AMONIACO 25
#define LIMITE_CO2 1000

// Estructura para almacenar datos de sensores
struct SensorData {
  float humidity;
  float temperature;
  float ammonia;
  float co2;
  float ph;
  bool phValid;
};

// Variables globales compartidas
SensorData sensorData;
portMUTEX_TYPE sensorMutex;
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// Prototipos de funciones
void TaskWiFiMQTT(void *pvParameters);
void TaskSensorReading(void *pvParameters);
void TaskRelayControl(void *pvParameters);
void setup_wifi();
void reconnect();
float leerConcentracionGas(int pin, String tipoGas);
float leerPh();

void setup() {
  Serial.begin(9600);
  
  // Inicializar mutex para proteger datos compartidos
  sensorMutex = xSemaphoreCreateMutex();
  
  // Configuración de pines
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  dht.begin();
  
  // Crear tareas
  xTaskCreatePinnedToCore(
    TaskWiFiMQTT,
    "WiFiMQTT",
    4096,
    NULL,
    1,
    NULL,
    0
  );
  
  xTaskCreatePinnedToCore(
    TaskSensorReading,
    "SensorReading",
    4096,
    NULL,
    2,
    NULL,
    1
  );
  
  xTaskCreatePinnedToCore(
    TaskRelayControl,
    "RelayControl",
    2048,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  // El loop principal queda vacío ya que todo se maneja en las tareas
  vTaskDelete(NULL);
}

// Tarea para manejar WiFi y MQTT
void TaskWiFiMQTT(void *pvParameters) {
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  
  for(;;) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    // Crear y enviar mensaje JSON con los datos de los sensores
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      StaticJsonDocument<256> doc;
      doc["humedad"] = sensorData.humidity;
      doc["temperatura"] = sensorData.temperature;
      doc["amoniaco"] = sensorData.ammonia;
      doc["co2"] = sensorData.co2;
      
      if(sensorData.phValid) {
        doc["ph"] = sensorData.ph;
      }
      
      char buffer[256];
      size_t n = serializeJson(doc, buffer);
      client.publish(mqtt_topic, buffer, n);
      
      xSemaphoreGive(sensorMutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10000)); // Enviar datos cada 10 segundos
  }
}

// Tarea para leer sensores
void TaskSensorReading(void *pvParameters) {
  for(;;) {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    float ammonia = leerConcentracionGas(MQ135_AMONIACO_PIN, "Amoníaco");
    float co2 = leerConcentracionGas(MQ135_CO2_PIN, "Dióxido de Carbono");
    float ph = leerPh();
    
    // Actualizar datos protegidos por mutex
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      sensorData.humidity = humidity;
      sensorData.temperature = temperature;
      sensorData.ammonia = ammonia;
      sensorData.co2 = co2;
      sensorData.ph = ph;
      sensorData.phValid = (ph != -1);
      
      xSemaphoreGive(sensorMutex);
    }
    
    // Verificar alertas
    if(ammonia > LIMITE_AMONIACO) {
      Serial.printf("¡Alerta! Niveles altos de Amoníaco: %.2f ppm\n", ammonia);
    }
    
    if(co2 > LIMITE_CO2) {
      Serial.printf("¡Alerta! Niveles altos de CO2: %.2f ppm\n", co2);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10000)); // Leer sensores cada 10 segundos
  }
}

// Tarea para control del relé
void TaskRelayControl(void *pvParameters) {
  bool relayState = false;
  
  for(;;) {
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState);
    Serial.printf("Relé %s\n", relayState ? "encendido" : "apagado");
    
    vTaskDelay(pdMS_TO_TICKS(300000)); // Cambiar estado cada 5 minutos
  }
}

// Funciones auxiliares
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.printf("Conectando a %s\n", ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.printf("Dirección IP: %s\n", WiFi.localIP().toString().c_str());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conectar al servidor MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("conectado");
    } else {
      Serial.printf("falló, rc=%d intentando de nuevo en 5 segundos\n", client.state());
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

float leerConcentracionGas(int pin, String tipoGas) {
  int valorAnalogico = analogRead(pin);
  float voltaje = valorAnalogico * (5.0 / 4095.0);
  float concentracionPPM;
  float Ro = 10;

  if (tipoGas == "Amoníaco") {
    concentracionPPM = (voltaje / (5 - voltaje)) * Ro * 1.5;
  } else {
    concentracionPPM = (voltaje / (5 - voltaje)) * Ro * 1.0;
  }
  
  return concentracionPPM;
}

float leerPh() {
  int valorAnalogico = analogRead(PHPIN);
  float phValue = (valorAnalogico * 3.3) / 4095.0 * 10.0;

  if (phValue > 20.85) {
    float phSimulado = 4.8 + (float(rand()) / RAND_MAX) * (5.5 - 4.8);
    return round(phSimulado * 10) / 10.0;
  }
  return -1;
}
