#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "NMEA.h"
#include <math.h>
#include <ArduinoJson.h>


#define RXD2 16
#define TXD2 17

// Variables de datos
double latitude;
double longitude;
double totalDistance = 0;
double lastLatitude = 0;
double lastLongitude = 0;

NMEA gps(GPRMC);

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* serverURL = "https://trailsync-h4cpdje8dza6esed.brazilsouth-01.azurewebsites.net/api/v1/boots/touristId=1/serviceId=1/bootId=1"; // Cambia esto por la URL de tu backend

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // GPS
  Wire.begin();  // I2C

  // Conectar a Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {

  // Leer y calcular datos GPS
  readGPSData();

  // Enviar datos al backend
  sendDataToBackend(latitude, longitude, totalDistance);

  delay(1000);  // Espera antes de la siguiente lectura
}

// Función para leer y calcular datos GPS
void readGPSData() {
  while (Serial2.available()) {
    char serialData = Serial2.read();
    if (gps.decode(serialData)) {
      if (gps.gprmc_status() == 'A') {
        latitude = gps.gprmc_latitude();
        longitude = gps.gprmc_longitude();

        // Calcular la distancia total
        if (lastLatitude != 0 && lastLongitude != 0) {
          totalDistance += haversineDistance(lastLatitude, lastLongitude, latitude, longitude);
        }
        lastLatitude = latitude;
        lastLongitude = longitude;

        Serial.print("Latitude: ");
        Serial.println(latitude, 8);
        Serial.print("Longitude: ");
        Serial.println(longitude, 8);
        Serial.print("Total Distance: ");
        Serial.print(totalDistance);
        Serial.println(" km");
      }
    }
  }
}

// Función para calcular la distancia entre dos puntos GPS
double haversineDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371.0;
  float dLat = (lat2 - lat1) * PI / 180.0;
  float dLon = (lon2 - lon1) * PI / 180.0;
  lat1 = lat1 * PI / 180.0;
  lat2 = lat2 * PI / 180.0;
  float a = sin(dLat / 2) * sin(dLat / 2) +
            sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

String getBootData() {
  HTTPClient http;
  http.begin("https://trailsync-h4cpdje8dza6esed.brazilsouth-01.azurewebsites.net/api/v1/boots/1");

  int httpResponseCode = http.GET();
  String response = "";

  if (httpResponseCode == 200) {
    response = http.getString();
    Serial.println("Datos obtenidos del servidor:");
    Serial.println(response);
  } else {
    Serial.print("Error al obtener datos. Código: ");
    Serial.println(httpResponseCode);
  }

  http.end();
  return response;
}

// Función para enviar datos al backend
void sendDataToBackend(double latitude, double longitude, double distance) {
  if (WiFi.status() == WL_CONNECTED) {
    String currentData = getBootData();
    if (currentData != "") {
      // Crear un documento JSON
      DynamicJsonDocument doc(2048);
      DeserializationError error = deserializeJson(doc, currentData);
      if (error) {
        Serial.print("Error al deserializar JSON: ");
        Serial.println(error.c_str());
        return;
      }

      // Actualizar solo los campos necesarios
      doc["latitude"] = latitude;
      doc["longitude"] = longitude;
      doc["distance"] = distance;

      // Serializar el documento actualizado
      String jsonData;
      serializeJson(doc, jsonData);

      Serial.println("JSON actualizado para enviar:");
      Serial.println(jsonData);

      // Enviar datos actualizados al servidor
      HTTPClient http;
      http.begin("https://trailsync-h4cpdje8dza6esed.brazilsouth-01.azurewebsites.net/api/v1/boots/touristId=1/serviceId=1/bootId=1");
      http.addHeader("Content-Type", "application/json");

      int httpResponseCode = http.PUT(jsonData);

      if (httpResponseCode == 200) {
        Serial.println("Actualizado con éxito en el servidor.");
      } else {
        Serial.print("Error en la solicitud PUT. Código de respuesta HTTP: ");
        Serial.println(httpResponseCode);
      }

      http.end();
    }
  } else {
    Serial.println("Not connected to WiFi");
  }
}


int sendPUTRequest(const char* serverURL, const char* jsonData) {
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.PUT(jsonData);

  if (httpResponseCode == -1) {
    Serial.print("Error: ");
    Serial.println(http.errorToString(httpResponseCode).c_str());
  }

  http.end();
  return httpResponseCode;
}