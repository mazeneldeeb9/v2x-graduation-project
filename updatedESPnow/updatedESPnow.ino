#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>


// Set your new MAC Address
uint8_t newMACAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x2E, 0x20};
//uint8_t recieverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t baseMac[6];


TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use Serial1 for GPS module

// Structure to hold GPS data
typedef struct struct_message {
    float latitude;
    float longitude;
} struct_message;

struct_message myData;

void printMacAddress(const uint8_t *mac) {
  Serial.printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


void readMacAddress(){
  
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    printMacAddress(baseMac);
  } else {
    Serial.println("Failed to read MAC address");
  }
}
void receiveCallback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int dataLen)
// Called when data is received
{
  // Only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);
  strncpy(buffer, (const char *)data, msgLen);
 
  // Make sure we are null terminated
  buffer[msgLen] = 0;
 
  // Format the MAC address
  Serial.println("---------------------------------------------------------");
   Serial.printf("Received message from :");
   printMacAddress(recv_info->src_addr);
  //  Serial.print("Rssi Is :");
  //  Serial.println(recv_info->rx_ctrl->rssi);
  //   Serial.print("Noise floor :");
  //  Serial.println(recv_info->rx_ctrl->noise_floor);
  //  Serial.print("timestamp :");
  //  Serial.println(recv_info->rx_ctrl->timestamp);
  // Send Debug log message to the serial port
  //Serial.printf("Received message is %s\n", buffer);
    Serial.print("Received message is : ");
    float latitude = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
    float longitude = (buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7];
    
    latitude /= 1000000;
    longitude /= 1000000;
    Serial.print("Latitude : ");
    Serial.print(latitude, 6);
    Serial.print(", Longitude : ");
    Serial.println(longitude, 6);
     Serial.println("---------------------------------------------------------");
 
 char espMaC[18];
   sprintf(espMaC, "%02X:%02X:%02X:%02X:%02X:%02X",
   baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  //  Serial.print("ESP32's MAC address (espMacStr): ");
  //  Serial.println(espMaC);

   char srcMacStr[18];
    strncpy(srcMacStr, buffer, 17); 
    srcMacStr[17] = '\0'; 
    //  Serial.print("Extracted MAC from message (srcMacStr): ");
    // Serial.println(srcMacStr);


if (strlen(srcMacStr) == 17 && srcMacStr[2] == ':' && srcMacStr[5] == ':' && srcMacStr[8] == ':' && srcMacStr[11] == ':' && srcMacStr[14] == ':'){
    if (strcmp(srcMacStr, espMaC) == 0) {
      Serial.println("Message is from this ESP, skipping rebroadcast.");
      return;
           }
      }else{
  Serial.println(" no MAC address in message or invalid, BroadCast.");
  char srcMac [25];
  sprintf(srcMac, "%02X:%02X:%02X:%02X:%02X:%02X",
            recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
            recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
  String message_with_mac = String (srcMac) + " : " + String(buffer);
  broadCast(message_with_mac);
  delay(2000);
  return;
}
}
  



void sentCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
 {
//  Serial.print("Last Packet Sent to: ");
//  printMacAddress(mac_addr);
//  Serial.print("Last Packet Send Status: ");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS  ? "Succeeded" : "Failed");
}

void broadCast(const String &message)
{
esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    //peerInfo.lmk=
    
  if (!esp_now_is_peer_exist(broadcastAddress))
  {
    esp_now_add_peer(&peerInfo);
  }
  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());
 
  // Print results to serial monitor
  // if (result == ESP_OK)
  // {
  //   Serial.println("Broadcast message success");
  // }
  // else 
  //  {
  //     Serial.println("Broadcast failed: " + String(result));
  //   }

}

void setup(){
  
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX for GPS module

  WiFi.mode(WIFI_STA);
   esp_wifi_set_protocol( WIFI_IF_STA , WIFI_PROTOCOL_LR);
Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
    esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  if (err == ESP_OK) {
    Serial.println("Success changing Mac Address");
  }
  Serial.print("[NEW] ESP32 Board MAC Address: ");
  readMacAddress();
  //-----------------------------
   WiFi.disconnect();
 // Initialize ESP-NOW
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESP-NOW Init Success");
    esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sentCallback);
  }
  else
  {
    Serial.println("ESP-NOW Init Failed");
    delay(3000);
    ESP.restart();
  }}
 void loop() {
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        //Serial.write(c); // Print incoming data for visibility
        gps.encode(c);
    }
    if (gps.location.isUpdated()) {
        Serial.println("---------------------------------------------------------");
        //broadCast("omar");
        myData.latitude = gps.location.lat();
        myData.longitude = gps.location.lng();
        Serial.print("Sent Message : Latitude : ");
        Serial.print(myData.latitude, 6);
        Serial.print(" Longitude : ");
        Serial.println(myData.longitude, 6);
        Serial.println("---------------------------------------------------------");
        
        // String y = String(myData.latitude) + " : " + String(myData.longitude);
        // broadCast(y); // Send GPS data via ESP-NOW
        char lat[4], lon[4];
        long temp = myData.latitude * 1000000;
        lat[0] = temp >> 24;
        lat[1] = (temp >> 16) & 0xFF;
        lat[2] = (temp >> 8) & 0xFF;
        lat[3] = temp & 0xFF;
        temp = myData.longitude * 1000000;
        lon[0] = temp >> 24;
        lon[1] = (temp >> 16) & 0xFF;
        lon[2] = (temp >> 8) & 0xFF;
        lon[3] = temp & 0xFF;

        String message = String(lat[0]) + String(lat[1]) + String(lat[2]) + String(lat[3]) + String(lon[0]) + String(lon[1]) + String(lon[2]) + String(lon[3]);

        broadCast(message);

        delay(1000);

   }
    

//      for (int i = 0;; i++) {
//     if (i >= 1000) {
//         i = 0;
//     }
//     String x = String(i);
//     Serial.print("X = ");
//     Serial.println(x);
//     Serial.print("I = ");
//     Serial.println(i);
//     broadCast(x);
//     delay(500);
// }
// broadCast("gemyyyyy");
// delay(2000);
}
