#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
// Set your new MAC Address
uint8_t newMACAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x20};
//uint8_t recieverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t baseMac[6];


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
   Serial.printf("Received message from :");
   printMacAddress(recv_info->src_addr);
   Serial.print("Rssi Is :");
   Serial.println(recv_info->rx_ctrl->rssi);
    Serial.print("Noise floor :");
   Serial.println(recv_info->rx_ctrl->noise_floor);
   Serial.print("timestamp :");
   Serial.println(recv_info->rx_ctrl->timestamp);
  // Send Debug log message to the serial port
  Serial.printf("Received message is %s\n", buffer);
 
}


void sentCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
 Serial.print("Last Packet Sent to: ");
 printMacAddress(mac_addr);
 Serial.print("Last Packet Send Status: ");
 Serial.println(status == ESP_NOW_SEND_SUCCESS  ? "Succeeded" : "Failed");
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
  if (result == ESP_OK)
  {
    Serial.println("Broadcast message success");
  }
  else 
   {
      Serial.println("Broadcast failed: " + String(result));
    }

}

void setup(){
  
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);

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
  }

}
 
void loop(){
  
 for (int i = 0;; i++) {
    if (i >= 1000) {
        i = 0;
    }
    String x = String(i);
    Serial.print("X = ");
    Serial.println(x);
    Serial.print("I = ");
    Serial.println(i);
    broadCast(x);
    delay(500);
}



}