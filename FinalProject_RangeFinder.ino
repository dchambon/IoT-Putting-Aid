#include <M5StickCPlus.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    M5.Lcd.println("ESPNow Init Success");
  }
  else {
    M5.Lcd.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  M5.Lcd.println("");
  if (scanResults == 0) {
    // M5.Lcd.println("No WiFi devices in AP Mode found");
  } else {
    // M5.Lcd.print("Found "); M5.Lcd.print(scanResults); M5.Lcd.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        M5.Lcd.print(i + 1);
        M5.Lcd.print(": ");
        M5.Lcd.print(SSID);
        M5.Lcd.print(" (");
        M5.Lcd.print(RSSI);
        M5.Lcd.print(")");
        M5.Lcd.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        M5.Lcd.println("Found Blake.");
        // M5.Lcd.print(i + 1); M5.Lcd.print(": "); M5.Lcd.print(SSID); M5.Lcd.print(" ["); M5.Lcd.print(BSSIDstr); M5.Lcd.print("]"); M5.Lcd.print(" ("); M5.Lcd.print(RSSI); M5.Lcd.print(")"); M5.Lcd.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    // M5.Lcd.println("Slave Found, processing..");
  } else {
    // M5.Lcd.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    // M5.Lcd.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if ( exists) {
      // Slave already paired.
      // M5.Lcd.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        // M5.Lcd.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        M5.Lcd.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        M5.Lcd.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        M5.Lcd.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        M5.Lcd.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        M5.Lcd.println("Peer Exists");
        return true;
      } else {
        M5.Lcd.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    M5.Lcd.println("Blake not found");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  M5.Lcd.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    M5.Lcd.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    M5.Lcd.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    M5.Lcd.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    M5.Lcd.println("Peer not found.");
  } else {
    M5.Lcd.println("Not sure what happened");
  }
}

uint8_t data = 99;
// send data
void sendData() {
  // data++;
  const uint8_t *peer_addr = slave.peer_addr;
  // M5.Lcd.print("Sending: "); Serial.println(data);
  esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
  // M5.Lcd.print("Send Status: ");
  if (result == ESP_OK) {
    // M5.Lcd.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    M5.Lcd.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    M5.Lcd.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    M5.Lcd.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    M5.Lcd.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    M5.Lcd.println("Peer not found.");
  } else {
    M5.Lcd.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // M5.Lcd.print("Last Packet Sent to: "); M5.Lcd.println(macStr);
  // M5.Lcd.print("Last Packet Send Status: "); M5.Lcd.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// ToF hat
VL53L0X sensorHat = VL53L0X();

// Declaring & Initializing variable(s)
unsigned long lastRead = 0;

void setup() {
  M5.begin();
  Wire.begin(0, 26, 400000UL);

  M5.Lcd.println("Script has started...");

  // Sensor working?
  if (sensorHat.init())
    {
      M5.Lcd.println("Sensor Works!");      
    }
    else if (!sensorHat.init())
    {
      M5.Lcd.println("Sensor Failed!");
    }

    sensorHat.startContinuous();

  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  // Init ESPNow with a fallback logic
  InitESPNow();
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  M5.update();

  // Reads distance and stores in dist
  uint16_t distMM = sensorHat.readRangeContinuousMillimeters();
  float distFT = distMM / 304.8;

  // // Prints distances in mm and ft
  // M5.Lcd.print("Latest Dist: ");
  // // M5.Lcd.print(distMM);
  // // M5.Lcd.print(" mm; ");
  // M5.Lcd.print(distFT);
  // M5.Lcd.println(" ft");  

  ScanForSlave();

  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired) {
      // pair success or already paired
      // Send data to device

      if (distFT >= 0 && distFT <= 3){            // Short putt
        M5.Lcd.println("Short");
        data = 0;
        sendData();
      } 
      else if (distFT > 3 && distFT <= 6){      // Long putt
        M5.Lcd.println("Long");
        data = 1;
        sendData();
      } 
      else {                                    // Out of Range
      // Need to identify what error code will be
        M5.Lcd.println("Out of Range");
        data = 69;
        sendData();
      }
    } else {
      // slave pair failed
      M5.Lcd.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }

  // wait for 3 seconds to run the logic again
  delay(3000);
}
