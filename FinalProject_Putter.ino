#include <M5StickCPlus.h>
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

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

void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    // M5.Lcd.println("AP Config failed.");
  } else {
    // M5.Lcd.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    // M5.Lcd.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

float accX = 0.0;
float accY = 0.0;
float accZ = 0.0;

const int servo_pin = 26;
int freq            = 50;
int ledChannel      = 0;
int resolution      = 10;

#define SAMPLE_COUNT 3000 // This is the number of readings that will be stored
#define BUFFER_SIZE SAMPLE_COUNT // Creates a buffer the size of the sample

float bufferX[BUFFER_SIZE]; // The variable that will store the swing data

void setup() { // Initiate the M5Stick
  M5.begin();
  M5.Imu.Init();
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(servo_pin, ledChannel);
  ledcWrite(ledChannel, 256); 
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // Init ESPNow with a fallback logic
  InitESPNow();
}

int x; // Maybe Remove

#define UPPER_LIMIT 4000 // Upper limit for short putt
#define LOWER_LIMIT 3000 // Lower limit for short putt, evaluated as negative

int collecting_idx = 0; // begin at the first index of collection
float complete = 0; // tracker to denote completed swing
int distance = 99; // 0 is short, 1 is long

#define MIN(a,b) (((a)<(b))?(a):(b)) // defining MIN argument
#define MAX(a,b) (((a)>(b))?(a):(b)) // defining MAX argument

float maxX = 0, minX = 0; // variables to capture min and max acceleration

void evaluate_sample(){ // function to set max and min acceleration values for the swing

  for(int i = 0; i < SAMPLE_COUNT; i++){ // iterate through the buffer and find the max and min values
    maxX = MAX(maxX, bufferX[i]);
    minX = MIN(minX, bufferX[i]);
  }

}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // M5.Lcd.print("Last Packet Recv from: "); M5.Lcd.println(macStr);
  // M5.Lcd.print("Putt Type: "); M5.Lcd.println(*data);
  // M5.Lcd.println("");

  if (int(*data) == 0){
    distance = 0;
  }
  else if (int(*data) == 1){
    distance = 1;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  complete = 0;
  maxX = 0;
  minX = 0;
  collecting_idx = 0; // set the index to 0 for our buffer


  if (M5.BtnA.wasPressed()) { // when the button is pressed...
    
    if(complete == 0){ // begin the loop of collecting data
      esp_now_register_recv_cb(OnDataRecv);

      if(distance == 0){
        M5.Lcd.println("Target: Short");
      }
      else if(distance == 1){
        M5.Lcd.println("Target: Long");
      }
      else{
        M5.Lcd.println("Error: No Distance");
        M5.update();
        return;
      }

      ledcWriteTone(ledChannel, 1000);
      delay(1000);
      ledcWriteTone(ledChannel, 0);

      delay(1000);

      ledcWriteTone(ledChannel, 1000);
      delay(1000);
      ledcWriteTone(ledChannel, 0);

      delay(1000);

      ledcWriteTone(ledChannel, 1000);
      delay(1000);
      ledcWriteTone(ledChannel, 0);

      while(collecting_idx < SAMPLE_COUNT){ // while the mode is collecting, gather data until buffer is full
        M5.IMU.getAccelData(&accX,&accY,&accZ);
        x = (int)(accX * 1000.0f);

        if(x < 100 && x > -100)
          x = 0;

        bufferX[collecting_idx] = x;
        collecting_idx++;
        delay(1);
      }

      evaluate_sample();

      if(maxX < UPPER_LIMIT && fabs(minX) < LOWER_LIMIT){
        M5.Lcd.println(" SHORT");
        M5.Lcd.println(maxX);
        M5.Lcd.println(minX);
        complete++;
        if(distance == 0){
          M5.Lcd.println(" Nice Putt, Tiger!");
          ledcWriteTone(ledChannel, 500);
          delay(500);
          ledcWriteTone(ledChannel, 1000);
          delay(500);
          ledcWriteTone(ledChannel, 0);
          distance = 99;
        }
        else{
          M5.Lcd.println(" You're Bad!");
          ledcWriteTone(ledChannel, 1000);
          delay(500);
          ledcWriteTone(ledChannel, 500);
          delay(500);
          ledcWriteTone(ledChannel, 0);
          distance = 99;
        }        
      }
      else{
        M5.Lcd.println(" LONG");
        M5.Lcd.println(maxX);
        M5.Lcd.println(minX);
        complete++;
        if(distance == 0){
          M5.Lcd.println(" Get A New Hobby!");
          ledcWriteTone(ledChannel, 1000);
          delay(500);
          ledcWriteTone(ledChannel, 500);
          delay(500);
          ledcWriteTone(ledChannel, 0);
          distance = 99;
        }
        else{
          M5.Lcd.println(" Ball Went Home!");
          ledcWriteTone(ledChannel, 500);
          delay(500);
          ledcWriteTone(ledChannel, 1000);
          delay(500);
          ledcWriteTone(ledChannel, 0);
          distance = 99;
        }
      }
      
    }
    
  }
  M5.update();
}