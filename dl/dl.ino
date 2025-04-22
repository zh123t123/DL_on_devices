#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include <OneWire.h>
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);

#define TOTAL_PACKETS 500
RTC_DATA_ATTR int bootCount = 0;
#define uS_TO_MIN_FACTOR 60000000

// Set the MASTER MAC Address
uint8_t masterAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
int packetReceived = 0;
int packetMissed = 0;
int lastReceivedPacketNumber = -1; 
int start_time = 0;
int end_time = 0;
bool SYNCED = false;
bool first_packet_received = false;

int sleep_time = 4970;
int previous_delta = 0;
int previous_rssi = 0;
bool previous_received = true;
float delta_history[3] = {0};
float rssi_history[3] = {0};

wifi_country_t country = {
  .cc = "JP",
  .schan = 1,
  .nchan = 14,
  .max_tx_power = 20,
  .policy = WIFI_COUNTRY_POLICY_AUTO,
};
esp_now_peer_info_t masterInfo;
// Define a data structure
typedef struct struct_message {
  unsigned int sender_id;
  unsigned long time;
  unsigned int packetNumber;
} struct_message;

typedef struct {
  String timestamp;
  int packetNumber;
  bool received;
  float successRate;
  int radioOnTime;
  int data; 
  int length; 
  String madeAdjustment; 
  int newDelay = 4970; 
  int rssi_display;
} PacketLog;

/////////////////////////////////////   RSSI  //////////////////////////////////////

int rssi_display;
int totalRssi = 0;
// Estructuras para calcular los paquetes, el RSSI, etc
typedef struct {
  unsigned frame_ctrl: 16;
  unsigned duration_id: 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl: 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

//La callback que hace la magia
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  int rssi = ppkt->rx_ctrl.rssi;
  rssi_display = rssi;
}

//////////////////////////////////// END RSSI /////////////////////////////////

String formatTimestamp(unsigned long totalMillis) {
  unsigned long totalSeconds = totalMillis / 1000;
  unsigned long totalMinutes = totalSeconds / 60;
  unsigned long totalHours = totalMinutes / 60;

  unsigned long ms = totalMillis % 1000;
  unsigned long seconds = totalSeconds % 60;
  unsigned long minutes = totalMinutes % 60;
  unsigned long hours = totalHours % 24;  // Uncomment if you want the hour to reset every 24 hours

  char timestamp[20];
  sprintf(timestamp, "%02lu:%02lu:%02lu.%03lu", hours, minutes, seconds, ms);

  return String(timestamp);
}

// Create a structured object
PacketLog packetLog;
struct_message myData;

int prev_pkt = 0;
float success_rate = 0;
unsigned long wifiOnTime = 0; 

void printPacketLog() {
  Serial.print(packetLog.timestamp);
  Serial.print(" | ");
  Serial.print(packetLog.packetNumber);
  Serial.print("  | ");
  Serial.print(packetLog.received ? "Yes" : "TIMEOUT");
  Serial.print(" | ");
  Serial.print(packetLog.successRate, 2);  
  Serial.print(" | ");
  Serial.print(packetLog.radioOnTime);
  Serial.print(" ms  | ");
  Serial.print(packetLog.rssi_display);
  Serial.print(" | ");
  Serial.print(packetLog.newDelay);
  //if(packetLog.madeAdjustment != "") {
  //  Serial.print(packetLog.madeAdjustment); 
  //  Serial.print("  | ");
  //  Serial.print(packetLog.newDelay);
  //} else {
  //  Serial.print("N/A | ");  // If no adjustment, print N/A and leave the new delay column blank
  //}
  Serial.println();
}


int predict_sleep_time(
  float delta_time,
  float rssi,
  float prev_delta,
  float prev_rssi,
  float prev_received,
  float prev_sleep_time,
  float delta_change,
  float rssi_change,
  float rolling_delta_avg,
  float rolling_rssi_avg
) {
    const float intercept = 1302.16988f;
    const float prediction =
      intercept
      + (-0.000996752f * delta_time)
      + (-0.0383709095f * rssi)
      + (-0.000241911288f * prev_delta)
      + (-0.0226541446f * prev_rssi)
      + (-0.254682147f * prev_received)
      + ( 0.73933496f * prev_sleep_time)
      + (-0.000754840655f * delta_change)
      + (-0.0157167649f * rssi_change)
      + ( 0.0000608629029f * rolling_delta_avg)
      + ( 0.075915945f * rolling_rssi_avg);
  
  return static_cast<int>(round(prediction));
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  if (!first_packet_received) { // A bit questionalbe since what if we receive 70th packet? It means that we missed 69 according to the code. However, what if I just connected the device?
      packetMissed += myData.packetNumber - 1;
      if (packetMissed != 0) {
        Serial.print("Missed a packet! Missed packet count: ");
        Serial.println(packetMissed);
        Serial.println(myData.packetNumber);
      }
      first_packet_received = true;
    }
  previous_rssi = packetLog.rssi_display;
  packetLog.rssi_display = rssi_display; 
  totalRssi += rssi_display;
  packetReceived++;
  
  
  if (lastReceivedPacketNumber != -1 && (myData.packetNumber - lastReceivedPacketNumber) > 1) {
      Serial.print("Missed a packet! Missed packet count: ");
      packetMissed += myData.packetNumber - lastReceivedPacketNumber - 1;
      Serial.println(packetMissed);
      packetLog.received = false;
  }
  success_rate = float(packetReceived) / float(packetReceived + packetMissed);
  lastReceivedPacketNumber = myData.packetNumber;
  
  // Turn off wifi module
  if (esp_wifi_stop() != ESP_OK) {
    Serial.println("Troubles with stop function");
  }
  end_time = millis(); // for metrics
  if (previous_delta <= 30) {
    previous_delta = packetLog.radioOnTime; 
  } else {
    previous_delta = 0; 
  }
  
   
  packetLog.radioOnTime = end_time - start_time;
  packetLog.timestamp = formatTimestamp(millis()); 
  packetLog.packetNumber = myData.packetNumber;
  if (packetLog.received == true) {
    previous_received = true;
  } else {
    previous_received = false;
  }
  packetLog.received = true; 
  packetLog.data = myData.time; 
  packetLog.length = len; 
  packetLog.successRate = success_rate * 100;
  printPacketLog();

  //calculating rolling delta and rolling rssi:
  //shifting existing vals:
  for (int i = 2; i > 0; i--) {
    delta_history[i] = delta_history[i - 1];
    rssi_history[i] = rssi_history[i - 1];
  }
  //storing current vals:
  if (end_time - start_time <= 30) {
      delta_history[0] = end_time - start_time;
  } else {
      delta_history[0] = 0;
  }
  rssi_history[0] = rssi_display;
  //excluding zero values:
  float delta_sum = 0.0;
  int delta_validCount = 0;
  float rssi_sum = 0.0;
  int rssi_validCount = 0;
  for (int i = 0; i < 3; i++) {
    if (delta_history[i] != 0) {
      delta_sum += delta_history[i];
      delta_validCount++;
    }
    if (rssi_history[i] != 0) {
      rssi_sum += rssi_history[i];
      rssi_validCount++;
    }
  }
  //if there are valid delta values, calculate the average, otherwise set it to 0
  float rolling_delta_avg = (delta_validCount > 0) ? delta_sum / delta_validCount : 0.0;
  //ff there are valid RSSI values, calculate the average, otherwise set it to 0
  float rolling_rssi_avg = (rssi_validCount > 0) ? rssi_sum / rssi_validCount : 0.0;

  
  //delta_time, rssi, prev_delta, prev_rssi, prev_received, 
  //prev_sleep_time, delta_change, rssi_change,
  //rolling_delta_avg, rolling_rssi_avg
  sleep_time = predict_sleep_time(end_time - start_time, rssi_display, previous_delta, previous_rssi, previous_received,
                                sleep_time, (end_time - start_time) - previous_delta, rssi_display - previous_rssi,
                                rolling_delta_avg, rolling_rssi_avg); 
   
  packetLog.newDelay = sleep_time;                              


  delay(sleep_time);
  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_country(&country);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  start_time = millis();
}

void printFinalStatistics() {
    float finalSuccessRate = 0;
    if (packetReceived + packetMissed != 0) {
        finalSuccessRate = (float)packetReceived / (packetReceived + packetMissed) * 100;
    }
    Serial.println("\n------ Final Statistics ------");
    Serial.print("Total Packets Received: ");
    Serial.println(packetReceived);
    Serial.print("Total Packets Missed: ");
    Serial.println(packetMissed);
    Serial.print("Final Success Rate: ");
    Serial.print(finalSuccessRate);
    Serial.println("%");
    if (packetReceived > 0) {  // Check to avoid division by zero
        float averageRssi = float(totalRssi) / packetReceived;
        Serial.print("Average RSS: ");
        Serial.println(averageRssi);
    }
    Serial.println("--------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  ++bootCount;
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
  Serial.println("Boot number: " + String(bootCount));
  packetReceived = 0;

  WiFi.mode(WIFI_STA);
  

  Serial.println("  Timestamp  |Pct#| RCV? | PRR | RAD(ms)| RSS | Sleep time");
  Serial.println("--------------------------------------------------------------------");

  if (esp_now_init() != ESP_OK) {
    Serial.println("There was an error initializing ESP-NOW");
    return;
  }
  memcpy(masterInfo.peer_addr, masterAddress, 6);
  masterInfo.channel = 14;
  esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_country(&country);
  
  if (esp_now_add_peer(&masterInfo) != ESP_OK) {
    Serial.println("There was an error registering the master");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  start_time = millis();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  esp_log_level_set("wifi", ESP_LOG_NONE);
}

void loop() {
    if ((packetReceived + packetMissed >= TOTAL_PACKETS) || lastReceivedPacketNumber == 500) {
      delay(sleep_time);  
      printFinalStatistics();
      esp_sleep_enable_timer_wakeup(2*uS_TO_MIN_FACTOR);
      if (bootCount == 10) {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
      }
      Serial.println("Turning off...");
      delay(1000);
      Serial.flush();
      esp_deep_sleep_start();
    }
}
