#include "SparkComms.h"


//hw_timer_t *timer_sp = NULL;

bool spark_timer_active = false;
unsigned long last_spark_time;

bool app_timer_active = false;
unsigned long last_app_time;



#define PASSTHRU_TABLE_SIZE 30

int passthru_packet_index = 0;
int passthru_packet_start[PASSTHRU_TABLE_SIZE];
int passthru_packet_length[PASSTHRU_TABLE_SIZE];

//SemaphoreHandle_t xFromAppMutex = NULL;  // Create a mutex object
QueueHandle_t qAppToSpark;


// this is to catch a good block that ends on a 10, 20 or 106 byte boundary via a timeout
//void ARDUINO_ISR_ATTR timer_cb_sp() {
void spark_comms_timer() {
  if (spark_timer_active && (millis() - last_spark_time > SPARK_TIMEOUT)) {
    spark_timer_active = false;
    if (from_spark_index != 0) {
      if (from_spark[from_spark_index-1] == 0xf7) {
        // mark this as good and wait for the receiver to clear the buffer
        got_spark_block = true;
        last_spark_was_bad = false;
        DEBUG("Timeout on block, think I got a block");
      }
      else {
        got_spark_block = false;
        last_spark_was_bad = true;
        // clear the buffer
        from_spark_index = 0;  
        DEBUG("Timeout on block, didn't get a block");
      }
    }  
  }

  if (app_timer_active && (millis() - last_app_time > APP_TIMEOUT)) {
    app_timer_active = false;
    if (from_app_index != 0) {
      if (from_app[from_app_index-1] == 0xf7) {
        // mark this as good and wait for the receiver to clear the buffer
        got_app_block = true;
        last_app_was_bad = false;
      }
      else {
        got_app_block = false;
        last_app_was_bad = true;
        // clear the buffer
        from_app_index = 0;  
      }
    }  
  }
}

/*
void start_timer() {
  timerRestart(timer_sp);
  timerAlarmWrite(timer_sp, TIMER, true);
  timerAlarmEnable(timer_sp);
  spark_timer_active = false;
  app_timer_active = false;
}

void setup_timer_sp() {
  timer_sp = timerBegin(2, 80, true);                    // timer 2, prescale 80       
  timerAttachInterrupt(timer_sp, &timer_cb_sp, true);    // count up
  start_timer();
  spark_timer_active = false;
  app_timer_active = false;
}
*/


void spark_comms_process() {
  int packets_waiting;
  int i;
  int index;
  int len, pos;

  // process the 
  spark_comms_timer();
  
  // do passthru to app
  // it seems to be important to send all queued packets close together else the app gets disconnected - especially the 170 message
  while (uxQueueMessagesWaiting(qAppToSpark) > 0) {
    xQueueReceive(qAppToSpark, &index, (TickType_t) 0);
    //Serial.print(index);
    //Serial.println();
    len = passthru_packet_length[index];
    if (len != 0) {
      pos = passthru_packet_start[index];
      //Serial.print("Sending passthru ");
      //Serial.print(index);
      //Serial.print(" : ");
      //Serial.print(pos);
      //Serial.print(" : ");
      //Serial.println(len);
      pSender_sp->writeValue((uint8_t *) &from_app[pos], len, false);    
      passthru_packet_length[index] = 0;
    }
  }
}


const uint8_t notifyOn[] = {0x1, 0x0};

// client callback for connection to Spark

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient) {
    DEBUG("Spark connected");
    ble_spark_connected = true;   
  }
  void onDisconnect(BLEClient *pclient) {
    connected_sp = false;    
    ble_spark_connected = false;     
    DEBUG("Spark disconnected");   
  }
};

// server callback for connection to BLE app

class MyServerCallback : public BLEServerCallbacks {
  void onConnect(BLEServer *pserver)  {
     if (pserver->getConnectedCount() == 1) {
      DEBUG("App connection event and is connected"); 
      ble_app_connected = true;
    }
    else {
      DEBUG("App connection event and is not really connected");   
    }
  }
  void onDisconnect(BLEServer *pserver) {
    ble_app_connected = false;
    DEBUG("App disconnected");
  }
};

#ifdef CLASSIC
// server callback for connection to BT classic app

void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    DEBUG("callback: Classic BT Spark app connected");
    bt_app_connected = true;
  }
 
  if(event == ESP_SPP_CLOSE_EVT ){
    DEBUG("callback: Classic BT Spark app disconnected");
    bt_app_connected = false;
  }
}
#endif




// Blocks sent to the app are max length 0xad or 173 bytes. This includes the 16 byte block header.
// Without that header the block is 157 bytes.
// For Spark 40 BLE they are sent as 100 bytes then 73 bytes.
// 
// From the amp, max size is 0x6a or 106 bytes (90 bytes plus 16 byte block header).
// For the Spark 40 these are sent in chunk of 106 bytes.
// MINI and GO do not use the block header so just transmit 90 bytes.
// From the MINI and GO, they are sent in chunks of 20 bytes up to the 90 bytes (so 20 + 20 + 20 + 20 + 10). 
// 
//
// Chunks sent from the amp have max size of 39 bytes, 0x27.
// This is 6 byte header, 1 byte footer, 4 data chunks of 7 bytes, 4 '8 bit' bytes. So 6 + 1 +32 = 39.
// Because of the multi-chunk header of 3 bytes, there are 32 - 3 = 29 data bytes (0x19)
//
// Example
// F0 01 04 1F 03 01   20  0E 00 19  00 00 59 24   00 39 44 32 46 32 41 41   00 33 2D 34 45 43 35 2D   00 34 42 44 37 2D 41 33   F7


// From the Spark

void notifyCB_sp(BLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  // copy to the buffer
  if (from_spark_index + length < BLE_BUFSIZE) {
    memcpy(&from_spark[from_spark_index], pData, length);
    from_spark_index += length; 
  }
  else {
    from_spark_index = 0;
    last_spark_was_bad = true;
    DEBUG("Exceeded block size");
  }

  // passthru
  if (ble_passthru && ble_app_connected) {
    pCharacteristic_send->setValue(pData, length);
    pCharacteristic_send->notify(true);
  }

  #ifdef CLASSIC
  if (ble_passthru && bt_app_connected) {
    bt->write(pData, length);
  }
  #endif

  // check to see if this is the end of a block - if not a standard packet size (20, 10, 106) then it definitely is
  // but it could also happen to be a standard size and the end of a block
  // in which case we set up a timer to catch it

  if (length != 20 && length != 10 && length != 19 && length != 106) {   // added 19 for Spark LIVE
    got_spark_block = true;
    spark_timer_active = false;
  }
  else {
    last_spark_time = millis();
    spark_timer_active = true;
  }
}

// From the app

class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {

    // copy to the buffer 
    std::string s = pCharacteristic->getValue();  // do this to avoid the issue here: https://github.com/h2zero/NimBLE-Arduino/issues/413
    int length = s.size();
    const char *data = s.c_str();
    int index = from_app_index;

    if (from_app_index + length < BLE_BUFSIZE) {
      memcpy(&from_app[from_app_index], data, length);
      from_app_index += length;
    }
    else {
      from_app_index = 0;
      last_app_was_bad = true;
      DEBUG("Exceeded app block size");
    }

    // passthru
    if (ble_passthru) {
      #ifdef CLASSIC 
        passthru_packet_start[passthru_packet_index] = index;
        passthru_packet_length[passthru_packet_index] = length;
        xQueueSend(qAppToSpark, &passthru_packet_index, (TickType_t) 0);
        passthru_packet_index++;
        if (passthru_packet_index >= PASSTHRU_TABLE_SIZE) 
          passthru_packet_index = 0;
      #else
        pSender_sp->writeValue((uint8_t *) s.c_str(), length, false);
      #endif
    }

    // For Spark 40,  MINI and GO will be 100 then 73 for a block of 173
    if (length != 100 && length != 73) {
      got_app_block = true;
      app_timer_active = false;
    }
    else {
      last_app_time = millis();
      app_timer_active = true;
    }

  };
};

static CharacteristicCallbacks chrCallbacks_s, chrCallbacks_r;


// Serial BT callback for data
void data_callback(const uint8_t *buffer, size_t size) {
  int index = from_app_index;

  if (from_app_index + size < BLE_BUFSIZE) {
    memcpy(&from_app[from_app_index], buffer, size);
    //for (int i=0; i < length; i++) {Serial.print(" "); Serial.print(from_app[from_app_index + i], HEX);}
    //Serial.println();
    from_app_index += size;
  }
  else {
    from_app_index = 0;
    last_app_was_bad = true;
    DEBUG("Exceeded app block size");
  }


  // passthru
  if (ble_passthru) {
    passthru_packet_start[passthru_packet_index] = index;
    passthru_packet_length[passthru_packet_index] = size;
    xQueueSend(qAppToSpark, &passthru_packet_index, (TickType_t) 0);
    passthru_packet_index++;
    if (passthru_packet_index >= PASSTHRU_TABLE_SIZE) 
      passthru_packet_index = 0;
  }
  
  /*
  // passthru
  if (ble_passthru) {
    passthru_packet_start[passthru_packet_index] = index;
    passthru_packet_length[passthru_packet_index] = size;
    passthru_packet_index++;
  }
  */

  // For Spark 40,  MINI and GO will be 100 then 73 for a block of 173
  if (size != 173) {
    got_app_block = true;
    app_timer_active = false;
  }
  else {
    last_app_time = millis();
    app_timer_active = true;
  }
}


BLEUUID SpServiceUuid(C_SERVICE);

void connect_spark() {
  if (found_sp && !connected_sp) {
    if (pClient_sp != nullptr && pClient_sp->isConnected())
       DEBUG("connect_spark() thinks I was already connected");
    
    if (pClient_sp->connect(sp_device)) {
#if defined CLASSIC  && !defined HELTEC_WIFI
        pClient_sp->setMTU(517);  
#endif
      connected_sp = true;
      pService_sp = pClient_sp->getService(SpServiceUuid);
      if (pService_sp != nullptr) {
        pSender_sp   = pService_sp->getCharacteristic(C_CHAR1);
        pReceiver_sp = pService_sp->getCharacteristic(C_CHAR2);
        if (pReceiver_sp && pReceiver_sp->canNotify()) {
#ifdef CLASSIC
          pReceiver_sp->registerForNotify(notifyCB_sp);
          p2902_sp = pReceiver_sp->getDescriptor(BLEUUID((uint16_t)0x2902));
          if (p2902_sp != nullptr)
             p2902_sp->writeValue((uint8_t*)notifyOn, 2, true);
#else
          if (!pReceiver_sp->subscribe(true, notifyCB_sp, true)) {
            connected_sp = false;
            DEBUG("Spark disconnected");
            NimBLEDevice::deleteClient(pClient_sp);
          }   
#endif
        } 
      }
      DEBUG("connect_spark(): Spark connected");
      ble_spark_connected = true;
    }
  }
}






bool connect_to_all() {
  int i, j;
  int counts;
  uint8_t b;
  int len;

  //xFromAppMutex = xSemaphoreCreateMutex();
  qAppToSpark = xQueueCreate(5, sizeof(int));

  strcpy(spark_ble_name, DEFAULT_SPARK_BLE_NAME);
  ble_spark_connected = false;
  ble_app_connected = false;
  bt_app_connected = false;    // only for Serial Bluetooth

  BLEDevice::init(spark_ble_name);        // put here for CLASSIC code
  BLEDevice::setMTU(517);
  pClient_sp = BLEDevice::createClient();
  pClient_sp->setClientCallbacks(new MyClientCallback());
 
  BLEDevice::getScan()->setInterval(40);
  BLEDevice::getScan()->setWindow(40);
  BLEDevice::getScan()->setActiveScan(true);
  pScan = BLEDevice::getScan();

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallback());  
  pService = pServer->createService(S_SERVICE);

#ifdef CLASSIC  
  pCharacteristic_receive = pService->createCharacteristic(S_CHAR1, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristic_send = pService->createCharacteristic(S_CHAR2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#else
  pCharacteristic_receive = pService->createCharacteristic(S_CHAR1, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pCharacteristic_send = pService->createCharacteristic(S_CHAR2, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY); 
#endif

  pCharacteristic_receive->setCallbacks(&chrCallbacks_r);
  pCharacteristic_send->setCallbacks(&chrCallbacks_s);
#ifdef CLASSIC
  pCharacteristic_send->addDescriptor(new BLE2902());
#endif

  pService->start();
#ifndef CLASSIC
  pServer->start(); 
#endif

  pAdvertising = BLEDevice::getAdvertising(); // create advertising instance
  
  pAdvertising->addServiceUUID(pService->getUUID()); // tell advertising the UUID of our service
  pAdvertising->setScanResponse(true);  

  // Connect to Spark
  connected_sp = false;
  found_sp = false;

  DEBUG("Scanning...");

  counts = 0;
  while (!found_sp && counts < MAX_SCAN_COUNT) {   // assume we only use a pedal if on already and hopefully found at same time as Spark, don't wait for it
    counts++;
    pResults = pScan->start(4);
    
    for(i = 0; i < pResults.getCount()  && !found_sp; i++) {
      device = pResults.getDevice(i);

      if (device.isAdvertisingService(SpServiceUuid)) {
        strncpy(spark_ble_name, device.getName().c_str(), SIZE_BLE_NAME);
        DEBUG("Found ");
        DEBUG(spark_ble_name);

        found_sp = true;
        connected_sp = false;
        sp_device = new BLEAdvertisedDevice(device);
      }
    }
  }

  if (!found_sp) return false;   // failed to find the Spark within the number of counts allowed (MAX_SCAN_COUNT)
  connect_spark();

#ifdef CLASSIC
  DEBUG("Starting classic bluetooth");
  // now advertise Serial Bluetooth
  bt = new BluetoothSerial();
  bt->register_callback(bt_callback);
  len = strlen(spark_ble_name);
  strncpy(spark_bt_name, spark_ble_name, len - 4);   // effectively strip off the ' BLE' at the end
  spark_bt_name[len - 4] = '\0';

  DEB("Creating classic bluetooth with name ");
  DEBUG(spark_bt_name);
  
  if (!bt->begin (spark_bt_name, false)) {
    DEBUG("Classic bluetooth init fail");
    while (true);
  }

  bt->onData(data_callback);

  // flush anything read from App - just in case
  while (bt->available())
    b = bt->read(); 
  DEBUG("Spark serial bluetooth set up");
#endif



  DEBUG("Available for app to connect...");  

  //== Start: try to look like a Spark Go
  //char scan_data[] = {0x0e,0x09,0x53,0x70,0x61,0x72,0x6b,0x20,0x47,0x4f,0x20,0x42,0x4c,0x45,0x00};
  //char adv_data[] =  {0x02,0x01,0x0a,0x03,0x03,0xc0,0xff,0x0b,0xff,0x06,0x10,0x00,0x00,0x08,0xeb,0xed,0x3d,0x5d,0x5a};

  //BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  //BLEAdvertisementData oScanAdvertisementData = BLEAdvertisementData();  

  //oScanAdvertisementData.addData(scan_data, sizeof(scan_data));
  //oAdvertisementData.addData(adv_data, sizeof(adv_data));

  //pAdvertising->setAdvertisementData(oAdvertisementData);
  //pAdvertising->setScanResponseData(oScanAdvertisementData);
  //== Stop: that code

#ifndef CLASSIC
  pAdvertising->setName(spark_ble_name);
#endif

  //pAdvertising->setManufacturerData(manuf_data);
  pAdvertising->start(); 

  // timers for timeout
  //setup_timer_sp();

  // flags for data availability
  got_app_block = false;
  got_spark_block = false;

  return true;
}

void send_to_spark(byte *buf, int len) {
  pSender_sp->writeValue(buf, len, false);
}


void send_to_app(byte *buf, int len) {
  if (ble_app_connected) {
    pCharacteristic_send->setValue(buf, len);
    pCharacteristic_send->notify(true);
  }
#if defined CLASSIC
  else {
    bt->write(buf, len);
  }
#endif
}

// for some reason getRssi() crashes with two clients!
int ble_getRSSI() { 
  return pClient_sp->getRssi();
}