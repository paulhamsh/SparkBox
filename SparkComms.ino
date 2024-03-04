#include "SparkComms.h"


// Timer routines
#define TIMER 100000
#define SPARK_TIMEOUT 200
#define APP_TIMEOUT 500

hw_timer_t *timer_sp = NULL;

bool spark_timer_active = false;
unsigned long last_spark_time;

bool app_timer_active = false;
unsigned long last_app_time;

// this is to catch a good block that ends on a 10, 20 or 106 byte boundary via a timeout
void ARDUINO_ISR_ATTR timer_cb_sp() {
  if (spark_timer_active && (millis() - last_spark_time > SPARK_TIMEOUT)) {
    spark_timer_active = false;
    if (from_spark_index != 0) {
      if (from_spark[from_spark_index-1] == 0xf7) {
        // mark this as good and wait for the receiver to clear the buffer
        got_spark_block = true;
        last_spark_was_bad = false;
      }
      else {
        got_spark_block = false;
        last_spark_was_bad = true;
        // clear the buffer
        from_spark_index = 0;  
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
  if (ble_passthru) {
    pCharacteristic_send->setValue(pData, length);
    pCharacteristic_send->notify(true);
  }
  
  // check to see if this is the end of a block - if not a standard packet size (20, 10, 106) then it definitely is
  // but it could also happen to be a standard size and the end of a block
  // in which case we set up a timer to catch it

  if (length != 20 && length != 10 && length != 106) {
    got_spark_block = true;
    spark_timer_active = false;
  }
  else {
    last_spark_time = millis();
    spark_timer_active = true;
  }
}


class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    // copy to the buffer 
    std::string s = pCharacteristic->getValue();  // do this to avoid the issue here: https://github.com/h2zero/NimBLE-Arduino/issues/413
    int length = s.size();

    if (from_app_index + length < BLE_BUFSIZE) {
      memcpy(&from_app[from_app_index], s.c_str(), length);
      from_app_index += length;
    }
    else {
      from_app_index = 0;
      last_app_was_bad = true;
      DEBUG("Exceeded app block size");
    }

    // passthru
    if (ble_passthru) 
      pSender_sp->writeValue((uint8_t *) s.c_str(), length, false);

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

BLEUUID SpServiceUuid(C_SERVICE);

void connect_spark() {
  if (found_sp && !connected_sp) {
    if (pClient_sp != nullptr && pClient_sp->isConnected())
       DEBUG("connect_spark() thinks I was already connected");
    
    if (pClient_sp->connect(sp_device)) {
      connected_sp = true;
      pService_sp = pClient_sp->getService(SpServiceUuid);
      if (pService_sp != nullptr) {
        pSender_sp   = pService_sp->getCharacteristic(C_CHAR1);
        pReceiver_sp = pService_sp->getCharacteristic(C_CHAR2);
        if (pReceiver_sp && pReceiver_sp->canNotify()) {
          if (!pReceiver_sp->subscribe(true, notifyCB_sp, true)) {
            connected_sp = false;
            DEBUG("Spark disconnected");
            NimBLEDevice::deleteClient(pClient_sp);
          }   
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

  strcpy(spark_ble_name, "");
  ble_spark_connected = false;
  ble_app_connected = false;

  BLEDevice::init("");
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

  pCharacteristic_receive = pService->createCharacteristic(S_CHAR1, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pCharacteristic_send = pService->createCharacteristic(S_CHAR2, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY); 


  pCharacteristic_receive->setCallbacks(&chrCallbacks_r);
  pCharacteristic_send->setCallbacks(&chrCallbacks_s);
  pService->start();
  pServer->start(); 

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
  
  DEBUG("Available for app to connect...");  
  pAdvertising->setName(spark_ble_name);
  pAdvertising->start(); 

  // timers for timeout
  setup_timer_sp();

  // flags for data availability
  got_app_block = false;
  got_spark_block = false;

  return true;
}

void send_to_spark(byte *buf, int len) {
  pSender_sp->writeValue(buf, len, false);
}


void send_to_app(byte *buf, int len) {
  pCharacteristic_send->setValue(buf, len);
  pCharacteristic_send->notify(true);
}

// for some reason getRssi() crashes with two clients!
int ble_getRSSI() { 
  return pClient_sp->getRssi();
}