#include "SparkComms.h"

//#define DEBUG_COMMS(...)  {char _b[100]; sprintf(_b, __VA_ARGS__); Serial.println(_b);}
#define DEBUG_COMMS(...) {}

//#define DEBUG_STATUS(...)  {char _b[100]; sprintf(_b, __VA_ARGS__); Serial.println(_b);}
#define DEBUG_STATUS(...) {}

//#define DUMP_BUFFER(p, s) {for (int _i=0; _i <=  (s); _i++) {Serial.print( (p)[_i], HEX); Serial.print(" ");}; Serial.println();}
#define DUMP_BUFFER(p, s) {}

struct packet_data {
  uint8_t *ptr;
  int size;
};


struct packet_data packet_spark;
struct packet_data packet_app;
unsigned long lastAppPacketTime;
unsigned long lastSparkPacketTime;

// simply copy the packet received and put pointer in the queue
void app_callback(uint8_t *buf, int size) {
  struct packet_data qe;

  new_packet_from_data(&qe, buf, size);
  xQueueSend (qFromApp, &qe, (TickType_t) 0);
}


void spark_callback(uint8_t *buf, int size) {
  struct packet_data qe;

  new_packet_from_data(&qe, buf, size);
  xQueueSend (qFromSpark, &qe, (TickType_t) 0);
}

// read from queue, pass-through to amp then check for a complete valid message to send on for processing

void setup_comms() {
  packet_spark.size = 0;
  packet_app.size = 0;
  lastAppPacketTime = millis();
  lastSparkPacketTime = millis();

  qFromApp         = xQueueCreate(20, sizeof (struct packet_data));
  qFromSpark       = xQueueCreate(20, sizeof (struct packet_data));

  qFromAppFilter   = xQueueCreate(20, sizeof (struct packet_data));
  qFromSparkFilter = xQueueCreate(20, sizeof (struct packet_data));
}


void process_app_packet(struct packet_data *pd, int *start, int *end) {
  int length;
  struct packet_data qe;

  length = *end - *start + 1;
  qe.ptr = (uint8_t *) malloc(length) ;
  qe.size = length;
  memcpy(qe.ptr, &pd->ptr[*start], length); 
  xQueueSend (qFromAppFilter, &qe, (TickType_t) 0);

  DEBUG_COMMS("Processing a packet %d to %d", *start, *end);
}

void process_spark_packet(struct packet_data *pd, int *start, int *end) {
  int length;
  struct packet_data qe;

  length = *end - *start + 1;
  qe.ptr = (uint8_t *) malloc(length) ;
  qe.size = length;
  memcpy(qe.ptr, &pd->ptr[*start], length); 
  xQueueSend (qFromSparkFilter, &qe, (TickType_t) 0);

  DEBUG_COMMS("Processing a packet %d to %d", *start, *end);
}


void remove_block_headers (struct packet_data *pd, int *f7_pos) {
  int p = 0;
  DUMP_BUFFER(pd->ptr, *f7_pos);
  //for (int i=0; i<=*f7_pos; i++) {Serial.print(pd->ptr[i], HEX); Serial.print(" ");}; Serial.println();
  while (p < *f7_pos) {
    if (pd->ptr[p] == 0x01 && pd->ptr[p + 1] == 0xfe) {
      for (int i = p; i < pd->size - 16; i++) 
        pd->ptr[i] = pd->ptr[i + 16];
      pd->size -= 16;
      *f7_pos -= 16; 
      pd->ptr = (uint8_t *) realloc(pd->ptr, pd->size);
    }
    else
      p++;
  }
  DUMP_BUFFER(pd->ptr, *f7_pos);
}

// Packet handling routines

void new_packet_from_data(struct packet_data *pd, uint8_t *data, int length) {
  pd->ptr = (uint8_t *) malloc(length) ;
  pd->size = length;
  memcpy(pd->ptr, data, length);
}


void clear_packet(struct packet_data *pd) {
    free(pd->ptr);
    pd->size = 0; 
}

void append_packet(struct packet_data *pd, struct packet_data *add) {
  if (pd->size == 0)
    pd->ptr = (uint8_t *)  malloc(add->size);
  else
    pd->ptr = (uint8_t *)  realloc(pd->ptr, pd->size + add->size);
  memcpy (&pd->ptr[pd->size], add->ptr, add->size);
  pd->size += add->size;
}


void remove_packet_start(struct packet_data *pd, int end) {
  if (end == pd->size - 1) {
    // processed the whole block
    pd->size = 0;
    free(pd->ptr);
  }
  else if (end != 0) {
    uint8_t *p = pd->ptr;
    int new_start = end + 1;
    int new_size = pd->size - new_start;
    p = (uint8_t *) malloc(new_size);

    for (i = 0; i < new_size; i++)
      p[i] = pd->ptr[new_start + i];
    free(pd->ptr);
    pd->ptr = p;
    pd->size = new_size;
  }
}

int packet_scan_from_end(struct packet_data *pd, uint8_t to_find) {
  int pos = -1;
  for (int i = pd->size - 1; (i >= 0) && (pos == -1); i--) 
    if (pd->ptr[i] == to_find) pos = i;
  return pos;
}
  

void handle_spark_packet() {
  struct packet_data qe; 
  int start, end;
  int f7_pos; 
  bool good_packet;
  int good_end;

  // process packets queued
  while (uxQueueMessagesWaiting(qFromSpark) > 0) {
    lastSparkPacketTime = millis();
    xQueueReceive(qFromSpark, &qe, (TickType_t) 0);

    // passthru
    if (ble_passthru && ble_app_connected) {
      pCharacteristic_send->setValue(qe.ptr, qe.size);
      pCharacteristic_send->notify(true);
    }

    #ifdef CLASSIC
    if (ble_passthru && bt_app_connected) {
      bt->write(qe.ptr, qe.size);
    }
    #endif


    append_packet(&packet_spark, &qe);
    clear_packet(&qe); // this was created in app_callback, no longer needed

    DEBUG_STATUS("pd size %d", packet_spark.size);

    // validate new buffer and try to extract message from it
    // seek a 'f7' starting at the end

    f7_pos = packet_scan_from_end(&packet_spark, 0xf7); 

    DEBUG_COMMS("f7 pos %d", f7_pos);

    // if we found an f7 we can seek a useful chunk / multichunk
    if (f7_pos != -1) {
      end = f7_pos;
      start = 0;
      good_end = 0;
      remove_block_headers(&packet_spark, &f7_pos);
      while (start < f7_pos) {
        if (scan_packet(&packet_spark, &start, &end, f7_pos)) {
          DEBUG_COMMS("Got a good packet %d %d", start, end);
          process_spark_packet (&packet_spark, &start, &end);
          good_end = end;
        }
        start = end + 1;
      }
      if (good_end != 0) {
        remove_packet_start(&packet_spark, good_end);
      }
    }
  }
  // check for timeouts and delete the packet, it took too long to get a proper packet
  if ((packet_spark.size > 0) && (millis() - lastSparkPacketTime > SPARK_TIMEOUT)) {
    clear_packet(&packet_spark);
  }
}

void handle_app_packet() {
  struct packet_data qe; 
  int start, end;
  int f7_pos; 
  bool good_packet;
  int good_end;

  // process packets queued
  while (uxQueueMessagesWaiting(qFromApp) > 0) {
    lastAppPacketTime = millis();
    xQueueReceive(qFromApp, &qe, (TickType_t) 0);

    if (ble_passthru) {
      pSender_sp->writeValue(qe.ptr, qe.size, false);
    }

    append_packet(&packet_app, &qe);
    clear_packet(&qe); // this was created in app_callback, no longer needed

    DEBUG_STATUS("pd size %d", packet_app.size);

    // validate new buffer and try to extract message from it
    // seek a 'f7' starting at the end

    f7_pos = packet_scan_from_end(&packet_app, 0xf7); 

    DEBUG_COMMS("f7 pos %d", f7_pos);

    // if we found an f7 we can seek a useful chunk / multichunk
    if (f7_pos != -1) {
      end = f7_pos;
      start = 0;
      good_end = 0;
      remove_block_headers(&packet_app, &f7_pos);
      while (start < f7_pos) {
        if (scan_packet(&packet_app, &start, &end, f7_pos)) {
          DEBUG_COMMS("Got a good packet %d %d", start, end);
          process_app_packet (&packet_app, &start, &end);
          good_end = end;
        }
        start = end + 1;
      }
      if (good_end != 0) {
        remove_packet_start(&packet_app, good_end);
      }
    }
  }
  // check for timeouts and delete the packet, it took too long to get a proper packet
  if ((packet_app.size > 0) && (millis() - lastAppPacketTime > APP_TIMEOUT)) {
    clear_packet(&packet_app);
  }
}


bool scan_packet (struct packet_data *pd, int *start, int *end, int f7_pos) {
  int cmd; 
  int sub;
  int checksum;
  int multi_total_chunks, multi_this_chunk, multi_last_chunk;
  int st = -1;
  int en = -1;
  int this_checksum = 0;
  bool is_good = true;
  bool is_done = false;
  bool is_multi = false;
  bool is_final_multi = false;
  bool is_first_multi = false;
  bool found_chunk = false;

  uint8_t *buf = pd->ptr;
  int len = pd->size;
  int p = *start;

  while (!is_done) {
    // check to see if past end of buffer
    if (p > f7_pos) {
      is_done = true;
      is_good = false;
      en = p;
    }
 
    // found start of a message - either single or multi-chunk
    else if (buf [p] == 0xf0 && buf[p + 1] == 0x01 && (f7_pos - p >= 6)) {

      //DEBUG_COMMS("Pos %3d: new header", p);
      found_chunk = true;
      checksum = buf[p + 3];
      cmd      = buf[p + 4];
      sub      = buf[p + 5];
      this_checksum = 0;

      if ((cmd == 0x01 || cmd == 0x03) && sub == 0x01)
        is_multi = true;
      else
       is_multi = false;
    
      if (is_multi) {
        multi_total_chunks = buf[p + 7] | (buf[p + 6] & 0x01? 0x80 : 0x00);
        multi_this_chunk   = buf[p + 8] | (buf[p + 6] & 0x02? 0x80 : 0x00);
        is_first_multi = (multi_this_chunk == 0);
        is_final_multi = (multi_this_chunk + 1 == multi_total_chunks);

        //DEBUG_COMMS("Pos %3d: multi-chunk %d of %d", p, multi_this_chunk, multi_total_chunks);
        if (!is_first_multi && (multi_this_chunk != multi_last_chunk + 1)) {
          //DEBUG_COMMS( "Gap in multi chunk numbers");
          is_good = false;
        }
      }
      // only mark start if first multi chunk or not multi at all
      if (!is_multi || (is_multi && is_first_multi)) {
        //DEBUG_COMMS("Mark as start of chunks");
        st = p;
        is_good = true;
      }

      // skip header
      p += 6;
    }

    // if we have an f7, check we found a header and if multi, we are at last chunk
    else if (buf[p] == 0xf7 && found_chunk) {
      //DEBUG_COMMS( "Pos %3d: got f7", p);
      //DEBUG_COMMS("Provided checksum: %2x Calculated checksum: %2x", checksum, this_checksum);
      if (checksum != this_checksum)
        is_good = false;
      if (is_multi)
        multi_last_chunk = multi_this_chunk;
      if (!is_multi| (is_multi && is_final_multi)) {
        en = p;
        is_done = true;
      }
      else
        p++;
    }
    // haven't found a block yet so just scanning
    else if (!found_chunk) {
      p++;
    }

    // must be processing a meaningful block so update checksum calc
    else {
      this_checksum ^= buf[p];
      p++;
    }
  }
  
  *start = st;
  *end = en;
  DEBUG_COMMS("Returning start: %3d end: %3d status: %s", st, en, is_good ? "good" : "bad");
  return is_good;
}


void spark_comms_process() {
  handle_app_packet();
  handle_spark_packet();
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
    #ifdef CLASSIC
      pAdvertising->start(); 
    #endif
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


// From the Spark

void notifyCB_sp(BLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {

#ifdef BLE_DUMP
  int i;
  byte b;

  DEB("FROM SPARK:        ");

  for (i = 0; i < length; i++) {
    b = pData[i];
    if (b < 16) DEB("0");
    DEB(b, HEX);    
    DEB(" ");
    if (i % 32 == 31) { 
      DEBUG("");
      DEB("                   ");
    }
  }
  DEBUG();
#endif

  struct packet_data qe;
  new_packet_from_data(&qe, pData, length);
  xQueueSend (qFromSpark, &qe, (TickType_t) 0);
}


class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string s = pCharacteristic->getValue(); 
    int size = s.size();
    const char *buf = s.c_str();

    //DEB("Got BLE callback size: ");
    //DEBUG(size);

    struct packet_data qe;
    new_packet_from_data(&qe, (uint8_t *) buf, size);
    xQueueSend (qFromApp, &qe, (TickType_t) 0);
  };
};

static CharacteristicCallbacks chrCallbacks_s, chrCallbacks_r;


// Serial BT callback for data
void data_callback(const uint8_t *buffer, size_t size) {
//  int index = from_app_index;

  //DEB("Got SerialBT callback size: ");
  //DEBUG(size);

#ifdef BLE_DUMP
    int i = 0;
    byte b;
    DEB("FROM APP:          ");
    for (i=0; i < size; i++) {
      b = buffer[i];
      if (b < 16) DEB("0");
      DEB(b, HEX);    
      DEB(" ");
      if (i % 32 == 31) { 
        DEBUG("");
        DEB("                   ");
      }   
    }
    DEBUG();
#endif

    struct packet_data qe;
    new_packet_from_data(&qe, (uint8_t *) buffer, size);
    xQueueSend (qFromApp, &qe, (TickType_t) 0);

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


  // init comms processing
  setup_comms();

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

        if (strcmp(spark_ble_name, "Spark 40 BLE") == 0) 
          spark_type = S40;
        else if (strcmp(spark_ble_name, "Spark GO BLE") == 0)
          spark_type = GO;
        else if (strcmp(spark_ble_name, "Spark MINI BLE") == 0)        
          spark_type = MINI;  
        else if (strcmp(spark_ble_name, "Spark LIVE BLE") == 0)     
          spark_type = LIVE; 

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
  strcat(spark_bt_name, " Audio");

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