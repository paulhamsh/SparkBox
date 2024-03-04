#ifndef SparkComms_h
#define SparkComms_h

enum {S40=0, MINI, GO} spark_type = MINI;
char *ble_names[]{"Spark 40 BLE", "Spark MINI BLE", "Spark GO BLE"};

#define SIZE_BLE_NAME 20
char spark_ble_name[SIZE_BLE_NAME + 1];

#define DEBUG_ON

#ifndef DEBUG
  #ifdef DEBUG_ON
  // found this hint with __VA_ARGS__ on the web, it accepts different sets of arguments /Copych
    #define DEB(...) Serial.print(__VA_ARGS__) 
    #define DEBUG(...) Serial.println(__VA_ARGS__) 
  #else
    #define DEB(...)
    #define DEBUG(...)
  #endif
#endif

#ifdef CLASSIC
#include "BluetoothSerial.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#else
#include "NimBLEDevice.h"
#endif


#define BLE_BUFSIZE 5000

byte from_app[BLE_BUFSIZE];
int from_app_index = 0;
bool got_app_block;
bool last_app_was_bad;

byte from_spark[BLE_BUFSIZE];
int from_spark_index = 0;
bool got_spark_block;
bool last_spark_was_bad;

bool ble_passthru;

#define C_SERVICE "ffc0"
#define C_CHAR1   "ffc1"
#define C_CHAR2   "ffc2"

#define S_SERVICE "ffc0"
#define S_CHAR1   "ffc1"
#define S_CHAR2   "ffc2"

#define MAX_SCAN_COUNT 2

bool connect_to_all();
void connect_spark();

void send_to_spark(); 
void send_to_app();

int ble_getRSSI();


bool ble_app_connected;
bool ble_spark_connected;

bool connected_sp;
bool found_sp;

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic_receive;
BLECharacteristic *pCharacteristic_send;

BLEAdvertising *pAdvertising;

BLEScan *pScan;
BLEScanResults pResults;
BLEAdvertisedDevice device;

BLEClient *pClient_sp;
BLERemoteService *pService_sp;
BLERemoteCharacteristic *pReceiver_sp;
BLERemoteCharacteristic *pSender_sp;
BLERemoteDescriptor* p2902_sp;
BLEAdvertisedDevice *sp_device;

#endif
