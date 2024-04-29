#ifndef SparkComms_h
#define SparkComms_h

// Timer routines
//#define TIMER 100000
#define SPARK_TIMEOUT 1000
#ifdef CLASSIC
#define APP_TIMEOUT 1000
#else
#define APP_TIMEOUT 1000
#endif


enum {S40, MINI, GO, LIVE} spark_type = MINI;

#define SIZE_BLE_NAME 20
char spark_ble_name[SIZE_BLE_NAME + 1];
char spark_bt_name[SIZE_BLE_NAME + 1];

//#define BLE_DUMP 

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

//#define SPARK_BT_NAME  "Spark 40 AUdio"
//#define SPARK_BT_NAME  "Spark MINI Audio"
//#define SPARK_BT_NAME  "Spark GO Audio"
//#define SPARK_BT_NAME "Spark LIVE Audio"
#define DEFAULT_SPARK_BLE_NAME "Spark 40 BLE"


QueueHandle_t qFromApp;
QueueHandle_t qFromSpark;
QueueHandle_t qFromAppFilter;
QueueHandle_t qFromSparkFilter;

struct packet_data {
  uint8_t *ptr;
  int size;
};

#ifdef CLASSIC
#include "BluetoothSerial.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BluetoothSerial *bt;
#else
#include "NimBLEDevice.h"
#endif

#define BLE_BUFSIZE 2000

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

void spark_comms_process();

int ble_getRSSI();


bool ble_app_connected;
bool ble_spark_connected;
bool bt_app_connected;

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
