#ifndef SparkIO_h
#define SparkIO_h

#include "SparkStructures.h"
#include "SparkComms.h"
#include "CircularArray.h"

uint8_t license_key[64];

#define OUT_BLOCK_SIZE 900 // largest preset seen so far is 800 bytes

// MESSAGE INPUT CLASS
class MessageIn
{
  public:
    MessageIn() {
      message_pos = 0;
    };

    bool check_for_acknowledgement();
    bool get_message(unsigned int *cmdsub, SparkMessage *msg, SparkPreset *preset);
    
    CircularArray message_in;
    int message_pos;

    void read_string(char *str);
    void read_prefixed_string(char *str);
    void read_onoff(bool *b);
    void read_float(float *f);
    void read_uint(uint8_t *b);
    void read_general_uint(uint32_t *b);
    void read_byte(uint8_t *b);
};

// MESSAGE OUTPUT CLASS
class MessageOut
{
  public:
    MessageOut(unsigned int base): cmd_base(base) {};

    // creating messages to send
    void start_message(int cmdsub);
    void end_message();
    //void add(byte b);
    void write_byte(byte b);
    void write_byte_no_chksum(byte b);
    
    void write_uint(byte b);
    void write_prefixed_string(const char *str);
    void write_long_string(const char *str);
    void write_string(const char *str);
    void write_float(float flt);
    void write_onoff(bool onoff);
    void write_uint32(uint32_t w);

    void select_live_input_1();
    void create_preset(SparkPreset *preset);
    void turn_effect_onoff(char *pedal, bool onoff);
    void turn_effect_onoff_input(char *pedal, bool onoff, uint8_t input);
    void change_hardware_preset(uint8_t curr_preset, uint8_t preset_num);
    void change_effect(char *pedal1, char *pedal2);
    void change_effect_input(char *pedal1, char *pedal2, uint8_t input);
    void change_effect_parameter(char *pedal, int param, float val);
    void change_effect_parameter_input(char *pedal, int param, float val, uint8_t input);   
    void get_serial();
    void get_name();
    void get_hardware_preset_number();
    void get_preset_details(unsigned int preset);
    void get_checksum_info();
    void get_firmware();
    void save_hardware_preset(uint8_t curr_preset, uint8_t preset_num);
    void send_firmware_version(uint32_t firmware);
    void send_0x022a_info(byte v1, byte v2, byte v3, byte v4);  
    void send_preset_number(uint8_t preset_h, uint8_t preset_l);
    void send_key_ack();
    void send_serial_number(char *serial);
    void send_ack(unsigned int cmdsub);
    void send_tap_tempo(float val);
    // trial message
    void tuner_on_off(bool onoff);

    int buf_size;
    int buf_pos;
    uint8_t buffer[OUT_BLOCK_SIZE];

    int cmd_base;
    int out_msg_chksum;

};


MessageIn spark_msg_in;
MessageIn app_msg_in;

MessageOut spark_msg_out(0x0100);
MessageOut app_msg_out(0x0300);

void process_sparkIO();

void spark_send();
void app_send();

void init_sparkIO();

#endif
      
