#ifndef SparkIO_h
#define SparkIO_h

#include "RingBuffer.h"
#include "SparkStructures.h"
#include "SparkComms.h"

uint8_t license_key[64];

// MESSAGE INPUT CLASS
class MessageIn
{
  public:
    MessageIn() {};
    void set_from_array(uint8_t *in, int size);
    void clear();
    bool check_for_acknowledgement();
    bool get_message(unsigned int *cmdsub, SparkMessage *msg, SparkPreset *preset);
    RingBuffer in_message;

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
    
    bool has_message();
    void copy_message_to_array(byte *blk, int *len);
    // creating messages to send
    void start_message(int cmdsub);
    void end_message();
    void write_byte(byte b);
    void write_byte_no_chksum(byte b);
    
    void write_uint(byte b);
    void write_prefixed_string(const char *str);
    void write_long_string(const char *str);
    void write_string(const char *str);
    void write_float(float flt);
    void write_onoff(bool onoff);
    void write_uint32(uint32_t w);

    void create_preset(SparkPreset *preset);
    void turn_effect_onoff(char *pedal, bool onoff);
    void change_hardware_preset(uint8_t curr_preset, uint8_t preset_num);
    void change_effect(char *pedal1, char *pedal2);
    void change_effect_parameter(char *pedal, int param, float val);
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
    // trial message
    void tuner_on_off(bool onoff);

    RingBuffer out_message;
    int cmd_base;
    int out_msg_chksum;
};


MessageIn spark_message_in;
MessageIn app_message_in;

MessageOut spark_message_out(0x0100);
MessageOut app_message_out(0x0300);

void spark_process();
void app_process();

void spark_send();
void app_send();

#endif
      
