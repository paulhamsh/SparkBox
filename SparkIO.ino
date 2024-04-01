#include "SparkIO.h"

/*  SparkIO
 *  
 *  SparkIO handles communication to and from the Positive Grid Spark amp over bluetooth for ESP32 boards
 *  
 *  From the programmers perspective, you create and read two formats - a Spark Message or a Spark Preset.
 *  The Preset has all the data for a full preset (amps, effects, values) and can be sent or received from the amp.
 *  The Message handles all other changes - change amp, change effect, change value of an effect parameter, change hardware preset and so on
 *  
 *  Conection is handled with the two commands:
 *  
 *    connect_to_all();
 *  
 *  Messages and presets from the amp and the app are then queued and processed.
 *  The essential thing is the have the spark_process() and app_process() function somewhere in loop() - this handles all the processing of the input queues
 *  
 *  loop() {
 *    ...
 *    spark_process();
 *    app_process();
 *    ...
 *    do something
 *    ...
 *  }
 * 
 * Sending functions:
 *     void create_preset(SparkPreset *preset);   
 *     void get_serial();    
 *     void turn_effect_onoff(char *pedal, bool onoff);    
 *     void change_hardware_preset(uint8_t preset_num);    
 *     void change_effect(char *pedal1, char *pedal2);    
 *     void change_effect_parameter(char *pedal, int param, float val);
 *     
 *     These all create a message or preset which is sent immediately to the app or amp
 *  
 * Receiving functions:
 *     bool get_message(unsigned int *cmdsub, SparkMessage *msg, SparkPreset *preset);
 * 
 *     This receives the front of the 'received' queue - if there is nothing it returns false
 *     
 *     Based on whatever was in the queue, it will populate fields of the msg parameter or the preset parameter.
 *     Eveything apart from a full preset sent from the amp will be a message.
 *     
 *     You can determine which by inspecting cmdsub - this will be 0x0301 for a preset.
 *     
 *     Other values are:
 *     
 *     cmdsub       str1                   str2              val           param1             param2                onoff
 *     0323         amp serial #
 *     0337         effect name                              effect val    effect number
 *     0306         old effect             new effect
 *     0338                                                                0                  new hw preset (0-3)
 * 
 * 
 * 
 */


/* Data sizes for streaming to and from the Spark are as below.
 *
 *                             To Spark                   To App
 * Data                        128   (0x80)               25   (0x19)  
 * 8 bit expanded              150   (0x96)               32   (0x20)
 * With header and trailer     157   (0x9d)               39   (0x29)
 * 
 * Packet size                 173   (0xad)               106  (0x6a) 
 *
 */

 
// UTILITY FUNCTIONS

void uint_to_bytes(unsigned int i, uint8_t *h, uint8_t *l) {
  *h = (i & 0xff00) / 256;
  *l = i & 0xff;
}

void bytes_to_uint(uint8_t h, uint8_t l,unsigned int *i) {
  *i = (h & 0xff) * 256 + (l & 0xff);
}


// ------------------------------------------------------------------------------------------------------------
// Shared global variables
//
// block_from_spark holds the raw data from the Spark amp and data is processed in-place
// block_from_app holds the raw data from the app and data is processed in-place
// ------------------------------------------------------------------------------------------------------------
#define BLOCK_SIZE 1000
byte block_from_spark[BLOCK_SIZE];
byte block_from_app[BLOCK_SIZE];

#define HEADER_LEN 6
#define CHUNK_HEADER_LEN 6

// ------------------------------------------------------------------------------------------------------------
// Routines to dump full blocks of data
// ------------------------------------------------------------------------------------------------------------

void dump_raw_block(byte *block, int block_length) {
  Serial.print("Raw block - length: ");
  Serial.println(block_length);

  int lc = 8;
  for (int i = 0; i < block_length; i++) {
    byte b = block[i];
    // 0xf001 chunk header
    if (b == 0xf0) {
      Serial.println();
      lc = 6;
    }
    // 0x01fe block header
    if (b == 0x01 && block[i+1] == 0xfe) {
      lc = 16;
      Serial.println();
    }
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
    if (--lc == 0) {
      Serial.println();
      lc = 8;
    }
  }
  Serial.println();
}

void dump_processed_block(byte *block, int block_length) {
  Serial.print("Processed block: length - ");
  Serial.println(block_length);

  int pos = 0;
  int len = 0;
  int lc;
  byte b;

  while (pos < block_length) {
    if (len == 0) {
      len = (block[pos+2] << 8) + block[pos+3];
      lc = HEADER_LEN;
      Serial.println();
    }
    b = block[pos];
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
    if (--lc == 0) {
      Serial.println();
      lc = 8;
    }
    len--;
    pos++;
  }
  Serial.println();
}


// ------------------------------------------------------------------------------------------------------------
// Global variables
//
// last_sequence_to_spark used for a response to a 0x0201 request for a preset - must have the same sequence in the response
// ------------------------------------------------------------------------------------------------------------
int last_sequence_to_spark;


// ------------------------------------------------------------------------------------------------------------
// Routines to process blocks of data to get to msgpack format
//
// remove_headers() - remove the 01fe and f001 headers, add 6 byte header to packet
// fix_bit_eight() - add the missing eighth bit to each data byte
// compact()       - remove the multi-chunk header and the eighth bit byte to get to msgpack data
// ------------------------------------------------------------------------------------------------------------

void clone(byte *to, byte *from, int len) {
  memcpy(to, from, len);
}

// remove_headers())
// Removes any headers (0x01fe and 0xf001) from the packets and leaves the rest
// Each new data block starts with a 6 byte SparkIO header
// 0  command
// 1  sub-command
// 2  total block length (inlcuding this header) (msb)
// 3  total block length (including this header) (lsb)
// 4  number of checksum errors in the original block
// 5  sequence number of the original block

int remove_headers(byte *out_block, byte *in_block, int in_len) {
  int new_len  = 0;
  int in_pos   = 0;
  int out_pos  = 0;
  int out_base = 0; 
  int last_sequence = -1;

  byte chk;

  byte sequence;
  byte command;
  byte sub_command;
  byte checksum;

  while (in_pos < in_len) {
    // check for 0xf7 chunk ending
    if (in_block[in_pos] == 0xf7) {
       in_pos++;
       out_block[out_base + 2] = (out_pos - out_base) >> 8;
       out_block[out_base + 3] = (out_pos - out_base) & 0xff;
       out_block[out_base + 4] += (checksum != chk);
       out_block[out_base + 5] = sequence;
    }    
    // check for 0x01fe start of Spark 40 16-byte block header
    else if (in_block[in_pos] == 0x01 && in_block[in_pos + 1] == 0xfe) {
      in_pos += 16;
    }
    // check for 0xf001 chunk header
    else if (in_block[in_pos] == 0xf0 && in_block[in_pos + 1] == 0x01) {
      sequence    = in_block[in_pos + 2];
      checksum    = in_block[in_pos + 3];
      command     = in_block[in_pos + 4];
      sub_command = in_block[in_pos + 5];


      chk = 0;
      in_pos += 6;

      if (sequence != last_sequence) {
        last_sequence = sequence;
        out_base = out_pos;                     // move to end of last data
        out_pos  = out_pos + HEADER_LEN;        // save space for header      
        out_block[out_base]     = command;
        out_block[out_base + 1] = sub_command;
        out_block[out_base + 4] = 0;
      }
    }
    else {
      out_block[out_pos] = in_block[in_pos];
      chk ^= in_block[in_pos];
      in_pos++;
      out_pos++;
    }
  }
  // keep a global record of the sequence number for a response to an 0x0201
  last_sequence_to_spark = sequence;

  return out_pos;
}

void fix_bit_eight(byte *in_block, int in_len) {
  int len = 0;
  int in_pos = 0;
  int counter = 0;
  int command = 0;

  byte bitmask;
  byte bits;
  int cmd_sub = 0;

  while (in_pos < in_len) {
    if (len == 0) {
      command = (in_block[in_pos] << 8) + in_block[in_pos];
      len = (in_block[in_pos + 2] << 8) + in_block[in_pos + 3];
      in_pos += HEADER_LEN;
      len    -= HEADER_LEN;
      counter = 0;
    }
    else {
      if (counter % 8 == 0) {
        bitmask = 1;
        bits = in_block[in_pos];
      }
      else {
        if (bits & bitmask) {
          in_block[in_pos] |= 0x80;
        }
        bitmask <<= 1;
      }
      counter++;
      len--;
      in_pos++;
    }
    if (command == 0x0101 && counter == 150) 
      counter = 0;
    if (command == 0x0301 && counter == 32) 
      counter = 0;    

  }
}


int compact(byte *out_block, byte *in_block, int in_len) {
  int len = 0;
  int in_pos = 0;
  int out_pos = 0;
  int counter = 0;
  int out_base = 0;

  int total_chunks;
  int this_chunk;
  int data_len;

  int command = 0;

  while (in_pos < in_len) {
    if (len == 0) {
      // start of new block so prepare header and new out_base pointer
      out_base = out_pos;
      command = (in_block[in_pos] << 8) +     in_block[in_pos + 1];
      len     = (in_block[in_pos + 2] << 8) + in_block[in_pos + 3];
      // fill in the out header (length will change!)
      memcpy(&out_block[out_base], &in_block[in_pos], HEADER_LEN);      
      in_pos  += HEADER_LEN;
      out_pos += HEADER_LEN;
      len     -= HEADER_LEN;
      counter = 0;

    }
    // if len is not 0
    else {
      // this is the bitmask, so we won't copy it
      if (counter % 8 == 0) {      
        in_pos++;
      }
      // this is the multi-chunk header - perhaps do some checks on this in future
      else if ((command == 0x0301 || command == 0x0101 ) && (counter >= 1 && counter <= 3)) { 
        if (counter == 1) total_chunks = in_block[in_pos++];
        if (counter == 2) this_chunk   = in_block[in_pos++];
        if (counter == 3) data_len     = in_block[in_pos++];         
      }
      // otherwise we can copy it
      else { 
        out_block[out_pos] = in_block[in_pos];
        out_pos++;
        in_pos++;
      }
      counter++;
      len--;
      // if at end of the block, update the header length
      if (len == 0) {
        out_block[out_base + 2] = (out_pos - out_base) >> 8;
        out_block[out_base + 3] = (out_pos - out_base) & 0xff;
      }
      if (command == 0x0101 && counter == 150) 
        counter = 0;
      if (command == 0x0301 && counter == 32) 
        counter = 0;    
    }
  }
  return out_pos;
}

// ------------------------------------------------------------------------------------------------------------
// Routines to process the raw data into msgpack format and put into the message buffer
// ------------------------------------------------------------------------------------------------------------

void spark_process() 
{
  int len;
  int trim_len;

  if (last_spark_was_bad) {
    last_spark_was_bad = false;
    spark_msg_in.clear(); 
    DEBUG("Spark sent a bad block");
  }

  if (got_spark_block) {
    // swiftly make a copy of everything and 'free' the ble block
    len = from_spark_index;
    clone(block_from_spark, from_spark, len);

    // these are from SparkComms - should think of a better approach
    got_spark_block = false;
    last_spark_was_bad = false;
    from_spark_index = 0;

    //dump_raw_block(block_from_spark, len);   
    trim_len = remove_headers(block_from_spark, block_from_spark, len);
    fix_bit_eight(block_from_spark, trim_len);
    len = compact(block_from_spark, block_from_spark, trim_len);
    //dump_processed_block(block_from_spark, len);

    spark_msg_in.set_from_array(block_from_spark, len); 
  }
}


void app_process() 
{
  int len;
  int trim_len;

  if (last_app_was_bad) {
    last_app_was_bad = false;
    app_msg_in.clear(); 
    DEBUG("App sent a bad block");
  }

  if (got_app_block) {
    // swiftly make a copy of everything and 'free' the ble block
    len = from_app_index;
    clone(block_from_app, from_app, len);

    // these are from SparkComms - should think of a better approach
    got_app_block = false;
    last_app_was_bad = false;
    from_app_index = 0;

    //dump_raw_block(block_from_app, len); 
    trim_len = remove_headers(block_from_app, block_from_app, len);
    fix_bit_eight(block_from_app, trim_len);
    len = compact(block_from_app, block_from_app, trim_len);
    //dump_processed_block(block_from_app, len);

    app_msg_in.set_from_array(block_from_app, len); 
  }
}

void process_sparkIO() {
  spark_comms_process();
  spark_process();
  app_process();
}

// ------------------------------------------------------------------------------------------------------------
// MessageIn class
// 
// Message formatting routines to read the msgpack 
// Read messages from the in_message ring buffer and copy to a SparkStructure format
// ------------------------------------------------------------------------------------------------------------

void MessageIn::set_from_array(uint8_t *in, int size) {
  in_message.set_from_array(in, size);
}

void MessageIn::clear() {
  in_message.clear();
}


void MessageIn::read_byte(uint8_t *b)
{
  uint8_t a;
  in_message.get(&a);
  *b = a;
}   

void MessageIn::read_uint(uint8_t *b)
{
  uint8_t a;
  in_message.get(&a);
  if (a == 0xCC)
    in_message.get(&a);
  *b = a;
}
   
void MessageIn::read_string(char *str)
{
  uint8_t a, len;
  int i;

  read_byte(&a);
  if (a == 0xd9) {
    read_byte(&len);
  }
  else if (a >= 0xa0) {
    len = a - 0xa0;
  }
  else {
    read_byte(&a);
    if (a < 0xa0 || a >= 0xc0) DEBUG("Bad read_string");
    len = a - 0xa0;
  }

  if (len > 0) {
    // process whole string but cap it at STR_LEN-1
    for (i = 0; i < len; i++) {
      read_byte(&a);
      if (a<0x20 || a>0x7e) a=0x20; // make sure it is in ASCII range - to cope with get_serial 
      if (i < STR_LEN -1) str[i]=a;
    }
    str[i > STR_LEN-1 ? STR_LEN-1 : i]='\0';
  }
  else {
    str[0]='\0';
  }
}   

void MessageIn::read_prefixed_string(char *str)
{
  uint8_t a, len;
  int i;

  read_byte(&a); 
  read_byte(&a);

  if (a < 0xa0 || a >= 0xc0) DEBUG("Bad read_prefixed_string");
  len = a-0xa0;

  if (len > 0) {
    for (i = 0; i < len; i++) {
      read_byte(&a);
      if (a<0x20 || a>0x7e) a=0x20; // make sure it is in ASCII range - to cope with get_serial 
      if (i < STR_LEN -1) str[i]=a;
    }
    str[i > STR_LEN-1 ? STR_LEN-1 : i]='\0';
  }
  else {
    str[0]='\0';
  }
}   

void MessageIn::read_float(float *f)
{
  union {
    float val;
    byte b[4];
  } conv;   
  uint8_t a;
  int i;

  read_byte(&a);  // should be 0xca
  if (a != 0xca) return;

  // Seems this creates the most significant byte in the last position, so for example
  // 120.0 = 0x42F00000 is stored as 0000F042  
   
  for (i=3; i>=0; i--) {
    read_byte(&a);
    conv.b[i] = a;
  } 
  *f = conv.val;
}

void MessageIn::read_onoff(bool *b)
{
  uint8_t a;
   
  read_byte(&a);
  if (a == 0xc3)
    *b = true;
  else // 0xc2
    *b = false;
}

// The functions to get the message

bool MessageIn::get_message(unsigned int *cmdsub, SparkMessage *msg, SparkPreset *preset)
{
  uint8_t cmd, sub, len_h, len_l;
  uint8_t sequence;
  uint8_t chksum_errors;

  unsigned int len;
  unsigned int cs;
   
  uint8_t junk;
  int i, j;
  uint8_t num, num1, num2;

  if (in_message.is_empty()) return false;

  read_byte(&cmd);
  read_byte(&sub);
  read_byte(&len_h);
  read_byte(&len_l);
  read_byte(&chksum_errors);
  read_byte(&sequence);


  bytes_to_uint(len_h, len_l, &len);
  bytes_to_uint(cmd, sub, &cs);

  if (chksum_errors > 0) {
    DEBUG("Got a checksum error - need to skip this chunk");
    for (i = HEADER_LEN; i < len; i++) read_byte(&junk);
    return false;
  }


  *cmdsub = cs;
  switch (cs) {

    // 0x02 series - requests
    // get preset information
    case 0x0201:
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      for (i=0; i < 30; i++) read_byte(&junk); // 30 bytes of 0x00
      break;            
    // get current hardware preset number - this is a request with no payload
    case 0x0210:
      break;
    // get amp name - no payload
    case 0x0211:
      break;
    // get name - this is a request with no payload
    case 0x0221:
      break;
    // get serial number - this is a request with no payload
    case 0x0223:
      break;
    // the UNKNOWN command - 0x0224 00 01 02 03
    case 0x0224:
    case 0x022a:
    case 0x032a:
      // the data is a fixed array of four bytes (0x94 00 01 02 03)
      read_byte(&junk);
      read_uint(&msg->param1);
      read_uint(&msg->param2);
      read_uint(&msg->param3);
      read_uint(&msg->param4);
    // get firmware version - this is a request with no payload
    case 0x022f:
      break;
    // change effect parameter
    case 0x0104:
      read_string(msg->str1);
      read_byte(&msg->param1);
      read_float(&msg->val);
      break;
    // change of effect model
    case 0x0306:
    case 0x0106:
      read_string(msg->str1);
      read_string(msg->str2);
      break;
    // get current hardware preset number
    case 0x0310:
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      break;
    // get name
    case 0x0311:
      read_string(msg->str1);
      break;
    // enable / disable an effect
    // and 0x0128 amp info command
    case 0x0315:
    case 0x0115:
    case 0x0128:
      read_string(msg->str1);
      read_onoff(&msg->onoff);
      break;
    // get serial number
    case 0x0323:
      read_string(msg->str1);
      break;
    // store into hardware preset
    case 0x0327:
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      break;
    // amp info   
    case 0x0328:
      read_float(&msg->val);
      break;  
    // firmware version number
    case 0x032f:
      // really this is a uint32 but it is just as easy to read into 4 uint8 - a bit of a cheat
      read_byte(&junk);           // this will be 0xce for a uint32
      read_byte(&msg->param1);      
      read_byte(&msg->param2); 
      read_byte(&msg->param3); 
      read_byte(&msg->param4); 
      break;
    // change of effect parameter
    case 0x0337:
      read_string(msg->str1);
      read_byte(&msg->param1);
      read_float(&msg->val);
      break;
    // tuner
    case 0x0364:
      read_byte(&msg->param1);
      read_float(&msg->val);
      break;
    case 0x0365:
      read_onoff(&msg->onoff);
      break;
    // change of preset number selected on the amp via the buttons
    case 0x0338:
    case 0x0138:
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      break;
    // license key
    case 0x0170:
      for (i = 0; i < 64; i++)
        read_uint(&license_key[i]);
      break;
    // response to a request for a full preset
    case 0x0301:
    case 0x0101:
      read_byte(&preset->curr_preset);
      read_byte(&preset->preset_num);
      read_string(preset->UUID); 
      read_string(preset->Name);
      read_string(preset->Version);
      read_string(preset->Description);
      read_string(preset->Icon);
      read_float(&preset->BPM);

      for (j=0; j < 7; j++) {
        read_string(preset->effects[j].EffectName);
        read_onoff(&preset->effects[j].OnOff);
        read_byte(&num);
        preset->effects[j].NumParameters = num - 0x90;
        for (i = 0; i < preset->effects[j].NumParameters; i++) {
          read_byte(&junk);
          read_byte(&junk);
          read_float(&preset->effects[j].Parameters[i]);
        }
      }
      read_byte(&preset->chksum);  

      break;
    // tap tempo!
    case 0x0363:
      read_float(&msg->val);  
      break;
    case 0x0470:
    case 0x0428:
      read_byte(&junk);
      break;
    // acks - no payload to read - no ack sent for an 0x0104
    case 0x0401:
    case 0x0501:
    case 0x0406:
    case 0x0415:
    case 0x0438:
    case 0x0465:
    // Serial.print("Got an ack ");
    // Serial.println(cs, HEX);
      break;




    // LIVE includes 0x031a with 0x0338 with change to preset via HW Button
    // Not sure what this represents, but it is an array of: byte byte boolean
    case 0x031a:
      DEBUG("LIVE message 0x031a");
      read_byte(&num);
      num -= 0x90;  // should be a fixed array
      DEB("Fixed array size: ");
      DEBUG(num);
      // Assume size 2 for now
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      read_onoff(&msg->bool1);
      read_byte(&msg->param3);
      read_byte(&msg->param4);
      read_onoff(&msg->bool2);   
      DEBUG(msg->param1);
      DEBUG(msg->param2);
      if (msg->bool1) DEBUG("On"); else DEBUG("Off");
      DEBUG(msg->param3);
      DEBUG(msg->param4);
      if (msg->bool2) DEBUG("On"); else DEBUG("Off");
      if (num != 2)
        in_message.clear();   // clear rest of message as we haven't used it
      break;
    // LIVE INPUT 1 Guitar Volume
    case 0x036b:
      DEB("LIVE INPUT 1 Guitar Volume ");
      read_float(&msg->val);
      DEBUG(msg->val);
      break;
    // LIVE Mixer
    case 0x0333:
      DEB("LIVE Mixer ");
      read_float(&msg->val);
      DEBUG(msg->val);
      break;
    // LIVE INPUT 2 Cable Insert
    case 0x0374:
      DEB("LIVE INPUT 2 Cable Insert");
      read_byte(&num);
      num -= 0x90;  // should be a fixed array
      DEB("Fixed array size: ");
      DEBUG(num);
      // Assume size 1 for now
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      DEBUG(msg->param1);
      DEBUG(msg->param1);
      if (num != 1)
        in_message.clear();   // clear rest of message as we haven't used it
      break;
    case 0x0373:
      DEB("LIVE INPUT 2 Cable Insert");
      read_byte(&num);
      num -= 0x90;  // should be a fixed array
      DEB("Fixed array size: ");
      DEBUG(num);
      // Assume size 2 for now
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      read_onoff(&msg->bool1);
      read_byte(&msg->param3);
      read_byte(&msg->param4);
      read_onoff(&msg->bool2);   
      DEBUG(msg->param1);
      DEBUG(msg->param2);
      if (msg->bool1) DEBUG("On"); else DEBUG("Off");
      DEBUG(msg->param3);
      DEBUG(msg->param4);
      if (msg->bool2) DEBUG("On"); else DEBUG("Off");
      if (num != 2)
        in_message.clear();   // clear rest of message as we haven't used it
      break;


    case 0x0371:
      DEBUG("undefined Battery?");
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      read_byte(&msg->param3);
      read_byte(&msg->param4);   
      read_byte(&num);  
      read_byte(&num); 
      DEBUG(num, HEX);    // should be 0xCD
      read_byte(&num1);
      read_byte(&num2);
      bytes_to_uint(num1, num2, &msg->param5);
      read_byte(&msg->param6);    
      read_byte(&msg->param7);   
      DEBUG(msg->param5, HEX);   
      in_message.clear();
      break;


    // unprocessed Spark GO/MINI messages
    case 0x0272:
    case 0x0372:
    case 0x0271:
    case 0x0304:
    case 0x0204:
      break;
    // Power Settings for MINI, GO, LIVE
    case 0x0472:
      DEBUG("undefined Power Settings");
      break;

    // LIVE INPUT Impedence
    case 0x0474:
      DEBUG("undefined LIVE Input Impedence");
      break;
    // unprocessed LIVE Connection Messages
    case 0x022b:
    case 0x032b:
      DEBUG("undefined LIVE connection messages");
      break;


    default:
      Serial.print("Unprocessed message ");
      Serial.print(cs, HEX);
      Serial.print(" length ");
      Serial.print(len);

      Serial.print(":");
      if (len != 0) {
        for (i = 0; i < len - 4; i++) {
          read_byte(&junk);
          Serial.print(junk, HEX);
          Serial.print(" ");
        }
      }
      Serial.println();
      // defensively clear the message buffer in case this is a bug
      in_message.clear();
  }
  return true;
}


// used when sending a preset to Spark to see if a block has been received, will lose the messages until acknowledgement one
bool MessageIn::check_for_acknowledgement() {
  uint8_t cmd, sub, len_h, len_l;
  uint8_t sequence;
  uint8_t chksum_errors;

  unsigned int len;
  unsigned int cs;
   
  uint8_t junk;
  int i, j;

  if (in_message.is_empty()) return false;

  read_byte(&cmd);
  read_byte(&sub);
  read_byte(&len_h);
  read_byte(&len_l);
  read_byte(&chksum_errors);
  read_byte(&sequence);

  bytes_to_uint(len_h, len_l, &len);
  bytes_to_uint(cmd, sub, &cs);

  for (i = HEADER_LEN; i < len; i++) read_byte(&junk);
  if (cs == 0x0401 || cs == 0x0501)  return true;
  return false;
};


// ------------------------------------------------------------------------------------------------------------
// MessageOut class
// 
// Message formatting routines to create the msgpack 
// ead messages from the SparkStructure format and place into the out_message ring buffer 
// ------------------------------------------------------------------------------------------------------------

bool MessageOut::has_message() {
  return !out_message.is_empty();
}

void MessageOut::start_message(int cmdsub)
{
  int om_cmd, om_sub;
  
  om_cmd = (cmdsub & 0xff00) >> 8;
  om_sub = cmdsub & 0xff;

  out_message.add(om_cmd);
  out_message.add(om_sub);
  out_message.add(0);      // placeholder for length
  out_message.add(0);      // placeholder for length
  out_message.add(0);      // placeholder for checksum errors
  out_message.add(0x60);   // placeholder for sequence number - setting to 0 will not work!

  out_msg_chksum = 0;
}

void MessageOut::end_message()
{
  unsigned int len;
  uint8_t len_h, len_l;
  
  len = out_message.get_len();
  uint_to_bytes(len, &len_h, &len_l);
  
  out_message.set_at_index(2, len_h);   
  out_message.set_at_index(3, len_l);
  out_message.commit();
}

void MessageOut::write_byte_no_chksum(byte b)
{
  out_message.add(b);
}

void MessageOut::write_byte(byte b)
{
  out_message.add(b);
  out_msg_chksum += int(b);
}

void MessageOut::write_uint(byte b)
{
  if (b > 127) {
    out_message.add(0xCC);
    out_msg_chksum += int(0xCC);  
  }
  out_message.add(b);
  out_msg_chksum += int(b);
}

void MessageOut::write_uint32(uint32_t w)
{
  int i;
  uint8_t b;
  uint32_t mask;

  mask = 0xFF000000;
  
  out_message.add(0xCE);
  out_msg_chksum += int(0xCE);  

  for (i = 3; i >= 0; i--) {
    b = (w & mask) >> (8*i);
    mask >>= 8;
    write_uint(b);
//    out_message.add(b);
//    out_msg_chksum += int(b);
  }
}


void MessageOut::write_prefixed_string(const char *str)
{
  int len, i;

  len = strnlen(str, STR_LEN);
  write_byte(byte(len));
  write_byte(byte(len + 0xa0));
  for (i=0; i<len; i++)
    write_byte(byte(str[i]));
}

void MessageOut::write_string(const char *str)
{
  int len, i;

  len = strnlen(str, STR_LEN);
  write_byte(byte(len + 0xa0));
  for (i=0; i<len; i++)
    write_byte(byte(str[i]));
}      
  
void MessageOut::write_long_string(const char *str)
{
  int len, i;

  len = strnlen(str, STR_LEN);
  write_byte(byte(0xd9));
  write_byte(byte(len));
  for (i=0; i<len; i++)
    write_byte(byte(str[i]));
}

void MessageOut::write_float (float flt)
{
  union {
    float val;
    byte b[4];
  } conv;
  int i;
   
  conv.val = flt;
  // Seems this creates the most significant byte in the last position, so for example
  // 120.0 = 0x42F00000 is stored as 0000F042  
   
  write_byte(0xca);
  for (i=3; i>=0; i--) {
    write_byte(byte(conv.b[i]));
  }
}

void MessageOut::write_onoff (bool onoff)
{
  byte b;

  if (onoff)
  // true is 'on'
    b = 0xc3;
  else
    b = 0xc2;
  write_byte(b);
}


void MessageOut::change_effect_parameter (char *pedal, int param, float val)
{
   if (cmd_base == 0x0100) 
     start_message (cmd_base + 0x04);
   else
     start_message (cmd_base + 0x37);
   write_prefixed_string (pedal);
   write_byte (byte(param));
   write_float(val);
   end_message();
}

void MessageOut::change_effect (char *pedal1, char *pedal2)
{
   start_message (cmd_base + 0x06);
   write_prefixed_string (pedal1);
   write_prefixed_string (pedal2);
   end_message();
}



void MessageOut::change_hardware_preset (uint8_t curr_preset, uint8_t preset_num)
{
   // preset_num is 0 to 3

   start_message (cmd_base + 0x38);
   write_byte (curr_preset);
   write_byte (preset_num)  ;     
   end_message();  
}


void MessageOut::turn_effect_onoff (char *pedal, bool onoff)
{
   start_message (cmd_base + 0x15);
   write_prefixed_string (pedal);
   write_onoff (onoff);
   end_message();
}

void MessageOut::get_serial()
{
   start_message (0x0223);
   end_message();  
}

void MessageOut::get_name()
{
   start_message (0x0211);
   end_message();  
}

void MessageOut::get_hardware_preset_number()
{
   start_message (0x0210);
   end_message();  
}

void MessageOut::get_checksum_info() {
   start_message (0x022a);
   write_byte(0x94);
   write_uint(0);
   write_uint(1);
   write_uint(2);
   write_uint(3);   
   end_message();   
}

void MessageOut::get_firmware() {
   start_message (0x022f);
   end_message(); 
}

void MessageOut::save_hardware_preset(uint8_t curr_preset, uint8_t preset_num)
{
   start_message (cmd_base + 0x27);
//   start_message (0x0327);
   write_byte (curr_preset);
   write_byte (preset_num);  
   end_message();
}

void MessageOut::send_firmware_version(uint32_t firmware)
{
   start_message (0x032f);
   write_uint32(firmware);  
   end_message();
}

void MessageOut::send_serial_number(char *serial)
{
   start_message (0x0323);
   write_prefixed_string(serial);
   end_message();
}

void MessageOut::send_ack(unsigned int cmdsub) {
   start_message (cmdsub);
   end_message();
}

void MessageOut::send_0x022a_info(byte v1, byte v2, byte v3, byte v4)
{
   start_message (0x032a);
   write_byte(0x94);
   write_uint(v1);
   write_uint(v2);
   write_uint(v3);
   write_uint(v4);      
   end_message();
}

void MessageOut::send_key_ack()
{
   start_message (0x0470);
   write_byte(0x00);
   end_message();
}

void MessageOut::send_preset_number(uint8_t preset_h, uint8_t preset_l)
{
   start_message (0x0310);
   write_byte(preset_h);
   write_byte(preset_l);
   end_message();
}

void MessageOut::send_tap_tempo(float val)
{
   if (cmd_base == 0x0100) 
     start_message (cmd_base + 0x62);
   else
     start_message (cmd_base + 0x63);   // is this right??
   write_float(val);
   write_byte(0x3f);
   write_byte(0x3f);
   end_message();
}


void MessageOut::tuner_on_off(bool onoff)
{
   start_message (0x0165);
   write_onoff (onoff);
   end_message();
}

void MessageOut::get_preset_details(unsigned int preset)
{
   int i;
   uint8_t h, l;

   uint_to_bytes(preset, &h, &l);
   
   start_message (0x0201);
   write_byte(h);
   write_byte(l);

   for (i=0; i<30; i++) {
     write_byte(0);
   }
   
   end_message(); 
}

void MessageOut::create_preset(SparkPreset *preset)
{
  int i, j, siz;

  start_message (cmd_base + 0x01);

  write_byte_no_chksum (0x00);
  write_byte_no_chksum (preset->preset_num);   
  write_long_string (preset->UUID);
  //write_string (preset->Name);
  if (strnlen (preset->Name, STR_LEN) > 31)
    write_long_string (preset->Name);
  else
    write_string (preset->Name);
    
  write_string (preset->Version);
  if (strnlen (preset->Description, STR_LEN) > 31)
    write_long_string (preset->Description);
  else
    write_string (preset->Description);
  write_string(preset->Icon);
  write_float (preset->BPM);
   
  write_byte (byte(0x90 + 7));       // always 7 pedals
  for (i=0; i<7; i++) {
    write_string (preset->effects[i].EffectName);
    write_onoff(preset->effects[i].OnOff);

    siz = preset->effects[i].NumParameters;
    write_byte ( 0x90 + siz); 
      
    for (j=0; j<siz; j++) {
      write_byte (j);
      write_byte (byte(0x91));
      write_float (preset->effects[i].Parameters[j]);
    }
  }
  write_byte_no_chksum (uint8_t(out_msg_chksum % 256));  
  end_message();
}

void MessageOut::copy_message_to_array(byte *blk, int *len) {
  out_message.copy_to_array(blk, len);
}

// ------------------------------------------------------------------------------------------------------------
// Routines to process the msgpack format into Spark blocks
// ------------------------------------------------------------------------------------------------------------

int expand(byte *out_block, byte *in_block, int in_len) {
  int len = 0;
  int in_pos = 0;
  int out_pos = 0;
  int counter = 0;

  int total_chunks;
  int this_chunk;
  int this_len;
  int chunk_size;
  int last_chunk_size;

  bool multi;

  int command = 0;
  uint8_t cmd = 0;
  uint8_t sub = 0;
  uint8_t sequence = 0;

  int i;

  cmd = in_block[in_pos];
  sub = in_block[in_pos + 1];
  command = (cmd << 8) + sub;
  len = (in_block[in_pos + 2] << 8) + in_block[in_pos + 3];
  len -= HEADER_LEN;
  sequence = in_block[in_pos + 5];
  
  in_pos += HEADER_LEN;
  chunk_size = len;
  multi = false;
  total_chunks = 1;
  last_chunk_size = len;

  if (command == 0x0101) {
    chunk_size = 128;
    multi = true;
  }
  if (command == 0x0301) {
    chunk_size = 25;
    multi = true;
  }
  // using the global as this should be a response to a 0x201
  // TODO - think of a better way to do this
  if (command == 0x0301) 
    sequence = last_sequence_to_spark;

  if (multi)
    total_chunks = int((len - 1) / chunk_size) + 1;

  if (chunk_size != 0) 
    last_chunk_size = len % chunk_size;
  else
    last_chunk_size = 0;

  if (last_chunk_size == 0) last_chunk_size = chunk_size;   // an exact number of bytes into the chunks

  for (this_chunk = 0; this_chunk < total_chunks; this_chunk++) {
    this_len = (this_chunk == total_chunks - 1) ? last_chunk_size : chunk_size;     // how big is the last chunk
    
    out_block[out_pos++] = 0xf0;
    out_block[out_pos++] = 0x01;
    out_block[out_pos++] = sequence;
    out_block[out_pos++] = 0;            // space for checksum
    out_block[out_pos++] = cmd;
    out_block[out_pos++] = sub;

    counter = 0;
    // do each data byte
    for (i = 0; i < this_len; i++) {
      if (counter % 8 == 0) {
        out_block[out_pos++] = 0;        // space for bitmap
        counter++;
      }
      if (multi && counter == 1) {       // counter == 1 because we have a first bitmap space
        out_block[out_pos++] = total_chunks;
        out_block[out_pos++] = this_chunk;
        out_block[out_pos++] = this_len;
        counter += 3;
      }
      out_block[out_pos++] = in_block[in_pos++];
      counter++;
    }
    out_block[out_pos++] = 0xf7;
  }
  return out_pos;
}

void add_bit_eight(byte *in_block, int in_len) {
  int in_pos = 0;
  int bit_pos;
  int counter = 0;

  int total_chunks;
  int this_chunk;
  int this_len;
  int chunk_size;
  int last_chunk_size;

  int checksum_pos;
  int checksum;

  bool multi;

  int command = 0;
  uint8_t cmd = 0;
  uint8_t sub = 0;

  byte bitmask;

  int i;

  cmd = in_block[in_pos + 4];
  sub = in_block[in_pos + 5];
  command = (cmd << 8) + sub;

  in_pos = 0;
  chunk_size = in_len;
  multi = false;
  total_chunks = 1;

  if (command == 0x0101) {
    chunk_size = 157;
    multi = true;
  }
  if (command == 0x0301) {
    chunk_size = 39;
    multi = true;
  }
  if (multi)
    total_chunks = int((in_len - 1) / chunk_size) + 1;

  last_chunk_size = in_len % chunk_size;
  if (last_chunk_size == 0) last_chunk_size = chunk_size;   // an exact number of bytes into the chunks


  for (this_chunk = 0; this_chunk < total_chunks; this_chunk++) {
    this_len = (this_chunk == total_chunks - 1) ? last_chunk_size : chunk_size;     
    counter = 0;
    checksum = 0;
    checksum_pos = in_pos + 3;
    in_pos += CHUNK_HEADER_LEN;
    // do each data byte
    for (i = CHUNK_HEADER_LEN; i < this_len - 1; i++) {   // skip header and trailing f7
      if (counter % 8 == 0) {
        in_block[in_pos] = 0;        // space for bitmap
        bitmask = 1;
        bit_pos = in_pos;
      }
      else {
        checksum ^= in_block[in_pos] & 0x7f;
        if (in_block[in_pos] & 0x80) {
          in_block[in_pos] &= 0x7f;
          in_block[bit_pos] |= bitmask;
          checksum ^= bitmask;
        };
        bitmask <<= 1;
      };
      counter++;
      in_pos++;
    }
    in_block[checksum_pos] = checksum;
    in_pos++;    // skip the trailing f7
  }
}  
 
uint8_t header_to_app[]    {0x01, 0xfe, 0x00, 0x00, 0x41, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t header_to_spark[]  {0x01, 0xfe, 0x00, 0x00, 0x53, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

int add_headers(byte *out_block, byte *in_block, int in_len) {
  int in_pos;
  int out_pos;

  int total_chunks;
  int this_chunk;
  int this_len;
  int chunk_size;
  int last_chunk_size;

  int command;
  uint8_t cmd;
  uint8_t sub;

  int i;

  in_pos = 0;
  out_pos = 0;

  cmd = in_block[in_pos + 4];
  sub = in_block[in_pos + 5];
  command = (cmd << 8) + sub;

  chunk_size = in_len;            // default if not a multi-chunk message
  total_chunks = 1;

  if (command == 0x0101) chunk_size = 157;
  if (command == 0x0301) chunk_size = 90;

  total_chunks = int((in_len - 1) / chunk_size) + 1;

  last_chunk_size = in_len % chunk_size;
  if (last_chunk_size == 0) last_chunk_size = chunk_size;   // an exact number of bytes into the chunks

  for (this_chunk = 0; this_chunk < total_chunks; this_chunk++) {
    this_len = (this_chunk == total_chunks - 1) ? last_chunk_size : chunk_size;   
    if (cmd == 0x01 || cmd == 0x02) {
      // sending to the amp
      memcpy(&out_block[out_pos], header_to_spark, 16);
    }
    else {
      memcpy(&out_block[out_pos], header_to_app, 16);
    };
    out_block[out_pos + 6] = this_len + 16;
    out_pos += 16;
    memcpy(&out_block[out_pos], &in_block[in_pos], this_len);
    out_pos += this_len;
    in_pos += this_len;
  }
  return out_pos;
}


// ------------------------------------------------------------------------------------------------------------
// Routines to send to the app and the amp
// ------------------------------------------------------------------------------------------------------------

// only need one block out as we won't send to app and amp at same time

byte block_out[BLOCK_SIZE];
byte block_out_temp[BLOCK_SIZE];

void spark_send() {
  int len;
  byte direction;

  int this_block;
  int num_blocks;
  int block_size;

  int last_block_len;
  int this_len;

  if (spark_msg_out.has_message()) {
    spark_msg_out.copy_message_to_array(block_out, &len);
    len = expand(block_out_temp, block_out, len);
    add_bit_eight(block_out_temp, len);
    len = add_headers(block_out, block_out_temp, len);

    // with the 16 byte header, position 4 is 0x53fe for data being sent to Spark, and 0x41ff for data going to the app
    // although should be onvious from the call used eg spark_send() sends to spark
    direction = block_out[4];
    if (direction == 0x53)      block_size = 173;
    else if (direction == 0x41) block_size = 106;

    num_blocks = int ((len - 1) / block_size) + 1;
    last_block_len = len % block_size;
    for (this_block = 0; this_block < num_blocks; this_block++) {
      this_len = (this_block == num_blocks - 1) ? last_block_len : block_size;
      send_to_spark(&block_out[this_block * block_size], this_len);
      //Serial.println("Sent a block");

      if (num_blocks != 1) {   // only do this for the multi blocks
        bool done = false;
        unsigned long t;
        t = millis();
        while (!done && (millis() - t) < 400) {  // add timeout just in case of no acknowledgement
          spark_process();
          done = spark_msg_in.check_for_acknowledgement();
        };
      }                             
    }
  }
}

void app_send() {
  int len;
  byte direction;

  int this_block;
  int num_blocks;
  int block_size;

  int last_block_len;
  int this_len;

  if (app_msg_out.has_message()) {
    app_msg_out.copy_message_to_array(block_out, &len);
    len = expand(block_out_temp, block_out, len);
    add_bit_eight(block_out_temp, len);
    len = add_headers(block_out, block_out_temp, len);

    // with the 16 byte header, position 4 is 0x53fe for data being sent to Spark, and 0x41ff for data going to the app
    // although should be onvious from the call used eg app_send() sends to app
    direction = block_out[4];
    if (direction == 0x53)      block_size = 173;
    else if (direction == 0x41) block_size = 106;

    num_blocks = int ((len - 1) / block_size) + 1;
    last_block_len = len % block_size;
    for (this_block = 0; this_block < num_blocks; this_block++) {
      this_len = (this_block == num_blocks - 1) ? last_block_len : block_size;
      send_to_app(&block_out[this_block * block_size], this_len);
    }
  }
}
