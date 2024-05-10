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
 *    process_sparkIO();
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


// ------------------------------------------------------------------------------------------------------------
// Shared global variables
//
// block_from_spark holds the raw data from the Spark amp and data is processed in-place
// block_from_app holds the raw data from the app and data is processed in-place
// ------------------------------------------------------------------------------------------------------------

#define HEADER_LEN 6
#define CHUNK_HEADER_LEN 6

CircularArray array_spark;
CircularArray array_app;

// ------------------------------------------------------------------------------------------------------------
// Routines to dump full blocks of data
// ------------------------------------------------------------------------------------------------------------

void dump_raw_block(byte *block, int block_length) {
  DEB("Raw block - length: ");
  DEBUG(block_length);

  int lc = 8;
  for (int i = 0; i < block_length; i++) {
    byte b = block[i];
    // 0xf001 chunk header
    if (b == 0xf0) {
      DEBUG();
      lc = 6;
    }
    // 0x01fe block header
    if (b == 0x01 && block[i+1] == 0xfe) {
      lc = 16;
      DEBUG();
    }
    if (b < 16) DEB("0");
    DEB(b, HEX);
    DEB(" ");
    if (--lc == 0) {
      DEBUG();
      lc = 8;
    }
  }
  DEBUG();
}

void dump_processed_block(byte *block, int block_length) {
  DEB("Processed block: length - ");
  DEBUG(block_length);

  int pos = 0;
  int len = 0;
  int lc;
  byte b;

  while (pos < block_length) {
    if (len == 0) {
      len = (block[pos+2] << 8) + block[pos+3];
      lc = HEADER_LEN;
      DEBUG();
    }
    b = block[pos];
    if (b < 16) DEB("0");
    DEB(b, HEX);
    DEB(" ");
    if (--lc == 0) {
      DEBUG();
      lc = 8;
    }
    len--;
    pos++;
  }
  DEBUG();
}

// ------------------------------------------------------------------------------------------------------------ 
// UTILITY FUNCTIONS
// ------------------------------------------------------------------------------------------------------------

void uint_to_bytes(unsigned int i, uint8_t *h, uint8_t *l) {
  *h = (i & 0xff00) / 256;
  *l = i & 0xff;
}

void bytes_to_uint(uint8_t h, uint8_t l,unsigned int *i) {
  *i = (h & 0xff) * 256 + (l & 0xff);
}


// ------------------------------------------------------------------------------------------------------------ 
// MEMORY FUNCTIONS
// ------------------------------------------------------------------------------------------------------------


//#define DEBUG_MEMORY(...)  {char _b[100]; sprintf(_b, __VA_ARGS__); Serial.println(_b);}
//#define DEBUG_HEAP(...)  {char _b[100]; sprintf(_b, __VA_ARGS__); Serial.println(_b);}
#define DEBUG_MEMORY(...) {}
#define DEBUG_HEAP(...) {}

void show_heap() {
  DEBUG_HEAP("Total heap: %d", ESP.getHeapSize());
  DEBUG_HEAP("Free heap: %d", ESP.getFreeHeap());
  DEBUG_HEAP("Total PSRAM: %d", ESP.getPsramSize());
  DEBUG_HEAP("Free PSRAM: %d", ESP.getFreePsram());
}

int memrnd(int mem) {
  int new_mem;
  return mem;

  // can round if we want to!
  if (mem <= 20) new_mem = 20;
  else if (mem <= 100) new_mem = 100;
  else if (mem <= 800) new_mem = 800;
  else new_mem = mem;
  return new_mem;
}


uint8_t *malloc_check(int size) {
#ifdef PSRAM
  uint8_t *p = (uint8_t *) ps_malloc(memrnd(size));
#else
  uint8_t *p = (uint8_t *) malloc(memrnd(size));
#endif
  DEBUG_MEMORY("Malloc: %p %d %d", p, size, memrnd(size));
  if (p == NULL) {
    DEBUG_MEMORY("MALLOC FAILED: %p %d", p, size);
  }
  //show_heap();    
  return p;
}

uint8_t *realloc_check(uint8_t *ptr, int new_size) {
#ifdef PSRAM
  uint8_t *p = (uint8_t *) ps_realloc(ptr, memrnd(new_size));
#else
  uint8_t *p = (uint8_t *) realloc(ptr, memrnd(new_size));
#endif
  DEBUG_MEMORY("Realloc: %p %p %d %d", p, ptr, new_size, memrnd(new_size));
  if (p == NULL) {
    DEBUG_MEMORY("REALLOC FAILED: %p %p %d", p, ptr, new_size);
  }
  //show_heap();    
  return p; 
}

void free_check(uint8_t *ptr) {
  DEBUG_MEMORY("Free: %p", ptr);
  free(ptr);
  //show_heap();     
}

// ------------------------------------------------------------------------------------------------------------
// Packet handling routines
// ------------------------------------------------------------------------------------------------------------

void new_packet(struct packet_data *pd, int length) {
  pd->ptr = (uint8_t *) malloc_check(length) ;
  pd->size = length;
}

void new_packet_from_data(struct packet_data *pd, uint8_t *data, int length) {
  pd->ptr = (uint8_t *) malloc_check(length) ;
  pd->size = length;
  for (int i = 0; i < length; i++)
    pd->ptr[i] = data[i];
}

void clear_packet(struct packet_data *pd) {
  free_check(pd->ptr);
  pd->size = 0; 
}

// ------------------------------------------------------------------------------------------------------------
// Routines to handle validating packets of data from SparkComms before further processing
// Uses the RTOS queues to receive the packets
// ------------------------------------------------------------------------------------------------------------

//#define DEBUG_COMMS(...)  {char _b[100]; sprintf(_b, __VA_ARGS__); Serial.println(_b);}
#define DEBUG_COMMS(...) {}
//#define DEBUG_STATUS(...)  {char _b[100]; sprintf(_b, __VA_ARGS__); Serial.println(_b);}
#define DEBUG_STATUS(...) {}
//#define DUMP_BUFFER(p, s) {for (int _i=0; _i <=  (s); _i++) {Serial.print( (p)[_i], HEX); Serial.print(" ");}; Serial.println();}
#define DUMP_BUFFER(p, s) {}

bool scan_packet (CircularArray &buf, int *start, int *this_end, int end) {
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

  int len = buf.length();
  int p = *start;

  while (!is_done) {
    // check to see if past end of buffer
    if (p > end ) {
      is_done = true;
      is_good = false;
      en = p-1;   // is this ok????
    }
 
    // skip a block header if we find one
    else if (end - p >= 2 && buf[p] == 0x01 && buf[p + 1] == 0xfe) {
      p += 16;
    }
    
    // found start of a message - either single or multi-chunk
    else if (end - p >= 6 && buf [p] == 0xf0 && buf[p + 1] == 0x01) {

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
    
      if (is_multi && end - p >= 9) {
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
  *this_end = en;
  DEBUG_COMMS("Returning start: %3d end: %3d status: %s", st, en, is_good ? "good" : "bad");
  return is_good;
}

void handle_spark_packet() {
  struct packet_data qe; 
  struct packet_data me;
  
  int start, end;
  bool good_packet;
  int good_end;

  int len, trim_len;

  // process packets queued
  while (uxQueueMessagesWaiting(qFromSpark) > 0) {
    lastSparkPacketTime = millis();
    xQueueReceive(qFromSpark, &qe, (TickType_t) 0);

    // passthru
    if (ble_passthru) {
      send_to_app(qe.ptr, qe.size);
    }

    array_spark.append(qe.ptr, qe.size);
    clear_packet(&qe); // this was created in app_callback, no longer needed
  }
  
  end = array_spark.length() - 1;
  start = 0;
  good_end = 0;

  if (array_spark.length() > 0) {  
    if (scan_packet(array_spark, &start, &end, array_spark.length())) {
      DEBUG_COMMS("Got a good packet %d %d", start, end);
      len = end - start + 1;
      int orig_len = len;

      if (start > 0) {
        array_spark.shrink(start);    // clear out any bad data
        end =- start;
      }

      // process the packet to extract the msgpack data
      trim_len = remove_headers(array_spark, array_spark, len);
      fix_bit_eight(array_spark, trim_len);
      len = compact(array_spark, array_spark, trim_len);

      array_spark.extract_append(spark_msg_in.message_in, len, orig_len);
    }
  }

  // check for timeouts and delete the packet, it took too long to get a proper packet
  if ((array_spark.length() > 0) && (millis() - lastSparkPacketTime > SPARK_TIMEOUT)) {
    array_spark.clear();
    Serial.println("CLEARED SPARK");
  }
}

void handle_app_packet() {
  struct packet_data qe; 
  struct packet_data me;
  int start, end;
  bool good_packet;
  int good_end;

  int len, trim_len;

  // process packets queued
  while (uxQueueMessagesWaiting(qFromApp) > 0) {
    lastAppPacketTime = millis();
    xQueueReceive(qFromApp, &qe, (TickType_t) 0);

    if (ble_passthru) {
      send_to_spark(qe.ptr, qe.size);
    }

    array_app.append(qe.ptr, qe.size);
    clear_packet(&qe); // this was created in app_callback, no longer needed
  }

  end = array_app.length() - 1;
  start = 0;
  good_end = 0;

  if (array_app.length() > 0) {  
    if (scan_packet(array_app, &start, &end, array_app.length())) {
      DEBUG_COMMS("Got a good packet %d %d", start, end);
      len = end - start + 1;
      int orig_len = len;

      if (start > 0) {
        array_app.shrink(start);    // clear out any bad data
        end =- start;
      }

      // process the packet to extract the msgpack data
      trim_len = remove_headers(array_app, array_app, len);
      fix_bit_eight(array_app, trim_len);
      len = compact(array_app, array_app, trim_len);

      array_app.extract_append(app_msg_in.message_in, len, orig_len);
    }
  }

  // check for timeouts and delete the packet, it took too long to get a proper packet
  if ((array_app.length() > 0) && (millis() - lastAppPacketTime > APP_TIMEOUT)) {
    array_app.clear();
    Serial.println("CLEARED APP");
  }

}


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

// remove_headers())
// Removes any headers (0x01fe and 0xf001) from the packets and leaves the rest
// Each new data block starts with a 6 byte SparkIO header
// 0  command
// 1  sub-command
// 2  total block length (inlcuding this header) (msb)
// 3  total block length (including this header) (lsb)
// 4  number of checksum errors in the original block
// 5  sequence number of the original block

int remove_headers(CircularArray &out_block, CircularArray &in_block, int in_len) {
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

void fix_bit_eight(CircularArray &in_block, int in_len) {
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

// TODO - this currently can cope with multiple messages in a sequence, but doesn't need to be able to do that any more!!!!

int compact(CircularArray &out_block, CircularArray &in_block, int in_len) {
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

void process_sparkIO() {
  handle_app_packet();
  handle_spark_packet();
}


// ------------------------------------------------------------------------------------------------------------
// MessageIn class
// 
// Message formatting routines to read the msgpack 
// Read messages from the in_message ring buffer and copy to a SparkStructure format
// ------------------------------------------------------------------------------------------------------------

void MessageIn::read_byte(uint8_t *b)
{
  *b = message_in[message_pos++];
}   

void MessageIn::read_uint(uint8_t *b)
{
  uint8_t a;
  read_byte(&a);
  if (a == 0xCC)
    read_byte(&a);
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
  uint8_t num_effects;

  if (message_in.length() == 0) return false;

  message_pos = 0;

  read_byte(&cmd);
  read_byte(&sub);
  read_byte(&len_h);
  read_byte(&len_l);
  read_byte(&chksum_errors);
  read_byte(&sequence);

  bytes_to_uint(len_h, len_l, &len);
  bytes_to_uint(cmd, sub, &cs);


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
    //case 0x0221:
    //  break;
    // get serial number - this is a request with no payload
    case 0x0223:
      break;

    case 0x022a:
      // Checksum request (40 / GO / MINI)
      // the data is a fixed array of four bytes (0x94 00 01 02 03)
      read_byte(&junk);
      read_uint(&msg->param1);
      read_uint(&msg->param2);
      read_uint(&msg->param3);
      read_uint(&msg->param4);
      break;   

    case 0x032a:
      // Checksum response (40 / GO / MINI)
      // the data is a fixed array of four bytes (0x94 00 01 02 03)
      read_byte(&junk);
      read_uint(&msg->param1);
      read_uint(&msg->param2);
      read_uint(&msg->param3);
      read_uint(&msg->param4);
      break;
    // get firmware version - this is a request with no payload
    case 0x022f:
      break;
    // change effect parameter
    case 0x0104:
      read_string(msg->str1);
      read_byte(&msg->param1);
      read_float(&msg->val);
      read_byte(&msg->param2);
      //in_message.clear();        // temporary fix for added Input byte with LIVE
      break;
    // change of effect model
    case 0x0306:
    case 0x0106:
      read_string(msg->str1);
      read_string(msg->str2);
      read_byte(&msg->param2);     
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
      read_string(msg->str1);
      read_onoff(&msg->onoff);
      read_byte(&msg->param1);
      //in_message.clear();        // temporary fix for added Input byte with LIVE
      break;
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
      read_byte(&msg->param2);   // input - new for LIVE
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
      read_byte(&num);
      num_effects = num - 0x90;
      preset->num_effects = num_effects;
      for (j=0; j < num_effects; j++) {
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
    case 0x0474: 
    // Serial.print("Got an ack ");
    // Serial.println(cs, HEX);
      break;

    //
    // LIVE messages  
    //

    // Power Settings - Auto Standby and Auto Shutdown
    //      boolean       ? (0xc2 = false)
    //      byte          auto-shutdown time in minutes (0 = Never, 30, 40, 50, 60)
    //      byte          ?
    //      byte          auto standby time in minute (0 = Never, 5, 10, 15, 30)

    case 0x0172:
      read_onoff(&msg->bool1);
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      read_byte(&msg->param3);

      DEB("LIVE set power setting ");
      if (msg->bool1) DEB("true "); else DEB("false ");
      DEB(msg->param1);
      DEB(" ");
      DEBUG(msg->param3);

      break;

    case 0x0372:
      read_onoff(&msg->bool1);
      read_byte(&msg->param1);
      read_byte(&msg->param2);
      read_byte(&msg->param3);

      DEB("LIVE power setting response ");
      if (msg->bool1) DEB("true "); else DEB("false ");
      DEB(msg->param1);
      DEB(" ");
      DEBUG(msg->param3);

      break;
    case 0x0272:
      DEBUG("LIVE get power setting ");
      break;

    case 0x0472:
      read_byte(&junk);
      break;

    // Impedance
    //      byte          0x91 - fixed array size 1
    //      byte          0 = IN1, 1 = IN2 1/4, 2 = IN2 XLR, 3 = IN3, 4 = IN4
    //      byte          0 = Standard, 1 = Hi-Z, 2 = Line, 3 = Mic

    case 0x0174:
      read_byte(&num);
      read_uint(&msg->param1); // 0 = IN1, 1 = IN2 1/4, 2 = IN2 XLR, 4 = IN3/4
      read_uint(&msg->param2); // 0 = Standard, 1 = Hi-Z, 2 = Line, 3 = Mic  

      DEB("LIVE impedance change ");
      DEB(msg->param1);
      DEB(" ");
      DEBUG(msg->param2);
      break;

    //  Get impedance
    case 0x0274:
      read_byte(&num);
      read_uint(&msg->param1); // 0 = IN1, 1 = IN2 1/4, 2 = IN2 XLR, 4 = IN3/4

      DEB("LIVE get impedance ");
      DEBUG(msg->param1);
      break;
    
    // Impedance response
    case 0x0374:
      read_byte(&num);
      read_uint(&msg->param1); // 0 = IN1, 1 = IN2 1/4, 2 = IN2 XLR, 4 = IN3/4
      read_uint(&msg->param2); // 0 = Standard, 1 = Hi-Z, 2 = Line, 3 = Mic  

      DEB("LIVE impedance response ");
      DEB(msg->param1);
      DEB(" ");
      DEBUG(msg->param2);
      break;


    // Mixer
    //      byte          0 = IN1, 1 = IN2 1/4, 2 = IN2 XLR, 3 = IN3, 4 = IN4, 5= MUSIC, 9 = MASTER
    //      float         value

    case 0x0133:
      read_uint(&msg->param1); 
      read_float(&msg->val);

      DEB("MIXER change channel ");
      DEB(msg->param1);
      DEB(" Value: ");
      DEBUG(msg->val);
      break;

    // LIVE INPUT 1 Guitar Volume
    case 0x036b:
      read_float(&msg->val);

      DEB("LIVE guitar volume ");
      DEBUG(msg->val);
      break;
      
    // LIVE Input 2 Cable Insert  
    case 0x0373:
      read_byte(&num);
      num -= 0x90;  // should be a fixed array
      msg->param5 = num;

      read_byte(&msg->param1);
      read_byte(&msg->param2);
      read_onoff(&msg->bool1);

      if (num ==2)  {
        read_byte(&msg->param3);
        read_byte(&msg->param4);
        read_onoff(&msg->bool2);
      }

      DEB("LIVE Input 2 cable insert ");
      DEB(msg->param1);
      DEB(" ");
      DEB(msg->param2);
      DEB(" ");
      if (msg->bool1) DEB("plugged in "); else DEB ("unplugged");

      if (num == 2) {
        DEB(msg->param3);
        DEB(" ");
        DEB(msg->param4);
        DEB(" ");
        if (msg->bool2) DEB("plugged in "); else DEB ("unplugged"); 
      }
      DEBUG("");
      break;


    case 0x0273:
      read_byte(&num);
      num -= 0x90;  // should be a fixed array
      msg->param5 = num;

      read_byte(&msg->param1);
      read_byte(&msg->param2);

      DEBUG("LIVE get input 2 ");
      DEB(msg->param1);
      DEB(" ");
      DEB(msg->param2);
      DEBUG(" ");      
      break;



    // LIVE request checksums
    //      byte          Input (0 = Input 1, 1 = Input 2)

    case 0x022b:
      read_uint(&msg->param1); 

      DEB("Request LIVE checkums, input ");
      DEBUG(msg->param1);
      break;

    // Checksum response (LIVE)
    case 0x032b:
      // the data is a fixed array of eight bytes (0x98 00 01 02 03)
      read_byte(&junk);
      read_uint(&msg->param1);
      read_uint(&msg->param2);
      read_uint(&msg->param3);
      read_uint(&msg->param4);
      read_uint(&msg->param5);
      read_uint(&msg->param6);
      read_uint(&msg->param7);
      read_uint(&msg->param8);

      DEB("LIVE checksums ");
      DEB(msg->param1, HEX); DEB(" ");
      DEB(msg->param2, HEX); DEB(" ");
      DEB(msg->param3, HEX); DEB(" ");
      DEB(msg->param4, HEX); DEB(" ");   
      DEB(msg->param5, HEX); DEB(" ");
      DEB(msg->param6, HEX); DEB(" ");
      DEB(msg->param7, HEX); DEB(" ");
      DEB(msg->param8, HEX); DEBUG(" ");      
      break;


    // LIVE Get mixer setting
    case 0x0233:
      read_byte(&msg->param1);

      DEB("LIVE Mixer request setting for input ");
      DEBUG(msg->param1);
      break;


    // LIVE Mixer
    case 0x0333:
      read_float(&msg->val);

      DEB("LIVE Mixer setting is ");
      DEBUG(msg->val);
      break;

    // LIVE includes 0x031a with 0x0338 with change to preset via HW Button

    // 0x31a in response to a 0x021a gives:
    // Unprocessed message 21A length 9:92 0 1 
    // LIVE hardware preset change 0x031a   0 2 Unchanged 1 6 Unchanged 

    //
    // But as a hardware generated event (pressed) it gives:
    // LIVE hardware preset change 0x031a   0 2 Unchanged
    // Message: 31A 

    case 0x031a:
      read_byte(&num);
      num -= 0x90;  // should be a fixed array, size 1 or 2
      msg->param5 = num;

      read_byte(&msg->param1);
      read_byte(&msg->param2);
      read_onoff(&msg->bool1);
      if (num == 2) {
        read_byte(&msg->param3);
        read_byte(&msg->param4);
        read_onoff(&msg->bool2);  
      }

      DEB("LIVE hardware preset information response ");      
      DEB(msg->param1);
      DEB(" ");
      DEB(msg->param2);
      if (msg->bool1) DEB(" Unsaved changes "); else DEB(" Unchanged ");
      if (num == 2) {
        DEB(msg->param3);
        DEB(" "); 
        DEB(msg->param4);
        if (msg->bool2) DEB(" Unsaved changes"); else DEB(" Unchanged ");  
      }
      DEBUG(""); 
      break;

    case 0x021a:
      read_byte(&num);
      num -= 0x90;  // should be a fixed array
      msg->param5 = num;

      read_byte(&msg->param1);
      read_byte(&msg->param2);

      DEB("LIVE get hardware preset information ");      
      DEB(msg->param1);
      DEB(" ");
      DEBUG(msg->param2);
      break;


    /*
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
      bytes_to_uint(num1, num2, &msg->param10);
      read_byte(&msg->param6);    
      read_byte(&msg->param7);   
      DEBUG(msg->param10, HEX);   
      in_message.clear();
      break;
    */

    // the UNKNOWN command - 0x0224 00 01 02 03
    //case 0x0224:

    default:
      DEB("Unprocessed message ");
      DEB(cs, HEX);
      DEB(" length ");
      DEB(len);

      DEB(":");
      if (len != 0) {
        for (i = 0; i < len - 6; i++) {
          read_byte(&junk);
          DEB(junk, HEX);
          DEB(" ");
        }
      }
      DEBUG();
  }

  message_in.shrink(len);
  return true;
}


// used when sending a preset to Spark to see if a block has been received, will lose the messages until acknowledgement one
bool MessageIn::check_for_acknowledgement() {
  uint8_t cmd, sub, len_h, len_l;
  uint8_t sequence;
  uint8_t chksum_errors;

  unsigned int len;
  unsigned int cs;
  
  if (message_in.length() == 0) return false;

  message_pos = 0;

  read_byte(&cmd);
  read_byte(&sub);
  read_byte(&len_h);
  read_byte(&len_l);
  read_byte(&chksum_errors);
  read_byte(&sequence);

  bytes_to_uint(len_h, len_l, &len);
  bytes_to_uint(cmd, sub, &cs);

  message_in.shrink(len);

  if (cs == 0x0401 || cs == 0x0501)  
    return true;
  else
    return false;
};


// ------------------------------------------------------------------------------------------------------------
// MessageOut class
// 
// Message formatting routines to create the msgpack 
// ead messages from the SparkStructure format and place into the out_message ring buffer 
// ------------------------------------------------------------------------------------------------------------


void MessageOut::start_message(int cmdsub)
{
  int om_cmd, om_sub;
  
  om_cmd = (cmdsub & 0xff00) >> 8;
  om_sub = cmdsub & 0xff;

  buf_size = sizeof(buffer);
  buf_pos = 0;

  buffer[0] = om_cmd;
  buffer[1] = om_sub;
  buffer[2] = 0;      // placeholder for length
  buffer[3] = 0;      // placeholder for length
  buffer[4] = 0;      // placeholder for checksum errors
  buffer[5] = 0x60;   // placeholder for sequence number - setting to 0 will not work!
  buf_pos = 6;

  out_msg_chksum = 0;
}

void MessageOut::end_message()
{
  unsigned int len;
  uint8_t len_h, len_l;
  
  len = buf_pos;
  uint_to_bytes(len, &len_h, &len_l);
  
  buffer[2] = len_h;   
  buffer[3] = len_l;
}

void MessageOut::write_byte_no_chksum(byte b)
{
  if (buf_pos < buf_size) 
    buffer[buf_pos++] = b;
  else {
    DEBUG("WRITE PAST END OF BUFFER");
  }
}

void MessageOut::write_byte(byte b)
{
  write_byte_no_chksum(b);
  out_msg_chksum += int(b);
}

void MessageOut::write_uint(byte b)
{
  if (b > 127) {
    write_byte_no_chksum(0xCC);
    out_msg_chksum += int(0xCC);  
  }
  write_byte_no_chksum(b);
  out_msg_chksum += int(b);
}

void MessageOut::write_uint32(uint32_t w)
{
  int i;
  uint8_t b;
  uint32_t mask;

  mask = 0xFF000000;
  
  write_byte_no_chksum(0xCE);
  out_msg_chksum += int(0xCE);  

  for (i = 3; i >= 0; i--) {
    b = (w & mask) >> (8*i);
    mask >>= 8;
    write_uint(b);
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
   // Added with LIVE 
   write_byte(0);  // 0 is Input 1
   end_message();
}

void MessageOut::change_effect_parameter_input (char *pedal, int param, float val, uint8_t input)
{
   if (cmd_base == 0x0100) 
     start_message (cmd_base + 0x04);
   else
     start_message (cmd_base + 0x37);
   write_prefixed_string (pedal);
   write_byte (byte(param));
   write_float(val);
   // Added with LIVE 
   write_byte(input);  // 0 is Input 1
   end_message();
}

void MessageOut::change_effect (char *pedal1, char *pedal2)
{
   start_message (cmd_base + 0x06);
   write_prefixed_string (pedal1);
   write_prefixed_string (pedal2);
   // Added with LIVE 
   write_byte(0);  // 0 is Input 1
   end_message();
}

void MessageOut::change_effect_input(char *pedal1, char *pedal2, uint8_t input)
{
   start_message (cmd_base + 0x06);
   write_prefixed_string (pedal1);
   write_prefixed_string (pedal2);
   // Added with LIVE 
   write_byte(input);  // 0 is Input 1
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
   // Added with LIVE 
   write_byte(0);  // 0 is Input 1
   end_message();
}

void MessageOut::turn_effect_onoff_input (char *pedal, bool onoff, uint8_t input)
{
   start_message (cmd_base + 0x15);
   write_prefixed_string (pedal);
   write_onoff (onoff);
   // Added with LIVE 
   write_byte(input);  // 0 is Input 1
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

/*
void MessageOut::send_0x022a_info(byte v1, byte v2, byte v3, byte v4)
{
   start_message (0x022a);
   write_byte(0x94);
   write_uint(v1);
   write_uint(v2);
   write_uint(v3);
   write_uint(v4);      
   end_message();
}
*/

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
   
  write_byte (byte(0x90 + preset->num_effects));       // always 7 pedals
  for (i = 0; i < preset->num_effects; i++) {
    write_string (preset->effects[i].EffectName);
    write_onoff(preset->effects[i].OnOff);

    siz = preset->effects[i].NumParameters;
    write_byte ( 0x90 + siz); 
      
    for (j = 0; j < siz; j++) {
      write_byte (j);
      write_byte (byte(0x91));
      write_float (preset->effects[i].Parameters[j]);
    }
  }
  write_byte_no_chksum (uint8_t(out_msg_chksum % 256));  
  end_message();
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

// only need one temp block out as we won't send to app and amp at same time (not thread safe!)

byte block_out_temp[OUT_BLOCK_SIZE];

void spark_send() {
  int len;
  byte direction;

  int this_block;
  int num_blocks;
  int block_size;

  int last_block_len;
  int this_len;

  uint8_t *block_out;

  block_out = spark_msg_out.buffer;
  len = spark_msg_out.buf_pos;

  //if (spark_msg_out.has_message()) {
  if (spark_msg_out.buf_pos > 0) {
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
          //spark_process();
          process_sparkIO();
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

  uint8_t *block_out;

  block_out = app_msg_out.buffer;
  len = app_msg_out.buf_pos;

  if (app_msg_out.buf_pos > 0) {
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
