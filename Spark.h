#ifndef Spark_h
#define Spark_h

#include "SparkIO.h"

// variables required to track spark state and also for communications generally
unsigned int cmdsub;
SparkMessage msg;
SparkPreset preset;
SparkPreset presets[10][2];           // max 8 presets plus temp and current

int current_input = 0;

int num_inputs;
int num_presets;                   // default num_presets
int max_preset;
//int current_preset_index;          // default CUR_EDITING
//int temp_preset_index;

enum ePresets_t {HW_PRESET_0, HW_PRESET_1, HW_PRESET_2, HW_PRESET_3, TMP_PRESET=8, CUR_EDITING=9, TMP_PRESET_ADDR=0x007f};

enum spark_status_values {SPARK_DISCONNECTED, SPARK_CONNECTED, SPARK_COMMUNICATING, SPARK_CHECKSUM, SPARK_SYNCING, SPARK_SYNCED};
spark_status_values spark_state;
unsigned long spark_ping_timer;

//SparkBox specific
bool setting_modified = false;      // Flag that user has modifed a setting
bool isTunerMode;                   // Tuner mode flag
char param_str[STR_LEN];               
int param = -1;
uint8_t display_preset_num;         // Referenced preset number on Spark

bool spark_state_tracker_start();
bool update_spark_state();
void update_ui();

void change_comp_model(char *new_eff);
void change_drive_model(char *new_eff);
void change_amp_model(char *new_eff);
void change_mod_model(char *new_eff);
void change_delay_model(char *new_eff);

void change_noisegate_onoff(bool onoff);
void change_comp_onoff(bool onoff);
void change_drive_onoff(bool onoff);
void change_amp_onoff(bool onoff);
void change_mod_onoff(bool onoff);
void change_delay_onoff(bool onoff);
void change_reverb_onoff(bool onoff);

void change_noisegate_toggle();
void change_comp_toggle();
void change_drive_toggle();
void change_amp_toggle();
void change_mod_toggle();
void change_delay_toggle();
void change_reverb_toggle();

void change_noisegate_param(int param, float val);
void change_comp_param(int param, float val);
void change_drive_param(int param, float val);
void change_amp_param(int param, float val);
void change_mod_param(int param, float val);
void change_delay_param(int param, float val);
void change_reverb_param(int param, float val);

void change_hardware_preset(int pres_num);
void change_custom_preset(SparkPreset *preset, int pres_num);

void tuner_on_off(bool on_off);
void send_tap_tempo(float tempo);

#define AMP_GAIN 0
#define AMP_TREBLE 1
#define AMP_MID 2
#define AMP_BASS 3
#define AMP_MASTER 4

#endif