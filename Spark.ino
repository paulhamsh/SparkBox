#include "Spark.h"

///// ROUTINES TO SYNC TO AMP SETTINGS

int selected_preset;
int preset_requested;
bool preset_received;
unsigned long sync_timer;

// SparkBox specific
int hw_preset_requested; // for UI sync of bank of presets
bool hw_preset_received;
unsigned long hw_preset_timer;


int get_effect_index(char *str) {
  int ind, i;

  ind = -1;
  for (i = 0; ind == -1 && i <= 6; i++) {
    if (strcmp(presets[CUR_EDITING][current_input].effects[i].EffectName, str) == 0) {
      ind  = i;
    }
  }
  return ind;
}

bool wait_for_spark(int command_expected) {
  bool got_it = false;
  unsigned long time_now;

  time_now = millis();

  while (!got_it && millis() - time_now < 2000) {
    process_sparkIO();
    if (spark_msg_in.get_message(&cmdsub, &msg, &preset)) {
      got_it = (cmdsub == command_expected);    
    }
  }
  // if got_it is false, this is a timeout
  return got_it;
};

bool wait_for_app(int command_expected) {
  bool got_it = false;
  unsigned long time_now;

  time_now = millis();

  while (!got_it && millis() - time_now < 2000) {
    process_sparkIO();
    if (app_msg_in.get_message(&cmdsub, &msg, &preset)) {
      DEBUG(cmdsub, HEX);
      got_it = (cmdsub == command_expected);    
    }
  }
  // if got_it is false, this is a timeout
  return got_it;
};


bool spark_state_tracker_start() {
  bool got;
  int pres;

  int preset_to_get;
  bool got_all_presets;

  spark_state = SPARK_DISCONNECTED;
  ble_passthru = true;
  // try to find and connect to Spark - returns false if failed to find Spark
  if (!connect_to_all()) return false;

  // handle different spark types
  DEB("SPARK TYPE ");
  switch (spark_type) {
    case S40:
      DEBUG("Spark 40");
      break;
    case GO:
      DEBUG("Spark GO");
      break;
    case MINI:
      DEBUG("Spark MINI");
      break;
    case LIVE:
      DEBUG("Spark LIVE");
      break;      
  }

  if (spark_type != LIVE) { // should be NOT LIVE but just doing this anyway
    num_inputs = 1;
    num_presets = 4;                 
    max_preset = 3;
  }  
  else {
    num_inputs = 2;
    num_presets = 8;          
    max_preset = 7;
  }

  spark_state = SPARK_CONNECTED;     // it has to be to have reached here
  spark_ping_timer = millis();
  selected_preset = 0;

  // Get serial number
  spark_msg_out.get_serial();
  spark_send();
  got = wait_for_spark(0x0323);
  if (got) DEBUG("Got serial number");
  else DEBUG("Failed to get serial number");

  // Get firmware version
  spark_msg_out.get_firmware();
  spark_send();
  got = wait_for_spark(0x032f);
  if (got) DEBUG("Got firmware version");
  else DEBUG("Failed to get firmware version");

  // Get
  spark_msg_out.get_checksum_info();
  spark_send();
  got = wait_for_spark(0x032a);
  if (got) DEBUG("Got checksum");
  else DEBUG("Failed to get checksum");


  // Get the presets
  preset_to_get = 0x0000;
  got_all_presets = false;
  while (!got_all_presets) {
    spark_msg_out.get_preset_details(preset_to_get);
    spark_send();
    got = wait_for_spark(0x0301);

    if (got) {
      pres = preset.preset_num; // won't get an 0x7f
      if (preset.curr_preset == 0x01) {
        pres = CUR_EDITING;
        got_all_presets = true;
      }
      presets[pres][0] = preset;

      DEB("Got preset: "); 
      DEBUG(pres);
      dump_preset(presets[pres][0]);
      
      preset_to_get++;
      if (preset_to_get > max_preset + 0x0000) preset_to_get = 0x0100;
    }
    else {
      DEB("Missed preset: "); 
      DEBUG(preset_to_get);
    };
  }

  if (spark_type == LIVE) got_all_presets = false;

  // Get the presets from INPUT 2 on LIVE
  preset_to_get = 0x0300;    // The LIVE presets
  while (!got_all_presets) {
    spark_msg_out.get_preset_details(preset_to_get);
    spark_send();
    got = wait_for_spark(0x0301);

    if (got) {
      pres = preset.preset_num; // won't get an 0x7f
      if (preset.curr_preset == 0x04) {
        pres = CUR_EDITING;
        got_all_presets = true;
      }
      presets[pres][1] = preset;

      DEB("Got preset: "); 
      DEBUG(pres);
      dump_preset(presets[pres][1]);
      
      preset_to_get++;
      if (preset_to_get > max_preset + 0x0300) preset_to_get = 0x0400;
    }
    else {
      DEB("Missed preset: "); 
      DEBUG(preset_to_get);
    };
  }


  spark_state = SPARK_SYNCED;
  DEBUG("End of setup");

  spark_ping_timer = millis();
  ble_passthru = true;
  return true;
}

// get changes from app or Spark and update internal state to reflect this
// this function has the side-effect of loading cmdsub, msg and preset which can be used later

bool  update_spark_state() {
  int pres, ind;
  int input;
  
  // sort out connection and sync progress
  if (!ble_spark_connected) {
    spark_state = SPARK_DISCONNECTED;
    DEBUG("Spark disconnected, try to reconnect...");
    if (millis() - spark_ping_timer > 200) {
      spark_ping_timer = millis();
      connect_spark();  // reconnects if any disconnects happen    
      if (ble_spark_connected)
        spark_state = SPARK_SYNCED; 
    }
  }


  process_sparkIO();
  
  // K&R: Expressions connected by && or || are evaluated left to right, 
  // and it is guaranteed that evaluation will stop as soon as the truth or falsehood is known.
  
  if (spark_msg_in.get_message(&cmdsub, &msg, &preset) || app_msg_in.get_message(&cmdsub, &msg, &preset)) {
    DEB("Message: ");
    DEBUG(cmdsub, HEX);

    // all the processing for sync
    switch (cmdsub) {
      // full preset details

      case 0x0301:  
      case 0x0101:
        pres = (preset.preset_num == 0x7f) ? TMP_PRESET : preset.preset_num;
        if (preset.curr_preset == 0x01 || preset.curr_preset == 0x04)
          pres = CUR_EDITING;
        input = (preset.curr_preset > 2);     // this makes input either 0 for 0x00 and 0x01 or 1 for 0x03 and 0x04
        
        DEB("Got preset ");
        DEB(input);
        DEB(" : ");
        DEB(pres);
        DEB(" = ");
        DEB(preset.curr_preset);
        DEB(" : ");
        DEBUG(preset.preset_num);

        presets[pres][input] = preset;          // don't use current input to store
        //dump_preset(presets[pres][input]);
        break;
      // change of amp model
      case 0x0306:
        strcpy(presets[CUR_EDITING][current_input].effects[3].EffectName, msg.str2);
        break;
      // change of effect
      case 0x0106:
        ind = get_effect_index(msg.str1);
        if (ind >= 0) 
          strcpy(presets[CUR_EDITING][current_input].effects[ind].EffectName, msg.str2);
          setting_modified = true;
        break;
      // effect on/off  
      case 0x0315:
      case 0x0115:
        ind = get_effect_index(msg.str1);
        if (ind >= 0) 
          presets[CUR_EDITING][current_input].effects[ind].OnOff = msg.onoff;
          setting_modified = true;
        break;
      // change parameter value  
      case 0x0337:
      case 0x0104:
        ind = get_effect_index(msg.str1);
        if (ind >= 0)
          presets[CUR_EDITING][current_input].effects[ind].Parameters[msg.param1] = msg.val;
        setting_modified = true;  
        // SparkBox specific
        strcpy(param_str, msg.str1);
        param = msg.param1;
        break;  
      // change to preset  
      case 0x0338:
      case 0x0138:
        selected_preset = (msg.param2 == 0x7f) ? TMP_PRESET : msg.param2;
        presets[CUR_EDITING][current_input] = presets[selected_preset][current_input];
        setting_modified = false;
        // SparkBox specific
        // Only update the displayed preset number for HW presets
        if (selected_preset < num_presets) {
          display_preset_num = selected_preset; 
        }                        
        break;
      // Send licence key  
      case 0x0170:
        break; 
      // store to preset  
      case 0x0327:
        selected_preset = (msg.param2 == 0x7f) ? TMP_PRESET : msg.param2;
        presets[selected_preset][current_input] = presets[CUR_EDITING][current_input];
        setting_modified = false;
        // SparkBox specific
        // Only update the displayed preset number for HW presets
        if (selected_preset < num_presets) {
          display_preset_num = selected_preset; 
        }  
        break;
      // current selected preset
      case 0x0310:
        selected_preset = (msg.param2 == 0x7f) ? TMP_PRESET : msg.param2;
        if (msg.param1 == 0x01 || msg.param1 == 0x04) 
          selected_preset = CUR_EDITING;
        presets[CUR_EDITING][current_input] = presets[selected_preset][current_input];
        // SparkBox specific
        // Only update the displayed preset number for HW presets
        if (selected_preset < num_presets) {
          display_preset_num = selected_preset; 
        }
        break;
      case 0x0364:
        // SparkBox specific
        isTunerMode = true;
        break;    
         
      case 0x0365:
      case 0x0465:
        // SparkBox specific
        isTunerMode = false;
        break;       
      
      case 0x0438:
        setting_modified = false;
        break;

      default:
        break;
    }
    return true;
  }
  else
    return false;
}

void update_ui() {
  bool got;

  if (ble_app_connected) {
    ble_passthru = false;
    app_msg_out.save_hardware_preset(0x00, 0x03);
    app_send();

    DEBUG("Updating UI");
    got = wait_for_app(0x0201);
    if (got) {
      strcpy(presets[CUR_EDITING][current_input].Name, "SyncPreset");
      strcpy(presets[CUR_EDITING][current_input].UUID, "F00DF00D-FEED-0123-4567-987654321000");  
      presets[CUR_EDITING][current_input].curr_preset = 0x00;
      presets[CUR_EDITING][current_input].preset_num = 0x03;
      app_msg_out.create_preset(&presets[CUR_EDITING][current_input]);
      app_send();
      delay(100);
      app_msg_out.change_hardware_preset(0x00, 0x00);
      app_send();
      app_msg_out.change_hardware_preset(0x00, 0x03);     
      app_send();
    }
    else {
      DEBUG("Didn't capture the new preset");
    }
    ble_passthru = true;
  }
}

// SparkBox specific
void update_ui_hardware() {
  bool got;
  int p;
  int i;
  bool done;

  if (ble_app_connected) {
    done = false;
    ble_passthru = false;

    DEBUG("Updating UI for hardware");

    i = 0;

    while (i < num_presets) {
      app_msg_out.save_hardware_preset(0x00, i);
      app_send();

      got = wait_for_app(0x0201);
      if (got) {
        DEB("Got hardware preset request ");
        DEB(msg.param2);
        DEB(" Looking for: ");
        DEBUG(i);
        presets[i][current_input].curr_preset = 0x00;
        presets[i][current_input].preset_num = i;
        app_msg_out.create_preset(&presets[i][current_input]);
        app_send();
        delay(1000);
        i++;
      }
      else {
        DEBUG("Didn't capture the new preset");
      }
    }
  }
}


void set_input1() {
  if (spark_type == LIVE) {
    spark_msg_out.select_live_input_1();
    spark_send();
  }
}
///// ROUTINES TO CHANGE AMP SETTINGS

void change_generic_model(char *new_eff, int slot) {
  if (strcmp(presets[CUR_EDITING][current_input].effects[slot].EffectName, new_eff) != 0) {
    set_input1();
    spark_msg_out.change_effect_input(presets[CUR_EDITING][current_input].effects[slot].EffectName, new_eff, current_input);
    strcpy(presets[CUR_EDITING][current_input].effects[slot].EffectName, new_eff);
    spark_send();
    delay(100);
  }
}

void change_comp_model(char *new_eff) {
  change_generic_model(new_eff, 1);
}

void change_drive_model(char *new_eff) {
  change_generic_model(new_eff, 2);
}

void change_amp_model(char *new_eff) {
  if (strcmp(presets[CUR_EDITING][current_input].effects[3].EffectName, new_eff) != 0) {
    spark_msg_out.change_effect_input(presets[CUR_EDITING][current_input].effects[3].EffectName, new_eff, current_input);
    app_msg_out.change_effect_input(presets[CUR_EDITING][current_input].effects[3].EffectName, new_eff, current_input);
    strcpy(presets[CUR_EDITING][current_input].effects[3].EffectName, new_eff);
    spark_send();
    app_send();
    delay(100);
  }
}

void change_mod_model(char *new_eff) {
  change_generic_model(new_eff, 4);
}

void change_delay_model(char *new_eff) {
  change_generic_model(new_eff, 5);
}



void change_generic_onoff(int slot,bool onoff) {
  
  spark_msg_out.turn_effect_onoff_input(presets[CUR_EDITING][current_input].effects[slot].EffectName, onoff, current_input);
  app_msg_out.turn_effect_onoff_input(presets[CUR_EDITING][current_input].effects[slot].EffectName, onoff, current_input);
  presets[CUR_EDITING][current_input].effects[slot].OnOff = onoff;
  spark_send();
  app_send();  
}

void change_noisegate_onoff(bool onoff) {
  change_generic_onoff(0, onoff);  
}

void change_comp_onoff(bool onoff) {
  change_generic_onoff(1, onoff);  
}

void change_drive_onoff(bool onoff) {
  change_generic_onoff(2, onoff);  
}

void change_amp_onoff(bool onoff) {
  change_generic_onoff(3, onoff);  
}

void change_mod_onoff(bool onoff) {
  change_generic_onoff(4, onoff);  
}

void change_delay_onoff(bool onoff) {
  change_generic_onoff(5, onoff);  
}

void change_reverb_onoff(bool onoff) {
  change_generic_onoff(6, onoff);  
}


void change_generic_toggle(int slot) {
  bool new_onoff;

  new_onoff = !presets[CUR_EDITING][current_input].effects[slot].OnOff;
  
  spark_msg_out.turn_effect_onoff_input(presets[CUR_EDITING][current_input].effects[slot].EffectName, new_onoff, current_input);
  app_msg_out.turn_effect_onoff_input(presets[CUR_EDITING][current_input].effects[slot].EffectName, new_onoff, current_input);
  presets[CUR_EDITING][current_input].effects[slot].OnOff = new_onoff;
  spark_send();
  app_send();  
}

void change_noisegate_toggle() {
  change_generic_toggle(0);  
}

void change_comp_toggle() {
  change_generic_toggle(1);  
}

void change_drive_toggle() {
  change_generic_toggle(2);  
}

void change_amp_toggle() {
  change_generic_toggle(3);  
}

void change_mod_toggle() {
  change_generic_toggle(4);  
}

void change_delay_toggle() {
  change_generic_toggle(5);  
}

void change_reverb_toggle() {
  change_generic_toggle(6);  
}


void change_generic_param(int slot, int param, float val) {
  float diff;

  // some code to reduce the number of changes
  diff = presets[CUR_EDITING][current_input].effects[slot].Parameters[param] - val;
  if (diff < 0) diff = -diff;
  if (diff > 0.04) {
    spark_msg_out.change_effect_parameter_input(presets[CUR_EDITING][current_input].effects[slot].EffectName, param, val, current_input);
    app_msg_out.change_effect_parameter_input(presets[CUR_EDITING][current_input].effects[slot].EffectName, param, val, current_input);
    presets[CUR_EDITING][current_input].effects[slot].Parameters[param] = val;
    spark_send();  
    app_send();
  }
}

void change_noisegate_param(int param, float val) {
  change_generic_param(0, param, val);
}

void change_comp_param(int param, float val) {
  change_generic_param(1, param, val);
}

void change_drive_param(int param, float val) {
  change_generic_param(2, param, val);
}

void change_amp_param(int param, float val) {
  change_generic_param(3, param, val);
}

void change_mod_param(int param, float val) {
  change_generic_param(4, param, val);
}

void change_delay_param(int param, float val) {
  change_generic_param(5, param, val);
}

void change_reverb_param(int param, float val){
  change_generic_param(6, param, val);
}


void change_hardware_preset(int pres_num) {
  if (pres_num >= 0 && pres_num <= 3) {  
    presets[CUR_EDITING][current_input] = presets[pres_num][current_input];
    
    spark_msg_out.change_hardware_preset(0, pres_num);
    app_msg_out.change_hardware_preset(0, pres_num);  
    spark_send();  
    app_send();
  }
}

void change_custom_preset(SparkPreset *preset, int pres_num) {
  if (pres_num >= 0 && pres_num <= max_preset) {
    preset->preset_num = (pres_num < num_presets) ? pres_num : 0x7f;
    presets[CUR_EDITING][current_input] = *preset;
    presets[pres_num][current_input] = *preset;
    
    spark_msg_out.create_preset(preset);
    spark_msg_out.change_hardware_preset(0, preset->preset_num);
  }
}

void tuner_on_off(bool on_off) {
  spark_msg_out.tuner_on_off(on_off); 
  spark_send();  
}


void send_tap_tempo(float tempo) {
  spark_msg_out.send_tap_tempo(tempo);
  spark_send();    
};