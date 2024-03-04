#include "Spark.h"

///// ROUTINES TO SYNC TO AMP SETTINGS

int selected_preset;
int preset_requested;
bool preset_received;
unsigned long sync_timer;

int get_effect_index(char *str) {
  int ind, i;

  ind = -1;
  for (i = 0; ind == -1 && i <= 6; i++) {
    if (strcmp(presets[5].effects[i].EffectName, str) == 0) {
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
    spark_process();
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
    app_process();
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

  spark_state = SPARK_DISCONNECTED;
  ble_passthru = true;
  // try to find and connect to Spark - returns false if failed to find Spark
  connect_to_all();
                
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
  int preset_to_get = 0;
  bool got_all_presets = false;
  while (!got_all_presets) {
    //pres = (i == 4) ? 0x0100 : i;
    spark_msg_out.get_preset_details(preset_to_get);
    spark_send();
    got = wait_for_spark(0x0301);

    //pres = (preset.preset_num == 0x7f) ? 4 : preset.preset_num;
    pres = preset.preset_num; // won't get an 0x7f
    if (preset.curr_preset == 0x01) {
      pres = 5;
      got_all_presets = true;
    }
    presets[pres] = preset;
    //dump_preset(&presets[pres]);

    if (got) {
      preset_to_get++;
      if (preset_to_get == 4) preset_to_get = 0x0100;
      DEBUG("PRESET: "); 
      DEBUG(pres);
    }
    else {
      DEBUG("MISSED PRESET: "); 
      DEBUG(pres);
    };
  }
  spark_state = SPARK_SYNCED;
  DEBUG("END OF SETUP");

  spark_ping_timer = millis();
  ble_passthru = true;
  return true;
}

// get changes from app or Spark and update internal state to reflect this
// this function has the side-effect of loading cmdsub, msg and preset which can be used later

bool  update_spark_state() {
  int pres, ind;
  
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

  spark_process();
  app_process();
  
  // K&R: Expressions connected by && or || are evaluated left to right, 
  // and it is guaranteed that evaluation will stop as soon as the truth or falsehood is known.
  
  if (spark_msg_in.get_message(&cmdsub, &msg, &preset) || app_msg_in.get_message(&cmdsub, &msg, & preset)) {
    DEBUG("Message: ");
    DEBUG(cmdsub, HEX);

    // all the processing for sync
    switch (cmdsub) {
      // full preset details
      case 0x0301:  
      case 0x0101:
        pres = (preset.preset_num == 0x7f) ? 4 : preset.preset_num;
        if (preset.curr_preset == 0x01)
          pres = 5;
        presets[pres] = preset;
        //dump_preset(&presets[pres]);
        DEBUG("Got preset ");
        DEBUG(pres);
        break;
      // change of amp model
      case 0x0306:
        strcpy(presets[5].effects[3].EffectName, msg.str2);
        break;
      // change of effect
      case 0x0106:
        ind = get_effect_index(msg.str1);
        if (ind >= 0) 
          strcpy(presets[5].effects[ind].EffectName, msg.str2);
        break;
      // effect on/off  
      case 0x0315:
      case 0x0115:
        ind = get_effect_index(msg.str1);
        if (ind >= 0) 
          presets[5].effects[ind].OnOff = msg.onoff;
        break;
      // change parameter value  
      case 0x0337:
      case 0x0104:
        ind = get_effect_index(msg.str1);
        if (ind >= 0)
          presets[5].effects[ind].Parameters[msg.param1] = msg.val;
        break;  
      // change to preset  
      case 0x0338:
      case 0x0138:
        selected_preset = (msg.param2 == 0x7f) ? 4 : msg.param2;
        presets[5] = presets[selected_preset];
        break;
      // store to preset  
      case 0x0327:
        selected_preset = (msg.param2 == 0x7f) ? 4 : msg.param2;
        presets[selected_preset] = presets[5];
        break;
      // current selected preset
      case 0x0310:
        selected_preset = (msg.param2 == 0x7f) ? 4 : msg.param2;
        if (msg.param1 == 0x01) 
          selected_preset = 5;
        presets[5] = presets[selected_preset];
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

  ble_passthru = false;
  app_msg_out.save_hardware_preset(0x00, 0x03);
  app_send();

  DEBUG("Updating UI");
  got = wait_for_app(0x0201);
  if (got) {
    strcpy(presets[5].Name, "SyncPreset");
    strcpy(presets[5].UUID, "F00DF00D-FEED-0123-4567-987654321000");  
    presets[5].curr_preset = 0x00;
    presets[5].preset_num = 0x03;
    app_msg_out.create_preset(&presets[5]);
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

// SparkBox specific
void update_ui_hardware() {

};

///// ROUTINES TO CHANGE AMP SETTINGS

void change_generic_model(char *new_eff, int slot) {
  if (strcmp(presets[5].effects[slot].EffectName, new_eff) != 0) {
    spark_msg_out.change_effect(presets[5].effects[slot].EffectName, new_eff);
    strcpy(presets[5].effects[slot].EffectName, new_eff);
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
  if (strcmp(presets[5].effects[3].EffectName, new_eff) != 0) {
    spark_msg_out.change_effect(presets[5].effects[3].EffectName, new_eff);
    app_msg_out.change_effect(presets[5].effects[3].EffectName, new_eff);
    strcpy(presets[5].effects[3].EffectName, new_eff);
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
  spark_msg_out.turn_effect_onoff(presets[5].effects[slot].EffectName, onoff);
  app_msg_out.turn_effect_onoff(presets[5].effects[slot].EffectName, onoff);
  presets[5].effects[slot].OnOff = onoff;
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

  new_onoff = !presets[5].effects[slot].OnOff;
  
  spark_msg_out.turn_effect_onoff(presets[5].effects[slot].EffectName, new_onoff);
  app_msg_out.turn_effect_onoff(presets[5].effects[slot].EffectName, new_onoff);
  presets[5].effects[slot].OnOff = new_onoff;
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
  diff = presets[5].effects[slot].Parameters[param] - val;
  if (diff < 0) diff = -diff;
  if (diff > 0.04) {
    spark_msg_out.change_effect_parameter(presets[5].effects[slot].EffectName, param, val);
    app_msg_out.change_effect_parameter(presets[5].effects[slot].EffectName, param, val);
    presets[5].effects[slot].Parameters[param] = val;
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
    presets[5] = presets[pres_num];
    
    spark_msg_out.change_hardware_preset(0, pres_num);
    app_msg_out.change_hardware_preset(0, pres_num);  
    spark_send();  
    app_send();
  }
}

void change_custom_preset(SparkPreset *preset, int pres_num) {
  if (pres_num >= 0 && pres_num <= 4) {
    preset->preset_num = (pres_num < 4) ? pres_num : 0x7f;
    presets[5] = *preset;
    presets[pres_num] = *preset;
    
    spark_msg_out.create_preset(preset);
    spark_msg_out.change_hardware_preset(0, preset->preset_num);
  }
}

void tuner_on_off(bool on_off) {
  spark_msg_out.tuner_on_off(on_off); 
  spark_send();  
}