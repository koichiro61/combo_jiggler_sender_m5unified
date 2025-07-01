/*
 * Extended Screen saver killer 
 * M5Unified version (For Atom Lite, Atom S3 Lite, Atom Matrix, Stamp C3, Stamp Pico)
 *
*/

#define VERSION "20250701"

#include "M5Unified.h"
#include <BleCombo.h>     // Use forked version https://github.com/the0duke0/ESP32-BLE-Combo
#include "EspEasyLED.h"   // https://github.com/tanakamasayuki/EspEasyUtils

#define LED_ATOM_LITE     (gpio_num_t)27
#define LED_STAMP_C3U     (gpio_num_t)2
#define LED_ATOM_S3_LITE  (gpio_num_t)35
#define LED_ATOM_MATRIX   (gpio_num_t)27
#define LED_STAMP_PICO    (gpio_num_t)27
#define LED_MAX_PIN       (gpio_num_t)99

#define MODE_BT_INACTIVE 0
#define MODE_NOP  1
#define MODE_MOUSE_JIGGLER  2
#define MODE_KEYCODE_SENDER  3
#define MODE_DEMONSTRATION 4

BleComboKeyboard atomKeyboard("M5Unified Keyboard");  // device name appears on PC
BleComboMouse atomMouse(&atomKeyboard);

/*
 * Parameters for keyboard / mouse action / demonstration
*/

struct keycode_sender_param {
  uint32_t keyShiftCapsLockDelay; // in mSec
  uint32_t keyHoldDuration;
  uint32_t keyLockDuration;
  uint32_t keycode_sender_interval;
};

// keycode_sender_param ksparam = {4, 10, 10, 20000};
keycode_sender_param ksparam = {10, 50, 50, 60000}; // 20250516

struct mouse_jiggler_param {
  uint32_t mouseMoverDuration;  // in mSec
  uint32_t mouseMoverTickTime;
  uint32_t mouseMoverLoopInterval;
};

mouse_jiggler_param mjparam{12, 4, 10000};

struct demonstration_param {
  keycode_sender_param  demo_ksparam;
  mouse_jiggler_param   demo_mjparam; 
};

demonstration_param demoparam = {
  {4, 1000, 1000, 1000}, 
  {1000, 5, 1000}
};

//------ end of parameters -------


uint8_t volatile mode = MODE_BT_INACTIVE;
bool volatile mode_changed = false;

EspEasyLED* rgbled;
// Builds structure for control of RGB LED and returns pointer to it
// ... and initialization 
EspEasyLED* get_rgbled() {
  gpio_num_t g = LED_MAX_PIN;
  switch (M5.getBoard()) {
    case m5::board_t::board_M5AtomMatrix:
      g = LED_ATOM_MATRIX;
      break;
    case m5::board_t::board_M5AtomLite:
      g = LED_ATOM_LITE;
      break;
    case m5::board_t::board_M5AtomS3Lite:
      g = LED_ATOM_S3_LITE;
      break;
    case m5::board_t::board_M5StampC3U:
      g = LED_STAMP_C3U;
      break;
    case m5::board_t::board_M5StampPico:
      g = (gpio_num_t)LED_STAMP_PICO;
      break;
    default:
      g = LED_MAX_PIN;
      break;
  }
  Serial.print("LED g=");
  Serial.println(g);
  if ( g < LED_MAX_PIN) {
    pinMode(g, OUTPUT); // added on 20250624
    return new EspEasyLED(g, 1, 20); // GPIO, number of LEDs, Max brightness
  } else {
    return NULL;
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  Serial.printf("combo_jiggler_sender_m5Unified version %s\n", VERSION);

  rgbled = get_rgbled();  // depending on model type

  atomKeyboard.begin();
  atomMouse.begin();

  // invoking BLE connection monitoring task
  xTaskCreatePinnedToCore(BLE_connection, "BLE_connection", 4096, NULL, 1, NULL, tskNO_AFFINITY);
}

TaskHandle_t task_mouse_jiggler;
TaskHandle_t task_keycode_sender;
TaskHandle_t task_demonstration;

void loop() {

  M5.update();

  show_mode_LED(mode);
  
  if (mode > 0) {
    if (M5.BtnA.wasPressed()) {
      mode += 1; if (mode > MODE_DEMONSTRATION) mode = MODE_NOP;
      Serial.println(mode);

      switch(mode) {
        case MODE_NOP:  // kill all the task running other than BLE connection monitoring
          if (task_demonstration != NULL) {
            vTaskDelete(task_demonstration);
            task_demonstration = NULL;
            Serial.println("demonstration task deleted");
          }
          if (task_mouse_jiggler != NULL) {
            vTaskDelete(task_mouse_jiggler);
            task_mouse_jiggler = NULL;
            Serial.println("mouse_jiggler task deleted");
          }
          if (task_keycode_sender != NULL) {
            vTaskDelete(task_keycode_sender);
            task_keycode_sender = NULL;
            Serial.println("keycode_sender task deleted");
          }
          break;  
        case MODE_MOUSE_JIGGLER:
          if (task_mouse_jiggler == NULL) {
            xTaskCreatePinnedToCore(mouse_jiggler, "mouse_jiggler", 4096, (void*)&mjparam, 1, &task_mouse_jiggler, tskNO_AFFINITY);
            Serial.println("mouse jiggler task started");
          }
          break;
        case MODE_KEYCODE_SENDER:
          if (task_mouse_jiggler != NULL) {
            vTaskDelete(task_mouse_jiggler);
            task_mouse_jiggler = NULL;
            Serial.println("mouse_jiggler task deleted");
          }
          if (task_keycode_sender == NULL) {
            xTaskCreatePinnedToCore(keycode_sender, "keycode_sender", 4096, (void*)&ksparam, 1, &task_keycode_sender, tskNO_AFFINITY);
            Serial.println("kecode_sender task started");
          }
          break;      
        case MODE_DEMONSTRATION:
          if (task_keycode_sender != NULL) {
            vTaskDelete(task_keycode_sender);
            task_keycode_sender = NULL;
            Serial.println("keycode_sender task deleted");
          }
          if (task_demonstration == NULL) {
            xTaskCreatePinnedToCore(demonstration, "demonstration", 4096, (void*)&demoparam, 1, &task_demonstration, tskNO_AFFINITY);
            Serial.println("demontration task started");
          }
          break;      
      }
    }
  }
  delay(50);
}

void BLE_connection(void* arg) {
  bool bleEnabled;
  while (1) {
    bleEnabled = atomKeyboard.isConnected();
    if(bleEnabled == true){
      if (mode == 0) {
        mode = 1; 
      } else {
        // no need to change mode when mode > 0 already
      }  
    } else {  // when ble disabled
      mode = 0;
    }
    vTaskDelay(50);
  }
}

void show_mode_LED(uint8_t mode) {
  static const EspEasyLED::color_t led_color[] = {
    EspEasyLEDColor::RED,    // [MODE_BT_INACTIVE]
    EspEasyLEDColor::GREEN,  // MODE_NOP 
    EspEasyLEDColor::BLUE,   // MODE_MOUSE_JIGGLER 
    EspEasyLEDColor::CYAN,   // MODE_KEYCODE_SENDER  
    EspEasyLEDColor::PURPLE,  // MODE_DEMONSTRATION
  };
  if (rgbled) {
    rgbled->showColor(led_color[mode]);
  }
}

void push_shift_capsLock(uint32_t keyShiftCapsLockDelay, uint32_t keyHoldDuration) {
    atomKeyboard.press(KEY_LEFT_SHIFT);
    vTaskDelay(keyShiftCapsLockDelay);
    atomKeyboard.press(KEY_CAPS_LOCK);
    vTaskDelay(keyHoldDuration);
    atomKeyboard.releaseAll();
}

void capsLockOnOff(uint32_t keyShiftCapsLockDelay, uint32_t keyHoldDuration, uint32_t keyLockDuration) {
    push_shift_capsLock(keyShiftCapsLockDelay, keyHoldDuration);  // capslock on
    vTaskDelay(keyLockDuration);
    push_shift_capsLock(keyShiftCapsLockDelay, keyHoldDuration);  // capslock off
}

void keycode_sender(void* arg) {
  keycode_sender_param* param_ptr;
  param_ptr = (keycode_sender_param*)arg;
  while(1) {
    capsLockOnOff(param_ptr->keyShiftCapsLockDelay, param_ptr->keyHoldDuration, param_ptr->keyLockDuration);
    vTaskDelay(param_ptr->keycode_sender_interval);
  }
}

void mouse_move_xy(uint32_t deltaX, uint32_t deltaY, uint32_t duration, uint32_t tickTime) {
    uint32_t startTime = millis();
    while (millis() < startTime + duration) {
      atomMouse.move(deltaX, deltaY);
      vTaskDelay(tickTime);
    }
}

void mouse_jiggle_onetime (uint32_t duration, uint32_t tickTime) {
  mouse_move_xy( 0, -1, duration, tickTime);
  mouse_move_xy(-1,  0, duration, tickTime);
  mouse_move_xy( 0,  1, duration, tickTime);
  mouse_move_xy( 1,  0, duration, tickTime);
}


void mouse_jiggler(void* arg) {
  mouse_jiggler_param* param_ptr;
  param_ptr = (mouse_jiggler_param*)arg;
  while(1) {
      mouse_jiggle_onetime(param_ptr->mouseMoverDuration, param_ptr->mouseMoverTickTime);
      vTaskDelay(param_ptr->mouseMoverLoopInterval);
  }
}

void demonstration(void* arg) {
  demonstration_param* param_ptr;
  param_ptr = (demonstration_param*)arg;
  while(1) {
    // 20250516 At first mousemove, then keycode Sender 
    // capsLockOnOff(param_ptr->demo_ksparam.keyShiftCapsLockDelay,\
    //  param_ptr->demo_ksparam.keyHoldDuration,param_ptr->demo_ksparam.keyLockDuration);
    // vTaskDelay(param_ptr->demo_ksparam.keycode_sender_interval);

    mouse_jiggle_onetime (param_ptr->demo_mjparam.mouseMoverDuration,\
     param_ptr->demo_mjparam.mouseMoverTickTime);
    vTaskDelay(param_ptr->demo_mjparam.mouseMoverLoopInterval);

    capsLockOnOff(param_ptr->demo_ksparam.keyShiftCapsLockDelay,\
     param_ptr->demo_ksparam.keyHoldDuration,param_ptr->demo_ksparam.keyLockDuration);
    vTaskDelay(param_ptr->demo_ksparam.keycode_sender_interval);

  }
}

