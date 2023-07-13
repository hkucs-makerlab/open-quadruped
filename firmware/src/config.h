#pragma once

#define Console Serial
// ---------------------------------------------------------------
#ifdef ESP32
#define FR_Hip_PIN 12
#define FR_Shoulder_PIN 14
#define FR_Wrist_PIN 27

#define FL_Hip_PIN 15
#define FL_Shoulder_PIN 2
#define FL_Wrist_PIN 4

#define BL_Hip_PIN 5
#define BL_Shoulder_PIN 18
#define BL_Wrist_PIN 19

#define BR_Hip_PIN 25
#define BR_Shoulder_PIN 33
#define BR_Wrist_PIN 32

#define BUZZER_PIN 23
#endif  // ESP32

// ---------------------------------------------------------------
#ifdef AVR
#define FR_Hip_PIN 2
#define FR_Shoulder_PIN 3
#define FR_Wrist_PIN 4

#define BR_Hip_PIN 12
#define BR_Shoulder_PIN 13
#define BR_Wrist_PIN 5

#define FL_Hip_PIN 9
#define FL_Shoulder_PIN 8
#define FL_Wrist_PIN 6

#define BL_Hip_PIN 10
#define BL_Shoulder_PIN 11
#define BL_Wrist_PIN 7

#define BUZZER_PIN A0

#define BT_RX_PIN A1
#define BT_TX_PIN A2
#endif  // AVR


