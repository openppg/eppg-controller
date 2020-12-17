#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <WInterrupts.h>

enum batteryStates {
  NOT_CHARGING,
  CHARGING,
  CHARGE_COMPLETE, 
  UNKNOWN
};
 
enum batteryStates batteryStatus;

void checkBatt(){
    int stat1 = digitalRead(PIN_BSTAT1);
    int stat2 = digitalRead(PIN_BSTAT2);
    int vbat = analogRead(PIN_VBAT);
    Serial.print(stat1);
    Serial.print(" s1 and s2 ");
    Serial.println(stat2);
    Serial.print("VBAT ");
    Serial.println(vbat);
    //delay(300);


    if ((stat1 == HIGH) && (stat2 == HIGH)) {
        batteryStatus = NOT_CHARGING;
    } else if ((stat1 == LOW) && (stat2 == HIGH)) {
        batteryStatus = CHARGING;
    } else if ((stat1 == HIGH) && (stat2 == LOW)) {
        batteryStatus = CHARGE_COMPLETE;
    } else {
        batteryStatus = UNKNOWN;
    }
    Serial.print("status: ");
    Serial.println(batteryStatus);
}

void powerOff() {
  uint8_t check;
  sd_softdevice_is_enabled(&check);

  if (check == 1) {
    sd_power_system_off();
  } else {
    NRF_POWER->SYSTEMOFF = 1;
  }
}
