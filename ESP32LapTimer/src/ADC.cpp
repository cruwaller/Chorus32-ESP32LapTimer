/*
 * This file is part of Chorus32-ESP32LapTimer
 * (see https://github.com/AlessandroAU/Chorus32-ESP32LapTimer).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
////Functions to Read RSSI from ADCs//////
#include <driver/adc.h>
#include <driver/timer.h>
#include <esp_adc_cal.h>
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_INA219.h>

#include "HardwareConfig.h"
#include "Comms.h"
#include "settings_eeprom.h"
#include "ADC.h"
#include "Timer.h"
#include "Output.h"
#include "Calibration.h"
#include "Laptime.h"
#include "Utils.h"
#include "RX5808.h"
#include "Queue.h"

#include "Filter.h"

static Adafruit_INA219 ina219;
static TwoWire i2c_wire(0);

static esp_adc_cal_characteristics_t adc_chars;

static uint16_t adcLoopCounter = 0;

static float VBATcalibration;
static float mAReadingFloat;
static float VbatReadingFloat;

// count of current active pilots
static uint8_t current_pilot_num = 0;

// number of cascading lowpass filter per pilot
#define PILOT_FILTER_NUM 1

static lowpass_filter_t adc_voltage_filter;

#ifdef DEBUG_SIGNAL_LOG
static uint16_t readings[DEBUG_SIGNAL_LOG_NUM][DEBUG_SIGNAL_LOG_SIZE];
static uint32_t readings_pos[DEBUG_SIGNAL_LOG_NUM];
#endif

enum pilot_state {
  PILOT_UNUSED,
  PILOT_ACTIVE,
};

typedef struct receiver_data_s receiver_data_t;

typedef struct pilot_data_s {
  uint16_t RSSIthreshold;
  uint16_t ADCReadingRAW;
  uint16_t ADCvalue;
  lowpass_filter_t filter[PILOT_FILTER_NUM];
  /// Pilot state 0: unsused, 1: active
  uint8_t state;
  receiver_data_t* current_rx;
  uint8_t number;
  uint32_t unused_time; // used to force a certain stay time. if we allow an instant switch, modules would be constantly switching with e.g. 6 modules and 7 pilots
  bool disable_multiplexing; // don't switch this pilot. scenario: 3 modules installed with 4 pilots. 2 pilots really care about their time and the other 2 just want to see their approximate time. so fix the first two pilots and the remaining module multiplexes the other 2
  // Used for finding the maximum rssi after reaching the threshold
  uint16_t max_adc;
  uint32_t max_time;
  // require a certain amount of samples below threshold to determine the maximum. prevents triggering a lap when one approaches the timer and the rssi fluctuates
  uint16_t samples;
} pilot_data_t;

typedef struct receiver_data_s {
  uint32_t last_hop;
  pilot_data_t* current_pilot;
} receiver_data_t;

static pilot_data_t pilots[MAX_NUM_PILOTS];
static receiver_data_t receivers[MAX_NUM_RECEIVERS];

// multiplexing queue
static queue_t pilot_queue;
static pilot_data_t* pilot_queue_data[MAX_NUM_PILOTS];

SemaphoreHandle_t pilot_queue_lock;
SemaphoreHandle_t pilots_lock;

static adc1_channel_t ADC_PINS[MAX_NUM_RECEIVERS] = {
  ADC1,
#if 1 < MAX_NUM_RECEIVERS
  ADC2,
#if 2 < MAX_NUM_RECEIVERS
  ADC3,
#if 3 < MAX_NUM_RECEIVERS
  ADC4,
#if 4 < MAX_NUM_RECEIVERS
  ADC5,
#if 5 < MAX_NUM_RECEIVERS
  ADC6
#endif
#endif
#endif
#endif
#endif
  };

uint16_t multisample_adc1(adc1_channel_t channel, uint8_t samples) {
  uint32_t val = 0;
  for(uint8_t i = 0; i < samples; ++i) {
    val += adc1_get_raw(channel);
  }
  return val/samples;
}

/**
 * Find next free pilot and set them to busy. Marks the old pilot as active. Sets the current pilot for the given module
 * \returns if a different pilot was found.
 */
static IRAM_ATTR bool setNextPilot(uint8_t adc) {
  // skip this receiver if the assigned pilot is fixed
  if(receivers[adc].current_pilot && receivers[adc].current_pilot->disable_multiplexing) {
    return false;
  }
  if(!xSemaphoreTake(pilot_queue_lock, 1)) {
    // failed to obtain mutex, so do nothing
    return false;
  }
  pilot_data_t* new_pilot = (pilot_data_t*)queue_peek(&pilot_queue);
  bool ret_val = false;
  if(new_pilot) {
    if(new_pilot->state == PILOT_ACTIVE) {
      if((micros() - new_pilot->unused_time) > MULTIPLEX_STAY_TIME_US + MIN_TUNE_TIME_US){
        new_pilot = (pilot_data_t*)queue_dequeue(&pilot_queue);
        // set old pilot to active again
        if(receivers[adc].current_pilot && receivers[adc].current_pilot->state != PILOT_UNUSED) {
          receivers[adc].current_pilot->state = PILOT_ACTIVE;
          receivers[adc].current_pilot->current_rx = NULL;
          // readd to multiplex queue
          queue_enqueue(&pilot_queue, receivers[adc].current_pilot);
          receivers[adc].current_pilot->unused_time = micros();
        }
        new_pilot->current_rx = &receivers[adc];
        receivers[adc].current_pilot = new_pilot;
        ret_val = true;
      }
    } else { // pilot went inactive
      new_pilot = (pilot_data_t*)queue_dequeue(&pilot_queue); // Dequeue inactive pilot
      if(new_pilot->current_rx) {
        new_pilot->current_rx->current_pilot = NULL;
        new_pilot->current_rx = NULL;
      }
    }
  }
  xSemaphoreGive(pilot_queue_lock);
  // Do nothing it the pilot is not active
  return ret_val;
}

void ConfigureADC(bool disable_all_modules) {
  #ifdef DEBUG_SIGNAL_LOG
  memset(readings, 0, DEBUG_SIGNAL_LOG_SIZE * DEBUG_SIGNAL_LOG_NUM * sizeof(uint16_t));
  memset(readings_pos, 0, DEBUG_SIGNAL_LOG_NUM * sizeof(uint32_t));
  #endif

  pilot_queue_lock = xSemaphoreCreateMutex();
  pilots_lock = xSemaphoreCreateMutex();

  memset(pilots, 0, MAX_NUM_PILOTS * sizeof(pilot_data_t));
  for(int i = 0; i < MAX_NUM_PILOTS; ++i) {
    pilots[i].number = i;
  }
  memset(receivers, 0, MAX_NUM_RECEIVERS * sizeof(receiver_data_t));
  pilot_queue.curr_size = 0;
  pilot_queue.max_size = MAX_NUM_PILOTS;
  pilot_queue.data = (void**)pilot_queue_data;

  adc1_config_width(ADC_WIDTH_BIT_12);

  for(int i = 0; i < MAX_NUM_RECEIVERS; i++) {
    adc1_config_channel_atten(ADC_PINS[i], ADC_ATTEN_6db);
  }

  //since the reference voltage can range from 1000mV to 1200mV we are using 1100mV as a default
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_6db, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  ina219.begin(&i2c_wire);
  i2c_wire.begin(I2C_SDA, I2C_SCL);
  ReadVBAT_INA219();
  uint16_t cutoff = getRXADCfilterCutoff();

  setPilotFilters(cutoff);

  filter_init(&adc_voltage_filter, ADC_VOLTAGE_CUTOFF, VOLTAGE_UPDATE_INTERVAL_MS/1000.0);
  uint16_t voltage = getVbatFloat(true) * 1000;
  Serial.printf("Voltage is %d minimum is %d\n", voltage, getMinVoltageModule());
  if(voltage >= getMinVoltageModule() && !disable_all_modules) {
    // By default enable getNumReceivers() pilots
    for(uint8_t i = 0; i < getNumReceivers() && i < MAX_NUM_PILOTS; ++i)  {
      setPilotActive(i, true);
    }
  }

}

adc1_channel_t IRAM_ATTR getADCChannel(uint8_t adc_num) {
  adc1_channel_t channel = ADC_PINS[MIN(adc_num, MAX_NUM_RECEIVERS - 1)];
  return channel;
}

void IRAM_ATTR nbADCread( void * pvParameters ) {
  static uint8_t current_adc = 0;
  uint32_t now = micros();
  if(xSemaphoreTake(pilots_lock, 1)) { // lock changes to pilots from other cores
    if(now - receivers[current_adc].last_hop > MULTIPLEX_STAY_TIME_US + MIN_TUNE_TIME_US && isRxReady(current_adc)) {
      if(setNextPilot(current_adc)) {
        // TODO: add class between this and rx5808
        // TODO: add better multiplexing. Maybe based on the last laptime?
        setModuleChannelBand(getRXChannelPilot(receivers[current_adc].current_pilot->number), getRXBandPilot(receivers[current_adc].current_pilot->number), current_adc);
        receivers[current_adc].last_hop = now;
      }
    }
    // go to next adc if vrx is not ready or has no assigned pilot
    if(isRxReady(current_adc) && receivers[current_adc].current_pilot) {
      adc1_channel_t channel = getADCChannel(current_adc);
      pilot_data_t* current_pilot = receivers[current_adc].current_pilot;
      if(current_pilot) {
        if(LIKELY(isInRaceMode())) {
          current_pilot->ADCReadingRAW = adc1_get_raw(channel);
        } else {
          // multisample when not in race mode (for threshold calibration etc)
          current_pilot->ADCReadingRAW = multisample_adc1(channel, 10);
        }

        // Applying calibration
        if (LIKELY(!isCalibrating())) {
          uint16_t rawRSSI = constrain(current_pilot->ADCReadingRAW, EepromSettings.RxCalibrationMin[current_adc], EepromSettings.RxCalibrationMax[current_adc]);
          current_pilot->ADCReadingRAW = map(rawRSSI, EepromSettings.RxCalibrationMin[current_adc], EepromSettings.RxCalibrationMax[current_adc], RSSI_ADC_READING_MIN, RSSI_ADC_READING_MAX);
        }

        current_pilot->ADCvalue = current_pilot->ADCReadingRAW;
        for(uint8_t j = 0; j < PILOT_FILTER_NUM; ++j) {
          current_pilot->ADCvalue = filter_add_value(&(current_pilot->filter[j]), current_pilot->ADCvalue);
        }

        if (LIKELY(isInRaceMode() > 0)) {
          CheckRSSIthresholdExceeded(receivers[current_adc].current_pilot->number);
        }

        if(current_pilot->number == 0){
           ++adcLoopCounter;
         }
         #ifdef DEBUG_SIGNAL_LOG
         if(current_pilot->number < DEBUG_SIGNAL_LOG_NUM) {
           readings[current_pilot->number][readings_pos[current_pilot->number]] = current_pilot->ADCReadingRAW;
           readings_pos[current_pilot->number] = (readings_pos[current_pilot->number] + 1) % DEBUG_SIGNAL_LOG_SIZE;
         }
         #endif // DEBUG_SIGNAL_LOG
      }
    } // end if isRxReady
    xSemaphoreGive(pilots_lock);
  }
  // use minimum here to ensure the first n receivers are used since the other ones might be offline in case we have more rx than pilots
  // use max to avoid division by 0
  current_adc = (current_adc + 1) % MAX(1, MIN(getNumReceivers(), current_pilot_num));
}


void ReadVBAT_INA219() {
  setVbatFloat(ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000));
  mAReadingFloat = ina219.getCurrent_mA();
}

void IRAM_ATTR CheckRSSIthresholdExceeded(uint8_t pilot) {
  uint32_t CurrTime = millis();
  // require a certain amount of samples below threshold to determine the maximum. prevents triggering a lap when one approaches the timer and the rssi fluctuates
  if ( pilots[pilot].ADCvalue > pilots[pilot].RSSIthreshold) {
    pilots[pilot].samples = 0;
    if (CurrTime > (getMinLapTime() + getLaptime(pilot))) {
      if(pilots[pilot].ADCvalue > pilots[pilot].max_adc && get_trigger_mode() == TRIGGER_PEAK) {
        pilots[pilot].max_adc = pilots[pilot].ADCvalue;
        pilots[pilot].max_time = CurrTime;
      } else if(get_trigger_mode() == TRIGGER_NORMAL){ // normal mode
        addLap(pilot, CurrTime);
        pilots[pilot].max_adc = 0;
        pilots[pilot].max_time = 0;
      }
    }
  } else if(pilots[pilot].max_adc && pilots[pilot].max_time) { // falling edge and max set
    ++pilots[pilot].samples;
    if(pilots[pilot].samples > 500) {
      pilots[pilot].samples = 0;
      addLap(pilot, pilots[pilot].max_time);
      pilots[pilot].max_adc = 0;
      pilots[pilot].max_time = 0;
      #ifdef DEBUG_SIGNAL_LOG
      // Print out signal at falling edge
      if(pilot < DEBUG_SIGNAL_LOG_NUM) {
        Serial.print("_");
        Serial.println(pilot);
        for(uint32_t i = 0; i < DEBUG_SIGNAL_LOG_SIZE; ++i) {
          Serial.println(readings[pilot][(i + readings_pos[pilot]) % DEBUG_SIGNAL_LOG_SIZE]);
        }
        readings_pos[pilot] = 0;
        Serial.println("-");
      }
      #endif
    }
  }
}

uint16_t getRSSI(uint8_t pilot) {
  if(pilot < MAX_NUM_PILOTS) {
    return pilots[pilot].ADCvalue;
  }
  return 0;
}

void setRSSIThreshold(uint8_t node, uint16_t threshold) {
  if(node < MAX_NUM_PILOTS) {
    pilots[node].RSSIthreshold = threshold;
  }
}

uint16_t getRSSIThreshold(uint8_t node){
  return pilots[node].RSSIthreshold;
}

uint16_t getADCLoopCount() {
  return adcLoopCounter;
}

void setADCLoopCount(uint16_t count) {
  adcLoopCounter = count;
}

void setVbatCal(float calibration) {
  VBATcalibration = calibration;
}

float getMaFloat() {
  return mAReadingFloat;
}

float getVbatFloat(bool force_read){
  static uint32_t last_voltage_update = 0;
  if((millis() - last_voltage_update) > VOLTAGE_UPDATE_INTERVAL_MS || force_read) {
    int32_t adc_reading = 0;
    switch (getADCVBATmode()) {
      case ADC_CH5:
        adc_reading = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC5), &adc_chars);
        setVbatFloat(adc_reading / 1000.0 * VBATcalibration);
        break;
      case ADC_CH6:
        adc_reading = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC6), &adc_chars);
        setVbatFloat(adc_reading / 1000.0 * VBATcalibration);
        break;
      case INA219:
        ReadVBAT_INA219();
      default:
        break;
    }
    last_voltage_update = millis();
    VbatReadingFloat = filter_add_value(&adc_voltage_filter, VbatReadingFloat);
  }
  return VbatReadingFloat;
}

void setVbatFloat(float val){
  VbatReadingFloat = val;
}

void setVBATcalibration(float val) {
  VBATcalibration = val;
}

float getVBATcalibration() {
  return VBATcalibration;
}

uint8_t getActivePilots() {
  return current_pilot_num;
}

bool isPilotActive(uint8_t pilot) {
  return pilots[pilot].state != PILOT_UNUSED;
}

void setPilotActive(uint8_t pilot, bool active) {
  // First we power up all modules. If we have less pilots than modules they get activated at a later stage. As this function will never be called during a race this should be okay
  // There might be a way better solution, but this will have to suffice for now
  // XXX: We have to reset the module since it won't come online with a simple power up

  // Power down all modules
  for(uint8_t i = 0; i < MAX_NUM_RECEIVERS; ++i) {
    RXPowerDown(i);
  }
  while(!xSemaphoreTake(pilot_queue_lock, portMAX_DELAY)); // Wait until this is free. this is a non critical section
  while(!xSemaphoreTake(pilots_lock, portMAX_DELAY));
  pilot_queue.curr_size = 0; // delete complete queue
  if(active) {
    if(pilots[pilot].state == PILOT_UNUSED) {
      // If the pilot was unused until now, we set them to active and add them to the queue
      pilots[pilot].state = PILOT_ACTIVE;
    }
  } else {
    if(pilots[pilot].state != PILOT_UNUSED) {
      pilots[pilot].state = PILOT_UNUSED;
      // Set adc values to 0 for display etc
      pilots[pilot].ADCvalue = 0;
      pilots[pilot].ADCReadingRAW = 0;
      // Since the pilot state is now unused, it won't get added back to the queue
    }
  }
  current_pilot_num = 0;
  // rebuild queue and delete all asignments. again this ist non-critical and the safest way to avoid bugs
  for(int i = 0; i < getNumReceivers(); ++i) {
    receivers[i].current_pilot = NULL;
  }
  for(int i = 0; i < MAX_NUM_PILOTS; ++i) {
    // first reset all rx assignments
    pilots[i].current_rx = NULL;
    if(pilots[i].state == PILOT_ACTIVE) {
      ++current_pilot_num;
      queue_enqueue(&pilot_queue, &pilots[i]);
    }
  }

  Serial.print("New pilot num: ");
  Serial.println(current_pilot_num);

  // only reset active modules. a user might have 6 modules installed but only uses 4. using the all function all modules would power up
  for(int i = 0; i < MIN(current_pilot_num, getNumReceivers()); ++i) {
    RXreset(i);
    while(!isRxReady(i));
    rxLowPower(i);
    while(!isRxReady(i));
  }

  // adjust the filters for all active pilots
  // Turns out just using the sample frequency and completely ignore the down time works the best. Still leaving this here in case this idea isn't so bad after all
  /*for(uint8_t i = 0; i < MAX_NUM_PILOTS; ++i) {
    // check both rx and pilots for 0 to avoid division by 0
    if(pilots[i].state != PILOT_UNUSED && getNumReceivers() && current_pilot_num) {
      uint32_t total_pilot_time_us = ((MULTIPLEX_STAY_TIME_US + MIN_TUNE_TIME_US) * current_pilot_num); // Total time for all pilots
      float on_fraction = MULTIPLEX_STAY_TIME_US / (float)total_pilot_time_us * getNumReceivers(); // on percentage of the pilot
      // special case for non multiplexing
      if(current_pilot_num <= getNumReceivers()) {
        on_fraction = 1.0/current_pilot_num;
      }
      for(uint8_t j = 0; j < PILOT_FILTER_NUM; ++j) {
        filter_adjust_dt(&pilots[i].filter[j], 1.0/(6000.0 * on_fraction)); // set sampling rate
        // when multiplexing we are using the average sampling rate per pilot as a timebase
      }
      Serial.printf("Set filtering cutoff to %f hz\n", (6000.0 * on_fraction));
    }
  }*/

  xSemaphoreGive(pilot_queue_lock);
  xSemaphoreGive(pilots_lock);
}

bool isPilotMultiplexOff(uint8_t pilot) {
  return pilots[pilot].disable_multiplexing;
}

void setPilotMultiplexOff(uint8_t pilot, bool off) {
  pilots[pilot].disable_multiplexing = off;
}

void setPilotFilters(uint16_t cutoff) {
  for(uint8_t i = 0; i < MAX_NUM_PILOTS; ++i) {
    for(uint8_t j = 0; j < PILOT_FILTER_NUM; ++j) {
      filter_init(&pilots[i].filter[j], cutoff, 166 * getNumReceivers() * 1e-6);
    }
  }
}

void setPilotChannel(uint8_t pilot, uint8_t channel) {
  setRXChannelPilot(pilot, channel);
  EepromSettings.RXChannel[pilot] = channel;
  setSaveRequired();
  if(isPilotActive(pilot)) {
    // Toggle pilot to force new channel
    setPilotActive(pilot, false);
    setPilotActive(pilot, true);
  }
}


void setPilotBand(uint8_t pilot, uint8_t band) {
  setRXBandPilot(pilot, band);
  EepromSettings.RXBand[pilot] = band;
  setSaveRequired();
  if(isPilotActive(pilot)) {
    // Toggle pilot to force new band
    setPilotActive(pilot, false);
    setPilotActive(pilot, true);
  }
}

void setPilotBandChannel(uint8_t pilot, uint8_t band, uint8_t channel)
{
  setRXChannelPilot(pilot, channel);
  setRXBandPilot(pilot, band);
  EepromSettings.RXChannel[pilot] = channel;
  EepromSettings.RXBand[pilot] = band;
  setSaveRequired();
  if(isPilotActive(pilot)) {
    // Toggle pilot to force new band
    setPilotActive(pilot, false);
    setPilotActive(pilot, true);
  }
}
