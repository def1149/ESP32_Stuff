/// Low Power Zigbee Temperature Sensor with Deep Sleep 
///
/// ---------------------------------------------------
/// Dependencies: 
///    H/W: ESP32-C6/H2 
///    Espressif Arduino Library 3.2.0
///       
///       With Configuration set as follows:
///         #define TIME_TO_SLEEP  60         // Sleep time in seconds between wakeup for temperature checks
///         #define TEMP_DELTA 0.3            // If temperature changes by more than this, report to host
///         #define  MAX_REPORT_SECS 3600     // Maximum time without reporting  
///
///         ESP32=C6 Clock: 80 Mhz
///           Sleeps at about 19 uA      
///           One hour average about 0.6 mA  reporting only once per hour YMMV
///         ESP32-H2
///           Sleeps at about 9 uA      
///           One hour average about 0.3 mA  reporting only once per hour YMMV
///
///        Includes Watchdog Timer in case device hangs
///        Includes flexible LED Blink function
///        Includes flexible Sleep function    
///        Has a simple moving average function to smooth out a jittery temperature sensor data to eliminate suprious reports
///
///
///   Occam   03/28/2025 https://github.com/def1149/ESP32_Stuff
///
///   Provided As Is without warranty, and without support unless I feel like it. 

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include <rom/rtc.h>
#include "esp_task_wdt.h"
#include <Preferences.h>

#define BOOT_PIN                  9   //Boot button for C6/H2

/* Zigbee temperature sensor configuration */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10

// Tailor report parameters 
#define uS_TO_S_FACTOR    1000000ULL                                  // Conversion factor for micro seconds to seconds 
#define TIME_TO_SLEEP     60                                          // Sleep time in seconds between wakeup for temperature check
#define LONG_SLEEP        (60*15)                                     // Long sleep if EP cant connect
#define TEMP_DELTA        0.3                                         // Temp Delta. If temperature changes by more than this, report 
#define MAX_REPORT_SECS   3600                                        // The maximum time without a report in seconds
#define MAX_CYCLES        ((MAX_REPORT_SECS/TIME_TO_SLEEP) - 1)       // Maximum number of temperature read cycles without reporting 

#define TRUE true
#define FALSE false

// Store in RTC memory to persist across deep sleep cycles
RTC_DATA_ATTR int cycleCount = 0;                // Keep track of number of wake up cycles wuithout reporting 
RTC_DATA_ATTR float temperature  = 0.0;          // Keep revious reported temperature for comparison with new
RTC_DATA_ATTR long int resetCount = 0;           // Keep track of number of attempts to connect 

//Moving Average Data persist across deep sleep cycles
#define WINDOW_SIZE 3                            // Minimum size = 1 No Average 
RTC_DATA_ATTR  float buffer[WINDOW_SIZE] = {0};  // Buffer to store MA values
RTC_DATA_ATTR  int idx = 0;                      // Current position in buffer
RTC_DATA_ATTR  int count = 0;                    // Number of values received
RTC_DATA_ATTR  float sum = 0;                    // Running sum of values

Preferences preferences;                         // Flash storage for ENROLLED flag so it persistes across resets

uint8_t button = BOOT_PIN;
ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

// Configure the Watchdog 
#define WDT_TIMEOUT 35000
#define CONFIG_FREERTOS_NUMBER_OF_CORES 1

esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 2,    // Bitmask of all cores
        .trigger_panic = true,
    };

// Blink the built-in LED
void ledBlink(int numBlinks, int duration, int secsDelay) { // duration mS, secsDelay in seconds
  // Used signal status when there's no serial output
  delay(secsDelay * 1000);             // Delay before the blink
  if(numBlinks == 0) {
    digitalWrite(LED_BUILTIN, LOW);    // Turn LED off
  } else if (duration == 0) {
    digitalWrite(LED_BUILTIN, HIGH);   // Turn LED on 
  } else {
    for(int i= 0; i<numBlinks; i++) {  // Blink LED
      digitalWrite(LED_BUILTIN, HIGH);
      delay(duration);
      digitalWrite(LED_BUILTIN, LOW);
      if(i<numBlinks)                 // No delay after last blink
        delay(duration); 
    }
  }
}

// Parameterized sleep function 
void sleep(bool persistent, int timeToSleep) { // timeToSleep in seconds
  if(persistent) {
     // Go to sleep and not wake up unless hard reset or power cycle
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);       // In sleepy app, ALL wakeup sources wopuld need to be disabled first to force persistent sleep
    ledBlink(5,300,2);                                           // Five blinks indicates persistent sleep
  } else { 
    esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);                      
  }
  esp_deep_sleep_start();   
}

// Simple Moving Average function
float movingAverage(float newValue) {
    // Smooth Temperature values with a simple moving average w/variable window size
    if(WINDOW_SIZE < 2) {
      return newValue;
    } else {

      // Subtract the oldest value from sum before overwriting
      if (count >= WINDOW_SIZE) {
          sum -= buffer[idx];
      }
      
      // Add new value to buffer and update sum
      buffer[idx] = newValue;
      sum += newValue;
      
      for (int i= 0;i < WINDOW_SIZE;i++) {
          log_v("buffer[%d] = %.2f\r\n", i, buffer[i]);
      }
        
      // Update index and count
      idx = (idx + 1) % WINDOW_SIZE;
      if (count < WINDOW_SIZE) {
          count++;
      }
    
      // Return average based on current number of values
      return (float)(sum / count);
    }
}


/********************* Intialize Zigbee and Connect **************************/
bool initializeZigbee(bool doFactoryReset) {             // If doFactoryResart is true, perform a factory reset 
  //Start Zigbee and Connnect

  preferences.begin("Zigbee", false);                    // Get device status, Enrolled? 
  bool enrolled =  preferences.getBool("ENROLLED");      
  preferences.end();

  esp_task_wdt_reset();

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("Occam", "Sleepy Zigbee TempSensor Example");

  // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
  zbTempSensor.setMinMaxValue(-30,80);    // NB. If the temperature value is lower than the set minimum, a garbage is reported
                                         
  // Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
  zbTempSensor.setTolerance(0.1);

  //Set Power Source
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100);

  // Add humidity cluster to the temperature sensor device with min, max and tolerance values
  zbTempSensor.addHumiditySensor(0, 100, 0.1);
  
  // Add endpoint to Zigbee Core
  log_v("Add Endpoint\r\n");
  Zigbee.addEndpoint(&zbTempSensor);
 
  // Create a custom Zigbee configuration for End Device with keep alive 10s to avoid interference with reporting data
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;
  
  // For battery powered devices, it can be better to set timeout for Zigbee Begin to lower value to save battery
  // If the timeout has been reached, the network channel mask will be reset and the device will try to connect again after reset (scanning all channels)
  Zigbee.setTimeout(15000);  // Set timeout for Zigbee Begin to 15s (default is 30s)
  
  esp_task_wdt_reset();

  log_v("Begin Zigbee\r\n");
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    log_v("Zigbee failed to start!\r\n");
    ledBlink(3,300,2);                 // Three Blinks Zigbee failed to start
    
    resetCount++;                      // Keep track of attempts to Start/Connect

    // If the device is curently NOT enrolled on coordinator, shutdown abd enter persistent sleep
    // Otherwise keep retrying but back-off retry interval after five tries
    if(!enrolled) {
      log_v("Shutting Down. Press reset to try again ...\r\n");  
      sleep(TRUE,0);
    }
    else if (resetCount<5) {    
      sleep(FALSE,15);                // 5 retries @15 seconds    
    } else {
      sleep(FALSE,LONG_SLEEP);        // Keep retrying every 15 minutes
    }  
     log_v("Rebooting...\r\n");    
  }

  log_v("Zigbee Started.\r\n");
  esp_task_wdt_reset();

  if(doFactoryReset) {
    // If boot button pressed for more than 5 secs at reset startup, factory reset Zigbee entger  persistent sleep
    ledBlink(4,300,2);                                          // Four Blinks means Factory Reset executed
    preferences.begin("Zigbee", false);                         // Save NOT Enrolled 
    preferences.putBool("ENROLLED",FALSE); 
    preferences.end();
    delay(50);
    Zigbee.factoryReset(false);                                 // Call Factory Reset without rebooting 
    sleep(TRUE,0);                                              // Enter persistent sleep
    
  }
  esp_task_wdt_reset();
  
  log_v("Connecting to network\r\n");

  int i = 0;
  while (!Zigbee.connected()) {
    if (i++ > 200) {   // ~ 20 seconds 
      
      resetCount++;

      // If the device is curently NOT enrolled on coordinator, shutdown abd enter persistent sleep
      // Otherwise keep retrying but back-off retry interval after five tries
      if(!enrolled) {
        sleep(TRUE,0);                                          // Enter presistent sleep
      }
      else if(resetCount<5) {                                   // 5 reties @15 seconds    
        sleep(FALSE,15);   
      } else {
        sleep(FALSE,LONG_SLEEP);                               // Keep trying every 15 minutes
      }     
    }
    log_v(".");
    delay(100);
  } 

  esp_task_wdt_reset();
  log_v("\r\nSuccessfully connected to Zigbee network\r\n");
  if(!enrolled) {
    preferences.begin("Zigbee", false);                         // Save Enrolled state in flash  
    preferences.putBool("ENROLLED",TRUE);                       
    preferences.end();
  }
  resetCount = 0; 
  return TRUE;
}

 // Send data to coordinator 
void report(float temperature)
{
     
  esp_task_wdt_reset();

  if(initializeZigbee(FALSE)) {
    // Report temperature and Hunidity

    log_v("Set Temp/Humidity");
    zbTempSensor.setTemperature(temperature);
    zbTempSensor.setHumidity(temperature);   // Use Temperature as Humidity 
    
    log_v("Report to Coordinator");
    zbTempSensor.report();
                                                                                         
    log_v("Reported temperature: %.2f°C, Battery Pct: %d%% \r\n", temperature, battPct );
  
    esp_task_wdt_reset();

   
    // Delay approx 10s during enrollmnt/reset (may be adjusted) to allow establishing proper connection with coordinator.
    if ((int)rtc_get_reset_reason(0) != 5 )  { // =  SW_CPU_RESET=12  // POWERON_RESET=1 //DEEPSLEEP_RESET=5
      delay(10000);
    } else {
      delay(30);   // Allow time for update. Needed? Is there a "delivered"  status that can be checked in loop? 
    }
  
  // Reset after reporting 
  cycleCount = 0;
  }
}
  
/************************ Temp sensor *****************************/
void checkTemp() {
  // Measure temperature sensor value and report data if channge > delta

  esp_task_wdt_reset();
               
  float newTemp = temperatureRead();             // Read the CPU temperature

  log_v("New temperatue (Raw)  = %.2f °C\r\n",newTemp);

  newTemp = movingAverage(newTemp);             // Filter any jitter 

  float tempDelta = abs(newTemp-temperature);   // What was the temperature change? 

  log_v("New temperatue        = %.2f °C\r\n",newTemp);
  log_v("Previous temperature  = %.2f °C\r\n",temperature);
  log_v("Temperature delta     = %.2f °C, Max delta = %.2f °C\r\n",tempDelta,TEMP_DELTA );
  log_v("Read cycle count, %d of %d Max cycles\r\n",cycleCount, MAX_CYCLES );
   
  // Report temp if temp change > than minimum  difference  or number of checks exceeds maximum or the boot button was pushed briefly
  if((tempDelta > TEMP_DELTA) || (cycleCount > MAX_CYCLES)) {
     temperature = newTemp;
    report(temperature);
  } 
}


/********************* Arduino functions **************************/
void setup() {

  //Init LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Init button switch
  pinMode(button, INPUT_PULLUP);
 
  bool doReset = FALSE;

   //Get reset reason
  int resetReason = (int)rtc_get_reset_reason(0);
  log_v("Restart = %d\r\n",resetReason);

 
  if (resetReason == 1)  {  // POWERON_RESET=1 //DEEPSLEEP_RESET=5
      ledBlink(1,2000,0);   // Let user know to press the Boot button if they want a Factory Reset
      
      //Check the button for factory reset
      if (digitalRead(button) == LOW) {  // Boot button pressed
        
        // Key debounce handling
        delay(100);
        int startTime = millis();
        while (digitalRead(button) == LOW) {
          delay(50);
          if ((millis() - startTime) > 3000) {
            log_v("Do Factory Reset\r\n");
            // If boot button pressed for more than 5 secs, factory reset Zigbee and reboot
            doReset = TRUE;       // Set the reset flag for use during Zigbee Start
            ledBlink(2,300,2);    // Indicate factory reset commanded
          }
        }
      }  
  } 

  // Start WDT
  esp_task_wdt_reconfigure(&twdt_config); 
  esp_task_wdt_add(NULL);

  if(doReset) {
    log_v("\r\nInitiate Factory Reset... \r\n");
    initializeZigbee(TRUE);
  } else {
    log_v("Read and check for temp change.\r\n");
    checkTemp();    
  } 
  cycleCount++; // Bump the read cycle count


  // Put device into deep sleep and wake up on timer
  log_v("Going to sleep now for %d seconds.\r\n",TIME_TO_SLEEP);
  // Configure Deep Sleep wakeup timer 
  sleep(FALSE,TIME_TO_SLEEP);

}

void loop() {
  // Never executes
}