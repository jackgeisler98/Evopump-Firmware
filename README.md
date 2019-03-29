# Evopump-Firmware
Development firmware for CamMED inc.'s Evopump


These files were written as a part of Nordic's SDK 15.2.0 and developed on the nRF52840 MCU and PCA 10056 board. Files compile when included in an application template in the nordic SDK. 

The fully functioning project features voltage output and current control in order to cause electrolysis in the pump. A sensor was integrated to read in temperature and pressure to allow for environmental compensation that will be added later. A Real Time Timetable algorithm was integrated to allow for continuos timeline compensation based on the volume of medication previously pumped. A timer was added to keep real time on a low frequency clock and this time was saved to the flash memory every 5 seconds in case of a system power off. Bluetooth low energy was added to transmit the recorded flash data only when the board is connected to the nRF Connect iOS app. Finally, an ADC function was used to measure battery voltage which will be monitored within a predefined range in the future. 

Currently working on a POST function for power on as well as a new shdn_timer_timeout_handler(); function to allow for user controled, real time compensated insulin deliveries. Files will be seperated and shared as individual features approach completion. 
