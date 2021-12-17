# Mennekes lock


Mennekes lock is the mechanism for locking a charging cable into a non-tethered socket (IEC 62196-2). The purpose of the lock is to avoid the hazard of the cable being disconnected during a charging session and secure the cable to avoid theft. 


IEC 62196-2 enabled vehicles (all vehicles sold in Europe) also lock the cable at the vehicle end, usually there is a button on the keyfob and or dashboard to unlock the vehicle end of the cable which also stops the changing session. 


## Firmware 


This documentation references openevse controller firmware 


Controller Firmware: V8.1.0a+
RAPI: V5.2.0
Issue: https://github.com/lincomatic/open_evse/issues/141


To enable Mennekes lock uncomment `#define MENNEKES_LOCK` in `open_evse.h`


## Hardware connections 


Mennekes lock currently uses the ISP pins as digital out pins


MOSI - Pin A - LOCK
MISO - Pin B - UNLOCK


Mennekes lock requires a 12V H-bridge, tested with L298N duel H-bridge drive


## Operation 


Mennekes lock has both automatic and manual operation 


### Automatic 


The Mennekes lock will be activated (locked) when a vehicle is connected and will be unlocked when a vehicle is disconnected. To disconnect a vehicle the vehicle end of the cable should be unlocked first. **Automatic is default mode,**


#### Manual 


Manuel operation mode was introduced to allow the Mennekes lock to be controlled by ESP32 WiFi module via RAPI. Manual mode could be controlled via OCPP, HTTP API manually via the web interface. Manual mode could allow the user to permanently lock the cable into the socket, e.g to prevent cable theft for home use when the cable is left permanently in between charging sessions. **Manual mode is volatile, the unit will always boot into automatic mode**


##### RAPI 


```
S5 A|M|0|1 - Mennekes lock setting
   A = enable automatic mode - locked when connected, unlocked otherwise
   M = enable manual control mode
   0 = unlock (valid only in manual mode)
   1 = lock (valid only in manual mode)
   n.b. requires MENNEKES_LOCK. manual mode is volatile - always boots in automatic mode
```

```
G5 - get Mennekes settings
 response: $OK state mode
   state: 0 = unlocked
          1 = locked
   mode: A = automatic mode - locked when connected, unlocked otherwise
         M = manual control mode
   Note: lock mode is also indicated by ECVF_MENNEKES_MANUAL
   n.b. requires MENNEKES_LOCK
```