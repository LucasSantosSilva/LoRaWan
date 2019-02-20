# LoRaWan
Projetc using a station with barometer, anemometer, windsock, pluviometer and ESP 32 LoraWan 


In this project was used a meteorological station, in which it contains a client and a getway. The client was about 1 km from the getway, the data was then processed and sent to the TTN (https://www.thethingsnetwork.org/) and then sent to the Cayenne.

In the TTN itself it is possible to link with the Cayenne, thus sending the data direct to Cayenne

Library used into client
https://github.com/matthijskooijman/arduino-lmic

Library used into Getaway
https://github.com/things4u/ESP-1ch-Gateway-v4.0
