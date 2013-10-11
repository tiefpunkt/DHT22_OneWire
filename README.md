# DHT22 on OneWire
Want to use a larger number of DHT22s with long cable runs inbetween? Hook them up using OneWire.

This project uses a ATtiny25 as an controller between the OneWire bus and the DHT22 sensor. It emulates a DS18B20 OneWire temperature sensor, courtesy to http://www.tm3d.de/index.php/1-wire-device-mit-avr. The humidity value from the DHT22 is stored in the 16bit space in the scratchpad, right after the temperature value, and can be read just like the temperature. A reference arduino sketch will be in this repo as well eventually

DHT22 sensor library from http://www.pgollor.de/cms/?page_id=1013

## Circuit
Just a plain ATtiny25. DHT22 connected to pin B1, OneWire Bus connected to B2.

## License
Whatever complies with both parts used. Probably GPL3 (even if I don't like that one).
