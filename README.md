# Real Time Fire Alarm
The Real Time Fire Alarm is a safe, reliable and efficient solution to ensure your safety. With use of concurrent tasks and multiple alarm mechanisms. RTFA device **will** ensure you are notified in case of an emergency with the following indicators:
1. Text messages through UART communication port
2. Flashing lights
3. LCD alarm display 

## System Process Flow

The alarm system works by sending IR rays through its IR producer and reading the bounce from the flames with its IR reciever. If the value is above a dangerous threshold the alarm will be triggered with use of a global flag. 

Other tasks such as the user interface, communication and alarm managment tasks pivot around the value of the global flag.

## Design Schematic
![circuit_schematic](https://github.com/hmolavi/Real_Time_Fire_Alarm/assets/75816912/64bb8f81-4405-4b97-a2ff-323629f57851)
