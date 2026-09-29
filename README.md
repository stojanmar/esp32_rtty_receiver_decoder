# esp32_rtty_receiver_decoder
This is a standalone RTTY receiver-decoder,  using a cheap yellow display and mini microphone board. Cheap 1,9" display integrates a standard ESP32 microcontroller.

This project demonstrates, how can you use a cheap yellow display, to decode the rtty signal.
You can decode rtty signals using PC with apropriate sowtware, but I wanted to have stand alone device, that can receive and display text without running a computer or tablet.
So I made this solution using just two cheap components with minimal wireing. A display with integrated popular ESP32 microcontroller and a mini microphone board. 
This way rtty receiver does not need to be connected to radio, but symply placed near the radio speaker.
I started my RTTY receiver development with intention first to decode a signal which comes from weather report transmitters. Those usually run continuously 24/7.
This way I didn need to chase RTTY signal on different frequences. Later this work may be extended to decode HAM radio rtty signals.

Components needed:
1x CYD ESP32-1732S019 + 1x MAX4466 Electret Microphone Amplifier with Adjustable Gain
Connect them like this:

<img width="4032" height="2268" alt="parts" src="https://github.com/user-attachments/assets/16279a84-60c5-4e95-958d-6b2070b26ad8" />

Tools needed:
Arduino IDE and ESP32 SDK core 2.0.17 ; other may work too.

Youtube video link: https://www.youtube.com/watch?v=vk9e7C9kq8o

<img width="1365" height="768" alt="IMGfTtiuMl7go" src="https://github.com/user-attachments/assets/5fc974fe-479e-4703-a549-38f8e7b9371e" />

