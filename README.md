This is an example program that works as a transceiver and shows data on the display for the LoRa32 HeltecV3 board using rust and the embassy framework

# How to use
Send the text you want to transmit via serial UART0 (basically usb) and see the message on the oled screen or serial terminal of the other devices

# Why?
I wanted to test rust on embedded systems, the embassy framework seemed like a good start with the no-os-async thing. Truth to be told the lora-rs lora high level abstraction is kinda restricted I had to manipulate the SX1262 class in order to read messages in a non-blocking way, it could also be a skill issue since is something that could be fixed with mo threads and mo mutexes lol