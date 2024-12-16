# VOLUME GRAPH

Volume Graph is a program targeting STM32L475VGTx that displays audio volume readings in a line graph on the Adafruit 2.8" TFT Capacitive Touch Display. The problem the project aimed to solve is monitoring sleep volume, with a reach goal of indicating sleep quality.

## How it works
Two MP34DT01-M microphones are available on the b-l475e-iot01 discovery board. Only one is required for absolute volume, it is configured to use Pulse Density Modulation (PDM) to provide an audio reading. That value is converted to decibels for the purpose of the graph, and stored into a circular buffer of size 256. Audio levels are read in at intervals of 500ms and pushed to the buffer. With each read, the program prints the graph to the TFT display.
