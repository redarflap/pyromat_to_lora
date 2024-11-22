<h1>Köb Pyromat to LoRa for ESP32 (ESP-IDF)</h1>
<br/>
<h3>What does it do?</h3>
- Connects to the TWAI (CAN) bus of a Köb Pyromat ECO wood heater.<br/>
- Continuously buffers selected data fields from CAN<br/>
- Transmit buffered data via Lora every 30 seconds<br/>

<br/>
The data can then be received by another LoRa device (openmqttgateway) and finally be passed to the Home Automation solution of your choice via MQTT.
<br/>
<h3>Requirements</h3>

ESP32 + CAN Transceiver + Lora Transceiver
