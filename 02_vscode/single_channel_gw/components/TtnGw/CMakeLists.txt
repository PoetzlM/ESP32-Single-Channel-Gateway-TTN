idf_component_register(
	SRCS "LoRaModemSX127xHandler.cpp" "LoRaModemSX127x.cpp" "TtnGwUdpHandler.cpp" "TtnGwTime.cpp" "OneShotTimerInterrupt.cpp" "TtnGwUdpParser.cpp" "TtnGwSerialUiConfig.cpp" "TtnGwDataTypes.cpp"
	INCLUDE_DIRS "."
	REQUIRES arduino-esp32 ezTime arduino_base64 ArduinoJson
)
