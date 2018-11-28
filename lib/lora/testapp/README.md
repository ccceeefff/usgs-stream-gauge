# LoRaMac Test App

This app exercises the LoRaWAN capabilities of this library. It does not follow
the standard esp-idf unit test procedures. The main code is based off Semtech's
LoRaMac node sample apps.

## To Run:

Set the IDF_PATH as any regular esp-idf app. Configure the sdk parameters using
make menuconfig, and flash to the device. It should continuously send lorawan
packets according to the parameters set in Commisioning.h.
