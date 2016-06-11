##BLE CUBE - Firmware Stack
This is the repository for the Firmware Stack of this project. The build file is found in [firmwarestack/pca10040/s132/arm5_no_packs/](https://github.com/blecube/firmwarestack/tree/master/pca10040/s132/arm5_no_packs)

####nRF52832
This firmware stack is written for use on Nordic Semiconductor's SoC [nRF52832](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy2/nRF52832)


####SoftDevice v2.0.0
The SoftDevice in use for this project is [v2.0.0](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.s132.sds%2Fdita%2Fsoftdevices%2Fs130%2Fs130sds.html)

####BLE_card.h and PCA10040.h
This firmware stack is written for our own PCB called BLE Card. In the preprocessor symbols this card is defined. For use with a development kit, BOARD_BLECARD should be substituted with BOARD_PCAxxxxx. The x's are printed on the debug-chip on your development kit. 

####Necessary Software
A good place to start with development on nRF52-series is [Nordic Semiconductor's own "Getting Started" guide.](https://devzone.nordicsemi.com/tutorials/2/) This web page will inform you of every software needed for this project. As well as help you flash your first example code



> Created by Nordic Semiconductor

> Edited by Ørjan Storås and Even Johan Christiansen