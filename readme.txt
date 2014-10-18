==================================
 Beacon v0.03
==================================

This is the firmware for a mobile GPS tracking device. It runs on an
AVR ATmega8 microcontroller and controls an Telit GM862-GPS module.
The firmware supports tracking with GPS coordinates send via SMS.
The SMS contains a formatted link to Google Maps. This SMS must be
send to an SMS-to-Email service, which forwards the email to your
email box. Clicking on the link in this email will open up Google
Maps with your position on the center.

For more details, please visit:
http://tinkerlog.com/2007/07/28/using-google-maps-with-a-mobile-gps-tracker/
http://tinkerlog.com/2007/07/13/interfacing-an-avr-controller-to-a-gps-mobile-phone/

Please note: This project is a DIY project. To rebuild it you should
have soldering skills and should be familliar with programming AVR
controllers.

Features
--------
* Displays menu via serial port if attached to PC
* Can run automated or interactive for debugging
* Fetches GPS positions and sends them via SMS every two minutes


Contents
--------
beacon.c		Main program
uart.c, uart.h		UART serial communication to the GM862
suart.c, suart.h	software UART for serial communication with the PC
readme.txt		This file


Setup
-----
* Buy all components and assemble them.
* Get yourself a SIM, if you don't want to switch it from your mobile phone.
* Edit beacon.c, replace all "EDIT THIS" parts with your settings. You have 
  to provide the PIN for your SIM. Also the number of the SMS-to-Email gateway
  has to be provided. Check your telco-provider for that service number.
* Compile it with WinAVR. I am not sure if it compiles out of the box with
  other AVR compilers but it should not be a problem to adopt it.
* Program your device.
* Attach your terminal to the device and you should see the menu and the 
  controller, trying to switch the module on.

Contact
-------
Visit http://tinkerlog.com for latest infos on this device. You can also leave
me a message at alex@tinkerlog.com.







