<h2>Schematic and Gerber Files</h2>

The PCB (printed circuit board) is a standard 2 layer board.  It can be manufactured by sending the Gerber files to sites like https://jlcpcb.com/.
It only costs $2 to get 5 PCBs.  That is quite the bargain.

You will need to create a free account, then click "Order Now".  There will be a button to upload the Gerber zip files.
It will automatically fill in most options once you have uploaded the file.  No changes to the options are required, but I like change these options.

* "PCB Color" defaults to Green, which is fine, but I like Black.
* "Mark on PCB" defaults to Order Number, but I like to Remove Mark, to keep the board clean. 

<h2>Schematic</h2>
<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/schematics.png" alt="Schematics" width=600>
<h2>Printed Circuit Board</h2>
<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/PCB.png" alt="PCB" width=400>

<h2>Electronics assembly</h2>

Solder all components on the PCB.  

<b>NOTE: Make sure to use the power switch standoff under the toggle switch so that it will be in the right height
so that you can operate it without opening the Skidsteer covers.</b>

<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/IMG_3643.JPEG" alt="Soldering" width=400>
<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/IMG_3644.JPEG" alt="Soldering" width=400>

<h3>5V Buck Converter</h3>
Assembly the 5V buck converter. Ensure that the 5V jumper has solder and you use an exacto knife to cut the trace above the ADJ jumper.

<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/S20250310_0001.jpg" alt="Buck Converter" width=600>

Assembly the PCB, batteries, fuse, and 5V buck converter.  Turn on power and ensure buck converter is generating +5V.
When the slide switch is towards to top of the board (closest to the batteries), it is in the ON position.  Down is off.
There are labeled test points on the servo pins on the back of the board that you can use to test for +5V.

<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/IMG_3645.JPEG" alt="Buck Converter" width=400>
<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/IMG_3686.JPEG" alt="Buck Converter" width=600>

<h3>DRV8833 Motor Drivers</h3>
Assembly the DRV8833 Motor Driver boards.  Ensure that J2 jumper is soldered.

<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/S20250310_0002.jpg" alt="DRV8833" width=600>

Add DRV8833 boards and ESP32 board to PCB.  Ensure that all 3 LEDs illuminate when power is on.

<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/IMG_3646.JPEG" alt="DRV8833" width=400>
<img src="https://github.com/swholmstead/Skidsteer/blob/main/pictures/IMG_3647.JPEG" alt="ESP32" width=400>

