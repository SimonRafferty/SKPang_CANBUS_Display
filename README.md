# SKPang_CANBUS_Display
Simple example of a CANBUS display based on SKPang / Copperhill board

https://www.skpang.co.uk/products/teensy-4-0-classic-can-can-fd-board-with-480x320-3-5-touch-lcd

or

https://copperhilltech.com/teensy-4-0-classic-can-can-fd-board-with-480x320-3-5-touch-lcd/

I'd bought one of these & found that the example files linked didn't work (for me).  So, here is an example, using the Teensy FLEXCAN library, which does!
I built this for my Polaris General, which I've converted to Electric.

You can see in the code, I'm only receiving two CAN IDs and derriving the display from those.  You'll have to figure out what ID's you want to receive and adapt the code & display accordingly.

I've not implemented the touchscreen - but the Adafruit library mentioned in the documentation seemed to work.

This is not intended to be the answer to all your prayers - turnkey solution for your application, just something to get you started in a fruitful direction.
I'm not able to re-write it for you - though I will answer questions when I'm able.

I've also included the files for a 3D printed case.
