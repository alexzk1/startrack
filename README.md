This project aims couple things:
1. Figure what and how arduino is.
2. FIX stupid child C/C++ mixes  met into arduino libs (believe or not, C++ templates has less bytes compiled then pure C).
3. I need alt/azimuth tracker for the telescope for the Dobson mount.
4. Very good if it will automatically show target in stellarium.

What is done:
1. It tracks. Connected to telescope, pointed to Polar Star,then MOVED 
with alltogether to the other room, and this device keeps tracking of 
what and where I see.
2. Now, it can be set from Stellarium:
* launch server
* connect server to device
* setup long/lat on server
* setup "telescope control" in Steallarium to server on localhost:10001 using J2000
* find visually be telescope any star (with device attached to telescope)
* pick the same star in steallrium and press ctrl + 1 (or any other number associated with connection, don't miss with alt+1 which do something else and fake for us)
* now device shows exact alt/azimut what was in steallarium and keeps tracking moves further

3. Todo: back-track on map