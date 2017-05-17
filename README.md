This project aims couple things:
1. Figure what and how arduino is.
2. FIX stupid child C/C++ mixes  met into arduino libs (believe or not, C++ templates has less bytes compiled then pure C).
3. I need alt/azimuth tracker for the telescope for the Dobson mount.
4. Very good if it will automatically show target in stellarium.
5. It seems that all my automation programs (this one + astroed +qtcampp) will form automated journaling and processing system some day - kinda - just point and picture, all records and marks will be automated.

TODO:
* send full sensor's qutarnium to the PC for the record. It can help with camera rotation detection.
* need a scripts, which will categorize that sql db and pictures from camera in folder automatically on desktop (initially DB is on laptop).
* addition to astroed - put all that data in use together in picture processing
* add some query to somewhere, which will reveal star pictured by that RA/DEC, not sure if stellarium can do that automatically

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
* store all data(ra, dec, qutarnium) into sql-lite db (once per 5seconds or so). Each new observation day makes new file/db.

3. It back tracks on map, once you have aligned device by any star it will start tracking on stellarium map any moves of the telescope.

Limitations:
It has drift. Pretty big as for telescope, but still device is usable. If you point some object, then "jump" to other it will track accurate enough.
So system is ok for home usage with high light pollution and limited room (when you have to move telescope) - it still gives position, just need to "reclibrate" on map each time.
