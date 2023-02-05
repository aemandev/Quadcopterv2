C:

cd C:\Program Files\FlightGear 2020.3.6

SET FG_ROOT=C:\Program Files\FlightGear 2020.3.6\data

START .\\bin\fgfs.exe --fdm=null --native-fdm=socket,in,30,localhost,5502,udp  --enable-terrasync --prop:/sim/rendering/shaders/quality-level=0 --aircraft=arducopter --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --airport=KSFO --runway=10L --altitude=16 --heading=0 --offset-distance=0.01 --offset-azimuth=0  
