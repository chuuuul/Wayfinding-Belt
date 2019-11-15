import serial

gps = serial.Serial("/dev/ttyUSB0", baudrate = 9600)

while True:
	line = gps.readline()
	data = line.split(",")
	if data[0] == "$GPRMC":
		if data[2] == "A":

			latgps = float(data[3])
			if data[4] == "N":
				latgps = latgps
			
			latdeg = int(latgps/100)
			latmin = latgps - latdeg*100
			lat = latdeg+(latmin/60)

			longps = float(data[5])
			if data[6] == "E":
				longps = longps

			londeg = int(longps/100)
			lonmin = longps - londeg*100
			lon = londeg+(lonmin/60)

			print "Lat : %s  /  Lon : %s " % (lat, lon)

			with open ("position.txt", "w") as pos:
				pos.write("%s, %s\n" % ( lat, lon ))
			