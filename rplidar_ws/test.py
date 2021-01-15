from adafruit_rplidar import RPLidar
lidar = RPLidar(None, '/dev/ttyUSB0')

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))
    if i > 10000:
        break
   
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
