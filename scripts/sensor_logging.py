#! /usr/bin/python
# Based off the code written by Dan Mandle http://dan.mandle.me September 2012
# Modified by Kevin Kingsbury: https://github.com/kmkingsbury
# License: GPL 2.0

import os
import time
import RPi.GPIO as GPIO

from time import gmtime, strftime
import threading
import csv

from math import log

#Sesnsor libraries
import Adafruit_DHT
import Adafruit_BMP.BMP085 as BMP085

GPIO.setmode(GPIO.BCM)
sensor = 11
DHTpin = 4

BMP_i2c_busnum=1

SPICLK = 18
SPIMISO = 23
SPIMOSI = 24
SPICS = 25

mybutton = 40
mywindspeed = 38
myraingauge = 37
light_adc = 1
winddir_adc = 7

def readDHTHumidityTemp():
    sensor = Adafruit_DHT.DHT22
    humidity = None
    # Use the read_retry method which will retry up
    # to 15 times to get a sensor reading (waiting 2 seconds between each retry).
    while humidity is None:
        humidity, temperature = Adafruit_DHT.read_retry(sensor, DHTpin)
    return humidity, temperature

def dewPoint(humidity, celsius):
    a = 17.271
    b = 237.7
    temp = (a * celsius) / (b + celsius) + log(humidity*0.01)
    Td = (b * temp) / (a - temp)
    return Td

def heatIndex(humidity, tempF):
  c1, c2, c3, c4, c5, c6, c7, c8, c9 = -42.38, 2.049, 10.14, -0.2248, -6.838e-3, -5.482e-2, 1.228e-3, 8.528e-4, -1.99e-6
  T = tempF
  R = humidity

  A = (( c5 * T) + c2) * T + c1
  B = ((c7 * T) + c4) * T + c3
  C = ((c9 * T) + c8) * T + c6

  rv = (C * R + B) * R + A
  return rv

windref = {}
windref[38]="West"
windref[67]="NW"
windref[100]="WNW"
windref[131]="North"
windref[182]="NNW"
windref[238]="SW"
windref[261]="WSW"
windref[379]="NE"
windref[433]="NNE"
windref[554]="South"
windref[611]="SSW"
windref[697]="SE"
windref[786]="SSE"
windref[844]="East"
windref[860]="ENE"
windref[892]="ESE"

def direction(number):
    for i in windref.keys():
        if i-8 < number < i+8 :
            return windref[i]
    return "Unknown"

runner = True
rain_count = 0
windspeed_count = 0
# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(cspin, True)

        GPIO.output(clockpin, False)  # start clock low
        GPIO.output(cspin, False)     # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3    # we only need to send 5 bits here
        for i in range(5):
                if (commandout & 0x80):
                        GPIO.output(mosipin, True)
                else:
                        GPIO.output(mosipin, False)
                commandout <<= 1
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)
                adcout <<= 1
                if (GPIO.input(misopin)):
                        adcout |= 0x1

        GPIO.output(cspin, True)

        adcout >>= 1       # first bit is 'null' so drop it
        return adcout

# handle the button event
def buttonEventHandler (pin):
    print "handling button event"
    os.system("shutdown -h now")
    global runner
    runner = False

def windEventHandler (pin):
    print "handling wind speed event"
    global windspeed_count
    windspeed_count += 1

def rainEventHandler (pin):
    print "handling rain event"
    global rain_count
    rain_count += 1

# Main Loop
if __name__ == '__main__':
  # Logger open CSV
  fp = open('test.csv', 'a')
  csv = csv.writer(fp, delimiter=',')

  bmp180Sensor = BMP085.BMP085(busnum=BMP_i2c_busnum)

  # set up the SPI interface pins
  GPIO.setup(SPIMOSI, GPIO.OUT)
  GPIO.setup(SPIMISO, GPIO.IN)
  GPIO.setup(SPICLK, GPIO.OUT)
  GPIO.setup(SPICS, GPIO.OUT)

  GPIO.setup(mybutton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(mywindspeed, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(myraingauge, GPIO.IN, pull_up_down=GPIO.PUD_UP)

  # tell the GPIO library to look out for an
  # event on pin x and deal with it by calling
  # the buttonEventHandler function
  GPIO.add_event_detect(mybutton,GPIO.FALLING)
  GPIO.add_event_callback(mybutton,buttonEventHandler)

  GPIO.add_event_detect(mywindspeed,GPIO.FALLING)
  GPIO.add_event_callback(mywindspeed,windEventHandler)

  GPIO.add_event_detect(myraingauge,GPIO.FALLING)
  GPIO.add_event_callback(myraingauge,rainEventHandler)

  try:
    while (runner == True):
      # It may take a second or two to get good data

      # Pull BMP180 Temp, Humidity, Pressure
      bmpaltitude = bmp180Sensor.read_altitude()
      bmptempC = bmp180Sensor.read_temperature()
      bmptempF = bmptempC * 9/5.0 + 32
      bmppressure = bmp180Sensor.read_pressure()/100.0 #hPa
      bmpsealevelpressure = bmp180Sensor.read_sealevel_pressure()/100.0

      # Pull DHT Humidity & Temp
      dhthumidity, dhttempC = readDHTHumidityTemp()
      dewpoint = dewPoint(dhthumidity, dhttempC)
      dhttempF = dhttempC * 9/5.0 + 32
      heatindex = heatIndex(dhthumidity, dhttempF)

      # GEt Light:
      #light = readadc(light_adc, SPICLK, SPIMOSI, SPIMISO, SPICS)

      #Rain:
      #Right now this is just recording "hits", so this would be x events since last call.
      rain = rain_count
      rain_count = 0;

      #Wind Dir:
      # These need to be mapped to a direction. Right now record raw, we'll convert later or in the graphs.
      winddir = readadc(winddir_adc, SPICLK, SPIMOSI, SPIMISO, SPICS)
      winddir = direction(winddir)

      #Wind Speed
      #interupt: 1 = 180deg, 2 int = 1 full rotation.
      #Like Rain, this is just recording "hits",
      windspeed = windspeed_count
      windspeed_count = 0;

      #Record to CSV
      #todo provide current time as first element in data
      timenow = 0
      data = [ timenow, bmpaltitude, bmptempF, bmppressure, bmpsealevelpressure, dhthumidity, dewpoint, heatindex, winddir, windspeed, rain ]
      print "Data: ",
      print (data)
      csv.writerow(data)

      #Sleep
      time.sleep(10) #set to whatever

  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "\nKilling Thread..."
    runner = False
  print "Almost done."
  fp.close()
  GPIO.cleanup()
  print "Done.\nExiting."
  exit();
