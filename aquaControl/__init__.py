#!env/bin/python
from flask import Flask, jsonify, request, abort, make_response, send_from_directory

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


import logging
logging.basicConfig(filename='aquaControl.log', format='%(asctime)s %(levelname)s:%(message)s', level=logging.WARNING)

import smbus

from influxdb import InfluxDBClient

import time
import numpy
import collections

from math import floor
import threading

from datetime import datetime

import re

from operator import itemgetter

import smbus
i2c = smbus.SMBus(1)

app = Flask(__name__)


modules = {
    'light_single': {
        'i2cAddress': 0x1a,
        'maxBrightness': 2**14 - 1,
        'maxLumen': 1980
    },
    'ph': {
        'i2cAddress': 0x1e,
    }
}


influxDB = InfluxDBClient(database='aquarien')

def logVal(measurement, value, tags = {}):
    try:
        json = [{
            "measurement": measurement,
            "tags": {
                "aquarium": "Badezimmer"
            },
            "fields": {
                "value": value
            }
        }]

        json[0]['tags'].update(tags)
        
        influxDB.write_points(json)
    except Exception, e:
        logging.warning("Fehler in Messschleife: %s", str(e))


reTime = re.compile("^[0-9]{2}(:[0-9]{2}){1,2}$")


@app.errorhandler(404)
def not_found(error):
    return make_response(jsonify({'error': 'Not found', 'msg': error.description}), 404)

@app.errorhandler(400)
def bad_request(error):
    return make_response(jsonify({'error': 'Bad request', 'msg': error.description}), 400)

i2c = smbus.SMBus(1)
i2cAddressPh = 0x1e

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


################################
## Gerneral Stuff
################################

def secondsSinceMidnight():
    now = datetime.now()
    return (now - now.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()

def time2light(t):
    return (t ** 3)

def light2time(l):
    return l ** (1/3)

def timeText2seconds(text):
    return (int(text[:2])*60+int(text[-2:]))*60



@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/static/')
def staticIndex():
    return send_from_directory('static', 'index.html')


import aquaControl.light_single
import aquaControl.pumps
import aquaControl.ph
