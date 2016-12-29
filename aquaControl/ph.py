from aquaControl import app, i2c, modules, logVal, is_number

import threading, time

from operator import itemgetter
from flask import jsonify, request, abort

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

from w1thermsensor import W1ThermSensor
sensor = W1ThermSensor()

conf = {
    'cal': {
        'kH': 4.0,
        'mV1': 1040.0,
        'pH1': 4.00,
        'mV2': 869.0,
        'pH2': 7.03
    },
    'state': 'running',
    'mean': 6.75,
    'pm': 0.1,
    'gpioPort': 9,
}

################################
## pH
################################

def pHfrommV(mV):
    return (conf['cal']['pH2'] - conf['cal']['pH1']) /        \
           (conf['cal']['mV2'] - conf['cal']['mV1']) *        \
           (mV - conf['cal']['mV1']) + conf['cal']['pH1']

def CO2frompH(pH):
    return float(conf['cal']['kH']) / 2.8 * 10**(7.9 - pH)

@app.route('/api/ph/cal', methods=['GET'])
def getApiPhCal():
    return jsonify(conf['cal'])


@app.route('/api/ph/cal', methods=['PUT'])
def editApiPhCal():
    if not request.json:
        abort(400)
    if not 'kH' in request.json or not is_number(request.json['kH']):
        abort(400, 'kH nicht (richtig) angegeben.')
    if request.json['kH'] < 0.0:
        abort(400, 'kH kleiner Null.')
    if not 'mV1' in request.json or not is_number(request.json['mV1']):
        abort(400, 'mV-Wert von Puffer 1  nicht (richtig) angegeben.')
    if request.json['mV1'] < 0.0 or request.json['mV1'] > 1668:
        abort(400, 'mV-Wert von Puffer 1 hat nicht erlaubten Wert.')
    if not 'mV2' in request.json or not is_number(request.json['mV2']):
        abort(400, 'mV-Wert von Puffer 2 nicht (richtig) angegeben.')
    if request.json['mV2'] < 0.0 or request.json['mV2'] > 1668:
        abort(400, 'mV-Wert von Puffer 2 hat nicht erlaubten Wert.')
    if not 'pH1' in request.json or not is_number(request.json['pH1']):
        abort(400, 'pH-Wert von Puffer 1 nicht (richtig) angegeben.')
    if request.json['pH1'] < 0.0 or request.json['pH1'] > 14.0:
        abort(400, 'pH-Wert von Puffer 1 hat nicht erlaubten Wert.')
    if not 'pH2' in request.json or not is_number(request.json['pH2']):
        abort(400, 'pH-Wert von Puffer 2 nicht (richtig) angegeben.')
    if request.json['pH2'] < 0.0 or request.json['pH2'] > 14.0:
        abort(400, 'pH-Wert von Puffer 2 hat nicht erlaubten Wert.')

    conf['cal'] = request.json

    return jsonify(conf['cal']), 200

def getADCab():
    while True:
        try:
            low = i2c.read_byte_data(modules['ph']['i2cAddress'], 0x00)
            high = i2c.read_byte_data(modules['ph']['i2cAddress'], 0x01)
            return float(low+high*256)
        except:
            continue

def getADCint():
    while True:
        try:
            low = i2c.read_byte_data(modules['ph']['i2cAddress'], 0x02)
            high = i2c.read_byte_data(modules['ph']['i2cAddress'], 0x03)
            return float(low+high*256)
        except:
            continue

def getpHVoltage(adcVal):
    return adcVal / (2**16-1) * 1668 # in mV




sensorsLock = threading.Lock()
sensorsState = {
    'temperature': None,
    'pH': None,
    'CO2': None,
    'CO2valve': False,
    'light': None
}
GPIO.setup(conf['gpioPort'], GPIO.OUT)

phRawValues = []
phLastValue = None
def measurePhThreadRunner():
    global phRawValues, sensorsLock, phLastValue
    loggingIntervallSeconds = 1

    while True:
        nextWakeup = time.time() + 1

        with sensorsLock:
            phLastValue = getADCint()
            phRawValues.append(phLastValue)

        dt = nextWakeup - time.time()
        time.sleep(dt if dt > 0 else 0)


measurePhThread = threading.Thread(target=measurePhThreadRunner)
measurePhThread.daemon = True
measurePhThread.start()


@app.route('/api/ph/current', methods=['GET'])
def getApiPhCurrent():
    with sensorsLock:
        mV = getpHVoltage(phLastValue)
        pH = pHfrommV(mV)
        co2 = CO2frompH(pH)

    return jsonify(
        {
            'mV': mV,
            'ph': pH,
            'co2': co2
        })


def phMedian(lst):
    lst = sorted(lst)
    if len(lst) < 1:
        return 7.0
    if len(lst) %2 == 1:
        return float(lst[((len(lst)+1)/2)-1])
    else:
        return float(sum(lst[(len(lst)/2)-1:(len(lst)/2)+1]))/2.0


def sensorsThreadRunner():
    global sensorsLock, phRawValues, sensorsState
    loggingIntervallSeconds = 60


    time.sleep(loggingIntervallSeconds)

    while True:

        with sensorsLock:
            nextWakeup = time.time() + loggingIntervallSeconds

            #print phRawValues

            sensorsState['pH'] = pHfrommV(getpHVoltage(phMedian(phRawValues)))
            phRawValues = []

            sensorsState['CO2'] = CO2frompH(sensorsState['pH'])

            sensorsState['temperature'] = sensor.get_temperature()

            if True:
                # CO2 reduziert den pH-Wert
                if (sensorsState['pH'] > conf['mean'] + conf['pm']):
                    sensorsState['CO2valve'] = True
                elif (sensorsState['pH'] < conf['mean'] - conf['pm']):
                    sensorsState['CO2valve'] = False

                # CO2 valve is inverted
                GPIO.output(conf['gpioPort'], not sensorsState['CO2valve'])

            logVal('water.temperature', sensorsState['temperature'])
            logVal('co2.dissolved', sensorsState['CO2'])
            logVal('co2.valve', float(sensorsState['CO2valve']))
            logVal('ph', sensorsState['pH'])


        # TODO not good when wasserwechsel
        time.sleep(nextWakeup - time.time())



@app.route('/api/sensors')
def getApiSensors():
    sensorsLock.acquire()
    s = sensorsState
    sensorsLock.release()
    s['pHgoal'] = conf['mean']
    s['CO2goal'] = CO2frompH(conf['mean'])
    return jsonify(s)


sensorsThread = threading.Thread(target=sensorsThreadRunner)
sensorsThread.daemon = True
sensorsThread.start()
