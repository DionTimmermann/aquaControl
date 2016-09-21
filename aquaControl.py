#!env/bin/python
from flask import Flask, jsonify, request, abort, make_response, send_from_directory

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

from w1thermsensor import W1ThermSensor
sensor = W1ThermSensor()

import smbus

from influxdb import InfluxDBClient
influxDB = InfluxDBClient(database='aquarien')

import time
import numpy
import collections

from math import floor
import threading

from datetime import datetime

import re

from operator import itemgetter

app = Flask(__name__)

state = {
    'light': {
        'manual': {
            'on': False,
            'brightness': 0.4
        },
        'nodes': [
            {
                'id': 0,
                'time': '06:00',
                'brightness': 0
            },
            {
                'id': 1,
                'time': '07:00',
                'brightness': 1
            },
            {
                'id': 2,
                'time': '18:00',
                'brightness': 1
            },
            {
                'id': 3,
                'time': '23:00',
                'brightness': 0
            }
        ],
        'i2cAddress': 0x1a
    },
    'pH': {
        'cal': {
            'kH': 4.0,
            'mV1': 1064.0,
            'pH1': 4.00,
            'mV2': 889.0,
            'pH2': 7.03
        },
        'state': 'running',
        'mean': 6.75,
        'pm': 0.1,
        'gpioPort': 27,
    },
    'pumps': {
        'cal': [
            {
                'id': 1,
                'gpioPort': 20,
                'name': 'NPK',
                'continious': 100,
                'impulse': 90,
                'remaining': 500
            },
            {
                'id': 2,
                'gpioPort': 20,
                'name': 'Fe',
                'continious': 100,
                'impulse': 90,
                'remaining': 500
            }
        ],
        'nodes': [
            {
                'id': 0,
                'pump': 1,
                'time': '06:00',
                'amount': 10
            },
            {
                'id': 1,
                'pump': 2,
                'time': '08:00',
                'amount': 20
            }
        ]
    }
}


# The maxint value in
maxBrightness = 2**14 - 1

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
## Pumps
################################

def setupPumps():
    for pump in state['pumps']['cal']:
        GPIO.setup(pump['gpioPort'], GPIO.OUT)
        GPIO.output(pump['gpioPort'], 0)

def runPumpFor(pumpId, seconds):
    pump = next(pump for pump in state['pumps']['cal'] if pump['id'] == pumpId)


    # TODO set timer

    thread.start_new_thread(runPumpForRunner, [pumpId, seconds])


def runPumpForRunner(pumpId, seconds):

    #wait for lock to open

    GPIO.output(pump['gpioPort'], 1)
    # sleep for  seconds
    GPIO.output(pump['gpioPort'], 0)

    #return lock

@app.route('/api/pumps/cal', methods=['GET'])
def getApiPumpsCal():
    return jsonify({'cal': state['pumps']['cal']})


@app.route('/api/pumps/cal/<int:pump_id>', methods=['GET'])
def getApiPumpCal(pump_id):
    pump = [pump for pump in state['pumps']['cal'] if pump['id'] == pump_id]
    if len(pump) != 1:
        abort(404)
    return jsonify(pump[0])

@app.route('/api/pumps/cal/<int:pump_id>', methods=['PUT'])
def editApiPumpCal(pump_id):
    if not request.json:
        abort(400)
    if not 'continious' in request.json or not is_number(request.json['continious']):
        abort(400, 'Dauerwert nicht (richtig) angebeben.')
    if request.json['continious'] < 0.0:
        abort(400, 'Dauerwert kann nicht negativ sein.')
    if not 'impulse' in request.json or not is_number(request.json['impulse']):
        abort(400, 'Impulswert nicht (richtig) angebeben.')
    if request.json['impulse'] < 0.0:
        abort(400, 'Impulswert kann nicht negativ sein.')
    if not 'remaining' in request.json or not is_number(request.json['remaining']):
        abort(400, 'Menge nicht (richtig) angebeben.')
    if request.json['remaining'] < 0.0:
        abort(400, 'Verbleibende Menge kann nicht negativ sein.')
    if not 'name' in request.json or not isinstance(request.json['name'], basestring):
        abort(400, 'Name nicht angebeben.')

    pump = [pump for pump in state['pumps']['cal'] if pump['id'] == pump_id]
    if len(pump) != 1:
        abort(400, 'Pumpe wurde nicht gefunden.')

    pump[0]['name'] = request.json['name']
    pump[0]['continious'] = float(request.json['continious'])
    pump[0]['impulse'] = float(request.json['impulse'])
    pump[0]['remaining'] = float(request.json['remaining'])

    return getApiPumpsCal(), 200

@app.route('/api/pumps/nodes', methods=['GET'])
def getApiPumpNodes():
    nodes = sorted(state['pumps']['nodes'], key=itemgetter('time'))
    for node in nodes:
        node['pumpName'] = [n['name'] for n in state['pumps']['cal'] if n['id'] == node['pump']][0]

    return jsonify({'nodes': nodes})

@app.route('/api/pumps/nodes/<int:node_id>', methods=['GET'])
def getApiPumpNode(node_id):
    node = [node for node in state['pumps']['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(404)
    return jsonify({'node': node[0]})

@app.route('/api/pumps/nodes', methods=['POST'])
def createApiPumpNode():
    if not request.json:
        abort(400)
    if not 'pump' in request.json or not is_number(request.json['pump']):
        abort(400, 'Pumpe nicht (richtig) angebeben.')
    # TODO Test if pump is valid number
    if not 'amount' in request.json or not is_number(request.json['amount']):
        abort(400, 'Menge nicht (richtig) angebeben.')
    if request.json['amount'] < 0.0:
        abort(400, 'Menge kann nicht negativ sein.')
    if not 'time' in request.json or not isinstance(request.json['time'], basestring):
        abort(400, 'Zeit nicht (richtig) angebeben.')
    if not reTime.match(request.json['time']):
        abort(400, 'Zeit muss im Format hh:mm:ss oder hh:mm sein.')

    maxId = -1
    for node in state['pumps']['nodes']:
        if node['id'] > maxId:
            maxId = node['id'];

    node = {
        'id': maxId + 1,
        'pump': request.json['pump'],
        'time': request.json['time'],
        'amount': request.json['amount']
    }
    state['pumps']['nodes'].append(node)

    return getApiPumpNodes(), 201

@app.route('/api/pumps/nodes/<int:node_id>', methods=['PUT'])
def editApiPumpNode(node_id):
    if not request.json:
        abort(400)
    if not 'pump' in request.json or not is_number(request.json['pump']):
        abort(400, 'Pumpe nicht (richtig) angebeben.')
    # TODO Test if pump is valid number
    if not 'amount' in request.json or not is_number(request.json['amount']):
        abort(400, 'Menge nicht (richtig) angebeben.')
    if request.json['amount'] < 0.0:
        abort(400, 'Menge kann nicht negativ sein.')
    if not 'time' in request.json or not isinstance(request.json['time'], basestring):
        abort(400, 'Zeit nicht (richtig) angebeben.')
    if not reTime.match(request.json['time']):
        abort(400, 'Zeit muss im Format hh:mm:ss oder hh:mm sein.')

    # Test of pump number is valid
    p = [pump for pump in state['pumps']['cal'] if pump['id'] == request.json['pump']]
    if len(p) != 1:
        abort(400, 'Pumpe nicht (richtig) angebeben.')

    node = [node for node in state['pumps']['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(400)

    node[0]['pump'] = request.json['pump']
    node[0]['time'] = request.json['time']
    node[0]['amount'] = request.json['amount']

    return getApiPumpNodes(), 200

@app.route('/api/pumps/nodes/<int:node_id>', methods=['DELETE'])
def deleteApiPumpNode(node_id):
    node = [node for node in state['pumps']['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(404)
    state['pumps']['nodes'].remove(node[0])

    return getApiPumpNodes(), 200

setupPumps()

################################
## Light
################################

def i2cConfirmedWordWrite(address, register, value):
	while True:
		try:
			i2c.write_word_data(address, register, value)
			valueRB = i2c.read_word_data(address, register)
			if value == valueRB:
				return
		except:
			continue

def i2cWordWrite(address, register, value):
    while True:
        try:
            i2c.write_word_data(address, register, value)
        except:
            print 'alert!'
            continue

        return

def setPwm(value):
    i2cWordWrite(state['light']['i2cAddress'], 0x00, value)
    i2cWordWrite(state['light']['i2cAddress'], 0x02, value)

def getPwm():
    while True:
		try:
			return i2c.read_word_data(state['light']['i2cAddress'], 0x00)
		except:
			continue

@app.route('/api/light/manual/on', methods=['GET'])
def getApiLightManualOn():
    return jsonify({'on': state['light']['manual']['on']})

@app.route('/api/light/manual/on', methods=['PUT'])
def putApiLightManualOn():
    if not request.json:
        abort(400)
    if not 'on' in request.json or not isinstance(request.json['on'], bool):
        abort(400)
    state['light']['manual']['on'] = request.json['on']
    lightUpdate()
    return getApiLightManualOn(), 201

@app.route('/api/light/manual/brightness', methods=['GET'])
def getApiLightManualBrightness():
    return jsonify({'brightness': state['light']['manual']['brightness']})

@app.route('/api/light/manual/brightness', methods=['PUT'])
def putApiLightManualBrightness():
    if not request.json:
        abort(400)
    if not 'brightness' in request.json or not is_number(request.json['brightness']):
        abort(400)
    if request.json['brightness'] < 0.0 or request.json['brightness'] > 1.0:
        abort(400)
    state['light']['manual']['brightness'] = request.json['brightness']

    lightUpdate()
    return getApiLightManualBrightness(), 201

@app.route('/api/light/nodes', methods=['GET'])
def getApiLightNodes():
    return jsonify({'nodes': sorted(state['light']['nodes'], key=itemgetter('time'))})

@app.route('/api/light/nodes/<int:node_id>', methods=['GET'])
def getApiLightNode(node_id):
    node = [node for node in state['light']['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(404)
    return jsonify({'node': node[0]})

@app.route('/api/light/nodes', methods=['POST'])
def createApiLightNode():
    if not request.json:
        abort(400)
    if not 'brightness' in request.json or not is_number(request.json['brightness']):
        abort(400)
    if request.json['brightness'] < 0.0 or request.json['brightness'] > 1.0:
        abort(400)
    if not 'time' in request.json or not isinstance(request.json['time'], basestring):
        abort(400)
    if not reTime.match(request.json['time']):
        abort(400)

    maxId = -1
    for node in state['light']['nodes']:
        if node['id'] > maxId:
            maxId = node['id'];

    node = {
        'id': maxId + 1,
        'time': request.json['time'],
        'brightness': request.json['brightness']
    }
    state['light']['nodes'].append(node)

    lightUpdate()
    return getApiLightNodes(), 201

@app.route('/api/light/nodes/<int:node_id>', methods=['PUT'])
def editApiLightNode(node_id):
    if not request.json:
        abort(400)
    if not 'brightness' in request.json or not is_number(request.json['brightness']):
        abort(400)
    if request.json['brightness'] < 0.0 or request.json['brightness'] > 1.0:
        abort(400)
    if not 'time' in request.json or not isinstance(request.json['time'], basestring):
        abort(400)
    if not reTime.match(request.json['time']):
        abort(400)

    node = [node for node in state['light']['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(400)

    node[0]['time'] = request.json['time']
    node[0]['brightness'] = request.json['brightness']

    lightUpdate()
    return getApiLightNodes(), 200

@app.route('/api/light/nodes/<int:node_id>', methods=['DELETE'])
def deleteApiLightNode(node_id):
    node = [node for node in state['light']['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(404)
    state['light']['nodes'].remove(node[0])

    lightUpdate()
    return getApiLightNodes(), 200



################################
## pH
################################

def pHfrommV(mV):
    return (state['pH']['cal']['pH2'] - state['pH']['cal']['pH1']) /        \
           (state['pH']['cal']['mV2'] - state['pH']['cal']['mV1']) *        \
           (mV - state['pH']['cal']['mV1']) + state['pH']['cal']['pH1']

def CO2frompH(pH):
    return float(state['pH']['cal']['kH']) / 2.8 * 10**(7.9 - pH)

@app.route('/api/ph/cal', methods=['GET'])
def getApiPhCal():
    return jsonify(state['pH']['cal'])


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

    state['pH']['cal'] = request.json

    return jsonify(state['pH']['cal']), 200

def getADCab():
    while True:
        try:
            low = i2c.read_byte_data(i2cAddressPh, 0x00)
            high = i2c.read_byte_data(i2cAddressPh, 0x01)
            return float(low+high*256)
        except:
            continue

def getADCint():
    while True:
        try:
            low = i2c.read_byte_data(i2cAddressPh, 0x02)
            high = i2c.read_byte_data(i2cAddressPh, 0x03)
            return float(low+high*256)
        except:
            continue

def getpHVoltage(adcVal):
    return adcVal / (2**16-1) * 1668 # in mV

@app.route('/api/ph/current', methods=['GET'])
def getApiPhCurrent():
    mV = getpHVoltage(getADCab())
    pH = pHfrommV(mV)
    co2 = CO2frompH(pH)

    return jsonify(
        {
            'voltageMean': mV,
            'voltageLast': getpHVoltage(getADCint()),
            'ph': pH,
            'co2': co2
        })


################################
## Gerneral Stuff
################################


def secondsSinceMidnight():
    now = datetime.now()
    return (now - now.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()

lightThreadCondition = threading.Condition()
lightState = {
    'manual': False,
    'manualVal': 0.0,
    'points': [],
    'updated': False}

def time2light(t):
    return (t ** 3)

def light2time(l):
    return l ** (1/3)

def timeText2seconds(text):
    return (int(text[:2])*60+int(text[-2:]))*60


def lightUpdate():

    global lightThreadCondition, lightState

    lightThreadCondition.acquire()

    if state['light']['manual']['on']:
        lightState['manual'] = True
        lightState['manualVal'] = int(state['light']['manual']['brightness']*maxBrightness)


    else:

        lightState['manual'] = False

        nodes = []
        # build list with brightness nodes with correct units
        for node in state['light']['nodes']:
            nodes.append(
                {
                    'time': timeText2seconds(node['time']),
                    'brightness': int(node['brightness']*maxBrightness)
                }
            )

        # sort, prepend last node of last day and append first node of next day
        nodes = sorted(nodes, key=itemgetter('time'))
        nodes.append(
            {
                'time': nodes[0]['time'] + 24*60*60,
                'brightness': nodes[0]['brightness']
            }
        )

        lightState['points'] = []
        for ii in range(0, len(nodes)-1):
            cB = nodes[ii]['brightness']
            cT = nodes[ii]['time']

            while (cB != nodes[ii+1]['brightness']):

                if nodes[ii+1]['brightness'] > cB:
                    cB = cB+1
                    rB = float(cB - nodes[ii]['brightness']) /                         \
                        float(nodes[ii+1]['brightness'] - nodes[ii]['brightness'])

                else:
                    cB = cB-1
                    rB = float(cB - nodes[ii]['brightness']) /                         \
                        float(nodes[ii+1]['brightness'] - nodes[ii]['brightness'])
                    rB = 1 - rB

                rT = rB ** 3    ## TODO Factor should not be magic number
                cT = nodes[ii]['time'] + rT*(nodes[ii+1]['time']-nodes[ii]['time'])

                cT = ((cT - 24*60*60) if cT > 24*60*60 else cT)

                lightState['points'].append(
                    {
                        'time': cT,
                        'brightness': cB
                    }
                )

        lightState['points'] = sorted(lightState['points'], key=itemgetter('time'))
        lightState['points'].insert(0,
            {
                'time': lightState['points'][-1]['time'] - 24*60*60,
                'brightness': lightState['points'][-1]['brightness']
            }
        )
        lightState['points'].append(
            {
                'time': lightState['points'][1]['time'] + 24*60*60,
                'brightness': lightState['points'][1]['brightness']
            }
        )

    lightState['updated'] = True
    lightThreadCondition.notify()
    lightThreadCondition.release()


def lightThreadRunner():

    global lightThreadCondition, lightState

    lightThreadCondition.acquire()
    while True:

        lightState['updated'] = False

        if lightState['manual']:
            setPwm(lightState['manualVal'])
            lightThreadCondition.wait()
            continue

        now = secondsSinceMidnight()
        nextPointIdx = [pIdx for pIdx, point in enumerate(lightState['points']) if point['time'] > now][0]
        brightness = lightState['points'][nextPointIdx-1]['brightness']


        while True:

            setPwm(brightness)

            now = secondsSinceMidnight()
            nextPoint = next(point for point in lightState['points'] if point['time'] > now)
            dt = nextPoint['time'] - now
            brightness = nextPoint['brightness']

            #calculate dt - Time to wait...
            lightThreadCondition.wait(dt)
            # check why wait ended
            if lightState['updated']:
                break

def calculatePumpTime(amount, pump):
    a = (pump['amountContinious'] - (pump['amountImpulse'] / pump['NImpulse'])) / \
        (pump['timeContinious'] - pump['timeImpulse'])
    return (amount - pump['amountContinious'] + a*pump['timeContinious']) / a

def pumpsUpdate():

    global pumpsThreadCondition, pumpsUpdated, pumpNodes


    pumpsThreadCondition.acquire()

    pumpNodes = []
    # build list with brightness nodes with correct units
    for node in state['pumps']['nodes']:

        pump = next(pump for pump in state['pumps']['cal'] if pump['id'] == node['pump'])

        time = timeText2seconds(node['time'])
        if len([n for n in pumpNodes if n['time'] == time]) == 0:
            pumpNodes.append(
                {
                    'time': time,
                    'nodes': []
                }
            )

        n = [n for n in pumpNodes if n['time'] == time][0]
        n.append(
            {
                'time': time,
                'pumpName': pump[name],
                'gpioPort': pump['gpioPort'],
                'duration': calculatePumpTime(node['amount'], pump)
            }
        )

    # sort, prepend last node of last day and append first node of next day
    pumpNodes = sorted(pumpNodes, key=itemgetter('time'))
    nodes.append(
        {
            'time': nodes[0]['time'] + 24*60*60,
            'nodes': nodes[0]['nodes']
        }
    )

    pumpsUpdated = True
    pumpsThreadCondition.notify()
    pumpsThreadCondition.release()


sensorsThreadCondition = threading.Condition()
sensorsState = {
    'temperature': None,
    'pH': None,
    'CO2': None,
    'CO2valve': False,
    'light': None
}
GPIO.setup(state['pH']['gpioPort'], GPIO.OUT)

def sensorsThreadRunner():

    nextSensorUpdate = time.time()

    while True:
        try:
            waterTemp = sensor.get_temperature()
            pH = pHfrommV(getpHVoltage(getADCab()))
            CO2 = CO2frompH(pH)
            light = float(getPwm())/maxBrightness*1980

            sensorsThreadCondition.acquire()
            sensorsState['temperature'] = waterTemp
            sensorsState['pH'] = pH
            sensorsState['CO2'] = CO2
            sensorsState['light'] = light

            # CO2 reduziert den pH-Wert
            if (sensorsState['pH'] > state['pH']['mean'] + state['pH']['pm']):
                sensorsState['CO2valve'] = True
            elif (sensorsState['pH'] < state['pH']['mean'] - state['pH']['pm']):
                sensorsState['CO2valve'] = False

            json_body = [
                {
                    "measurement": "water.temperature",
                    "tags": {
                        "aquarium": "Badezimmer"
                    },
                    "fields": {
                        "value": sensorsState['temperature']
                    }
                },
                {
                    "measurement": "water.pH",
                    "tags": {
                        "aquarium": "Badezimmer"
                    },
                    "fields": {
                        "value": sensorsState['pH']
                    }
                },
                {
                    "measurement": "water.co2",
                    "tags": {
                        "aquarium": "Badezimmer"
                    },
                    "fields": {
                        "value": sensorsState['CO2']
                    }
                },
                {
                    "measurement": "addition.light",
                    "tags": {
                        "aquarium": "Badezimmer",
                        "unit": "lumen"
                    },
                    "fields": {
                        "value": sensorsState['light']
                    }
                },
                {
                    "measurement": "addition.co2",
                    "tags": {
                        "aquarium": "Badezimmer"
                    },
                    "fields": {
                        "value": sensorsState['CO2valve']
                    }
                }
            ]

            sensorsThreadCondition.release()

            influxDB.write_points(json_body)

            # CO2 valve is inverted
            GPIO.output(state['pH']['gpioPort'], not sensorsState['CO2valve'])

        except:
            print "Fehler beim in Messschleife"

        # TODO not good when wasserwechsel
        nextSensorUpdate += 60
        time.sleep(nextSensorUpdate - time.time())


@app.route('/api/sensors')
def getApiSensors():
    sensorsThreadCondition.acquire()
    s = sensorsState
    sensorsThreadCondition.release()
    s['pHgoal'] = state['pH']['mean']
    s['CO2goal'] = CO2frompH(state['pH']['mean'])
    return jsonify(s)

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/static/')
def staticIndex():
    return send_from_directory('static', 'index.html')

lightUpdate()

lightThread = threading.Thread(target=lightThreadRunner)
lightThread.daemon = True
lightThread.start()

sensorsThread = threading.Thread(target=sensorsThreadRunner)
sensorsThread.daemon = True
sensorsThread.start()

if __name__ == '__main__':
    app.run(debug=False, host='0.0.0.0')
