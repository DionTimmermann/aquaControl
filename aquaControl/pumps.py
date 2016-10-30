from aquaControl import app, timeText2seconds, secondsSinceMidnight, is_number, reTime, logVal

import threading, time

from operator import itemgetter
from flask import jsonify, request, abort

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

conf = {
    'cal': [
        {
            'id': 1,
            'gpioPort': 14,
            'name': 'NPK',
            'continuous': 100,
            'impulse': 104,
            'remaining': 500
        },
        {
            'id': 2,
            'gpioPort': 15,
            'name': 'Fe',
            'continuous': 100,
            'impulse': 104,
            'remaining': 500
        }
    ],
    'nodes': [
        {
            'id': 0,
            'pump': 1,
            'time': '08:00',
            'amount': 2
        },
        {
            'id': 1,
            'pump': 2,
            'time': '08:00',
            'amount': 2
        }
    ],
    'NImpulse' : 100,
    'timeImpulse': 1,
    'timeContinuous': 100,
}


################################
## Pumps
################################

pumpLocks = {}
statesLock = threading.Lock()
states = {}

def setupPumps():
    global pumpLocks, statesLock

    statesLock.acquire()

    for pump in conf['cal']:
        GPIO.setup(pump['gpioPort'], GPIO.OUT)
        GPIO.output(pump['gpioPort'], 0)
        pumpLocks[pump['id']] = threading.Lock()
        states[pump['id']] = 'off'

    statesLock.release()

def runPumpThreadRunner(pump, amount):
    global pumpLocks, statesLock

    seconds = calculatePumpTime(amount, pump)

    #wait for lock, this ensures the pump is not already running
    with pumpLocks[pump['id']]:

        logVal('pumps', amount, {'fluid': pump['name']})
        confPump = [confPump for confPump in conf['cal'] if confPump['id'] == pump['id']]
        confPump[0]['remaining'] = confPump[0]['remaining'] - amount

        with statesLock:
            states[pump['id']] = 'on'

        #start, sleep, stop
        GPIO.output(pump['gpioPort'], 1)
        time.sleep(seconds)
        GPIO.output(pump['gpioPort'], 0)

        with statesLock:
            states[pump['id']] = 'off'

def runPump(pumpId, amount):
    pump = next(pump for pump in conf['cal'] if pump['id'] == pumpId)

    newThread = threading.Thread(target=runPumpThreadRunner, args=[pump, amount])
    newThread.start()

def calPumpContinuousThreadRunner(pump):
    global pumpLocks, statesLock

    #wait for lock, this ensures the pump is not already running
    with pumpLocks[pump['id']]:

        with statesLock:
            states[pump['id']] = 'calibrateContinuous'

        #start, sleep, stop
        GPIO.output(pump['gpioPort'], 1)
        time.sleep(conf['timeContinuous'])
        GPIO.output(pump['gpioPort'], 0)

        with statesLock:
            states[pump['id']] = 'off'


def calPumpContinuous(pumpId):
    pump = next(pump for pump in conf['cal'] if pump['id'] == pumpId)

    newThread = threading.Thread(target=calPumpContinuousThreadRunner, args=[pump])
    newThread.start()

def calPumpImpulseThreadRunner(pump):
    # we also need the lock while the pump is not running. This is why we can not use runPump
    global pumpLocks, statesLock

    #wait for lock, this ensures the pump is not already running
    with pumpLocks[pump['id']]:

        with statesLock:
            states[pump['id']] = 'calibrateImpulse'

        for _ in range(conf['NImpulse']):
            #start, sleep, stop
            GPIO.output(pump['gpioPort'], 1)
            time.sleep(conf['timeImpulse'])
            GPIO.output(pump['gpioPort'], 0)
            time.sleep(conf['timeImpulse'])

        with statesLock:
            states[pump['id']] = 'off'

def calPumpImpulse(pumpId):
    pump = next(pump for pump in conf['cal'] if pump['id'] == pumpId)

    newThread = threading.Thread(target=calPumpImpulseThreadRunner, args=[pump])
    newThread.start()


@app.route('/api/pumps/state', methods=['GET'])
def getApiPumpsState():
    global statesLock
    with statesLock:
        return jsonify({'state': states})

@app.route('/api/pumps/state/<int:pump_id>', methods=['GET'])
def getApiPumpState(pump_id):
    global statesLock, states
    with statesLock:
        if not pump_id in states:
            abort(404)
        return jsonify({'state': states[pump_id]})


@app.route('/api/pumps/state/<int:pump_id>', methods=['PUT'])
def setApiPumpState(pump_id):
    global statesLock, states
    with statesLock:
        if not pump_id in states:
            abort(404)

        if not request.json:
            abort(400)
        if not 'state' in request.json or not isinstance(request.json['state'], basestring):
            abort(400, 'Zustand nicht angebeben.')

        if request.json['state'] == 'calibrateContinuous':
            calPumpContinuous(pump_id)
        elif request.json['state'] == 'calibrateImpulse':
            calPumpImpulse(pump_id)
        elif request.json['state'] == 'run':
            if not 'amount' in request.json or not is_number(request.json['amount']):
                abort(400, 'Menge nicht angebeben.')
            runPump(pump_id, float(request.json['amount']))
        else:
            abort(400, 'Nicht definierter Zustand.')

        return jsonify({'states': states})


@app.route('/api/pumps/cal', methods=['GET'])
def getApiPumpsCal():
    return jsonify({'cal': conf['cal']})


@app.route('/api/pumps/cal/<int:pump_id>', methods=['GET'])
def getApiPumpCal(pump_id):
    pump = [pump for pump in conf['cal'] if pump['id'] == pump_id]
    if len(pump) != 1:
        abort(404)
    return jsonify(pump[0])

@app.route('/api/pumps/cal/<int:pump_id>', methods=['PUT'])
def editApiPumpCal(pump_id):
    if not request.json:
        abort(400)
    if not 'continuous' in request.json or not is_number(request.json['continuous']):
        abort(400, 'Dauerwert nicht (richtig) angebeben.')
    if request.json['continuous'] < 0.0:
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

    pump = [pump for pump in conf['cal'] if pump['id'] == pump_id]
    if len(pump) != 1:
        abort(400, 'Pumpe wurde nicht gefunden.')

    pump[0]['name'] = request.json['name']
    pump[0]['continuous'] = float(request.json['continuous'])
    pump[0]['impulse'] = float(request.json['impulse'])
    pump[0]['remaining'] = float(request.json['remaining'])

    return getApiPumpsCal(), 200

@app.route('/api/pumps/nodes', methods=['GET'])
def getApiPumpNodes():
    nodes = sorted(conf['nodes'], key=itemgetter('time'))
    for node in nodes:
        node['pumpName'] = [n['name'] for n in conf['cal'] if n['id'] == node['pump']][0]

    return jsonify({'nodes': nodes})

@app.route('/api/pumps/nodes/<int:node_id>', methods=['GET'])
def getApiPumpNode(node_id):
    node = [node for node in conf['nodes'] if node['id'] == node_id]
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
    for node in conf['nodes']:
        if node['id'] > maxId:
            maxId = node['id'];

    node = {
        'id': maxId + 1,
        'pump': request.json['pump'],
        'time': request.json['time'],
        'amount': request.json['amount']
    }
    conf['nodes'].append(node)

    pumpsUpdate()

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
    p = [pump for pump in conf['cal'] if pump['id'] == request.json['pump']]
    if len(p) != 1:
        abort(400, 'Pumpe nicht (richtig) angebeben.')

    node = [node for node in conf['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(400)

    node[0]['pump'] = request.json['pump']
    node[0]['time'] = request.json['time']
    node[0]['amount'] = request.json['amount']

    pumpsUpdate()

    return getApiPumpNodes(), 200

@app.route('/api/pumps/nodes/<int:node_id>', methods=['DELETE'])
def deleteApiPumpNode(node_id):
    node = [node for node in conf['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(404)
    conf['nodes'].remove(node[0])

    pumpsUpdate()

    return getApiPumpNodes(), 200

setupPumps()


def calculatePumpTime(amount, pump):
    a = (pump['continuous'] - (pump['impulse'] / conf['NImpulse'])) / \
        (conf['timeContinuous'] - conf['timeImpulse'])

    return (amount - pump['continuous'] + a*conf['timeContinuous']) / a

runPumpsThreadCondition = threading.Condition()

nodesUpdated = False
pumpNodes = []

def pumpsUpdate():

    global runPumpsThreadCondition, nodesUpdated, pumpNodes


    runPumpsThreadCondition.acquire()

    pumpNodes = []
    # build list with brightness nodes with correct units
    for node in conf['nodes']:

        pump = next(pump for pump in conf['cal'] if pump['id'] == node['pump'])

        time = timeText2seconds(node['time'])
        if len([n for n in pumpNodes if n['time'] == time]) == 0:
            pumpNodes.append(
                {
                    'time': time,
                    'nodes': []
                }
            )

        n = [n for n in pumpNodes if n['time'] == time][0]
        n['nodes'].append(
            {
                'pumpId': pump['id'],
                'amount': node['amount']
            }
        )

    # sort, append first node of next day
    pumpNodes = sorted(pumpNodes, key=itemgetter('time'))
    pumpNodes.append(
        {
            'time': pumpNodes[0]['time'] + 24*60*60,
            'nodes': pumpNodes[0]['nodes']
        }
    )

    nodesUpdated = True
    runPumpsThreadCondition.notify()
    runPumpsThreadCondition.release()


def runPumpsThreadRunner():

    global runPumpsThreadCondition, nodesUpdated

    runPumpsThreadCondition.acquire()


    while True:

        nodesUpdated = False
        lastNodeTime = secondsSinceMidnight()

        while True:

            nextNode = next(node for node in pumpNodes if node['time'] > lastNodeTime)

            #FIXME we need to reintroduce the current time here!
            #calculate dt - Time to wait...
            dt = nextNode['time'] - secondsSinceMidnight()
            if dt > 0:
                #print "waiting for ", dt
                runPumpsThreadCondition.wait(dt)
            # check why wait ended, if the nodes did not update, it is just time to run the node
            if nodesUpdated:
                break

            for node in nextNode['nodes']:
                runPump(node['pumpId'], node['amount'])

            lastNodeTime = nextNode['time'] if nextNode['time'] < 24*60*60 else nextNode['time'] - 24*60*60

pumpsUpdate()

runPumpsThread = threading.Thread(target=runPumpsThreadRunner)
runPumpsThread.daemon = True
runPumpsThread.start()
