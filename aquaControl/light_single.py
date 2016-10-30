from aquaControl import app, timeText2seconds, secondsSinceMidnight, is_number, i2c, modules, logVal, reTime

import threading, time, json

from operator import itemgetter
from flask import jsonify, request, abort

confLock = threading.Lock()
conf = {
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
    ]
}


def loadConf():
    with open('light_single.json', 'r') as infile:
        conf = json.load(infile)

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


currentPwmVal = 0 #Read and replace operations are atomic
def setPwm(value):
	global currentPwmVal
	currentPwmVal = value
	i2cWordWrite(modules['light_single']['i2cAddress'], 0x00, value)
	i2cWordWrite(modules['light_single']['i2cAddress'], 0x02, value)

def getPwm():
    while True:
		try:
			return i2c.read_word_data(modules['light_single']['i2cAddress'], 0x00)
		except:
			continue

@app.route('/api/light/manual/on', methods=['GET'])
def getApiLightManualOn():
    return jsonify({'on': conf['manual']['on']})

@app.route('/api/light/manual/on', methods=['PUT'])
def putApiLightManualOn():
    if not request.json:
        abort(400)
    if not 'on' in request.json or not isinstance(request.json['on'], bool):
        abort(400)
    conf['manual']['on'] = request.json['on']
    updateState()
    return getApiLightManualOn(), 201

@app.route('/api/light/manual/brightness', methods=['GET'])
def getApiLightManualBrightness():
    return jsonify({'brightness': conf['manual']['brightness']})

@app.route('/api/light/manual/brightness', methods=['PUT'])
def putApiLightManualBrightness():
    if not request.json:
        abort(400)
    if not 'brightness' in request.json or not is_number(request.json['brightness']):
        abort(400)
    if request.json['brightness'] < 0.0 or request.json['brightness'] > 1.0:
        abort(400)
    conf['manual']['brightness'] = request.json['brightness']

    updateState()
    return getApiLightManualBrightness(), 201

@app.route('/api/light/nodes', methods=['GET'])
def getApiLightNodes():
    return jsonify({'nodes': sorted(conf['nodes'], key=itemgetter('time'))})

@app.route('/api/light/nodes/<int:node_id>', methods=['GET'])
def getApiLightNode(node_id):
    node = [node for node in conf['nodes'] if node['id'] == node_id]
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
    for node in conf['nodes']:
        if node['id'] > maxId:
            maxId = node['id'];

    node = {
        'id': maxId + 1,
        'time': request.json['time'],
        'brightness': request.json['brightness']
    }
    conf['nodes'].append(node)

    updateState()
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

    node = [node for node in conf['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(400)

    node[0]['time'] = request.json['time']
    node[0]['brightness'] = request.json['brightness']

    updateState()
    return getApiLightNodes(), 200

@app.route('/api/light/nodes/<int:node_id>', methods=['DELETE'])
def deleteApiLightNode(node_id):
    node = [node for node in conf['nodes'] if node['id'] == node_id]
    if len(node) != 1:
        abort(404)
    conf['nodes'].remove(node[0])

    updateState()
    return getApiLightNodes(), 200


updatePwmThreadCondition = threading.Condition()
state = {
    'manual': False,
    'manualVal': 0.0,
    'points': [],
    'updated': False
}


def updateState():

    global updatePwmThreadCondition, state

    with open('light_single.json', 'w') as outfile:
        json.dump(conf, outfile)

    updatePwmThreadCondition.acquire()

    if conf['manual']['on']:
        state['manual'] = True
        state['manualVal'] = int(conf['manual']['brightness']*modules['light_single']['maxBrightness'])


    else:

        state['manual'] = False

        nodes = []
        # build list with brightness nodes with correct units
        for node in conf['nodes']:
            nodes.append(
                {
                    'time': timeText2seconds(node['time']),
                    'brightness': int(node['brightness']*modules['light_single']['maxBrightness'])
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

        state['points'] = []
        for ii in range(0, len(nodes)-1):
            cB = nodes[ii]['brightness']
            cT = nodes[ii]['time']

            while (cB != nodes[ii+1]['brightness']):

                if cB < nodes[ii+1]['brightness']:
                    cB = cB+1
                else:
                    cB = cB-1

                rB = abs(float(cB - nodes[ii]['brightness']) /                         \
                    float(nodes[ii+1]['brightness'] - nodes[ii]['brightness']))

                if cB < nodes[ii+1]['brightness']:
                    rT = rB ** (1.0/3)    ## TODO Factor should not be magic number
                else:
                    rT = rB ** (3)    ## TODO Factor should not be magic number

                cT = nodes[ii]['time'] + rT*(nodes[ii+1]['time']-nodes[ii]['time'])

                cT = ((cT - 24*60*60) if cT > 24*60*60 else cT)

                state['points'].append(
                    {
                        'time': cT,
                        'brightness': cB
                    }
                )

        state['points'] = sorted(state['points'], key=itemgetter('time'))
        state['points'].insert(0,
            {
                'time': state['points'][-1]['time'] - 24*60*60,
                'brightness': state['points'][-1]['brightness']
            }
        )
        state['points'].append(
            {
                'time': state['points'][1]['time'] + 24*60*60,
                'brightness': state['points'][1]['brightness']
            }
        )

    state['updated'] = True
    updatePwmThreadCondition.notify()
    updatePwmThreadCondition.release()


def updatePwmThreadRunner():

    global updatePwmThreadCondition, state

    updatePwmThreadCondition.acquire()
    while True:

        state['updated'] = False

        if state['manual']:
            setPwm(state['manualVal'])
            updatePwmThreadCondition.wait()
            continue

        now = secondsSinceMidnight()
        nextPointIdx = [pIdx for pIdx, point in enumerate(state['points']) if point['time'] > now][0]
        brightness = state['points'][nextPointIdx-1]['brightness']


        while True:

            setPwm(brightness)

            now = secondsSinceMidnight()
            nextPoint = next(point for point in state['points'] if point['time'] > now)
            dt = nextPoint['time'] - now
            brightness = nextPoint['brightness']

            #calculate dt - Time to wait...
            updatePwmThreadCondition.wait(dt)
            # check why wait ended, if the state has not updated, it is just time for the next step
            if state['updated']:
                break

def logPwmThreadRunner():

	loggingIntervallSeconds = 60
	time.sleep(loggingIntervallSeconds)

	while True:
		nextWakeup = time.time() + loggingIntervallSeconds
		logVal('light', float(currentPwmVal)/modules['light_single']['maxBrightness']*modules['light_single']['maxLumen'])
		time.sleep(nextWakeup - time.time())
        print 'r'

updateState()

updatePwmThread = threading.Thread(target=updatePwmThreadRunner)
updatePwmThread.daemon = True
updatePwmThread.start()

logPwmThread = threading.Thread(target=logPwmThreadRunner)
logPwmThread.daemon = True
logPwmThread.start()
