
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
