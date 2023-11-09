# Base64 to encode/decode from digital bytes to ASCII text
from base64 import b64decode,b64encode
# JSON for package uplink and downlink
import json
# HTTP.CLIENT for HTTP connections
import http.client
# Use struct for logical structure decoding
import struct
# Datetime for Date and Timezone calcs
from datetime import datetime, timezone
# The boto3 and botocore libraries for using Python with AWS IoT Core
import boto3
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError

# Dynamodb as a database module
dynamodb = boto3.resource('dynamodb')
# IoTWireless library supporting Sidewalk and AICL (LoRaWAN)
client = boto3.client('iotwireless')
# Dynamodb table for storing uplink fragments (id as primary key, sort by time)
frag_table = dynamodb.Table('frag-test-CND36VJ1W901') 
# Dynamodb table for storing previously defragmented packets (deduplication)
dedup_table = dynamodb.Table('dedup-CND36VJ1W901') 
# Target URI for processing LoRa Cloud messages
API_URI='mgs.loracloud.com'
# API for sending data messages
DEV_API='/api/v1/uplink/send'
ADD_DEV_URI=API_URI+DEV_API
# Key for LoRa Cloud user 
# !!! NOTE !!! Must be replaced by customer's LoRa Cloud key (loracloud.com)
MGS_KEY={'Authorization': '<YOUR_LORACLOUD_KEY>'}
# Header for processing data
mgs_header= {'Authorization': MGS_KEY['Authorization'],'Content-Type': 'application/json'}

# Destination URI for posting location results
# !!! NOTE !!! Must be replaced by customer's destination URI
LOCATOR_URI='7xmbkh0zah.execute-api.eu-west-1.amazonaws.com'
# Destination POST/GET URI for posting location results
# !!! NOTE !!! Must be replaced by customer's destination API
LOCATOR_POST_API='/default/HttpInjectLocationFeed'
LOCATOR_GET_API='/default/HttpInjectLocationFeed'
# Destination KEY for posting location results
# !!! NOTE !!! Must be replaced by customer's destination authorization key 
LOCATOR_KEY = '<YOUR_LOCATION_POST_KEY>'
# Destination header for posting location results
locator_post_header = {'X-API-Key': LOCATOR_KEY,'Content-Type': 'application/json'}
locator_get_header = {'X-API-Key': LOCATOR_KEY}

    
# Convert ISO time format to timestamp
iso2ts   = lambda iso: datetime.strptime(iso[0:19]+'Z', '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc).timestamp()
iso2ts_tz   = lambda iso: datetime.strptime(iso, '%Y-%m-%dT%H:%M:%S.%f%z').timestamp()
ts2iso   = lambda ts: datetime.utcfromtimestamp(ts).strftime('%Y-%m-%dT%H:%M:%SZ')

# PayloadFragmentPush will store a current fragment from a device
def PayloadFragmentPush(params: dict):
    item = {}
    item['wdid'] = params['wdid']
    item['TSTAMPMS'] = int(params['timestamp']*1000)
    item['counter_up'] = params['counter_up']
    item['fragTot'] = params['fragTot']
    item['fragSeq'] = params['fragSeq']
    item['port'] = params['port']
    item['partHexPayload'] = params['hexASCIIPayload'][2:]
    print('PUSH fragment:: {}'.format(item))
    response = frag_table.put_item(Item=item)
    print('Fragment PUSH response: {}'.format(response))

# FullPayloadPush will store a current full payload from a device
def FullPayloadPush(params: dict, recovered: dict):
    item = {}
    item['wdid'] = params['wdid']
    item['TSTAMPMS'] = int(recovered['timestamp']*1000)
    item['fcnt'] = recovered['fcnt']
    item['fragTot'] = recovered['fragments']
    item['port'] = recovered['port']
    item['hexPayload'] = recovered['payload']
    print('PUSH recovered payload:: {}'.format(item))
    response = dedup_table.put_item(Item=item)
    print('Recovered payload PUSH response: {}'.format(response))

# Sorting function by counter_up
def sortByCounter(e):
    return (e['counter_up'])

# Sorting function by fragmentation segment
def sortBySeq(e):
    return (e['fragSeq'])

# Check function to determine if the defragmented payload is unique 
def PayloadIsNew(params: dict, recovered: dict)->bool:
    isNew = True
    wdid = params['wdid']
    try:
        response = dedup_table.query(
            KeyConditionExpression=Key("wdid").eq(wdid),
            Limit=5,
            ScanIndexForward=False # true = ascending, false = descending
        )
        print('frag table response: {}'.format(response))
    except:
        print('Failure: dedup table read')
        # Save payload
        print('Push recovered payload: {}'.format(recovered))
        FullPayloadPush(params, recovered)
        return (isNew)
    
    # check that the current has not already been saved
    fcnt = recovered['fcnt']
    if 'Items' in response:
        for item in response['Items']:
            if (fcnt == item['fcnt']):
                isNew = False
        if isNew:
            print('Save payload: {}'.format(recovered))
            FullPayloadPush(params, recovered)
        else:
            print('Duplicate payload found')
    
    return (isNew)
                
                
# Use past fragments to see if a completely defragmented payload can
#    be completed. If a completed payload is recovered, it is returned to the 
#    calling routine                
def DefragmentPayload(params: dict)->dict:
    retDict = {}
    wdid = params['wdid']
    totFrag = int(params['fragTot'])
    try:
        response = frag_table.query(
            KeyConditionExpression=Key("wdid").eq(wdid),
            Limit=totFrag*2,
            ScanIndexForward=False # true = ascending, false = descending
        )
        print('frag table response: {}'.format(response))
    except:
        print('Failure: frag table read')
        return (retDict)

    if 'Items' in response:
        fragList = []
        TSTAMPMS = 0
        FRAG = ''
        for item in response['Items']:
            print('defrag item: {}'.format(item))
            TSTAMPMS = item['TSTAMPMS']
            TSTAMP = float(int(TSTAMPMS))/1000.0
            FRAG = item['partHexPayload']
            counter_up = item['counter_up']
            fragTot = item['fragTot']
            fragSeq = item['fragSeq']
            port = item['port']
    
            vDict = {'partPayload': FRAG, 'fragTot': fragTot, 'fragSeq' : fragSeq,\
                'counter_up': counter_up, 'port': port, 'timestamp': TSTAMP}
            fragList.append(vDict)
        fragList.sort(key=sortByCounter, reverse=True)
        print('payload recall:{} '.format(fragList))
        # now check the list for a full frame
        currFrag = totFrag - 1
        defragValid = False
        frames = []
        recoveredPayload = ''
        recoveryTime = 0
        fcnt = 0
        for frag in fragList:                   # step through fragment list
            if (currFrag == frag['fragSeq']) \
                and (frag['fragTot']==totFrag):     # look for the current seq counter
                frames.append(frag)                 # if the seq we're looking for, add to frame
                lastPort = frag['port']
                currFrag = currFrag - 1             # look for next-highest seq
                if (currFrag < 0) and (len(frames) == totFrag):  # if frame full-sized, check and reconstruct
                    frames.sort(key=sortBySeq, reverse=False)    # sort by seq
                    counter = 0
                    print('defrag frames: {}'.format(frames))
                    for item in frames:                          # step through frames
                        if (counter==item['fragSeq']) and \
                            (lastPort == item['port']):           # enforce sequence and port
                            recoveredPayload = recoveredPayload + item['partPayload']
                            recoveryTime = item['timestamp']
                            fcnt = item['counter_up']
                            counter = counter + 1
                    if (counter==totFrag):  # recovered fragments are complete
                        defragValid = True
                        port = lastPort
        if defragValid:
            retDict = {'payload': recoveredPayload, 'port': port, 'fcnt': fcnt, 'fragments': totFrag, \
                       'timestamp': recoveryTime}
        else:
            print('Could not recover valid sequence')
            
    else:
         print('no Items in defrag response')


    return retDict



# Send an HTTPS request (returns a dict)
def send_https_dict(uri: str, method: str, api: str, myData: str, mgs_header: str)->dict:
    print('method:{}, uri:{}, api:{}, data:{}'.format(method,uri,api,myData))
    conn = http.client.HTTPSConnection(uri)
    conn.request(method, api, myData, mgs_header)
    response = conn.getresponse()
    print('Raw http type:{}, response:{}'.format(type(response),response))
    retVal = json.loads(response.read().decode())
    conn.close()
    return (retVal)

# Send an HTTPS request (returns a string)
def send_https_str(uri: str, method: str, api: str, myData: str, mgs_header: str)->str:
    print('method:{}, uri:{}, api:{}, data:{}'.format(method,uri,api,myData))
    conn = http.client.HTTPSConnection(uri)
    conn.request(method, api, myData, mgs_header)
    response = conn.getresponse()
    print('Raw http type:{}, response:{}'.format(type(response),response))
    retVal = response.read().decode()
    conn.close()
    return (retVal)

# Convert a wireless device ID to EUI by using the last 6 bytes of the WDID
#    Prepend '0200' to complete the ficticious DevEUI 
def Wdid2DevEUI(wdid: str)->str:
    dev_eui_nd = '0200'+wdid[-12:]
    deveui = '-'.join(dev_eui_nd[i:i+2] for i in range(0,len(dev_eui_nd),2)).upper()
    return deveui

# Once a payload is defragmented, process it by LoRa Cloud
def ProcessPayload(params: dict, network: dict):
    # First save the latest fragment
    PayloadFragmentPush(params)
    # This is the last in the sequence, defragment
    response = DefragmentPayload(params)
    if 'payload' in response:
        print('Valid payload recovery: {}'.format(response))
        if PayloadIsNew(params, response):
            port = response['port']
            deveui = Wdid2DevEUI(params['wdid'])
            if (port in [192,197,198,199]):
                # Sending a reference tracker message to MGS
                dmsmsg = json.dumps({
                deveui: {
                    "fcnt":       int(response['fcnt']),
                    "port":       int(response['port']),
                    "payload":    response['payload'],
                    "timestamp":  response['timestamp']
                }
                })
                print('dmsmsg:{}'.format(dmsmsg))
                dmsResponse = send_https_str(API_URI, 'POST', DEV_API, dmsmsg, mgs_header)
                print('dmsResponse type:{}, dmsResponse:{}'.format(type(dmsResponse),dmsResponse))
                jsonResponse = json.loads(dmsResponse)
                if 'result' in jsonResponse:
                    theDevEUI = list(jsonResponse['result'].keys())[0]
                    if 'result' in jsonResponse['result'][theDevEUI]:
                        result = jsonResponse['result'][theDevEUI]['result']
                        xmitMsg = json.dumps(
                                    {'deveui': result['deveui'], \
                                    'counter_up': int(response['fcnt']), \
                                    'port': int(response['port']),
                                   'position_solution': result['position_solution'], \
                                   'solution_type': result['operation']
                                    }
                                    )
                        print('Sending message |{}| to URI:{} on API:{}'.format(xmitMsg, LOCATOR_URI, LOCATOR_POST_API))
                        locatorResponse = send_https_str(LOCATOR_URI, 'POST', LOCATOR_POST_API, xmitMsg, locator_post_header)      
                        print('Response to Locator POST:{}'.format(locatorResponse)) 
            else:
                print('pp: unrecognized port: {}'.format(port))


# Take the Sidewalk input and apply defragmentation and LoRa Cloud processing
def ProcessSidewalkInput(qsp: dict)->tuple:
    statusCode = 200
    responseBody = 'Accepted'
    # promote data from input structures
    params = {}
    if 'wdid' in qsp:
        params['wdid'] = qsp['wdid']
    if 'seq' in qsp:
        params['counter_up'] = qsp['seq']
    if 'payload' in qsp:
        params['payload'] = qsp['payload']
        hexPayload = b64decode(qsp['payload']).decode('ascii') # sidewalk encodes ascii hex this way
        params['hexASCIIPayload'] = hexPayload
        fragTypeByte = int(hexPayload[0:2], 16)
        fragSeq =  fragTypeByte & 0x07          # lowest 3 bits, fragment counter
        fragTot = (fragTypeByte & 0x38) >> 3    # next 3 bits, total fragments
        fragTyp = (fragTypeByte & 0xC0) >> 6    # highest 2 bits, message type 
        if fragTyp == 0:
            port = 198  # frag type 0 is regular GNSS scan
        elif fragTyp == 1:
            port = 197  # frag type 1 is a WIFI scan
        elif fragTyp == 2:
            port = 199  # frag type 2 is a modem message
        else:
            port = 192  # frag type 3 is a GNSS-NG scan
        params['fragTot'] = fragTot
        params['fragSeq'] = fragSeq
        params['fragTyp'] = fragTyp
        params['port'] = port
    if 'timestamp' in qsp:
        timestampNIX = float(qsp['timestamp'])
        timestampISO = ts2iso(int(timestampNIX))
        params['timestampISO'] = timestampISO
        params['timestamp'] = timestampNIX
    if 'type' in qsp:
        msgType = qsp['type']
        params['type'] = msgType
    print('Sidewalk params: {}'.format(params))
    if ('wdid' in params) and ('timestamp' in params):
        if ('port' in params) and ('payload' in params) and \
            ('counter_up' in params) and (msgType=='uplink'):
            # This is an actual uplink
            network = dict(name='Sidewalk',type='sidewalk', \
                topic='sidewalk/downlink', id=params['wdid'], portName='port', \
                    payloadName='payload_raw', payloadFormat='base64', \
                        DEVEUI=params['wdid'].upper(), timestamp=timestampNIX)
            print('up_counter: {}, port: {}, base64_payload: {}, hex_payload: {}'.format(params['counter_up'],\
                                         params['port'], params['payload'],  params['hexASCIIPayload']))
 
            ProcessPayload(params, network)
            print('Payload processed')
        else:
            print('Missing port, payload, or fcnt')
    else:
        print('No wdid, port, payload, counter_up or timestamp')

    return (statusCode, responseBody)

# AWS Lambda serverless function to process input packets
def lambda_handler(event, context):
    print('event: {}'.format(event))
    #print('context: {}'.format(context))
    statusCode = 200
    responseBody = 'Accepted'
    # First check if this is an AICL
    if 'WirelessMetadata' in event:
        if 'LoRaWAN' in event['WirelessMetadata']:
            data = {}
            lw = event['WirelessMetadata']['LoRaWAN']
            DevEui = lw['DevEui']
            dev_eui_nd = DevEui
            data['dev_eui_nd'] = dev_eui_nd
            deveui = '-'.join(DevEui[i:i+2] for i in range(0,len(DevEui),2))
            data['deveui'] = deveui.upper()
            DEVEUI = deveui.upper()
            deveui = DEVEUI.lower()
            data['fcnt'] = int(lw.get('FCnt',0))
            data['port'] = int(lw.get('FPort', 0))
            data['dr'] = lw['DataRate']
            data['freq'] = lw['Frequency']
            data['timestamp'] = iso2ts(lw['Timestamp'])
            data['id'] = event['WirelessDeviceId']
            data['devaddr']  =  lw['DevAddr']
            data['payload']  =  event['PayloadData']
            if 'DataUp' in lw['MType']:
                data['type'] = 'uplink'
            else:
                data['type'] = 'unknown'
            data['lora'] = lw
            print('Input for AICL processing: {}'.format(data))
            # This implementation NOT for AICL
            #statusCode, responseBody = ProcessAICLInput(data)
        elif 'Sidewalk' in event['WirelessMetadata']:
            data = {}
            sw = event['WirelessMetadata']['Sidewalk']
            wdid = event['WirelessDeviceId']
            data['wdid'] = wdid
            data['WDID'] = wdid.upper()
            data['seq'] = int(sw.get('Seq',0))
            data['timestamp'] = datetime.timestamp(datetime.now())
            data['id'] = event['WirelessDeviceId']
            data['sidewalkid']  =  sw['SidewalkId']
            data['payload']  =  event['PayloadData']
            data['type'] = 'uplink'
            data['sidewalk'] = sw
            print('Input for Sidewalk processing: {}'.format(data))
            # This implementation NOT for AICL
            statusCode, responseBody = ProcessSidewalkInput(data)
            
            
    return {
        'statusCode': statusCode,
        'body': responseBody
    }
