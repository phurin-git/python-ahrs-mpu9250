import paho.mqtt.publish as pub, paho.mqtt.subscribe as sub, json

def publish(topic:str, payload:dict, hostname:str):
    pub.single(topic = topic, payload = json.dumps(payload), hostname = hostname)

def subscribe(topic:str, hostname:str):
    msg = sub.simple(topics = topic, hostname = hostname)
    data = json.loads(msg.payload.decode()) # type: ignore
    return data