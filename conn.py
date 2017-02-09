import network
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect('EEERover','exhibition')
print(sta_if.isconnected())

from umqtt.simple import MQTTClient
client = MQTTClient('unnamed1','192.168.0.10')
client.connect()
client.publish('esys/unnamed1/',bytes("Afrazinator", 'utf-8'))
