#!/usr/bin/env python3

# from https://stackoverflow.com/questions/54292179/saving-mqtt-data-from-subscribe-topic-on-a-text-file

import paho.mqtt.client as mqttClient
import time

def on_connect(client, userdata, flags, rc):


    if rc == 0:

        print("Connected to broker")

        global Connected                #Use global variable
        Connected = True                #Signal connection

    else:
      
        print("Connection failed")
        
def on_message(client, userdata, message):
    print("")
    print("Message received: "  + str(message.payload))

    with open('myData.txt','a+') as f: # You can change the data file name here
         f.write(str(message.payload)[2:-1]+"\n")

Connected = False   #global variable for the state of the connection

broker_address= "nam1.cloud.thethings.network"  #host
port = 1883                         #Broker port
user = "food-cabinet-2024@ttn" #<--  Put your TTN V3 app here                    #Connection username
password = "NNSXS.CTRZE653QMT54LDATALWEQEND7RUTGVW43ZOSAY.PWFMYW35UQC6XMOH4EKSZAPIKUFQKAKEB5LM3OO2GW44UXHOWHZQ" #<--  Put your TTN V3 API key in quotes           #Connection password

client = mqttClient.Client("Python")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.on_message= on_message                      #attach function to callback
client.connect(broker_address,port,60) #connect
client.subscribe(f"v3/{user}/devices/+/up") #subscribe
client.loop_forever() #then keep listening forever