#!/usr/bin/env python3.7

import paho.mqtt as carlaCommunication
import json


class MqttClient(object):
    # @brief Initialize the MQTT client
    # @param broker_address The IP address of the MQTT broker
    # @param broker_port The port of the MQTT broker
    # @return None
    def __init__(self, broker_address, broker_port):
        self.client = mqtt.Client()
        self.client.connect(broker_address, broker_port, 60)
        self.client.loop_start()

    # @brief Publish a message to a topic
    # @param topic The topic to publish to
    # @param message The message to publish
    # @return None
    def publish(self, topic, message):
        self.client.publish(topic, message)

    # @brief Subscribe to a topic
    # @param topic The topic to subscribe to
    # @return None
    def subscribe(self, topic):
        self.client.subscribe(topic)

    # @brief Unsubscribe from a topic
    # @param topic The topic to unsubscribe from
    # @return None
    def unsubscribe(self, topic):
        self.client.unsubscribe(topic)

    # @brief Set the callback for when a message is received
    # @param callback The callback function
    # @return None
    def set_on_message_callback(self, callback):
        self.client.on_message = callback

    # @brief Set the callback for when a message is published
    # @param callback The callback function
    # @return None
    def set_on_publish_callback(self, callback):
        self.client.on_publish = callback

    # @brief Disconnect from the broker
    # @return None
    def disconnect(self):
        self.client.disconnect()


# MQTT broker details
broker_address = "localhost"
broker_port = 1883

# Define IDs (HARD CODED)
trgt_id = '03'
front_id = '02'
leader_id = '01'

# Define states
trgt_state = None
front_state = None
leader_state = None

trgt_flag = False
front_flag = False
leader_flag = False

# Define topics
publish_topic = "trgt/" + trgt_id + "/actions"

subscribe_topics = {"trgt/" + trgt_id + "/sensors",
                    "front/" + front_id + "/sensors",
                    "leader/" + leader_id + "/sensors"}


# Callback when a message is published
def on_publish(client, userdata, mid):
    print(f"Message {mid} published to topic {publish_topic}")


# Callback when a message is received from the subscribed topic
def on_message(client, userdata, msg):
    print(f"Received message: {msg.payload.decode()} from topic {msg.topic}")
    content = msg.payload.decode()
    if msg.topic == "trgt/" + trgt_id + "/sensors":
        trgt_state = json.loads(content)
        trgt_flag = True
    elif msg.topic == "front/" + front_id + "/sensors":
        front_state = json.loads(content)
        front_flag = True
    elif msg.topic == "leader/" + leader_id + "/sensors":
        leader_state = json.loads(content)
        leader_flag = True


def initComms():
    client = MqttClient(broker_address, broker_port)  # Initialize the MQTT client
    for subscribe_topic in subscribe_topics:
        client.subscribe(subscribe_topic)  # Subscribe to the topic
    client.set_on_message_callback(on_message) # Set the callback for when a message is received
    client.set_on_publish_callback(on_publish) # Set the callback for when a message is published

    return client # Return the client object


def sendControls(client, controls):
    client.publish(publish_topic, json.dumps(controls)) # Publish the controls to the topic
