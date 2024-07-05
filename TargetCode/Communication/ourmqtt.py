#!/usr/bin/env python3.7

import paho.mqtt.client as mqtt
import json

ourclient = None


class MqttClient(object):
    # @brief Initialize the MQTT client
    # @param broker_address The IP address of the MQTT broker
    # @param broker_port The port of the MQTT broker
    # @return None
    def __init__(self, broker_address="localhost", broker_port=1883, id=1):
        self.client = mqtt.Client()
        self.client.connect(broker_address, broker_port, 60)
        self.client.loop_start()

        self.id = id

        trgt_id = str(id)
        front_id = str(id - 1)
        leader_id = '0'

        self.trgt_state = None
        self.front_state = None
        self.leader_state = None

        self.trgt_flag = False
        self.front_flag = False
        self.leader_flag = False

        self.before = 0

        # Define topics
        self.publish_topic = "trgt/" + trgt_id + "/actions"

        self.subscribe_topics = [
            "trgt/" + trgt_id + "/states",
            "trgt/" + front_id + "/states",
            "trgt/" + leader_id + "/states"
        ]

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


# Callback when a message is published
# def on_publish(client, userdata, mid):
# print(f"Message {mid} published to topic {publish_topic}")


# Callback when a message is received from the subscribed topic
def on_message(client, userdata, msg):
    print(f"Received message: {msg.payload.decode()} from topic {msg.topic}")
    content = msg.payload.decode()
    if msg.topic == ourclient.subscribe_topics[0]:
        ourclient.trgt_state = json.loads(content)
        ourclient.trgt_flag = True
    elif msg.topic == ourclient.subscribe_topics[1]:
        ourclient.front_state = json.loads(content)
        ourclient.front_flag = True
    if msg.topic == ourclient.subscribe_topics[2]:
        ourclient.leader_state = json.loads(content)
        ourclient.leader_flag = True

    if ((msg.topic == ourclient.subscribe_topics[2])
            and (msg.topic == ourclient.subscribe_topics[1])):
        ourclient.before += 1
    ourclient.before += 1
    if ourclient.before == 3:
        ourclient.publish(ourclient.publish_topic, "[0.0, 0.0, 0.0]")
        ourclient.before = 0


def initComms(broker_address, broker_port, id):
    global ourclient
    ourclient = MqttClient(broker_address, broker_port,
                           id)  # Initialize the MQTT client
    for subscribe_topic in ourclient.subscribe_topics:
        ourclient.subscribe(subscribe_topic)  # Subscribe to the topic
    ourclient.set_on_message_callback(
        on_message)  # Set the callback for when a message is received
    # ourclient.set_on_publish_callback(on_publish) # Set the callback for when a message is published
    return ourclient


def sendControls(controls):
    data = json.dumps(controls)
    ourclient.publish(ourclient.publish_topic,
                      data)  # Publish the controls to the topic
