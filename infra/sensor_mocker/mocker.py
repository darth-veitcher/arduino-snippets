import json
import os
import random
import time

import paho.mqtt.client as mqtt

MQTT_HOST: str = os.environ.get("MQTT_HOST", "mosquitto")
MQTT_PORT: int = int(os.environ.get("MQTT_PORT", 1883))
MQTT_TOPICS: list[str] = [
    t.strip()
    for t in os.environ.get(
        "MQTT_TOPICS",
        "sensors/bedroom/temperature, sensors/kitchen/temperature, sensors/mancave/temperature",
    ).split(",")
]

# Global variable to hold the latest state for each topic
latest_states = {}

# Callback when connected
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {str(rc)}")
    client.subscribe("#")  # Subscribing to all topics

# Callback when a message is received
def on_message(client, userdata, msg):
    global latest_states
    print(f"Topic: {msg.topic}\nMessage: {str(msg.payload)}")
    
    topic = msg.topic
    payload = json.loads(msg.payload)
    latest_states[topic] = payload  # Update the latest state

# Function to publish random messages to random topics
def publish_random_messages(client):
    topics = ["home/kitchen", "home/livingroom", "home/bedroom"]  # Define your topics
    
    while True:
        topic = random.choice(topics)
        
        # Initialize state if not already
        if topic not in latest_states:
            latest_states[topic] = {
                "temperature": round(random.uniform(20.0, 30.0), 2),
                "humidity": round(random.uniform(40.0, 60.0), 2)
            }
        
        # Generate new state based on last state
        last_state = latest_states[topic]
        new_temperature = round(random.uniform(last_state["temperature"] - 1, last_state["temperature"] + 1), 2)
        new_humidity = round(random.uniform(last_state["humidity"] - 2, last_state["humidity"] + 2), 2)
        
        # Boundary checks
        new_temperature = max(20.0, min(30.0, new_temperature))
        new_humidity = max(40.0, min(60.0, new_humidity))
        
        # Publish new state
        message = {
            "temperature": new_temperature,
            "humidity": new_humidity
        }
        print(f"Publishing to {topic}: {json.dumps(message)}")
        client.publish(topic, json.dumps(message))
        
        time.sleep(random.randint(1, 5))  # Sleep for a random interval between 1 and 5 seconds

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_HOST, MQTT_PORT, 60)

# Start a new thread for listening to messages and another for publishing messages
client.loop_start()
publish_random_messages(client)