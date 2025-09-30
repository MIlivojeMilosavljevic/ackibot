#include <iostream>
#include <cstring>
#include <mosquitto.h>

// Define MQTT broker address and ports
#define MQTT_SERVER_ADDRESS "localhost"
#define MQTT_SERVER_PORT 1883

// Define MQTT topics for sensors and actuators
#define SENSORS "sensors/+"
#define TEMPERATURE_TOPIC "sensors/temperature"
#define RELAY_VENT_TOPIC "actuators/relay_vent"
//za pumpu
#define PUMP_TOPIC "actuators/relay_pump"
#define HUMIDITY_TOPIC "sensors/humidity"

using namespace std;


// Define command values for actuators
#define COMMAND_ON "ON"
#define COMMAND_OFF "OFF"

// MQTT callback function
void on_message_callback(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* message) {
    if (message->payloadlen) {
        std::cout << "Message received on topic: " << message->topic << std::endl;
        std::cout << "Payload: " << (char*)message->payload << std::endl;

        // Process the received message here
        if (strcmp(message->topic, TEMPERATURE_TOPIC) == 0) {
            double temperature = std::stod((char*)message->payload);
            const char* command = (temperature > 29.0) ? COMMAND_ON : (temperature < 24.0) ? COMMAND_OFF : nullptr;

            if (command) {
                // Publish command to turn on/off ventilator
                mosquitto_publish(mosq, NULL, RELAY_VENT_TOPIC, strlen(command), command, 1, true);
                // Output to terminal
                std::cout << "Published command: " << command << " on topic: " << RELAY_VENT_TOPIC << std::endl;
            }
        }

        if(strcmp(message->topic , HUMIDITY_TOPIC) == 0) {
            double humidity = stod((char*)message->payload);
            //TO DO probeniti vent
            const char* command = (humidity < 50) ? COMMAND_ON : (humidity > 70.0) ? COMMAND_OFF : nullptr;

            if (command) {
                // Publish command to turn on/off ventilator
                mosquitto_publish(mosq, NULL, PUMP_TOPIC, strlen(command), command, 1, true);
                // Output to terminal
                std::cout << "Published command: " << command << " on topic: " << PUMP_TOPIC << std::endl;
            }
        }
    }
}

int main(int argc, char* argv[]) {
    struct mosquitto* mosq = NULL;

    // Initialize mosquitto library
    mosquitto_lib_init();

    // Create mosquitto client instance
    mosq = mosquitto_new("controller_client", true, NULL);
    if (!mosq) {
        std::cerr << "Error: Unable to create mosquitto client instance" << std::endl;
        mosquitto_lib_cleanup();
        return 1;
    }

    // Connect to the MQTT broker
    if (mosquitto_connect(mosq, MQTT_SERVER_ADDRESS, MQTT_SERVER_PORT, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Error: Unable to connect to MQTT broker" << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }
    cout << "Connected to MQTT broker" << std::endl;

    // Subscribe to sensor topics
    mosquitto_subscribe(mosq, NULL, SENSORS, 1);
    cout << "Subscribed to " << SENSORS " topic" << endl;

    // Set message callback
    mosquitto_message_callback_set(mosq, on_message_callback);

    // Enter mosquitto loop
    mosquitto_loop_forever(mosq, -1, 1);

    // Cleanup and disconnect from the MQTT broker
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    cout << "Disconnected from MQTT broker" << std::endl;

    return 0;
}
