// Put project specific (private) header files here

#ifndef MAIN_H
#define MAIN_H

// Pin definitions
#define PIN_HDMI 33

// MQTT Topics
#define PUBLISH_TOPIC "T3nz/UPN/coffeeShop/sendData"
#define SUBSCRIBE_TOPIC "T3nz/UPN/coffeeShop/getData"

// Task stack sizes
#define MQTT_TASK_STACK_SIZE 8192

// Task priorities
#define MQTT_TASK_PRIORITY 2

// MQTT settings
#define MQTT_BUFFER_SIZE 512
#define MQTT_KEEPALIVE 60

#endif // MAIN_H
