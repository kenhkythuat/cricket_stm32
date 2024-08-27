/*
 * config.h
 *
 *  Created on: Nov 15, 2023
 *      Author: nguyenthanhtoan1095
 */


#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

//#include "main.h"
//#include "stdbool.h"
//#include "stdio.h"
//#include "stdlib.h"
//#include "string.h"

// Codename of the farm, where we deploy this node to.
#ifndef FARM
  #define FARM "demox"
#endif

// Serial number. Must be lower case.
#ifndef SERIAL_NUMBER
  #define SERIAL_NUMBER "sw000183"
#endif


/** MQTT
 * Global broker: mqtt.agriconnect.vn
 */
#define MQTT_HOST "tcp://mqtt.agriconnect.vn"           		// MQTT broker
//#define MQTT_USER "mqttnode"                          // User - connect to MQTT broker
//#define MQTT_PASS "congamo"
// demox sử dụng để test
#define MQTT_USER "node" // User - connect to MQTT broker
#define MQTT_PASS "654321"// Password - connect to MQTT broker
#define MQTT_CLIENT_ID  SERIAL_NUMBER
#define MQTT_PORT 1883

#define MQTT_TOPIC_ACTUATOR_STATUS FARM "/sn/" SERIAL_NUMBER "/as/"
// MQTT topic to subscribe and get command to switch on/off actuator
#define MQTT_TOPIC_ACTUATOR_CONTROL FARM "/snac/" SERIAL_NUMBER "/"

#define NUMBER_LOADS 4
#define LENGTH_STATUS_PAYLOAD_0_9  6*NUMBER_LOADS + 1
#define LENGTH_STATUS_PAYLOAD_10_18  6*NUMBER_LOADS + 2 + NUMBER_LOADS%10


#endif /* INC_CONFIG_H_ */
