#ifndef DHT11_H
#define DHT11_H

#include <stdbool.h>

#include "main.h"

typedef enum {
	DHT11_INPUT, DHT11_OUTPUT,
} dht11_pin_mode;

typedef enum {
	DHT11_OK,
	DHT11_ERROR_INIT_REQUIRED,
	DHT11_ERROR_INVALID_CHECKSUM,
	DHT11_ERROR_START_TIMEOUT_LOW,		// MCU start signal low time response
	DHT11_ERROR_START_TIMEOUT_HIGH,		// MCU start signal high time response
	DHT11_ERROR_DATA_TIMEOUT_LOW,    	// Data bit low pulse timeout
	DHT11_ERROR_DATA_TIMEOUT_HIGH   	// Data bit high pulse timeout
} dht11_status;

typedef struct {
	float temperature;
	float humidity;
	dht11_status status;
} dht11_response;

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
	TIM_HandleTypeDef *tim;
	dht11_response response;
	uint8_t raw_dat[5];
} dht11_handle_t;

void dht11_init(dht11_handle_t *handle, GPIO_TypeDef *dht11_port,
		uint16_t dht11_pin, TIM_HandleTypeDef *dht11_tim);
dht11_response dht11_read(dht11_handle_t *handle);
const char* dht11_status_to_string(dht11_status status);

#endif
