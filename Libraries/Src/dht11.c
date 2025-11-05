#include <stdbool.h>

#include "main.h"

#include "dht11.h"

const uint16_t TIMEOUT_DURATION_US = 200;
const uint16_t MCU_START_SIGNAL_DURATION_US = 20 * 1000;
const uint16_t DATA_LENGTH_COMPARE_US = 50;

static void delay_us(dht11_handle_t *handle, uint16_t delay_duration) {
	__HAL_TIM_SET_COUNTER(handle->tim, 0);
	while (__HAL_TIM_GET_COUNTER(handle->tim) < delay_duration)
		;
}

void dht11_init(dht11_handle_t *handle, GPIO_TypeDef *dht11_port,
		uint16_t dht11_pin, TIM_HandleTypeDef *dht11_tim) {

	handle->port = dht11_port;
	handle->pin = dht11_pin;
	handle->tim = dht11_tim;

	handle->response.status = DHT11_ERROR_INIT_REQUIRED;
	handle->response.temperature = 0;
	handle->response.humidity = 0;

	HAL_TIM_Base_Start(handle->tim);
}

void set_gpio_pin_mode(dht11_handle_t *handle, dht11_pin_mode pin_mode) {
	GPIO_InitTypeDef GPIO_Struct;

	if (pin_mode == DHT11_INPUT) {
		GPIO_Struct.Mode = GPIO_MODE_INPUT;
	} else if (pin_mode == DHT11_OUTPUT) {
		GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
	}
	GPIO_Struct.Pin = handle->pin;
	GPIO_Struct.Pull = GPIO_PULLUP;
	GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(handle->port, &GPIO_Struct);
}

bool check_state_change(dht11_handle_t *handle, GPIO_PinState pin_state,
		dht11_status error_status) {
	__HAL_TIM_SET_COUNTER(handle->tim, 0);
	while (HAL_GPIO_ReadPin(handle->port, handle->pin) == pin_state) {
		if (__HAL_TIM_GET_COUNTER(handle->tim) > TIMEOUT_DURATION_US) {
			handle->response.status = error_status;
			return false;
		}
	}
	return true;
}

const char* dht11_status_to_string(dht11_status status) {
	switch (status) {
	case DHT11_OK:
		return "Success";
	case DHT11_ERROR_INIT_REQUIRED:
		return "DHT11 Initialization failed.";
	case DHT11_ERROR_INVALID_CHECKSUM:
		return "DHT11 Checksum validation failed.";
	case DHT11_ERROR_START_TIMEOUT_LOW:
		return "DHT11 MCU start signal low timeout.";
	case DHT11_ERROR_START_TIMEOUT_HIGH:
		return "DHT11 MCU start signal high timeout.";
	case DHT11_ERROR_DATA_TIMEOUT_LOW:
		return "DHT11 Data bit low pulse timeout.";
	case DHT11_ERROR_DATA_TIMEOUT_HIGH:
		return "DHT11 Data bit high pulse timeout.";
	default:
		return "Unknown Status";
	}
}

dht11_response dht11_read(dht11_handle_t *handle) {
	uint8_t dht11_data[40] = { 0 };

	if (handle->port == NULL || handle->tim == NULL) {
		return handle->response;
	}

	else {
		set_gpio_pin_mode(handle, DHT11_OUTPUT);

		HAL_GPIO_WritePin(handle->port, handle->pin, GPIO_PIN_RESET);
		delay_us(handle, MCU_START_SIGNAL_DURATION_US);
		HAL_GPIO_WritePin(handle->port, handle->pin, GPIO_PIN_SET);

		set_gpio_pin_mode(handle, DHT11_INPUT);

		__HAL_TIM_SET_COUNTER(handle->tim, 0);
		if (!check_state_change(handle, GPIO_PIN_SET,
				DHT11_ERROR_START_TIMEOUT_HIGH))
			return handle->response;
		if (!check_state_change(handle, GPIO_PIN_RESET,
				DHT11_ERROR_START_TIMEOUT_LOW))
			return handle->response;
		if (!check_state_change(handle, GPIO_PIN_SET,
				DHT11_ERROR_START_TIMEOUT_HIGH))
			return handle->response;

		for (uint8_t i = 0; i < 40; i++) {
			if (!check_state_change(handle, GPIO_PIN_RESET,
					DHT11_ERROR_DATA_TIMEOUT_LOW))
				return handle->response;
			delay_us(handle, DATA_LENGTH_COMPARE_US);
			dht11_data[i] = (HAL_GPIO_ReadPin(handle->port, handle->pin)
					== GPIO_PIN_SET);
			if (!check_state_change(handle, GPIO_PIN_SET,
					DHT11_ERROR_DATA_TIMEOUT_HIGH))
				return handle->response;
		}

		uint8_t humidity_int = 0;
		uint8_t humidity_frac = 0;
		uint8_t temperature_int = 0;
		uint8_t temperature_frac = 0;
		uint8_t checksum = 0;

		for (uint8_t i = 0; i < 8; i++)
			humidity_int = (humidity_int << 1) | dht11_data[i];
		for (uint8_t i = 8; i < 16; i++)
			humidity_frac = (humidity_frac << 1) | dht11_data[i];
		for (uint8_t i = 16; i < 24; i++)
			temperature_int = (temperature_int << 1) | dht11_data[i];
		for (uint8_t i = 24; i < 32; i++)
			temperature_frac = (temperature_frac << 1) | dht11_data[i];
		for (uint8_t i = 32; i < 40; i++)
			checksum = (checksum << 1) | dht11_data[i];

		uint8_t calculated_sum = humidity_int + humidity_frac + temperature_int
				+ temperature_frac;
		if (calculated_sum != checksum) {
			handle->response.status = DHT11_ERROR_INVALID_CHECKSUM;
			return handle->response;
		}

		handle->raw_dat[0] = humidity_int;
		handle->raw_dat[1] = humidity_frac;
		handle->raw_dat[2] = temperature_int;
		handle->raw_dat[3] = temperature_frac;
		handle->raw_dat[4] = checksum;

		handle->response.humidity = humidity_int + (humidity_frac / 10.0);
		handle->response.temperature = temperature_int
				+ (temperature_frac / 10.0);
		handle->response.status = DHT11_OK;
	}

	return handle->response;
}
