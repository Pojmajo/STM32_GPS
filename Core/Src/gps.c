/*
 * gps.c
 *
 *  Created on: 07.11.2019
 *      Author: marci
 */
#include "gps.h"
#include "main.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

char* strtoke(char *str, const char *delim)
{
  static char *start = NULL; /* stores string str for consecutive calls */
  char *token = NULL; /* found token */
  /* assign new start in case */
  if (str) start = str;
  /* check whether text to parse left */
  if (!start) return NULL;
  /* remember current start as found token */
  token = start;
  /* find next occurrence of delim */
  start = strpbrk(start, delim);
  /* replace delim with terminator and move start to follower */
  if (start) *start++ = '\0';
  /* done */
  return token;
}

struct gps_state gps_init(UART_HandleTypeDef * uart) {
	struct gps_state state;

	state.uart = uart;
	for(uint8_t i=0; i<GPS_BUFFER_SIZE; i++) state.output_buffer[i] = '\0';
	state.writer_position = 0;

	//Init values//
	//Time and Date
	state.hour = 0;
	state.minute = 0;
	state.second = 0;
	state.day = 0;
	state.month = 0;
	state.year = 0;

	//Position
	state.latitude = 0;
	state.latitudeDirection = 0;
	state.longitude = 0;
	state.longitudeDirection = 0;
	state.altitude = 0;

	//Speed
	state.speedKnots = 0;
	state.speedKilometers = 0;

	//Satelites parameters
	uint8_t satelitesNumber = 0;
	uint8_t quality = 0;
	uint8_t fixMode = 0;
	double dop = 0;
	double hdop = 0;
	double vdop = 0;

	return state;
}

void configureUblox()
{
	//Info arrays
    char config_start[] = "Rozpoczecie konfiguracji\r\n";
    char config_end[] = "Konfiguracja zakonczona\r\n";
    char clear_terminal[] = "\033[2J\033[;H";

    //Configuration byte arrays
	uint8_t gps_only_msg[] = {0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x10,0x07,0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,0x02,0x04,0x08,0x00,0x00,0x00,0x01,0x01,0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01,0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,0x00,0x00,0x01,0x01,0x1C,0xAD};
	uint8_t set_baudrate[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xDE,0xC9};
	uint8_t freq_rate_10Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
	uint8_t save_settings[] = {0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0x31,0xBF};

	//Setting NEO M8N
	HAL_UART_Transmit(&huart2, config_start, sizeof(config_start)/sizeof(uint8_t), 10);
	HAL_Delay(100);

	HAL_UART_Transmit(&huart1, gps_only_msg, sizeof(gps_only_msg)/sizeof(uint8_t), 10);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1, freq_rate_10Hz, sizeof(freq_rate_10Hz)/sizeof(uint8_t), 10);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1, set_baudrate, sizeof(set_baudrate)/sizeof(uint8_t), 10);
	HAL_Delay(1000);
	changeUartBaudrate(&huart1);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart1, save_settings, sizeof(save_settings)/sizeof(uint8_t), 10);

	//Status message
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart2, clear_terminal, sizeof(clear_terminal)/sizeof(uint8_t), 10);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart2, config_end, sizeof(config_end)/sizeof(uint8_t), 10);
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart2, clear_terminal, sizeof(clear_terminal)/sizeof(uint8_t), 10);

}


void receiveChar(struct gps_state * state, char recv_char) {
	if (state->writer_position == 0 && recv_char == '$') {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
		state->output_buffer[state->writer_position] = recv_char;
		state->writer_position++;
	} else if (state->writer_position >= 1 && state->writer_position < GPS_BUFFER_SIZE - 1) {
		if (recv_char == '\r' || recv_char == '\n') {
			state->output_buffer[state->writer_position] = '\r';
			state->writer_position++;
			state->output_buffer[state->writer_position] = '\n';
			ParseLine(state);
			state->writer_position = 0;
		} else {
			state->output_buffer[state->writer_position] = recv_char;
			state->writer_position++;
		}
	} else {
		state->writer_position = 0;
	}
}

void ParseLine(struct gps_state * state)
{
char* ParsePoiner = strtoke((char*)state->output_buffer, ",");
uint8_t Message[64];
uint8_t MessageLength;

if(strcmp(ParsePoiner, "$GPRMC") == 0)
	{
	parseRMC(state);
	//HAL_UART_Transmit_IT(&huart2, Message, MessageLength);
	}
else if(strcmp(ParsePoiner, "$GPVTG") == 0)
	{
	//parseVTG(state);
	//MessageLength = sprintf((char*)Message, "Speed: %.2f knots, %f km/h\n\r", state->speedKnots, state->speedKilometers);
	//HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);
	}
else if(strcmp(ParsePoiner, "$GPGGA") == 0)
	{
	parseGGA(state);
//	MessageLength = sprintf((char*)Message, "%02d:%02d:%02d;%02d.%02d.20%02d;%.2f;%.2f;%d\n\r", state->hour, state->minute, state->second, state->day, state->month, state->year, state->latitude, state->longitude, state->satelitesNumber);
	}
else if(strcmp(ParsePoiner, "$GPGLL") == 0)
	{
	//parseGLL(state);
	//MessageLength = sprintf((char*)Message, "GLL;%02d:%02d:%02d;%02d.%02d.20%02d;%.2f;%.2f\n\r", state->hour, state->minute, state->second, state->day, state->month, state->year, state->latitude, state->longitude);
	//HAL_UART_Transmit_IT(&huart2, Message, MessageLength);
	}
MessageLength = sprintf((char*)Message, "%02d:%02d:%02d;%02d.%02d.20%02d;%.2f;%.2f;%d\n\r", state->hour, state->minute, state->second, state->day, state->month, state->year, state->latitude, state->longitude, state->satelitesNumber);
HAL_UART_Transmit_IT(&huart2, Message, MessageLength);
}

void parseRMC(struct gps_state * state)
{
	char *ParsePoiner;
	uint32_t Temp;

	ParsePoiner = strtoke(NULL, ",");
	//if(strlen(ParsePoiner) > 0)
	//{
	//	Temp = atoi(ParsePoiner);
	//	state->second = Temp % 100;
	//	state->minute = (Temp / 100) % 100;
	//	state->hour = (Temp / 10000) % 100;
	//}

	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	//if(strlen(ParsePoiner) > 0)
	//{
	//	state->latitude = atof(ParsePoiner);
	//}
	ParsePoiner = strtoke(NULL, ",");
	//if(strlen(ParsePoiner) > 0)
	//{
	//	state->latitudeDirection = *ParsePoiner;
	//}
	ParsePoiner = strtoke(NULL, ",");
	//if(strlen(ParsePoiner) > 0)
	//{
	//	state->longitude = atof(ParsePoiner);
	//}
	ParsePoiner = strtoke(NULL, ",");
	//if(strlen(ParsePoiner) > 0)
	//{
	//	state->longitudeDirection = *ParsePoiner;
	//}
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		Temp = atoi(ParsePoiner);
		state->year = Temp % 100;
		state->month = (Temp/100) % 100;
		state->day = (Temp/10000) % 100;
	}
}

void parseVTG(struct gps_state * state)
{
	char  *ParsePoiner;
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->speedKnots = atof(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->speedKilometers = atof(ParsePoiner);
	}
}


void parseGGA(struct gps_state * state)
{
	char *ParsePoiner;
	uint32_t Temp;

	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		Temp = atoi(ParsePoiner);
		state->second = Temp % 100;
		state->minute = (Temp / 100) % 100;
		state->hour = (Temp / 10000) % 100;
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->latitude = atof(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->latitudeDirection = *ParsePoiner;
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->longitude = atof(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->longitudeDirection = *ParsePoiner;
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->quality = atoi(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->satelitesNumber = atoi(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->altitude = atof(ParsePoiner);
	}
}

void parseGSA(struct gps_state * state)
{
	char *ParsePoiner;
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->fixMode = atoi(ParsePoiner);
	}
	if(strlen(ParsePoiner) > 0)
	{
		state->altitude = atof(ParsePoiner);
	}
	for(uint8_t i=0; i < 12; i++)
	{
		ParsePoiner = strtoke(NULL, ",");
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->dop = atof(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->hdop = atof(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->hdop = atof(ParsePoiner);
	}

}


void parseGLL(struct gps_state * state)
{
	char *ParsePoiner;
	uint32_t Temp;

	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->latitude = atof(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
	state->latitudeDirection = *ParsePoiner;
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->longitude = atof(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		state->latitudeDirection = *ParsePoiner;
	}
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		Temp = atoi(ParsePoiner);
		state->second = Temp % 100;
		state->minute = (Temp / 100) % 100;
		state->hour = (Temp / 10000) % 100;
	}

}




void lineBufferCorrection(struct gps_state * state)
{
	for(uint8_t i=((state->writer_position)+1); i<GPS_BUFFER_SIZE; i++) state->output_buffer[i] = '\0';
}

void transmitNmeaMessage(struct gps_state * state) {
	lineBufferCorrection(state);
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)state->output_buffer, 128);
}

void changeUartBaudrate(UART_HandleTypeDef * uart) {
	while (__HAL_UART_GET_FLAG(uart, UART_FLAG_TC) == RESET){
	}
	HAL_UART_DeInit(uart);
	MX_USART1_UART_Init_57600();
}


