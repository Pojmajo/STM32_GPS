#ifndef gps_header
#define gps_header

#include "stm32l1xx_hal.h"
#include "stdlib.h"
#include "string.h"

#define GPS_BUFFER_SIZE 128

struct gps_state {
	UART_HandleTypeDef * uart;
	uint8_t output_buffer[GPS_BUFFER_SIZE];
	uint8_t writer_position;

	//Time and Date
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t day;
	uint8_t month;
	uint8_t year;

	//Position
	double latitude;
	char latitudeDirection;
	double longitude;
	char longitudeDirection;
	double altitude;

	//Speed
	double speedKnots;
	double speedKilometers;

	//Satelites parameters
	uint8_t satelitesNumber;
	uint8_t quality;
	uint8_t fixMode;
	double dop;
	double hdop;
	double vdop;
};

struct gps_state gps_init(UART_HandleTypeDef * _uart);
void configureUblox();
void receiveChar(struct gps_state * state, char recv_char);
void transmitNmeaMessage(struct gps_state * state);
void lineBufferCorrection(struct gps_state * state);
void changeUartBaudrate(UART_HandleTypeDef * uart);

void ParseLine(struct gps_state * state);
void parseRMC(struct gps_state * state);
void parseVTG(struct gps_state * state);
void parseGGA(struct gps_state * state);
void parseGSA(struct gps_state * state);
void parseGLL(struct gps_state * state);

#endif
