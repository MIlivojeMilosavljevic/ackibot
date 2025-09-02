
#include "sabertooth.h"

uint8_t crc7(
	const uint8_t* data,
	uint8_t length
) {
	uint8_t crc = 0x7f;
	uint8_t i, bit;
	for(i = 0; i < length; i++){
		crc ^= data[i];
		for(bit = 0; bit < 8; bit++){
			if(crc & 1){
				crc >>= 1;
				crc ^= 0x76;
			}else{
				crc >>= 1;
			}
		}
	}
	return crc ^ 0x7f;
}

uint16_t crc14(
	const uint8_t* data,
	uint8_t length
) {
	uint16_t crc = 0x3fff;
	uint8_t i, bit;
	for(i = 0; i < length; i ++){
		crc ^= data[i];
		for(bit = 0; bit < 8; bit ++){
			if(crc & 1){
				crc >>= 1;
				crc ^= 0x22f0;
			}else{
				crc >>= 1;
			}
		}
	}
	return crc ^ 0x3fff;
}

uint8_t writeCRCSabertoothCommand(
	uint8_t address,
	uint8_t command,
	uint8_t value,
	const uint8_t* data,
	uint8_t length,
	uint8_t* buffer
) {
	uint8_t i;
	uint16_t crc;
	buffer[0] = address + 112;
	buffer[1] = command;
	buffer[2] = value;
	buffer[3] = crc7(buffer, 3);
	if(length == 0){
		return 4;
	}else{
		for(i = 0; i < length; i ++){
			buffer[4 + i] = data[i];
		}
		crc = crc14(buffer + 4, length);
		buffer[4 + length] = (uint8_t)((crc >> 0) & 127);
		buffer[5 + length] = (uint8_t)((crc >> 7) & 127);
		return 6 + length;
	}
}

uint8_t writeCRCSabertoothSetCommand(
	uint8_t address,
	uint8_t setType,
	uint8_t targetType,
	uint8_t targetNumber,
	int16_t value,
	uint8_t* buffer
) {
	uint8_t data[4];
	data[2] = targetType;
	data[3] = targetNumber;
	if(value < 0){
		value = -value;
		data[0] = (uint8_t)((value >> 0) & 127);
		data[1] = (uint8_t)((value >> 7) & 127);
		return writeCRCSabertoothCommand(
			address,
			40,
			setType + 1,
			data,
			4,
			buffer
		);
	}else{
		data[0] = (uint8_t)((value >> 0) & 127);
		data[1] = (uint8_t)((value >> 7) & 127);
		return writeCRCSabertoothCommand(
			address,
			40,
			setType,
			data,
			4,
			buffer
		);
	}
}