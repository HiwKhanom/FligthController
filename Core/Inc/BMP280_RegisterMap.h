/*
 * BMP280_RegisterMap.h
 *
 *  Created on: Jun 2, 2024
 *      Author: panna
 */

#ifndef INC_BMP280_REGISTERMAP_H_
#define INC_BMP280_REGISTERMAP_H_

enum BMP280_register{
	BMP280_addr = 0x76 << 1,

	BMP280_who_am_i = 0xD0,
	BMP280_ID = 0x58,

	BMP280_reset = 0xE0,
	BMP280_status = 0xF3,
	BMP280_ctrl_meas = 0xF4,
	BMP280_config = 0xF5,

	BMP280_press_msb = 0xF7,
	BMP280_press_lsb = 0xF8,
	BMP280_press_xlsb = 0xF9,

	BMP280_temp_msb = 0xFA,
	BMP280_temp_lsb = 0xFB,
	BMP280_temp_xlsb = 0xFC,

	BMP280_calib00 = 0x88
};

#endif /* INC_BMP280_REGISTERMAP_H_ */
