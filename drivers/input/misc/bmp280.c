/*!
 * @section LICENSE
 * @license$
 *
 * @filename bmp280.c
 * @date     2014/07/21
 * @id       "271c0ae"
 *
 * @brief
 * API for accessing the BMP280 sensor
 *
 * Revision: 2.0(Pressure and Temperature compensation code revision is 1.1
 */
/****************************************************************************/

#include "bmp280.h"
static struct bmp280_t *p_bmp280;                      /**< pointer to BMP280 */
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP280 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	 \param p_bmp280 *bmp280 structure pointer.
 *
 *	While changing the parameter of the p_bmp280
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_init(struct bmp280_t *bmp280)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_data_u8r = BMP280_Zero_U8X;

	p_bmp280 = bmp280;                         /* assign BMP280 ptr */
	comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr,
	BMP280_CHIPID_REG, &v_data_u8r, 1);    /* read Chip Id */
	p_bmp280->chip_id = v_data_u8r;

	bmp280_get_calib_param(); /* readout bmp280 calibparam structure */
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	0xFA -> MSB -> bit from 0 to 7
 *	0xFB -> LSB -> bit from 0 to 7
 *	0xFC -> LSB -> bit from 4 to 7
 *
 *	\param s32 utemperature : Pointer holding
 *			the uncompensated temperature.
 *
 *
 *
 *	\return	results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_ut(s32 *utemperature)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 a_data_u8r[3] = {0, 0, 0};

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_TEMPERATURE_MSB_REG, a_data_u8r, 3);
			*utemperature = (s32)(((
			(u32) (a_data_u8r[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8r[1]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8r[2]
			>> SHIFT_RIGHT_4_POSITION));
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief Reads actual temperature
 *	from uncompensated temperature and returns the value in 0.01 degree Centigrade
 *                    Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  \param s32 : value of uncompensated temperature.
 *
 *
 *
 *  \return
 *			s32 : actual temperature
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s32 bmp280_compensate_T_int32(s32 adc_t)
{
	s32 v_x1_u32r = BMP280_Zero_U8X;
	s32 v_x2_u32r = BMP280_Zero_U8X;
	s32 temperature = BMP280_Zero_U8X;

	v_x1_u32r  = ((((adc_t >> 3) - ((s32)
	p_bmp280->cal_param.dig_T1 << 1))) *
	((s32)p_bmp280->cal_param.dig_T2)) >> 11;
	v_x2_u32r  = (((((adc_t >> 4) -
	((s32)p_bmp280->cal_param.dig_T1)) * ((adc_t >> 4) -
	((s32)p_bmp280->cal_param.dig_T1))) >> 12) *
	((s32)p_bmp280->cal_param.dig_T3)) >> 14;
	p_bmp280->cal_param.t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (p_bmp280->cal_param.t_fine * 5 + 128) >> 8;

	return temperature;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to read uncompensated pressure.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	0xF7 -> MSB -> bit from 0 to 7
 *	0xF8 -> LSB -> bit from 0 to 7
 *	0xF9 -> LSB -> bit from 4 to 7
 *
 *
 *
 *	\param s32 upressure : Pointer holding the uncompensated pressure.
 *
 *
 *
 *	\return: results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_up(s32 *upressure)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 a_data_u8r[3] = {0, 0, 0};

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_PRESSURE_MSB_REG, a_data_u8r, 3);
			*upressure = (s32)(
			(((u32)(a_data_u8r[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8r[1]))
			<< SHIFT_LEFT_4_POSITION) |
			((u32)a_data_u8r[2] >>
			SHIFT_RIGHT_4_POSITION));
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns the value in Pascal(Pa)
 *                          Output value of "96386" equals 96386 Pa =
 *                          963.86 hPa = 963.86 millibar

 *
 *
 *
 *  \param s32 : value of uncompensated pressure
 *
 *
 *
 *  \return
 *			u32 : actual pressure
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
u32 bmp280_compensate_P_int32(s32 adc_p)
{
	s32 v_x1_u32r = BMP280_Zero_U8X;
	s32 v_x2_u32r = BMP280_Zero_U8X;
	u32 pressure = BMP280_Zero_U8X;

	v_x1_u32r = (((s32)p_bmp280->cal_param.t_fine) >> 1) -
	(s32)64000;
	v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) *
	((s32)p_bmp280->cal_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *
	((s32)p_bmp280->cal_param.dig_P5)) << 1);
	v_x2_u32r = (v_x2_u32r >> 2) +
	(((s32)p_bmp280->cal_param.dig_P4) << 16);
	v_x1_u32r = (((p_bmp280->cal_param.dig_P3 * (((v_x1_u32r >> 2) *
	(v_x1_u32r >> 2)) >> 13)) >> 3) +
	((((s32)p_bmp280->cal_param.dig_P2) *
	v_x1_u32r) >> 1)) >> 18;
	v_x1_u32r = ((((32768+v_x1_u32r)) *
	((s32)p_bmp280->cal_param.dig_P1))	>> 15);
	pressure = (((u32)(((s32)1048576) - adc_p) -
	(v_x2_u32r >> 12))) * 3125;
	if (pressure < 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != BMP280_Zero_U8X)
			pressure = (pressure << 1) / ((u32)v_x1_u32r);
		else
			return BMP280_Zero_U8X;
	else
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != BMP280_Zero_U8X)
			pressure = (pressure /
			(u32)v_x1_u32r) * 2;
		else
			return BMP280_Zero_U8X;
		v_x1_u32r = (((s32)
		p_bmp280->cal_param.dig_P9) *
		((s32)(((pressure >> 3)
		* (pressure >> 3)) >> 13)))
		>> 12;
		v_x2_u32r = (((s32)(pressure >> 2)) *
		((s32)p_bmp280->cal_param.dig_P8)) >> 13;
		pressure = (u32)
		((s32)pressure +
		((v_x1_u32r + v_x2_u32r +
		p_bmp280->cal_param.dig_P7) >> 4));

	return pressure;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated pressure and temperature
 *
 *
 * \param s32 upressure: Pointer holding
 *	                    the uncompensated pressure.
 * \param s32 utemperature: Pointer holding
 *			           the uncompensated temperature.
 *
 *  \return: results of bus communication function
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uput(s32 *upressure,
s32 *utemperature)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_PRESSURE_MSB_REG, a_data_u8r, 6);
			*upressure = (s32)(
			(((u32)(a_data_u8r[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8r[1]))
			<< SHIFT_LEFT_4_POSITION) |
			((u32)a_data_u8r[2] >>
			SHIFT_RIGHT_4_POSITION));

			/* Temperature */
			*utemperature = (s32)(((
			(u32) (a_data_u8r[3]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8r[4]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8r[5]
			>> SHIFT_RIGHT_4_POSITION));
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads pressure and temperature
 *
 *
 *  \param u32 pressure : Pointer holding the compensated pressure.
 *  \param s32 temperature : Pointer holding
 *                          the compensated temperature.
 *
 *
 *  \return: results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_pt(u32 *pressure,
s32 *temperature)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	s32 upressure = BMP280_Zero_U8X;
	s32 utemperature = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += bmp280_read_uput(
			&upressure, &utemperature);
			*temperature = bmp280_compensate_T_int32(
			utemperature);
			*pressure = bmp280_compensate_P_int32(
			upressure);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to
 *	calibration parameters used for calculation in the registers
 *	parameter	Register address	bit
 *	dig_T1			0x88/0x89		0 : 7 / 8: 15
 *	dig_T2			0x8A/0x8B		0 : 7 / 8: 15
 *	dig_T3			0x8C/0x8D		0 : 7 / 8: 15
 *	dig_P1			0x8E/0x8F		0 : 7 / 8: 15
 *	dig_P2			0x90/0x91		0 : 7 / 8: 15
 *	dig_P3			0x92/0x93		0 : 7 / 8: 15
 *	dig_P4			0x94/0x95		0 : 7 / 8: 15
 *	dig_P5			0x96/0x97		0 : 7 / 8: 15
 *	dig_P6			0x98/0x99		0 : 7 / 8: 15
 *	dig_P7			0x9A/0x9B		0 : 7 / 8: 15
 *	dig_P8			0x9C/0x9D		0 : 7 / 8: 15
 *	dig_P9			0x9E/0x9F		0 : 7 / 8: 15
 *
 *	\param:  None
 *
 *
 *
 *	\return: results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_calib_param(void)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 a_data_u8r[26] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_DIG_T1_LSB_REG, a_data_u8r, 24);

			p_bmp280->cal_param.dig_T1 = (u16)(((
			(u16)((u8)a_data_u8r[1])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);
			p_bmp280->cal_param.dig_T2 = (s16)(((
			(s16)((s8)a_data_u8r[3])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[2]);
			p_bmp280->cal_param.dig_T3 = (s16)(((
			(s16)((s8)a_data_u8r[5])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[4]);
			p_bmp280->cal_param.dig_P1 = (u16)(((
			(u16)((u8)a_data_u8r[7])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[6]);
			p_bmp280->cal_param.dig_P2 = (s16)(((
			(s16)((s8)a_data_u8r[9])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[8]);
			p_bmp280->cal_param.dig_P3 = (s16)(((
			(s16)((s8)a_data_u8r[11])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[10]);
			p_bmp280->cal_param.dig_P4 = (s16)(((
			(s16)((s8)a_data_u8r[13])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[12]);
			p_bmp280->cal_param.dig_P5 = (s16)(((
			(s16)((s8)a_data_u8r[15])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[14]);
			p_bmp280->cal_param.dig_P6 = (s16)(((
			(s16)((s8)a_data_u8r[17])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[16]);
			p_bmp280->cal_param.dig_P7 = (s16)(((
			(s16)((s8)a_data_u8r[19])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[18]);
			p_bmp280->cal_param.dig_P8 = (s16)(((
			(s16)((s8)a_data_u8r[21])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[20]);
			p_bmp280->cal_param.dig_P9 = (s16)(((
			(s16)((s8)a_data_u8r[23])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[22]);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *	bit					temperature oversampling
 *	0x00						Skipped
 *	0x01						BMP280_OVERSAMPLING_1X
 *	0x02						BMP280_OVERSAMPLING_2X
 *	0x03						BMP280_OVERSAMPLING_4X
 *	0x04						BMP280_OVERSAMPLING_8X
 *	0x05,0x06 and 0x07			BMP280_OVERSAMPLING_16X
 *
 *
 *  \param u8 *value : Pointer holding the osrs_t value
 *
 *
 *
 *  \return: results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_osrs_t(
u8 *value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRLMEAS_REG_OSRST__REG,
			&v_data_u8r, 1);
			*value = BMP280_GET_BITSLICE(v_data_u8r,
			BMP280_CTRLMEAS_REG_OSRST);

			p_bmp280->osrs_t = *value;
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the temperature oversampling in the register 0xF4
 *	bits from 5 to 7
 *
 *	bit					temperature oversampling
 *	0x00						Skipped
 *	0x01						BMP280_OVERSAMPLING_1X
 *	0x02						BMP280_OVERSAMPLING_2X
 *	0x03						BMP280_OVERSAMPLING_4X
 *	0x04						BMP280_OVERSAMPLING_8X
 *	0x05,0x06 and 0x07			BMP280_OVERSAMPLING_16X
 *
 *
 *  \param u8 value : the osrs_t value
 *
 *
 *
 *  \return: results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_osrs_t(
u8 value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRLMEAS_REG_OSRST__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BMP280_SET_BITSLICE(v_data_u8r,
				BMP280_CTRLMEAS_REG_OSRST, value);
				comres +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRLMEAS_REG_OSRST__REG,
				&v_data_u8r, 1);
				p_bmp280->osrs_t = value;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *	bit					pressure oversampling
 *	0x00						Skipped
 *	0x01						BMP280_OVERSAMPLING_1X
 *	0x02						BMP280_OVERSAMPLING_2X
 *	0x03						BMP280_OVERSAMPLING_4X
 *	0x04						BMP280_OVERSAMPLING_8X
 *	0x05,0x06 and 0x07			BMP280_OVERSAMPLING_16X
 *
 *
 *  \param u8 value : Pointer holding the osrs_p value
 *
 *
 *
 *  \return: results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_osrs_p(
u8 *value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRLMEAS_REG_OSRSP__REG,
			&v_data_u8r, 1);
			*value = BMP280_GET_BITSLICE(v_data_u8r,
			BMP280_CTRLMEAS_REG_OSRSP);

			p_bmp280->osrs_p = *value;
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the pressure oversampling in the register 0xF4
 *	bits from 2 to 4
 *
 *	bit					pressure oversampling
 *	0x00						Skipped
 *	0x01						BMP280_OVERSAMPLING_1X
 *	0x02						BMP280_OVERSAMPLING_2X
 *	0x03						BMP280_OVERSAMPLING_4X
 *	0x04						BMP280_OVERSAMPLING_8X
 *	0x05,0x06 and 0x07			BMP280_OVERSAMPLING_16X
 *
 *
 *  \param u8 value : the osrs_p value
 *
 *
 *
 *  \return: results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_osrs_p(
	u8 value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRLMEAS_REG_OSRSP__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r = BMP280_SET_BITSLICE(
				v_data_u8r,
				BMP280_CTRLMEAS_REG_OSRSP, value);
				comres +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRLMEAS_REG_OSRSP__REG,
				&v_data_u8r, 1);

				p_bmp280->osrs_p = value;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to get the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	\param u8 *mode : Pointer holding the mode value.
 *	0x00			->	BMP280_SLEEP_MODE
 *	0x01 and 0x02	->	BMP280_FORCED_MODE
 *	0x03			->	BMP280_NORMAL_MODE
 *  \return : results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_mode(u8 *mode)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_mode_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRLMEAS_REG_MODE__REG,
			&v_mode_u8r, 1);
			*mode = BMP280_GET_BITSLICE(v_mode_u8r,
			BMP280_CTRLMEAS_REG_MODE);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	\param u8 mode : the value of mode.
 *	0x00			->	BMP280_SLEEP_MODE
 *	0x01 and 0x02	->	BMP280_FORCED_MODE
 *	0x03			->	BMP280_NORMAL_MODE
 *
 *
 *  \return : results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_mode(u8 mode)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_mode_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			if (mode < BMP280_Four_U8X) {
				v_mode_u8r = (p_bmp280->osrs_t <<
				SHIFT_LEFT_5_POSITION) + (p_bmp280->osrs_p <<
				SHIFT_LEFT_2_POSITION) + mode;
				comres += p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRLMEAS_REG_MODE__REG, &v_mode_u8r, 1);
			} else {
			comres = E_BMP280_OUT_OF_RANGE;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0 register the device is reset using the
 * complete power-on-reset procedure.
 * Softreset can be easily set using bmp280_set_softreset().
 *
 * Usage Hint : bmp280_set_softreset()
 *
 *  \param:	None
 *
 *
 *
 *  \return: result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_softreset(void)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_data_u8r = BMP280_SOFT_RESET_CODE;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,
			BMP280_RESET_REG, &v_data_u8r, 1);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to get the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	\param  u8 *enable_disable : Pointer holding the
 *	spi3 enable or disable state.
 *
 *
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_spi3(u8 *enable_disable)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_SPI3WEN__REG,
			&v_data_u8r, 1);
			*enable_disable = BMP280_GET_BITSLICE(
			v_data_u8r,
			BMP280_CONFIG_REG_SPI3WEN);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to set the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	\param  u8 enable_disable : the spi3 enable or disable value.
 *
 *
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_spi3(u8 enable_disable)
{
	BMP280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_SPI3WEN__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r = BMP280_SET_BITSLICE(
				v_data_u8r,
				BMP280_CONFIG_REG_SPI3WEN,
				enable_disable);
				comres +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_SPI3WEN__REG,
				&v_data_u8r, 1);
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to reads filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	\param u8 *value : Pointer holding the filter value.
 *
 *	value			Filter coefficient
 *	0x00				Filter Off
 *	0x01				2
 *	0x02				4
 *	0x03				8
 *	0x04				16
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_filter(u8 *value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_FILTER__REG,
			&v_data_u8r, 1);
			*value = BMP280_GET_BITSLICE(v_data_u8r,
			BMP280_CONFIG_REG_FILTER);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to set filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	\param u8 value : The filter coefficient value
 *
 *	value			Filter coefficient
 *	0x00				Filter Off
 *	0x01				2
 *	0x02				4
 *	0x03				8
 *	0x04				16
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_filter(u8 value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_FILTER__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r = BMP280_SET_BITSLICE(
				v_data_u8r,
				BMP280_CONFIG_REG_FILTER, value);
				comres +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_FILTER__REG,
				&v_data_u8r, 1);
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to Read the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *
 *	\param u8 *time : Pointer holding
 *                        the standby duration time value.
 *              0x00 - BMP280_STANDBYTIME_1_MS
 *              0x01 - BMP280_STANDBYTIME_63_MS
 *              0x02 - BMP280_STANDBYTIME_125_MS
 *              0x03 - BMP280_STANDBYTIME_250_MS
 *              0x04 - BMP280_STANDBYTIME_500_MS
 *              0x05 - BMP280_STANDBYTIME_1000_MS
 *              0x06 - BMP280_STANDBYTIME_2000_MS
 *              0x07 - BMP280_STANDBYTIME_4000_MS
 *
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_standbydur(u8 *time)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_TSB__REG, &v_data_u8r, 1);
			*time = BMP280_GET_BITSLICE(v_data_u8r,
			BMP280_CONFIG_REG_TSB);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to write
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *	Normal mode comprises an automated perpetual cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	The standby time is determined by the contents of the register t_sb.
 *	Standby time can be set using BME280_STANDBYTIME_125_MS.
 *
 *	Usage Hint : bme280_set_standbydur(BME280_STANDBYTIME_125_MS)
 *
 *	\param u8 time : Value of the standby duration
 *              0x00 - BMP280_STANDBYTIME_1_MS
 *              0x01 - BMP280_STANDBYTIME_63_MS
 *              0x02 - BMP280_STANDBYTIME_125_MS
 *              0x03 - BMP280_STANDBYTIME_250_MS
 *              0x04 - BMP280_STANDBYTIME_500_MS
 *              0x05 - BMP280_STANDBYTIME_1000_MS
 *              0x06 - BMP280_STANDBYTIME_2000_MS
 *              0x07 - BMP280_STANDBYTIME_4000_MS
 *
 *
 *
 *
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_standbydur(u8 time)
{
	BMP280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	u8 v_data_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_TSB__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BMP280_SET_BITSLICE(v_data_u8r,
				BMP280_CONFIG_REG_TSB, time);
				comres +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_TSB__REG,
				&v_data_u8r, 1);
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API is used to write
 *	 the working mode of the sensor
 *
 *
 *  \param u8 : Mode to be set
 *				0 -> BMP280_ULTRALOWPOWER_MODE
 *				1 -> BMP280_LOWPOWER_MODE
 *				2 -> BMP280_STANDARDRESOLUTION_MODE
 *				3 -> BMP280_HIGHRESOLUTION_MODE
 *				4 -> BMP280_ULTRAHIGHRESOLUTION_MODE
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_workmode(u8 mode)
{
BMP280_RETURN_FUNCTION_TYPE comres = SUCCESS;
u8 v_data_u8r = BMP280_Zero_U8X;
if (p_bmp280 == BMP280_NULL) {
	return  E_BMP280_NULL_PTR;
	} else {
		if (mode <= BMP280_Four_U8X) {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,	BMP280_CTRLMEAS_REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				switch (mode) {
				case BMP280_ULTRALOWPOWER_MODE:
					p_bmp280->osrs_t =
					BMP280_ULTRALOWPOWER_OSRS_T;
					p_bmp280->osrs_p =
					BMP280_ULTRALOWPOWER_OSRS_P;
					break;
				case BMP280_LOWPOWER_MODE:
					p_bmp280->osrs_t =
					BMP280_LOWPOWER_OSRS_T;
					p_bmp280->osrs_p =
					BMP280_LOWPOWER_OSRS_P;
					break;
				case BMP280_STANDARDRESOLUTION_MODE:
					p_bmp280->osrs_t =
					BMP280_STANDARDRESOLUTION_OSRS_T;
					p_bmp280->osrs_p =
					BMP280_STANDARDRESOLUTION_OSRS_P;
					break;
				case BMP280_HIGHRESOLUTION_MODE:
					p_bmp280->osrs_t =
					BMP280_HIGHRESOLUTION_OSRS_T;
					p_bmp280->osrs_p =
					BMP280_HIGHRESOLUTION_OSRS_P;
					break;
				case BMP280_ULTRAHIGHRESOLUTION_MODE:
					p_bmp280->osrs_t =
					BMP280_ULTRAHIGHRESOLUTION_OSRS_T;
					p_bmp280->osrs_p =
					BMP280_ULTRAHIGHRESOLUTION_OSRS_P;
					break;
				}
				v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r,
				BMP280_CTRLMEAS_REG_OSRST, p_bmp280->osrs_t);
				v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r,
				BMP280_CTRLMEAS_REG_OSRSP, p_bmp280->osrs_p);
				comres += p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,	BMP280_CTRLMEAS_REG,
				&v_data_u8r, 1);
			}
		} else {
		comres = E_BMP280_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**\brief This API used to read both
 *	uncompensated pressure and temperature in forced mode
 *
 *
 *  \param s32 upressure: Pointer holding the uncompensated pressure.
 *  \param s32 utemperature: Pointer holding
 *                       the uncompensated temperature.
 *
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_forced_uput(s32 *upressure,
s32 *utemperature)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;
	u8 v_data_u8r = BMP280_Zero_U8X;
	u8 v_waittime_u8r = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			v_data_u8r = (p_bmp280->osrs_t
			<< SHIFT_LEFT_5_POSITION) +
			(p_bmp280->osrs_p << SHIFT_LEFT_2_POSITION) +
			BMP280_FORCED_MODE;
			comres += p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,	BMP280_CTRLMEAS_REG,
			&v_data_u8r, 1);
			bmp280_compute_wait_time(&v_waittime_u8r);
			p_bmp280->delay_msec(v_waittime_u8r);
			comres += bmp280_read_uput(upressure, utemperature);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                          the data is written in the corresponding register
 *							address
 *
 *
 *
 *  \param u8 addr, u8 data, u8 len
 *          addr -> Address of the register
 *          data -> Data to be written to the register
 *          len  -> Length of the Data
 *
 *
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_write_register(u8 addr,
u8 *data, u8 len)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,
			addr, data, len);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief This API reads the data from the given register
 *							address
 *
 *
 *
 *  \param u8 addr, u8 *data, u8 len
 *         addr -> Address of the register
 *         data -> address of the variable, read value will be kept
 *         len  -> Length of the data
 *
 *
 *
 *
 *  \return result of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_register(u8 addr,
u8 *data, u8 len)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;

	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			addr, data, len);
		}
	return comres;
}
#ifdef BMP280_ENABLE_FLOAT
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *							and returns the value in Degree centigrade
 *                          Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  \param s32 : value of uncompensated temperature
 *
 *
 *
 *  \return
 *			double : actual temperature in floating point
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
double bmp280_compensate_T_double(s32 adc_t)
{
	double v_x1_u32r = BMP280_Zero_U8X;
	double v_x2_u32r = BMP280_Zero_U8X;
	double temperature = BMP280_Zero_U8X;

	v_x1_u32r  = (((double)adc_t) / 16384.0 -
	((double)p_bmp280->cal_param.dig_T1) / 1024.0) *
	((double)p_bmp280->cal_param.dig_T2);
	v_x2_u32r  = ((((double)adc_t) / 131072.0 -
	((double)p_bmp280->cal_param.dig_T1) / 8192.0) *
	(((double)adc_t) / 131072.0 -
	((double)p_bmp280->cal_param.dig_T1) / 8192.0)) *
	((double)p_bmp280->cal_param.dig_T3);
	p_bmp280->cal_param.t_fine = (s32)(v_x1_u32r + v_x2_u32r);
	temperature  = (v_x1_u32r + v_x2_u32r) / 5120.0;


	return temperature;
}
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns pressure in Pa as double.
 *                          Output value of "96386.2"
 *                          equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *
 *  \param s32 : value of uncompensated pressure
 *
 *
 *
 *  \return
 *			double : actual pressure in floating point
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
double bmp280_compensate_P_double(s32 adc_p)
{
	double v_x1_u32r = BMP280_Zero_U8X;
	double v_x2_u32r = BMP280_Zero_U8X;
	double pressure = BMP280_Zero_U8X;

	v_x1_u32r = ((double)p_bmp280->cal_param.t_fine/2.0) - 64000.0;
	v_x2_u32r = v_x1_u32r * v_x1_u32r *
	((double)p_bmp280->cal_param.dig_P6) / 32768.0;
	v_x2_u32r = v_x2_u32r + v_x1_u32r *
	((double)p_bmp280->cal_param.dig_P5) * 2.0;
	v_x2_u32r = (v_x2_u32r / 4.0) +
	(((double)p_bmp280->cal_param.dig_P4) * 65536.0);
	v_x1_u32r = (((double)p_bmp280->cal_param.dig_P3) *
	v_x1_u32r * v_x1_u32r / 524288.0 +
	((double)p_bmp280->cal_param.dig_P2) * v_x1_u32r) / 524288.0;
	v_x1_u32r = (1.0 + v_x1_u32r / 32768.0) *
	((double)p_bmp280->cal_param.dig_P1);
	pressure = 1048576.0 - (double)adc_p;
	/* Avoid exception caused by division by zero */
	if (v_x1_u32r != 0.0)
		pressure = (pressure - (v_x2_u32r / 4096.0)) *
		6250.0 / v_x1_u32r;
	else
		return 0;
	v_x1_u32r = ((double)p_bmp280->cal_param.dig_P9) *
	pressure * pressure / 2147483648.0;
	v_x2_u32r = pressure * ((double)p_bmp280->cal_param.dig_P8) / 32768.0;
	pressure = pressure + (v_x1_u32r + v_x2_u32r +
	((double)p_bmp280->cal_param.dig_P7)) / 16.0;

	return pressure;
}
#endif
#if defined(BMP280_ENABLE_INT64) && defined(BMP280_64BITSUPPORT_PRESENT)
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *                          and returns the value in Pa as unsigned 32 bit
 *                          integer in Q24.8 format (24 integer bits and
 *                          8 fractional bits). Output value of "24674867"
 *                          represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  \param s32 : value of uncompensated temperature
 *
 *
 *
 *  \return
 *			u32 : actual pressure
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
u32 bmp280_compensate_P_int64(s32 adc_p)
{
	s64 v_x1_s64r = BMP280_Zero_U8X;
	s64 v_x2_s64r = BMP280_Zero_U8X;
	s64 pressure = BMP280_Zero_U8X;

	v_x1_s64r = ((s64)p_bmp280->cal_param.t_fine) - 128000;
	v_x2_s64r = v_x1_s64r * v_x1_s64r *
	(s64)p_bmp280->cal_param.dig_P6;
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r *
	(s64)p_bmp280->cal_param.dig_P5) << 17);
	v_x2_s64r = v_x2_s64r +
	(((s64)p_bmp280->cal_param.dig_P4) << 35);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r *
	(s64)p_bmp280->cal_param.dig_P3) >> 8) +
	((v_x1_s64r * (s64)p_bmp280->cal_param.dig_P2) << 12);
	v_x1_s64r = (((((s64)1) << 47) + v_x1_s64r)) *
	((s64)p_bmp280->cal_param.dig_P1) >> 33;
	pressure = 1048576 - adc_p;
	if (v_x1_s64r != BMP280_Zero_U8X)
		#if defined __KERNEL__
			pressure = div64_s64((((pressure << 31) - v_x2_s64r)
			* 3125), v_x1_s64r);
		#else
			pressure = (((pressure << 31) - v_x2_s64r)
			* 3125) / v_x1_s64r;
		#endif
	else
		return BMP280_Zero_U8X;
	v_x1_s64r = (((s64)p_bmp280->cal_param.dig_P9) *
	(pressure >> 13) * (pressure >> 13)) >> 25;
	v_x2_s64r = (((s64)p_bmp280->cal_param.dig_P8) *
	pressure) >> 19;
	pressure = ((pressure + v_x1_s64r + v_x2_s64r) >> 8) +
	(((s64)p_bmp280->cal_param.dig_P7) << 4);
	return (u32)pressure;
}
#endif
/*******************************************************************************
 * Description: *//**\brief Computing waiting time for sensor data read
 *
 *
 *
 *
 *  \param
 *			u8 : value of time
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_compute_wait_time(u8
*v_delaytime_u8r)
{
	BMP280_RETURN_FUNCTION_TYPE comres = BMP280_Zero_U8X;

	*v_delaytime_u8r = (T_INIT_MAX + T_MEASURE_PER_OSRS_MAX *
	(((1 << p_bmp280->osrs_t) >> 1) + ((1 << p_bmp280->osrs_p)
	>> 1)) + (p_bmp280->osrs_p ? T_SETUP_PRESSURE_MAX : 0) + 15)
	/ 16;
	return comres;
}
