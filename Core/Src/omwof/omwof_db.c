/*
 * omwof_db.c
 *
 *  Created on: 18 Dec 2019
 *      Author: oli
 */
#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>
#include <omwof/omwof_db.h>

float32_t * temp_array(uint32_t array_length) {

	static float32_t * temp_array = NULL;
	static volatile uint32_t memory_allocated = 0;
	if (!temp_array) {
		memory_allocated = (array_length * 4);
		temp_array = malloc(memory_allocated);
		return temp_array;
	}
	if (temp_array && memory_allocated == array_length) {
		return temp_array;
	} else {
		free(temp_array);
		memory_allocated = (array_length * 4);
		temp_array = malloc(memory_allocated);
		return temp_array;

	}

}

void remove_dc_from_mag(float32_t *mag, uint32_t array_length) {

	float32_t dc_component = *mag;
	float32_t *mag_bin = mag;

	for (uint32_t i = 0; i < ((array_length / 2) - 1); ++i) {
		mag_bin += 2;
		*mag_bin = *mag_bin - dc_component;
	}
}

void remove_rms_from_wave(float32_t *samples, uint32_t array_length) {
	float32_t temp_mean = 0.0f;

	float32_t t_float[array_length];

	arm_mean_f32(samples, array_length, &temp_mean);
	arm_offset_f32(samples, ((temp_mean * -1) / array_length), &t_float[0],
			array_length); // multiply by -1 for -offset
	memcpy(samples, &t_float[0], array_length * 4);

}

volatile float32_t dynamic_range_max = 130.0f;
volatile float32_t dynamic_range_min = 63555.0f;

void mag2db(float32_t *array, float32_t *db_array, uint32_t array_len) {

	static volatile float32_t u;
	static volatile float32_t arm_max_result = 0.0f;
	static float32_t arm_min_result = 63555.0f;

	arm_abs_f32(array, db_array, array_len);

	for (uint32_t i = 0; i < array_len; ++i) {
		if (*array > 0.0f) {
			*db_array = 20 * log10(*array);
			++array;
			++db_array;
		} else {
			*db_array = -99.999f;
			++array;
			++db_array;
		}
	}

}

//float32_t shift_dB_array[64];  //  TODO : cannot hardcode shiftdb array code must be #DEFINE from header re:fft_len

void shift_db_to_100(float32_t *db_array, float32_t *output_array,
		uint32_t array_len) {

	static float32_t t_max;
	static uint32_t i;

	arm_max_f32(db_array, array_len, &t_max, &i);
	arm_offset_f32(db_array, (t_max * -1), output_array, array_len);

	for (uint32_t i = 0; i < array_len; i++) {
		*output_array = *output_array > -100 ? *output_array : -99.99f;

	}

	arm_offset_f32(output_array, 100, db_array, array_len);

}

float32_t min;

void normalize_db(float32_t *dB_array, float32_t *temp_array,
		uint32_t array_len) {

	static uint32_t pIndex;

	arm_scale_f32(dB_array, 0.01f, temp_array, array_len);
	arm_min_f32(temp_array, array_len, &min, &pIndex);
	arm_offset_f32(temp_array, (-1.0 * min), dB_array, array_len);
	arm_scale_f32(temp_array, (1 / (1.0f - min)), temp_array, array_len);
	memcpy(dB_array, temp_array, array_len * 4);

}

void square_bins(float32_t *input_bins, uint32_t number_of_bins) {
	static uint32_t i = 0;
	for (i = 0; i < number_of_bins; i++) {
		*input_bins = pow(*input_bins, 2);
		++i;
	}

}

void cube_bins(float32_t *input_bins, uint32_t number_of_bins) {
	static uint32_t i = 0;
	for (i = 0; i < number_of_bins; i++) {
		*input_bins = pow(*input_bins, 3);
		++i;
	}

}

void power_spectra(float32_t *input_bins, float32_t *power_spectra,
		uint32_t number_of_bins) {

	mag2db(input_bins, power_spectra, number_of_bins);
	shift_db_to_100(power_spectra,temp_array(number_of_bins), number_of_bins);
	normalize_db(power_spectra,temp_array(number_of_bins), number_of_bins);
}
void power_spectra1(float32_t *input_bins, float32_t *power_spectra,
		uint32_t number_of_bins) {

	remove_dc_from_mag(input_bins, number_of_bins);
	mag2db(input_bins, power_spectra, number_of_bins);
	shift_db_to_100(power_spectra,temp_array(number_of_bins), number_of_bins);
	normalize_db(power_spectra,temp_array(number_of_bins), number_of_bins);
}
void power_spectra2(float32_t *input_bins, float32_t *power_spectra,
		uint32_t number_of_bins) {

	mag2db(input_bins, power_spectra, number_of_bins);
	shift_db_to_100(power_spectra,temp_array(number_of_bins), number_of_bins);
	normalize_db(power_spectra,temp_array(number_of_bins), number_of_bins);
	square_bins(power_spectra, number_of_bins);
}
void power_spectra3(float32_t *input_bins, float32_t *power_spectra,
		uint32_t number_of_bins) {

	mag2db(input_bins, power_spectra, number_of_bins);
	shift_db_to_100(power_spectra,temp_array(number_of_bins), number_of_bins);
	normalize_db(power_spectra,temp_array(number_of_bins), number_of_bins);
	cube_bins(power_spectra, number_of_bins);
}

void power_spectra4(float32_t *input_bins, float32_t *power_spectra,
		uint32_t number_of_bins) {

	mag2db(input_bins, power_spectra, number_of_bins);
	shift_db_to_100(power_spectra,temp_array(number_of_bins), number_of_bins);

	cube_bins(power_spectra, number_of_bins);
	normalize_db(power_spectra,temp_array(number_of_bins), number_of_bins);
}
