#include "adpcm.h"

static const int16_t step_size_table[89] = {
	7, 8, 9, 10, 11, 12, 13, 14,
	16, 17, 19, 21, 23, 25, 28, 31,
	34, 37, 41, 45, 50, 55, 60, 66,
	73, 80, 88, 97, 107, 118, 130, 143,
	157, 173, 190, 209, 230, 253, 279, 307,
	337, 371, 408, 449, 494, 544, 598, 658,
	724, 796, 876, 963, 1060, 1166, 1282, 1411,
	1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
	3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
	7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
	15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
	32767
};

static const int8_t index_adjust[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };

void adpcm_encode(struct adpcm_state *state, const int16_t *pcm, uint8_t *adpcm, size_t sample_count)
{
	int16_t predictor = state->predictor;
	uint8_t step_index = state->step_index;
	size_t byte_idx = 0;

	for (size_t i = 0; i < sample_count; i++) {
		int32_t diff = (int32_t)pcm[i] - predictor;
		uint8_t nibble = 0;

		if (diff < 0) {
			nibble = 8;
			diff = -diff;
		}

		int16_t step = step_size_table[step_index];
		int32_t delta = step >> 3;

		if (diff >= step) {
			nibble |= 4;
			diff -= step;
			delta += step;
		}
		step >>= 1;
		if (diff >= step) {
			nibble |= 2;
			diff -= step;
			delta += step;
		}
		step >>= 1;
		if (diff >= step) {
			nibble |= 1;
			delta += step;
		}

		if (nibble & 8) {
			predictor -= delta;
		} else {
			predictor += delta;
		}

		if (predictor > 32767) predictor = 32767;
		if (predictor < -32768) predictor = -32768;

		step_index += index_adjust[nibble & 7];
		if (step_index < 0) step_index = 0;
		if (step_index > 88) step_index = 88;

		if (i & 1) {
			adpcm[byte_idx] |= nibble;
			byte_idx++;
		} else {
			adpcm[byte_idx] = nibble << 4;
		}
	}

	if (sample_count & 1) {
		byte_idx++;
	}

	state->predictor = predictor;
	state->step_index = step_index;
}

void adpcm_decode(struct adpcm_state *state, const uint8_t *adpcm, int16_t *pcm, size_t sample_count)
{
	int16_t predictor = state->predictor;
	uint8_t step_index = state->step_index;

	for (size_t i = 0; i < sample_count; i++) {
		uint8_t nibble;

		if (i & 1) {
			nibble = adpcm[i / 2] & 0x0F;
		} else {
			nibble = (adpcm[i / 2] >> 4) & 0x0F;
		}

		int32_t diff = 0;
		int16_t step = step_size_table[step_index];

		if (nibble & 4) diff += step;
		if (nibble & 2) diff += step >> 1;
		if (nibble & 1) diff += step >> 2;
		diff += step >> 3;

		if (nibble & 8) {
			predictor -= diff;
		} else {
			predictor += diff;
		}

		if (predictor > 32767) predictor = 32767;
		if (predictor < -32768) predictor = -32768;

		step_index += index_adjust[nibble & 7];
		if (step_index < 0) step_index = 0;
		if (step_index > 88) step_index = 88;

		pcm[i] = predictor;
	}

	state->predictor = predictor;
	state->step_index = step_index;
}
