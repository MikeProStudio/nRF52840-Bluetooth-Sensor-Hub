#ifndef ADPCM_H
#define ADPCM_H

#include <stdint.h>
#include <stddef.h>

struct adpcm_state {
	int16_t predictor;
	uint8_t step_index;
};

void adpcm_encode(struct adpcm_state *state, const int16_t *pcm, uint8_t *adpcm, size_t sample_count);

void adpcm_decode(struct adpcm_state *state, const uint8_t *adpcm, int16_t *pcm, size_t sample_count);

#endif
