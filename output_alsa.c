/* 
 *  Squeezelite - lightweight headless squeezebox emulator
 *
 *  (c) Adrian Smith 2012-2015, triode1@btinternet.com
 *      Ralph Irving 2015-2023, ralph_irving@hotmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Additions (c) Paul Hermann, 2015-2023 under the same license terms
 *   -Control of Raspberry pi GPIO for amplifier power
 *   -Launch script on power status change from LMS
 */

// Output using Alsa

#define _GNU_SOURCE


#include "squeezelite.h"

#if ALSA

#include <alsa/asoundlib.h>
#include <sys/mman.h>
#include <malloc.h>
#include <math.h>
#include <sched.h>
#include <unistd.h>


#define MAX_DEVICE_LEN 128



static snd_pcm_format_t fmts[] = { SND_PCM_FORMAT_S32_LE, SND_PCM_FORMAT_S24_LE, SND_PCM_FORMAT_S24_3LE, SND_PCM_FORMAT_S16_LE,
								   SND_PCM_FORMAT_UNKNOWN };

#if SL_LITTLE_ENDIAN
#define NATIVE_FORMAT SND_PCM_FORMAT_S32_LE
#else
#define NATIVE_FORMAT SND_PCM_FORMAT_S32_BE
#endif

// output device
static struct {
	char device[MAX_DEVICE_LEN + 1];
	char *ctl;
	char *mixer_ctl;
	snd_pcm_format_t format;
#if DSD
	dsd_format outfmt;
	snd_pcm_format_t pcmfmt;
#endif
	snd_pcm_uframes_t buffer_size;
	snd_pcm_uframes_t period_size;
	unsigned rate;
	bool mmap;
	bool reopen;
	u8_t *write_buf;
	const char *volume_mixer_name;
	bool mixer_linear;
	bool linear_dB_internal;
	bool output_affinity;
	snd_mixer_elem_t* mixer_elem;
	snd_mixer_t *mixer_handle;
	long mixer_min;
	long mixer_max;
	long mixer_mindb;
	long mixer_maxdb;
} alsa;

static snd_pcm_t *pcmp = NULL;

extern u8_t *silencebuf;
#if DSD
extern u8_t *silencebuf_dsd;
#endif

static log_level loglevel;

static bool running = true;

extern struct outputstate output;
extern struct buffer *outputbuf;

#define LOCK   mutex_lock(outputbuf->mutex)
#define UNLOCK mutex_unlock(outputbuf->mutex)

static char *ctl4device(const char *device) {
	char *ctl = NULL;
	
	if (!strncmp(device, "hw:", 3))
		ctl = strdup(device);
	else if (!strncmp(device, "plughw:", 7))
		ctl = strdup(device + 4);

	if (ctl) {
		char *comma;
		if ((comma = strrchr(ctl, ',')))
			*comma = '\0';
	} else
		ctl = strdup(device);

	return ctl;
}

void list_devices(void) {
	void **hints, **n;
	if (snd_device_name_hint(-1, "pcm", &hints) >= 0) {
		n = hints;
		printf("Output devices:\n");
		while (*n) {
			char *name = snd_device_name_get_hint(*n, "NAME");
			char *desc = snd_device_name_get_hint(*n, "DESC");
			if (name) printf("  %-30s", name);
			if (desc) {
				char *s1 = strtok(desc, "\n");
				char *s2 = strtok(NULL, "\n");
				if (s1) printf(" - %s", s1);
				if (s2) printf(" - %s", s2);
			}
			printf("\n");
			if (name) free(name);
			if (desc) free(desc);
			n++;
		}
		snd_device_name_free_hint(hints);
	}
	printf("\n");
}

void list_mixers(const char *output_device) {
	int err;
	snd_mixer_t *handle;
	snd_mixer_selem_id_t *sid;
	snd_mixer_elem_t *elem;
	char *ctl = ctl4device(output_device);
	snd_mixer_selem_id_alloca(&sid);

	LOG_INFO("listing mixers for: %s", output_device);

	if ((err = snd_mixer_open(&handle, 0)) < 0) {
		LOG_ERROR("open error: %s", snd_strerror(err));
		return;
	}
	if ((err = snd_mixer_attach(handle, ctl)) < 0) {
		LOG_ERROR("attach error: %s", snd_strerror(err));
		snd_mixer_close(handle);
		free(ctl);
		return;
	}
	free(ctl);
	if ((err = snd_mixer_selem_register(handle, NULL, NULL)) < 0) {
		LOG_ERROR("register error: %s", snd_strerror(err));
		snd_mixer_close(handle);
		return;
	}
	if ((err = snd_mixer_load(handle)) < 0) {
		LOG_ERROR("load error: %s", snd_strerror(err));
		snd_mixer_close(handle);
		return;
	}

	printf("Volume controls for %s\n", output_device);
	for (elem = snd_mixer_first_elem(handle); elem; elem = snd_mixer_elem_next(elem)) {
		if (snd_mixer_selem_has_playback_volume(elem)) {
			snd_mixer_selem_get_id(elem, sid);
			printf("   %s", snd_mixer_selem_id_get_name(sid));
			if (snd_mixer_selem_id_get_index(sid)) {
				printf(",%d", snd_mixer_selem_id_get_index(sid));
			}
			printf("\n");
		}
	}

	snd_mixer_close(handle);
}

#define MINVOL_DB -72        // LMS volume map for SqueezePlay sends values in range ~ -72..0 dB
		
static void set_mixer(bool setmax, bool setmin, long ldb, long rdb) {
	int err;
	long nldb, nrdb;

	if (ldb < MINVOL_DB || ldb == SND_CTL_TLV_DB_GAIN_MUTE) 
		setmin = 1;

	if (setmax) 
		ldb = rdb = alsa.mixer_maxdb / 100;
	if (setmin) 
		ldb = rdb = alsa.mixer_mindb / 100;

	//FIXME: mute setting -999999.99 at mixer_mindb - currently covered by setmin.
	//FIXME: if (alsa.mixer_mindb == SND_CTL_TLV_DB_GAIN_MUTE) {


	LOG_DEBUG("(VC) external set: setmax: %d setmin: %d", setmax, setmin);
	LOG_DEBUG("(VC) external set level in dB: %s, l: %ld r: %ld", alsa.volume_mixer_name, ldb, rdb);
	
	snd_mixer_handle_events(alsa.mixer_handle); 

	if ((err = snd_mixer_selem_set_playback_dB_all(alsa.mixer_elem, 100 * ldb, 0)) < 0) {
			LOG_ERROR("error setting left volume(dB): %s", snd_strerror(err));
	}

// function calls not needed if not in debug mode
#ifdef LOG_DEBUG

	snd_mixer_handle_events(alsa.mixer_handle); 
	if ((err = snd_mixer_selem_get_playback_dB(alsa.mixer_elem, SND_MIXER_SCHN_FRONT_LEFT, &nldb)) < 0) {
		LOG_ERROR("error getting left vol dB: %s", snd_strerror(err));
	}
	snd_mixer_handle_events(alsa.mixer_handle); 
	if ((err = snd_mixer_selem_get_playback_dB(alsa.mixer_elem, SND_MIXER_SCHN_FRONT_RIGHT, &nrdb)) < 0) {
		LOG_ERROR("error getting right vol dB: %s", snd_strerror(err));
	}
	
	LOG_DEBUG("(VC) external get level in dB: %s, l: %ld r: %ld", alsa.volume_mixer_name, nldb / 100 , nrdb / 100);

#endif


}


void set_volume(unsigned left, unsigned right) {
	long l = 0;
	long r = 0;
	int i;
	bool setmin = 0;
	bool setmax = 0;

	const long gscale[101] = { 65536, 61952, 58624, 55296, 52224, 49408, 46592, 44032, 41728, 39424,
                                    37120, 35072, 33024, 31232, 29696, 27904, 26368, 24832, 23552, 22272,
                                    20992, 19968, 18688, 17664, 16640, 15872, 14848, 14080, 13312, 12544,
                                    12032, 11264, 10752,  9984,  9472,  8960,  8448,  7936,  7680,  7168,
                                     6656,  6400,  6144,  5632,  5376,  5120,  4864,  4608,  4352,  4096,
                                     3840,  3584,  3328, 3328,  3072,  2816, 2816,  2560,  2304, 2304, 
                                     2048,  2050,  1937,  1830,  1729,  1634,  1543,  1458,  1378,  1302,
                                     1230,  1162,  1098,  1037,   980,   926,   781,   658,   555,   468,
                                      395,   333,   281,   237,   200,   168,   142,   120,   101,    85,
                                       72,    61,    51,    43,    36,    31,    26,    22,    18,    16,
                                        0, };

	// gscale consists of 101 values reflecting the full LMS volume control scale
	// it reflects the LMS special tailored logarithmic VC curve, see LMS squeezeplayer2 player sources
	// the numbers are supplied by LMS through slimproto (left/right). 
	// the 1st field equals 100, 2nd = 99, 3rd = ....  101st = 0 on the VC 100->0 scale
	// LMS issue 1: non lineraties and ambigeous values 48/47=3328 45/44=2816 42/41=2304 
	// LMS issue 2: Swapped order by LMS: 40=2048, 39=2050 
	// Workaround: fields 2048/2050 swapped in array
	// Troubletickets on LMS have been issues
	// Not tested yet: 
	// replay gain and crossfade - & applicable at all? 
	// unbalanced channel gain - applicable at all? 

	const long gscalenew[101] = { 65536,58409,52057,46396,41350,36854,32768,29274,26090,
                                  23253,20724,18471,16384,14672,13076,11654,10387,9257,8192,
                                  7353,6554,5841,5206,4640,4096,3685,3285,2927,2609,2325,
                                  2048,1847,1646,1467,1308,1165,1024,926,825,735,655,
                                  584,512,464,414,369,328,293,256,233,207,185,165,147,128,
                                  117,104,93,83,74,64,58,52,46,41,37,32,29,26,23,21,18,16,
                                  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, };


	// SL internal software volume control
	if (!alsa.volume_mixer_name) {
		LOG_DEBUG("(VC) internal level: l: %u r: %u", left, right);

		/////////////////////////////////////////////////////////
		//new internal VC mapping start
		/////////////////////////////////////////////////////////
		if (alsa.linear_dB_internal) {
			
			int lower = 0;
			
			if (left > 0) {
				
				for(i = 0; i < sizeof(gscale) / sizeof(gscale[0]); i++) 
				{
					if (gscale[i] < left) 
					{
						lower = i;
						break;
					}
				}
			l = gscalenew[lower - 1];
			r = gscalenew[lower - 1];
			
			LOG_DEBUG("(VC) new mapping - internal level: l: %u r: %u", l, r);
			LOG_DEBUG("(VC) new mapping - internal position: %u", lower - 1 );
			}
			
		} else {
			
			l = left;
			r = right;
			
		}
		/////////////////////////////////////////////////////
		//new internal VC mapping end
		/////////////////////////////////////////////////////

		LOCK;
		output.gainL = l;
		output.gainR = r;
		UNLOCK;
		return;
	} else {
		LOCK;
		output.gainL = FIXED_ONE;
		output.gainR = FIXED_ONE;
		UNLOCK;
	}

    if ( ! alsa.mixer_linear) {
		if (left > 0) {
			// convert 16.16 fixed point to dB
			l = r = floor(20 * log10( left  / 65536.0F ));
		} else {
			setmin = 1;
			l = r = alsa.mixer_mindb / 100;
		}
		LOG_DEBUG("(VC) external: non-linear mode active");
	} else {
		// HW volume control in db mapped against LMS input
		// find equal or nearest value in gscale array
		int low = 0;
		if (left > 0) {
			for(i = 0; i < sizeof(gscale) / sizeof(gscale[0]); i++) {
				if (gscale[i] < left) {
					low = i;
					break;
				}
			}
		l = r = (gscale[low - 1] - left ) >= ( left - gscale[low])  ? (-1) * low : (-1) * (low - 1);
		} else {
			setmin = 1;
			l = r = alsa.mixer_mindb / 100;
		}
		LOG_DEBUG("(VC) external: linear mode active");
	}

	LOG_DEBUG("(VC) external level in dB: l: %ld r: %ld", l, r);

	set_mixer(setmax, setmin, l, r);
}

static void *alsa_error_handler(const char *file, int line, const char *function, int err, const char *fmt, ...) {
	va_list args;
	if ((loglevel >= lINFO && err == 0) || loglevel >= lDEBUG) {
		fprintf(stderr, "%s ALSA %s:%d ", logtime(), function, line);
		va_start(args, fmt);
		vfprintf(stderr, fmt, args);
		fprintf(stderr, "\n");
		fflush(stderr);
	}
	return NULL;
}

static void alsa_close(void) {
	int err;
	if ((err = snd_pcm_close(pcmp)) < 0) {
		LOG_INFO("snd_pcm_close error: %s", snd_strerror(err));
	}
}

bool test_open(const char *device, unsigned rates[], bool userdef_rates) {
	int err;
	snd_pcm_t *pcm;
	snd_pcm_hw_params_t *hw_params;
	hw_params = (snd_pcm_hw_params_t *) alloca(snd_pcm_hw_params_sizeof());
	memset(hw_params, 0, snd_pcm_hw_params_sizeof());

	// open device
	if ((err = snd_pcm_open(&pcm, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
		LOG_ERROR("playback open error: %s", snd_strerror(err));
		return false;
	}

	// get max params
	if ((err = snd_pcm_hw_params_any(pcm, hw_params)) < 0) {
		LOG_ERROR("hwparam init error: %s", snd_strerror(err));
		return false;
	}

	// find supported sample rates to enable client side resampling of non supported rates
	if (!userdef_rates) {
		unsigned i, ind;
		unsigned ref[] TEST_RATES;

		for (i = 0, ind = 0; ref[i]; ++i) {
			if (snd_pcm_hw_params_test_rate(pcm, hw_params, ref[i], 0) == 0) {
				rates[ind++] = ref[i];
			}
			else {
				LOG_DEBUG("sample rate %u not supported", ref[i]);
			}
		}
	}

	if ((err = snd_pcm_close(pcm)) < 0) {
		LOG_ERROR("snd_pcm_close error: %s", snd_strerror(err));
		return false;
	}

	return true;
}

static bool pcm_probe(const char *device) {
	int err;
	snd_pcm_t *pcm;

	if ((err = snd_pcm_open(&pcm, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
		return false;
	}

	if ((err = snd_pcm_close(pcm)) < 0) {
		LOG_ERROR("snd_pcm_close error: %s", snd_strerror(err));
	}

	return true;
}

#if DSD
static int alsa_open(const char *device, unsigned sample_rate, unsigned alsa_buffer, unsigned alsa_period, dsd_format outfmt) {
#else
static int alsa_open(const char *device, unsigned sample_rate, unsigned alsa_buffer, unsigned alsa_period) {
#endif
	int err;
	snd_pcm_hw_params_t *hw_params;
	snd_pcm_hw_params_alloca(&hw_params);

	// close if already open
	if (pcmp) alsa_close();

	// reset params
	alsa.rate = 0;
#if DSD
	alsa.outfmt = PCM;
#endif
	alsa.period_size = 0;
	strcpy(alsa.device, device);

	if (strlen(device) > MAX_DEVICE_LEN - 4 - 1) {
		LOG_ERROR("device name too long: %s", device);
		return -1;
	}

	LOG_INFO("opening device at: %u", sample_rate);

	bool retry;
	do {
		// open device
		if ((err = snd_pcm_open(&pcmp, alsa.device, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
			LOG_ERROR("playback open error: %s", snd_strerror(err));
			return err;
		}

		// init params
		memset(hw_params, 0, snd_pcm_hw_params_sizeof());
		if ((err = snd_pcm_hw_params_any(pcmp, hw_params)) < 0) {
			LOG_ERROR("hwparam init error: %s", snd_strerror(err));
			return err;
		}

		// open hw: devices without resampling, if sample rate fails try plughw: with resampling
		bool hw = !strncmp(alsa.device, "hw:", 3);
		retry = false;

		if ((err = snd_pcm_hw_params_set_rate_resample(pcmp, hw_params, !hw)) < 0) {
			LOG_ERROR("resampling setup failed: %s", snd_strerror(err));
			return err;
		}

		if ((err = snd_pcm_hw_params_set_rate(pcmp, hw_params, sample_rate, 0)) < 0) {
			if (hw) {
				strcpy(alsa.device + 4, device);
				memcpy(alsa.device, "plug", 4);
				LOG_INFO("reopening device %s in plug mode as %s for resampling", device, alsa.device);
				snd_pcm_close(pcmp);
				retry = true;
			}
		}

	} while (retry);

	// set access 
	if (!alsa.mmap || snd_pcm_hw_params_set_access(pcmp, hw_params, SND_PCM_ACCESS_MMAP_INTERLEAVED) < 0) {
		if ((err = snd_pcm_hw_params_set_access(pcmp, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
			LOG_ERROR("access type not available: %s", snd_strerror(err));
			return err;
		}
		alsa.mmap = false;
	}

	// set the sample format
#if DSD
	switch (outfmt) {
	case DSD_U8:
		alsa.format = SND_PCM_FORMAT_DSD_U8; break;
	case DSD_U16_LE:
		alsa.format = SND_PCM_FORMAT_DSD_U16_LE; break;
	case DSD_U16_BE:
		alsa.format = SND_PCM_FORMAT_DSD_U16_BE; break;
	case DSD_U32_LE:
		alsa.format = SND_PCM_FORMAT_DSD_U32_LE; break;
	case DSD_U32_BE:
		alsa.format = SND_PCM_FORMAT_DSD_U32_BE; break;
	case DOP_S24_LE:
		alsa.format = SND_PCM_FORMAT_S24_LE; break;
	case DOP_S24_3LE:
		alsa.format = SND_PCM_FORMAT_S24_3LE; break;
	default:
		alsa.format = alsa.pcmfmt;
	}
#endif
	snd_pcm_format_t *fmt = alsa.format ? &alsa.format : (snd_pcm_format_t *)fmts;
	do {
		if (snd_pcm_hw_params_set_format(pcmp, hw_params, *fmt) >= 0) {
			LOG_INFO("opened device %s using format: %s sample rate: %u mmap: %u", alsa.device, snd_pcm_format_name(*fmt), sample_rate, alsa.mmap);
			alsa.format = *fmt;
			break;
		}
		if (alsa.format) {
			LOG_ERROR("unable to open audio device requested format: %s", snd_pcm_format_name(alsa.format));
			return -1;
		}
		++fmt; 
		if (*fmt == SND_PCM_FORMAT_UNKNOWN) {
			LOG_ERROR("unable to open audio device with any supported format");
			return -1;
		}
	} while (*fmt != SND_PCM_FORMAT_UNKNOWN);

	// set the output format to be used by _scale_and_pack
	switch(alsa.format) {
	case SND_PCM_FORMAT_S32_LE:
		output.format = S32_LE; break;
	case SND_PCM_FORMAT_S24_LE: 
		output.format = S24_LE; break;
	case SND_PCM_FORMAT_S24_3LE:
		output.format = S24_3LE; break;
	case SND_PCM_FORMAT_S16_LE: 
		output.format = S16_LE; break;
#if DSD
	case SND_PCM_FORMAT_DSD_U32_LE:
		output.format = U32_LE; break;
	case SND_PCM_FORMAT_DSD_U32_BE:
		output.format = U32_BE; break;
	case SND_PCM_FORMAT_DSD_U16_LE:
		output.format = U16_LE; break;
	case SND_PCM_FORMAT_DSD_U16_BE:
		output.format = U16_BE; break;
	case SND_PCM_FORMAT_DSD_U8:
		output.format = U8; break;
#endif
	default: 
		break;
	}

	// set channels
	if ((err = snd_pcm_hw_params_set_channels (pcmp, hw_params, 2)) < 0) {
		LOG_ERROR("channel count not available: %s", snd_strerror(err));
		return err;
	}

	// set period size - value of < 50 treated as period count, otherwise size in bytes
	if (alsa_period < 50) {
		unsigned count = alsa_period;
		if ((err = snd_pcm_hw_params_set_periods_near(pcmp, hw_params, &count, 0)) < 0) {
			LOG_ERROR("unable to set period count %s", snd_strerror(err));
			return err;
		}
	} else {
		snd_pcm_uframes_t size = alsa_period;
		int dir = 0;
		if ((err = snd_pcm_hw_params_set_period_size_near(pcmp, hw_params, &size, &dir)) < 0) {
			LOG_ERROR("unable to set period size %s", snd_strerror(err));
			return err;
		}
	}

	// set buffer size - value of < 500 treated as buffer time in ms, otherwise size in bytes
	if (alsa_buffer < 500) {
		unsigned time = alsa_buffer * 1000;
		int dir = 0;
		if ((err = snd_pcm_hw_params_set_buffer_time_near(pcmp, hw_params, &time, &dir)) < 0) {
			LOG_ERROR("unable to set buffer time %s", snd_strerror(err));
			return err;
		}
	} else {
		snd_pcm_uframes_t size = alsa_buffer;
		if ((err = snd_pcm_hw_params_set_buffer_size_near(pcmp, hw_params, &size)) < 0) {
			LOG_ERROR("unable to set buffer size %s", snd_strerror(err));
			return err;
		}
	}

	// get period_size
	if ((err = snd_pcm_hw_params_get_period_size(hw_params, &alsa.period_size, 0)) < 0) {
		LOG_ERROR("unable to get period size: %s", snd_strerror(err));
		return err;
	}

	// get buffer_size
	if ((err = snd_pcm_hw_params_get_buffer_size(hw_params, &alsa.buffer_size)) < 0) {
		LOG_ERROR("unable to get buffer size: %s", snd_strerror(err));
		return err;
	}

	LOG_INFO("buffer: %u period: %u -> buffer size: %u period size: %u", alsa_buffer, alsa_period, alsa.buffer_size, alsa.period_size);

	// ensure we have two buffer sizes of samples before starting output
	output.start_frames = alsa.buffer_size * 2;

	// create an intermediate buffer for non mmap case for all but NATIVE_FORMAT
	// this is used to pack samples into the output format before calling writei
	if (!alsa.mmap && !alsa.write_buf && alsa.format != NATIVE_FORMAT) {
		alsa.write_buf = malloc(alsa.buffer_size * BYTES_PER_FRAME);
		if (!alsa.write_buf) {
			LOG_ERROR("unable to malloc write_buf");
			return -1;
		}
	}

	// set params
	if ((err = snd_pcm_hw_params(pcmp, hw_params)) < 0) {
		LOG_ERROR("unable to set hw params: %s", snd_strerror(err));
		return err;
	}

	// dump info
	if (loglevel == lSDEBUG) {
		static snd_output_t *debug_output;
		snd_output_stdio_attach(&debug_output, stderr, 0);
		snd_pcm_dump(pcmp, debug_output);
	}

	// this indicates we have opened the device ok
	alsa.rate = sample_rate;
#if DSD
	alsa.outfmt = outfmt;
#endif
	
	return 0;
}

static int _write_frames(frames_t out_frames, bool silence, s32_t gainL, s32_t gainR, u8_t flags,
						 s32_t cross_gain_in, s32_t cross_gain_out, s32_t **cross_ptr) {

	const snd_pcm_channel_area_t *areas;
	snd_pcm_uframes_t offset;
	void  *outputptr;
	s32_t *inputptr;
	int err;

	if (alsa.mmap) {
		snd_pcm_uframes_t alsa_frames = (snd_pcm_uframes_t)out_frames;
		
		snd_pcm_avail_update(pcmp);
		
		if ((err = snd_pcm_mmap_begin(pcmp, &areas, &offset, &alsa_frames)) < 0) {
			LOG_WARN("error from mmap_begin: %s", snd_strerror(err));
			return -1;
		}
		
		out_frames = (frames_t)alsa_frames;
	}

	if (!silence) {
		// applying cross fade is delayed until this point as mmap_begin can change out_frames
		if (output.fade == FADE_ACTIVE && output.fade_dir == FADE_CROSS && *cross_ptr) {
			_apply_cross(outputbuf, out_frames, cross_gain_in, cross_gain_out, cross_ptr);
		}
	}

	inputptr = (s32_t *) (silence ? silencebuf : outputbuf->readp);

	IF_DSD(
		if (output.outfmt != PCM) {
			if (silence) {
				inputptr = (s32_t *) silencebuf_dsd;
			}
			if (output.outfmt == DOP || output.outfmt == DOP_S24_LE || output.outfmt == DOP_S24_3LE)
				update_dop((u32_t *) inputptr, out_frames, output.invert && !silence);
			else if (output.invert && !silence)
				dsd_invert((u32_t *) inputptr, out_frames);
		}
	)

	if (alsa.mmap || alsa.format != NATIVE_FORMAT) {

		outputptr = alsa.mmap ? (areas[0].addr + (areas[0].first + offset * areas[0].step) / 8) : alsa.write_buf;

		_scale_and_pack_frames(outputptr, inputptr, out_frames, gainL, gainR, flags, output.format);

	} else {

		outputptr = (void *)inputptr;

		if (!silence) {

			if (gainL != FIXED_ONE || gainR != FIXED_ONE) {
				_apply_gain(outputbuf, out_frames, gainL, gainR, flags);
			}
		}
	}

	if (alsa.mmap) {

		snd_pcm_sframes_t w = snd_pcm_mmap_commit(pcmp, offset, out_frames);
		if (w < 0 || w != out_frames) {
			LOG_WARN("mmap_commit error");
			return -1;
		}

	} else {

		snd_pcm_sframes_t w = snd_pcm_writei(pcmp, outputptr, out_frames);
		if (w < 0) {
			//if (w != -EAGAIN && ((err = snd_pcm_recover(pcmp, w, 1)) < 0)) {
			if (((err = snd_pcm_recover(pcmp, w, 1)) < 0)) {
				static unsigned recover_count = 0;
				LOG_WARN("recover failed: %s [%u]", snd_strerror(err), ++recover_count);
				if (recover_count >= 10) {				
					recover_count = 0;
					alsa_close();
					pcmp = NULL;
				}
			}
			return -1;
		} else {
			if (w != out_frames) {
				LOG_WARN("writei only wrote %u of %u", w, out_frames);
			}						
			out_frames = w;
		}
	}

	return (int)out_frames;
}

static void *output_thread(void *arg) {
	bool start = true;
	bool output_off = (output.state == OUTPUT_OFF);
	bool probe_device = (arg != NULL);
	int err;
	const char* threadname = "output_alsa\0";

	
	while (running) {
		
		if (prctl(PR_SET_NAME, (unsigned long) threadname) != 0) {
			LOG_DEBUG("setting threadname failed: %s", strerror(errno));
		}

		// disabled output - player is off
		while (output_off) {
			usleep(100000);
			LOCK;
			output_off = (output.state == OUTPUT_OFF);
			UNLOCK;
			if (!running) return 0;
		}

		// wait until device returns - to allow usb audio devices to be turned off
		if (probe_device) {
			while (!pcm_probe(output.device)) {
				LOG_DEBUG("waiting for device %s to return", output.device);
				sleep(5);
			}
			probe_device = false;
		}
#if DSD
		if (!pcmp || alsa.rate != output.current_sample_rate || alsa.outfmt != output.outfmt ) {
#else

		if (!pcmp || alsa.rate != output.current_sample_rate) {
#endif
#if GPIO
			// Wake up amp
			if (gpio_active) { 
				relay(1);
			}
			if (power_script != NULL) {
				relay_script(1);
			}
#endif
			LOG_INFO("open output device: %s", output.device);
			LOCK;

			// FIXME - some alsa hardware requires opening twice for a new sample rate to work
			// this is a workaround which should be removed
			if (alsa.reopen) {
#if DSD
				alsa_open(output.device, output.current_sample_rate, output.buffer, output.period, output.outfmt);
#else
				alsa_open(output.device, output.current_sample_rate, output.buffer, output.period);
#endif
			}
#if DSD
			if (!!alsa_open(output.device, output.current_sample_rate, output.buffer, output.period, output.outfmt)) {
#else
			if (!!alsa_open(output.device, output.current_sample_rate, output.buffer, output.period)) {
#endif
				output.error_opening = true;
				UNLOCK;
				sleep(5);
				continue;
			}
			output.error_opening = false;
			start = true;
			UNLOCK;
		}

		snd_pcm_state_t state = snd_pcm_state(pcmp);

		if (state == SND_PCM_STATE_XRUN) {
			LOG_INFO("XRUN");
			if ((err = snd_pcm_recover(pcmp, -EPIPE, 1)) < 0) {
				LOG_INFO("XRUN recover failed: %s", snd_strerror(err));
				usleep(10000);
			}
			start = true;
			continue;
		} else if (state == SND_PCM_STATE_SUSPENDED) {
			if ((err = snd_pcm_recover(pcmp, -ESTRPIPE, 1)) < 0) {
				LOG_INFO("SUSPEND recover failed: %s", snd_strerror(err));
			}
		} else if (state == SND_PCM_STATE_DISCONNECTED) {
			LOG_INFO("Device %s no longer available", output.device);
			alsa_close();
			pcmp = NULL;
			probe_device = true;
			continue;
		}

		snd_pcm_sframes_t avail = snd_pcm_avail_update(pcmp);

		if (avail < 0) {
			if ((err = snd_pcm_recover(pcmp, avail, 1)) < 0) {
				if (err == -ENODEV) {
					LOG_INFO("Device %s no longer available", output.device);
					alsa_close();
					pcmp = NULL;
					probe_device = true;
					continue;
				}
				LOG_WARN("recover failed: %s", snd_strerror(err));
			}
			start = true;
			continue;
		}

		if (avail < alsa.period_size) {
			if (start) {
				if (alsa.mmap && ((err = snd_pcm_start(pcmp)) < 0)) {
					if ((err = snd_pcm_recover(pcmp, err, 1)) < 0) {
						if (err == -ENODEV) {
							LOG_INFO("Device %s no longer available", output.device);
							alsa_close();
							pcmp = NULL;
							probe_device = true;
							continue;
						}
						LOG_INFO("start error: %s", snd_strerror(err));
						usleep(10000);
					}
				} else {
					start = false;
				}
			} else {
				usleep(10000);
				if ((err = snd_pcm_wait(pcmp, 1000)) <= 0) {
					if ( err == 0 ) {
						LOG_INFO("pcm wait timeout");
					}
					if ((err = snd_pcm_recover(pcmp, err, 1)) < 0) {
						LOG_INFO("pcm wait error: %s", snd_strerror(err));
					}
					start = true;
				}
			}
			continue;
		}

		// restrict avail to within sensible limits as alsa drivers can return erroneous large values
		// in writei mode restrict to period_size due to size of write_buf
		if (alsa.mmap) {
			avail = min(avail, alsa.buffer_size);
		} else {
			avail = min(avail, alsa.period_size);
		}

		// avoid spinning in cases where wait returns but no bytes available (seen with pulse audio)
		if (avail == 0) {
			LOG_SDEBUG("avail 0 - sleeping");
			usleep(10000);
			continue;
		}

		LOCK;

		// turn off if requested
		if (output.state == OUTPUT_OFF) {
			UNLOCK;
			LOG_INFO("disabling output");
			alsa_close();
			pcmp = NULL;
			output_off = true;
			vis_stop();
#if GPIO
			//  Put Amp to Sleep
			if (gpio_active){
				relay(0);
			}
			if (power_script != NULL ){
				relay_script(0);
			}
#endif
			continue;
		}

		// measure output delay
		snd_pcm_sframes_t delay;
		if ((err = snd_pcm_delay(pcmp, &delay)) < 0) {
			if (err == -EPIPE) {
				// EPIPE indicates underrun - attempt to recover
				UNLOCK;
				continue;
			} else if (err == -EIO) {
				// EIO can occur with non existant pulse server
				UNLOCK;
				LOG_SDEBUG("snd_pcm_delay returns: EIO - sleeping");
				usleep(100000);
				continue;
			} else {
				LOG_DEBUG("snd_pcm_delay returns: %d", err);
			}
		} else {
			output.device_frames = delay;
			output.updated = gettime_ms();
			output.frames_played_dmp = output.frames_played;
		}

		// process frames
		frames_t wrote = _output_frames(avail);

		UNLOCK;

		// some output devices such as alsa null refuse any data, avoid spinning
		if (!wrote) {
			LOG_SDEBUG("wrote 0 - sleeping");
			usleep(10000);
		}
	}

	return 0;
}

int mixer_init_alsa(const char *device, const char *mixer, int mixer_index) {
	int err;
	snd_mixer_selem_id_t *sid;

	if ((err = snd_mixer_open(&alsa.mixer_handle, 0)) < 0) {
		LOG_ERROR("open error: %s", snd_strerror(err));
		return -1;
	}
	if ((err = snd_mixer_attach(alsa.mixer_handle, device)) < 0) {
		LOG_ERROR("attach error: %s", snd_strerror(err));
		snd_mixer_close(alsa.mixer_handle);
		return -1;
	}
	if ((err = snd_mixer_selem_register(alsa.mixer_handle, NULL, NULL)) < 0) {
		LOG_ERROR("register error: %s", snd_strerror(err));
		snd_mixer_close(alsa.mixer_handle);
		return -1;
	}
	if ((err = snd_mixer_load(alsa.mixer_handle)) < 0) {
		LOG_ERROR("load error: %s", snd_strerror(err));
		snd_mixer_close(alsa.mixer_handle);
		return -1;
	}

	snd_mixer_selem_id_alloca(&sid);
	snd_mixer_selem_id_set_index(sid, mixer_index);
	snd_mixer_selem_id_set_name(sid, mixer);

	if ((alsa.mixer_elem = snd_mixer_find_selem(alsa.mixer_handle, sid)) == NULL) {
		LOG_ERROR("error find selem %s", alsa.mixer_handle);
		snd_mixer_close(alsa.mixer_handle);
		return -1;
	}

	snd_mixer_handle_events(alsa.mixer_handle); 


	if (snd_mixer_selem_has_playback_switch(alsa.mixer_elem)) {
		snd_mixer_selem_set_playback_switch_all(alsa.mixer_elem, 1); // unmute
	}
	
	snd_mixer_handle_events(alsa.mixer_handle); 


	if ((err = snd_mixer_selem_get_playback_dB_range(alsa.mixer_elem, &alsa.mixer_mindb, &alsa.mixer_maxdb)) < 0 ) {
			LOG_ERROR("Unable to get volume dB range");
			return -1;
	}

	LOG_DEBUG("(VC) external - range in db: min: %ld max: %ld", alsa.mixer_mindb / 100, alsa.mixer_maxdb / 100);

	return 0;
}

static pthread_t thread;

void output_init_alsa(log_level level, const char *device, unsigned output_buf_size, char *params, unsigned rates[], 
						unsigned rate_delay, unsigned rt_priority, unsigned idle, char *mixer_device, char *volume_mixer, 
						bool mixer_unmute, bool mixer_linear, bool linear_dB_internal, bool output_affinity) {

	unsigned alsa_buffer = ALSA_BUFFER_TIME;
	unsigned alsa_period = ALSA_PERIOD_COUNT;
	char *alsa_sample_fmt = NULL;
	bool alsa_mmap = true;
	bool alsa_reopen = false;
	int err; 

	char *volume_mixer_name = next_param(volume_mixer, ',');
	char *volume_mixer_index = next_param(NULL, ',');

	char *t = next_param(params, ':');
	char *c = next_param(NULL, ':');
	char *s = next_param(NULL, ':');
	char *m = next_param(NULL, ':');
	char *r = next_param(NULL, ':');

	if (t) alsa_buffer = atoi(t);
	if (c) alsa_period = atoi(c);
	if (s) alsa_sample_fmt = s;
	if (m) alsa_mmap = atoi(m);
	if (r) alsa_reopen = atoi(r);

	loglevel = level;

	LOG_INFO("init output");

	memset(&output, 0, sizeof(output));

	alsa.mmap = alsa_mmap;
	alsa.write_buf = NULL;
#if DSD
	alsa.pcmfmt = 0;
#else
	alsa.format = 0;
#endif
	alsa.reopen = alsa_reopen;
	alsa.mixer_handle = NULL;
	alsa.ctl = ctl4device(device);
	alsa.mixer_ctl = mixer_device ? ctl4device(mixer_device) : alsa.ctl;
	alsa.volume_mixer_name = volume_mixer_name;
	alsa.mixer_linear = mixer_linear;
	alsa.linear_dB_internal = linear_dB_internal;
	alsa.output_affinity = output_affinity;

	output.format = 0;
	output.buffer = alsa_buffer;
	output.period = alsa_period;
	output.start_frames = 0;
	output.write_cb = &_write_frames;
	output.rate_delay = rate_delay;

	if (alsa_sample_fmt) {
#if DSD
		if (!strcmp(alsa_sample_fmt, "32"))	alsa.pcmfmt = SND_PCM_FORMAT_S32_LE;
		if (!strcmp(alsa_sample_fmt, "24")) alsa.pcmfmt = SND_PCM_FORMAT_S24_LE;
		if (!strcmp(alsa_sample_fmt, "24_3")) alsa.pcmfmt = SND_PCM_FORMAT_S24_3LE;
		if (!strcmp(alsa_sample_fmt, "16")) alsa.pcmfmt = SND_PCM_FORMAT_S16_LE;
#else
		if (!strcmp(alsa_sample_fmt, "32"))	alsa.format = SND_PCM_FORMAT_S32_LE;
		if (!strcmp(alsa_sample_fmt, "24")) alsa.format = SND_PCM_FORMAT_S24_LE;
		if (!strcmp(alsa_sample_fmt, "24_3")) alsa.format = SND_PCM_FORMAT_S24_3LE;
		if (!strcmp(alsa_sample_fmt, "16")) alsa.format = SND_PCM_FORMAT_S16_LE;
#endif
	}

	LOG_INFO("requested alsa_buffer: %u alsa_period: %u format: %s mmap: %u", output.buffer, output.period, 
			 alsa_sample_fmt ? alsa_sample_fmt : "any", alsa.mmap);

	snd_lib_error_set_handler((snd_lib_error_handler_t)alsa_error_handler);

	output_init_common(level, device, output_buf_size, rates, idle);
	
	if (volume_mixer_name) {
	        if (mixer_init_alsa(alsa.mixer_ctl, alsa.volume_mixer_name, volume_mixer_index ?
			atoi(volume_mixer_index) : 0) < 0)
		{
			LOG_ERROR("Initialization of mixer failed, reverting to software volume");
			alsa.mixer_handle = NULL;
			alsa.volume_mixer_name = NULL;
		}
	}
	if (mixer_unmute && alsa.volume_mixer_name) {
		set_mixer(1, 0, 0, 0);
		alsa.volume_mixer_name = NULL;
	}

#if LINUX
	// RT linux - aim to avoid pagefaults by locking memory: 
	// https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
		LOG_INFO("(rt) - unable to lock memory: %s", strerror(errno));
	} else {
		LOG_INFO("(rt) - memory locked");
	}

#ifdef __GLIBC__
	//turn off malloc trimming
	mallopt(M_TRIM_THRESHOLD, -1);
	//turn off mmap usuage
	mallopt(M_MMAP_MAX, 0);
	LOG_INFO("(rt) - glibc detected using mallopt");
#endif

	touch_memory(silencebuf, MAX_SILENCE_FRAMES * BYTES_PER_FRAME);
	touch_memory(outputbuf->buf, outputbuf->size);
#endif

	// start output thread
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + OUTPUT_THREAD_STACK_SIZE);
	pthread_create(&thread, &attr, output_thread, rates[0] ? "probe" : NULL);
	pthread_attr_destroy(&attr);

	// try to set this thread to real-time scheduler class, only works as root or if user has permission
	int policy = SCHED_FIFO;
	struct sched_param param;
	param.sched_priority = rt_priority;

	if ((err = pthread_setschedparam(thread, policy, &param)) < 0) {
		LOG_DEBUG("unable to set sched params %s", strerror(errno));
	}
	
	if (alsa.output_affinity) {
	
		cpu_set_t mask;
		ulong ncores = sysconf(_SC_NPROCESSORS_CONF);

		LOG_DEBUG("number of processors %d", ncores);

		//clears the actual mask
		CPU_ZERO(&mask);

		// assign output thread to last CPU = ncores - 1
		CPU_SET( ncores - 1 , &mask);

		LOG_DEBUG("affinity assigned to processor %d", ncores - 1);

	
		if ((err = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &mask)) < 0) {
			LOG_DEBUG("unable to set thread affinity %s", strerror(errno));
			
		CPU_FREE(&mask);
		}
	}



#ifdef LOG_DEBUG

	const char* threadname = "output_alsa";

	if ((err = pthread_getschedparam(thread, &policy, &param)) < 0) { 
		LOG_DEBUG("unable to get sched params %s", strerror(errno));
	}

	LOG_DEBUG("%s: pthread: sched - policy: %s, priority: %u", threadname,
										(policy == SCHED_FIFO)  ? "SCHED_FIFO" :
										(policy == SCHED_RR)    ? "SCHED_RR" :
										(policy == SCHED_OTHER) ? "SCHED_OTHER" :
										"???",
										param.sched_priority);

#endif

}

void output_close_alsa(void) {
	LOG_INFO("close output");

	LOCK;
	running = false;
	UNLOCK;

	pthread_join(thread, NULL);

	if (alsa.write_buf) free(alsa.write_buf);
	if (alsa.ctl) free(alsa.ctl);
	if (alsa.mixer_ctl) free(alsa.mixer_ctl);
	if (alsa.mixer_handle != NULL) snd_mixer_close(alsa.mixer_handle);

	output_close_common();
}

#endif // ALSA

