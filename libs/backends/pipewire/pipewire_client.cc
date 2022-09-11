/*
 * Copyright (C) 2019 Robin Gareus <robin@gareus.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <math.h>
#include <regex.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "pipewire_backend.h"
#include "pipewire_client.h"

using namespace ARDOUR;

/*
 * FIXME: might be ok to broadcast done at the end of on_process
 * after all we don't actually
 */
static void on_trigger_done (void *userdata)
{

}

static void on_process (void *userdata)
{
	/*
	 * FIXME: userdata is set during PipewireClient::client_init
	 * we assume here that the PipewireClient is actually a PipewireAudioBackend object
	 * and cast it directly!
	 * 
	 * FIXME: dynamic_cast might be slow?
	 */
	PipewireAudioBackend *backend = static_cast<PipewireAudioBackend*> (userdata);
	assert(backend);
}

static const struct pw_stream_events stream_events = {
        PW_VERSION_STREAM_EVENTS,
        .process = on_process,
};

PipewireClient::PipewireClient ()
	: audio_process_mtex (PTHREAD_MUTEX_INITIALIZER)
	, audio_process_trigger_done (PTHREAD_COND_INITIALIZER)
{
}

int PipewireClient::client_init (std::string &instance_name, int samplerate)
{
	// This buffer is used to build the spa_pod with the audio format info
	// it will then be assigned to PipewireClient::audio_format
	void *buf = malloc(1024);
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, 1024);

	pw_init (1, &&instance_name.c_str());

	loop = pw_main_loop_new(NULL);
	playback_stream = pw_stream_new_simple (pw_main_loop_get_loop (loop),
			"ardour-playback",
			pw_properties_new (
				PW_KEY_MEDIA_TYPE, "Audio",
				PW_KEY_MEDIA_CATEGORY, "Playback",
				PW_KEY_MEDIA_ROLE, "Production",
				NULL),
			&stream_events,
			this);

	audio_format = spa_format_audio_raw_build (&b, SPA_PARAM_EnumFormat,
		&SPA_AUDIO_INFO_RAW_INIT(
			.format = SPA_AUDIO_FORMAT_F32,
			.channels = N_CHANNELS,
			.rate = samplerate));
	
	pw_stream_connect(playback_stream, PW_DIRECTION_OUTPUT,
		PW_ID_ANY,
		PW_STREAM_FLAG_AUTOCONNECT |
		PW_STREAM_FLAG_MAP_BUFFERS |
		PW_STREAM_FLAG_RT_PROCESS |
		PW_STREAM_FLAG_DRIVER, // We will tell the stream when data is ready
		&audio_format, 1);
	
	pw_main_loop_run (loop);
}
