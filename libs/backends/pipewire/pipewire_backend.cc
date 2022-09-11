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

#include <glibmm.h>

#include "pbd/compose.h"
#include "pbd/error.h"
#include "pbd/file_utils.h"
#include "pbd/pthread_utils.h"

#include "ardour/port_manager.h"

#include "pipewire_backend.h"
#include "pipewire_client.h"

#include "pbd/i18n.h"

using namespace ARDOUR;

static std::string s_instance_name;

const size_t PipewireAudioBackend::_max_buffer_size = 8192;

PipewireAudioBackend::PipewireAudioBackend (AudioEngine& e, AudioBackendInfo& info)
	: AudioBackend (e, info)
	, PortEngineSharedImpl (e, s_instance_name)
	, PipewireClient ()
	, p_stream (0)
	, p_context (0)
	, p_mainloop (0)
	, _run (false)
	, _active (false)
	, _freewheel (false)
	, _freewheeling (false)
	, _last_process_start (0)
	, _samplerate (48000)
	, _samples_per_period (1024)
	, _systemic_audio_output_latency (0)
	, _dsp_load (0)
	, _processed_samples (0)
{
	_instance_name = s_instance_name;
}

static void*
pthread_process (void* arg)
{
	PipewireAudioBackend* d = static_cast<PipewireAudioBackend*> (arg);
	d->main_process_thread ();
	pthread_exit (0);
	return NULL;
}

int
PipewireAudioBackend::init_pipewire ()
{
	client_init (s_instance_name);
}

int
PipewireAudioBackend::_start (bool for_latency_measurement)
{
	if (!_active && _run) {
		PBD::error << _("PipewireAudioBackend: already active.") << endmsg;
		/* recover from 'halted', reap threads */
		stop ();
	}

	if (_active || _run) {
		PBD::info << _("PipewireAudioBackend: already active.") << endmsg;
		return BackendReinitializationError;
	}

	clear_ports ();

	/* reset internal state */
	_dsp_load                      = 0;
	_freewheeling                  = false;
	_freewheel                     = false;
	_last_process_start            = 0;
	_systemic_audio_output_latency = 0;

	init_pipewire ();

	if (register_system_ports ()) {
		PBD::error << _("PulseAudioBackend: failed to register system ports.") << endmsg;
		close_pulse ();
		return PortRegistrationError;
	}

	engine.sample_rate_change (_samplerate);
	engine.buffer_size_change (_samples_per_period);

	if (engine.reestablish_ports ()) {
		PBD::error << _("PipewireAudioBackend: Could not re-establish ports.") << endmsg;
		return PortReconnectError;
	}

	_run = true;
	g_atomic_int_set (&_port_change_flag, 0);

	if (pbd_realtime_pthread_create (PBD_SCHED_FIFO, PBD_RT_PRI_MAIN, PBD_RT_STACKSIZE_PROC,
	                                 &_main_thread, pthread_process, this)) {
		if (pbd_pthread_create (PBD_RT_STACKSIZE_PROC, &_main_thread, pthread_process, this)) {
			PBD::error << _("PipewireAudioBackend: failed to create process thread.") << endmsg;
			stop ();
			_run = false;
			return ProcessThreadStartError;
		} else {
			PBD::warning << _("PipewireAudioBackend: cannot acquire realtime permissions.") << endmsg;
		}
	}

	int timeout = 5000;
	while (!_active && --timeout > 0) {
		Glib::usleep (1000);
	}

	if (timeout == 0 || !_active) {
		PBD::error << _("PipewireAudioBackend: failed to start process thread.") << endmsg;
		_run = false;
		close_pulse ();
		return ProcessThreadStartError;
	}

	return NoError;
}

PipewireAudioBackend::~PipewireAudioBackend()
{
	clear_ports ();
	pw_deinit ();
}

void
PipewireAudioBackend::main_process_thread()
{
	size_t bytes_to_write;
	int64_t time_start;

	AudioEngine::thread_init_callback (this);
	_active            = true;
	_processed_samples = 0;

	manager.registration_callback ();
	manager.graph_order_callback ();

	_dsp_load_calc.reset ();
	stream_latency_update_cb (p_stream, this);

	while (_run) {
		// FIXME: implement freewheeling for exporting

		time_start = g_get_monotonic_time ();

		pthread_mutex_lock (&audio_process_mtex);
		if (engine.process_callback (_samples_per_period)) {
			_active = false;
			pthread_mutex_unlock (&audio_process_mtex);
			return 0;
		}

		pw_stream_trigger_process (playback_stream);
		pthread_cond_wait (&audio_process_trigger_done, &audio_process_mtex);
		pthread_mutex_unlock (&audio_process_mtex);

		bool connections_changed = false;
		bool ports_changed       = false;
		if (!pthread_mutex_trylock (&_port_callback_mutex)) {
			if (g_atomic_int_compare_and_exchange (&_port_change_flag, 1, 0)) {
				ports_changed = true;
			}
			if (!_port_connection_queue.empty ()) {
				connections_changed = true;
			}
			while (!_port_connection_queue.empty ()) {
				PortConnectData* c = _port_connection_queue.back ();
				manager.connect_callback (c->a, c->b, c->c);
				_port_connection_queue.pop_back ();
				delete c;
			}
			pthread_mutex_unlock (&_port_callback_mutex);
		}
		if (ports_changed) {
			manager.registration_callback ();
		}
		if (connections_changed) {
			manager.graph_order_callback ();
		}
		if (connections_changed || ports_changed) {
			update_system_port_latencies ();
			engine.latency_callback (false);
			engine.latency_callback (true);
		}
	}
}

int
PipewireAudioBackend::register_system_ports ()
{
	LatencyRange lr;
	/* audio ports */
	lr.min = lr.max = _systemic_audio_output_latency;
	for (int i = 1; i <= N_CHANNELS; ++i) {
		char tmp[32];
		snprintf (tmp, sizeof (tmp), "system:playback_%d", i);
		BackendPortPtr p = add_port (std::string (tmp), DataType::AUDIO,
		                static_cast<PortFlags> (IsInput | IsPhysical | IsTerminal));
		if (!p) {
			return -1;
		}
		set_latency_range (p, true, lr);
		//p->set_hw_port_name ("")
		_system_outputs.push_back (p);
	}
	return 0;
}

BackendPort*
PipewireAudioBackend::port_factory (std::string const & name, ARDOUR::DataType type, ARDOUR::PortFlags flags)
{
	BackendPort* port = 0;

	switch (type) {
		case DataType::AUDIO:
			port = new PipewireAudioPort (*this, name, flags);
			break;
		case DataType::MIDI:
		default:
			PBD::error << string_compose (_("%1::register_port: Invalid Data Type."), _instance_name) << endmsg;
			return 0;
	}

	return port;
}

/***** Thread code which should probably be in AudioEngine itself */

void*
PipewireAudioBackend::pipewire_process_thread (void* arg)
{
	ThreadData*             td = reinterpret_cast<ThreadData*> (arg);
	boost::function<void()> f  = td->f;
	delete td;
	f ();
	return 0;
}

int
PipewireAudioBackend::create_process_thread (boost::function<void()> func)
{
	pthread_t   thread_id;
	ThreadData* td = new ThreadData (this, func, PBD_RT_STACKSIZE_PROC);

	if (pbd_realtime_pthread_create (PBD_SCHED_FIFO, PBD_RT_PRI_PROC, PBD_RT_STACKSIZE_PROC,
	                                 &thread_id, pulse_process_thread, td)) {
		if (pbd_pthread_create (PBD_RT_STACKSIZE_PROC, &thread_id, pulse_process_thread, td)) {
			PBD::error << _("AudioEngine: cannot create process thread.") << endmsg;
			return -1;
		}
	}

	_threads.push_back (thread_id);
	return 0;
}

int
PipewireAudioBackend::join_process_threads ()
{
	int rv = 0;

	for (std::vector<pthread_t>::const_iterator i = _threads.begin (); i != _threads.end (); ++i) {
		void* status;
		if (pthread_join (*i, &status)) {
			PBD::error << _("AudioEngine: cannot terminate process thread.") << endmsg;
			rv -= 1;
		}
	}
	_threads.clear ();
	return rv;
}

bool
PipewireAudioBackend::in_process_thread ()
{
	if (pthread_equal (_main_thread, pthread_self ()) != 0) {
		return true;
	}

	for (std::vector<pthread_t>::const_iterator i = _threads.begin (); i != _threads.end (); ++i) {
		if (pthread_equal (*i, pthread_self ()) != 0) {
			return true;
		}
	}
	return false;
}

uint32_t
PipewireAudioBackend::process_thread_count ()
{
	return _threads.size ();
}

/***************************************/

static boost::shared_ptr<PipewireAudioBackend> _instance;

static boost::shared_ptr<AudioBackend> backend_factory (AudioEngine& e);
static int  instantiate (const std::string& arg1, const std::string& arg2);
static int  deinstantiate ();
static bool already_configured ();
static bool available ();

static ARDOUR::AudioBackendInfo _descriptor = {
	"Pipewire",
	instantiate,
	deinstantiate,
	backend_factory,
	already_configured,
	available
};

static boost::shared_ptr<AudioBackend>
backend_factory (AudioEngine& e)
{
	if (!_instance) {
		_instance.reset (new PipewireAudioBackend (e, _descriptor));
	}
	return _instance;
}

static int
instantiate (const std::string& arg1, const std::string& arg2)
{
	s_instance_name = arg1;
	return 0;
}

static int
deinstantiate ()
{
	_instance.reset ();
	return 0;
}

static bool
already_configured ()
{
	return false;
}

static bool
available ()
{
	return true;
}

extern "C" ARDOURBACKEND_API ARDOUR::AudioBackendInfo* descriptor ()
{
	return &_descriptor;
}

}
