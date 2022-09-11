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

#include <pipewire/pipewire.h>

#include "pipewire_backend.h"

#ifndef __libbackend_pipewire_registry_h__
#define __libbackend_pipewire_registry_h__

namespace ARDOUR {

class PipewireClient {
public:
	PipewireClient();

protected:
	struct pw_stream *playback_stream;
	struct pw_main_loop *loop;
	struct spa_pod *audio_format;

	pthread_mutex_t audio_process_mtex;
	pthread_cond_t audio_process_trigger_done;

	int client_init (std::string &instance_name);
	void client_close ();
}

} // namespace ARDOUR

#endif // __libbackend_pipewire_registry_h__