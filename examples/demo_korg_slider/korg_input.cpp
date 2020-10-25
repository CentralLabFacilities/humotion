#include "korg_input.h"
#include <assert.h>
#include <stdlib.h>

#define DEBUG_SHOW_MIDI_DEVICES 1
#define DEBUG_INCOMING_DATA 1

// defines for portmidi
#define INPUT_BUFFER_SIZE 100
#define OUTPUT_BUFFER_SIZE 0
#define DRIVER_INFO NULL
#define TIME_PROC ((int32_t(*)(void*))Pt_Time)
#define TIME_INFO NULL
#define TIME_START Pt_Start(1, 0, 0) /* timer started w/millisecond accuracy */

//! constructor
KorgInput::KorgInput() {
	midi = NULL;
	// first try old device:
	device_open = open_device();
	for (int i = 0; i < 256; i++) {
		data[i] = 64;
		data_new[i] = false;
	}
}

//! destructor
KorgInput::~KorgInput() {
	if (device_open) {
		// close
		Pm_Close(midi);
	}
}

//! fetch device id
//! \param char array with device name
//! \return id of device, or -1 if not found
int KorgInput::fetch_device_id(const char* name, bool output = false) {
	// fetch device id:
	int midi_device_id = -1;

	for (int i = 0; i < Pm_CountDevices(); i++) {
		const PmDeviceInfo* info = Pm_GetDeviceInfo(i);
		if (DEBUG_SHOW_MIDI_DEVICES)
			printf("%d: %s, %s\n", i, info->interf, info->name);

		if (output) {
			if (info->output) {
				if (strcmp(info->name, name) == 0) {
					midi_device_id = i;
				}
			}
		}
		else {
			if (info->input) {
				if (strcmp(info->name, name) == 0) {
					midi_device_id = i;
				}
			}
		}
	}

	return midi_device_id;
}

//! open device
//! \return true if device is found, false otherwise
bool KorgInput::open_device() {
	// standard is type1:
	device_type = NANO_KONTROL;

	int midi_device_id = fetch_device_id("nanoKONTROL MIDI 1");
	int midi_device_id_out = fetch_device_id("nanoKONTROL MIDI 1", true);
	if (midi_device_id == -1) {
		device_type = NANO_KONTROL2;
		midi_device_id = fetch_device_id("nanoKONTROL2 MIDI 1");
		midi_device_id_out = fetch_device_id("nanoKONTROL2 MIDI 1", true);
	}

	if (midi_device_id == -1) {
		printf("> can not open device (input). Error: device not found\n");
		return false;
	}

	if (midi_device_id_out == -1) {
		printf("> can not open output device\n");
		return false;
	}

	// success, found device
	printf("> found korg controller with id %d (%s)\n", midi_device_id,
	       device_type == NANO_KONTROL ? "nanoKONTROL" : "nanoKONTROL2");
	printf("> if setting leds on/off does not work, make sure to set led mode to external with the korg windows software\n");

	// open input
	Pm_OpenInput(&midi, midi_device_id, DRIVER_INFO, INPUT_BUFFER_SIZE, TIME_PROC, TIME_INFO);

	// open output
	Pm_OpenOutput(&midi_out, midi_device_id_out, DRIVER_INFO, OUTPUT_BUFFER_SIZE, NULL, NULL, 0);

	// set filter
	Pm_SetFilter(midi, PM_FILT_ACTIVE | PM_FILT_CLOCK | PM_FILT_SYSEX);
	Pm_SetFilter(midi_out, PM_FILT_ACTIVE | PM_FILT_CLOCK | PM_FILT_SYSEX);

	// clear buffer with unfiltered messages
	// printf("> clearing buffer\n");
	// while (Pm_Poll(midi)) {
	// Pm_Read(midi, buffer, 1);
	//}
	printf("> init done \n");

	// disable all leds:
	// 	for(unsigned char id = 0; id<127; id++){
	// 		Pm_WriteShort(midi_out, TIME_PROC(TIME_INFO), Pm_Message(0xB0, id, 0));
	// 		usleep(10*1000);
	// 	}

	// fancy animation:
	for (unsigned char id = 0; id < 80 + 4; id++) {
		if (id < 127)
			Pm_WriteShort(midi_out, TIME_PROC(TIME_INFO), Pm_Message(0xB0, id, 127));
		if (id > 4)
			Pm_WriteShort(midi_out, TIME_PROC(TIME_INFO), Pm_Message(0xB0, id - 4, 0));
		usleep(25 * 1000);
	}

	// 	for(unsigned char id = 0; id<127; id++){
	// 		Pm_WriteShort(midi_out, TIME_PROC(TIME_INFO), Pm_Message(0xB0, id, 0));
	// 	}
	//
	return true;
}

//! device opened sucessfully
//! \return true if device is open and working
bool KorgInput::device_available() {
	return device_open;
}

//! fetch new data
//! \return int count how many datasets were fetched
int KorgInput::fetch_data() {
	int new_data_fetched = 0;
	// poll all data:
	while (Pm_Poll(midi)) {
		// new data fetch sucessfull? build up dataset:
		if (fetch_single()) {
			new_data_fetched++;
		}
	}

	if (new_data_fetched > 0) {
		printf("> fetched %d datasets\n", new_data_fetched);
	}

	return new_data_fetched;
}

bool KorgInput::fetch_single() {
	PmEvent buffer[1];

	// poll data:
	PmError status = Pm_Poll(midi);

	if (status) {
		int length = Pm_Read(midi, buffer, 1);
		if (length > 0) {
			// store incoming data:
			if (Pm_MessageStatus(buffer[0].message) == 0xb0) {
				unsigned int idx = Pm_MessageData1(buffer[0].message) & 0xFF;
				data[idx] = Pm_MessageData2(buffer[0].message) & 0xFF;
				data_new[idx] = true;
			}

			if (DEBUG_INCOMING_DATA) {
				printf("> incoming message: time %ld, %2lx %2lx %2lx\n", (long)buffer[0].timestamp,
				       (long)Pm_MessageStatus(buffer[0].message), (long)Pm_MessageData1(buffer[0].message),
				       (long)Pm_MessageData2(buffer[0].message));
			}

			return true;
		}
		else {
			printf("> error, message len <= 0?!\n");
			return false;
		}
	}
}

//! access buffered data
//! \param index of data channel to fetch
//! \return unsigned char value for data channel <index> (0..127)
unsigned char KorgInput::get_buffered_raw_value(unsigned char index) {
	return data[index];
}

//! fetch a value + new flag & clear
//! \param idx index
//! \param uchar ptr to data
//! \return true if new data available
bool KorgInput::get_and_reset(unsigned char idx, unsigned char* val) {
	// fetch state & reset:
	bool new_data = data_new[idx];
	data_new[idx] = false;

	// override data only if new val arrived:
	if (new_data) {
		*val = get_buffered_raw_value(idx);
	}

	return new_data;
}

//! access slider
//! \param slider index 0..8
//! \param uchar ptr to value
//! \return bool if new value arrived
bool KorgInput::get_slider(unsigned char index, unsigned char* val) {
	assert(index <= 8);
	if (device_type == NANO_KONTROL) {
		switch (index) {
			default:

				return get_and_reset(0x02 + index, val);
			case (5):
			case (6):
				return get_and_reset(0x02 + index + 1, val);
			case (7):
			case (8):
				return get_and_reset(0x02 + index + 3, val);
		}
	}
	else {
		// NANO_KONTROL2
		return get_and_reset(index, val);
	}
}

//! access button
//! \param button index
//! \param uchar ptr to value
//! \return bool if new value arrived
bool KorgInput::get_button(unsigned char index, unsigned char* val) {
	assert(index <= 127);

	if (device_type == NANO_KONTROL) {
		printf("> NANO_KONTROL NOT SUPPORTED, different layout of buttons! exiting\n");
		exit(1);
		/*if (row == 0){
		   return get_and_reset(0x17+index, val);
		}else if (row == 1){
		   printf("> NANO_KONTROL does not have center row. ignored\n");
		   return false;
		}else{
		   return get_and_reset(0x21+index, val);
		}*/
	}
	else {
		// NANO_KONTROL2 - fetch button value
		return get_and_reset(index, val);
	}
}

//! access pot
//! \param pot index 0..8
//! \param unsigned char ptr to value (0..127)
//! \param bool true if new data available
bool KorgInput::get_pot(unsigned char index, unsigned char* val) {
	assert(index <= 8);
	if (device_type == NANO_KONTROL) {
		return get_and_reset(0x0E + index, val);
	}
	else {
		// NANO_KONTROL2
		return get_and_reset(0x10 + index, val);
	}
}

void KorgInput::set_led(unsigned char index, bool status) {
	Pm_WriteShort(midi_out, TIME_PROC(TIME_INFO), Pm_Message(0xB0, index, status ? 127 : 0));
}