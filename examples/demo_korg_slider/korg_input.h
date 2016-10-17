#pragma once

#include <portmidi.h>
#include <porttime.h>
#include <cstdio>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

class KorgInput{
public:
	KorgInput();
	~KorgInput();
	
	bool  get_slider(unsigned char index, unsigned char *val);
	bool  get_pot(unsigned char index, unsigned char *val);
	bool  get_button(unsigned char index, unsigned char *val);
	void  set_led(unsigned char index, bool status);
	
	bool get_and_reset(unsigned char idx, unsigned char *val);
	void set_pot(unsigned char index, int value);
	
	int  fetch_data();
	bool device_available();
	
	
private:
	int resolve_id();
	
	unsigned char data[256];
	bool data_new[256];
	int device_type;
	enum DEVICE_TYPE {NANO_KONTROL, NANO_KONTROL2};
	bool open_device();
	bool fetch_single();
	int fetch_device_id(const char*, bool);
	unsigned char get_buffered_raw_value(unsigned char index);
	
	bool device_open;
	PmStream * midi;
	PmStream * midi_out;
};