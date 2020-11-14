#include <cstdio>
#include <stdio.h>
#include <string.h>

#include "korg_input.h"
#include "humotion/client/client.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/thread/thread_time.hpp>
#include <boost/thread/thread.hpp>

using namespace boost;
humotion::GazeState last_awake_state;
bool awake_state;

enum JOINT_SLIDER_ID
{
	MOUTH_POSITION_LEFT = 0,
	MOUTH_POSITION_CENTER,
	MOUTH_POSITION_RIGHT,
	PAN,
	TILT
	// NECK_ROLL,
};

enum JOINT_POTI_ID
{
	MOUTH_OPENING_LEFT = 0,
	MOUTH_OPENING_CENTER,
	MOUTH_OPENING_RIGHT,
	EYE_VERGENCE,
	EYELID_OPENING_UPPER,
	EYELID_OPENING_LOWER,
	EYEBROW_RIGHT,
	EYEBROW_LEFT
};

float to_float(unsigned char val) {
	return ((float)(val)) / 127.0;
}

void slidervalues_to_mouthstate(KorgInput* input, humotion::MouthState* result) {
	unsigned char tmp;

	// do not handle updates when not awake
	if (!awake_state) {
		return;
	}

	if (input->get_slider(MOUTH_POSITION_LEFT, &tmp))
		result->position_left = 0.0 + 40.0 * (1.0 - to_float(tmp));
	if (input->get_slider(MOUTH_POSITION_CENTER, &tmp))
		result->position_center = 0.0 + 40.0 * (1.0 - to_float(tmp));
	if (input->get_slider(MOUTH_POSITION_RIGHT, &tmp))
		result->position_right = 0.0 + 40.0 * (1.0 - to_float(tmp));

	if (input->get_pot(MOUTH_OPENING_LEFT, &tmp))
		result->opening_left = 40.0 * to_float(tmp);
	if (input->get_pot(MOUTH_OPENING_CENTER, &tmp))
		result->opening_center = 40.0 * to_float(tmp);
	if (input->get_pot(MOUTH_OPENING_RIGHT, &tmp))
		result->opening_right = 40.0 * to_float(tmp);

	// printf("OOO %4.2f %4.2f\n",result.opening_center, result.position_center);
}

void wake_up(humotion::GazeState* result) {
	if (awake_state) {
		// already awake, return
		return;
	}

	// restore last awake state
	*result = last_awake_state;
	// force reopening
	result->eyeblink_request_left = 10;
	result->eyeblink_request_right = 10;

	awake_state = true;
}

void sleep(humotion::GazeState* result) {
	if (!awake_state) {
		// already sleeping, return
		return;
	}

	// store last awake state
	last_awake_state = *result;
	awake_state = false;

	// override values:
	result->pan = 0.0;
	result->tilt = -20.0;
	result->roll = 0.0;

	// result->eyelid_opening = 0.0;
	result->eyebrow_left = 0.0;
	result->eyebrow_right = 0.0;

	result->eyeblink_request_left = -1;
	result->eyeblink_request_right = -1;
}

void slidervalues_to_gazestate(KorgInput* input, humotion::GazeState* result) {
	unsigned char tmp;

	// PLAY
	if (input->get_button(0x29, &tmp)) {
		if (tmp != 0) {
			wake_up(result);
		}
	}

	// STOP
	if (input->get_button(0x2A, &tmp)) {
		if (tmp != 0) {
			sleep(result);
		}
	}

	// do not handle updates when not awake
	if (!awake_state) {
		return;
	}

	if (input->get_slider(PAN, &tmp))
		result->pan = -50.0 + 100.0 * to_float(tmp);
	if (input->get_slider(TILT, &tmp))
		result->tilt = -40.0 + 80.0 * to_float(tmp);
	result->roll = 0.0;

	if (input->get_pot(EYELID_OPENING_UPPER, &tmp))
		result->eyelid_opening_upper = 0.0 + 120.0 * to_float(tmp);
	if (input->get_pot(EYELID_OPENING_LOWER, &tmp))
		result->eyelid_opening_lower = 0.0 + 120.0 * to_float(tmp);

	if (input->get_pot(EYEBROW_LEFT, &tmp))
		result->eyebrow_left = 60.0 - 120.0 * to_float(tmp);
	if (input->get_pot(EYEBROW_RIGHT, &tmp))
		result->eyebrow_right = 60.0 - 120.0 * to_float(tmp);

	if (input->get_pot(EYE_VERGENCE, &tmp))
		result->vergence = -20.0 * to_float(tmp);

	// check for blink requests:
	if (input->get_button(0x24, &tmp)) {
		if (tmp != 0) {
			// blink left
			result->eyeblink_request_left = 300;
		}
	}
	if (input->get_button(0x44, &tmp)) {
		if (tmp != 0) {
			result->eyeblink_request_right = 300;
		}
	}
	if (input->get_button(0x34, &tmp)) {
		if (tmp != 0) {
			// blink both
			result->eyeblink_request_right = 150;
			result->eyeblink_request_left = 150;
		}
	}

	// printf("PTR = %3.1f %3.1f %3.1f\n",result->pan,result->tilt,result->roll);
}

int main(int argc, char** argv) {
	humotion::MouthState mouth_state;
	humotion::GazeState gaze_state;
	awake_state = true;

	if (argc != 2) {
		printf("> ERROR: invalid parameter count.\n\nusage: %s <scope>\n       (e.g. %s /flobi)\n\n", argv[0], argv[0]);
		exit(EXIT_FAILURE);
	}

	printf("> starting korg kontrol demo. please make sure the korg device is connected via usb\n");

	KorgInput* kinput = new KorgInput();

	if (!kinput->device_available()) {
		printf("> no korg kontrol device found on usb bus... exiting\n");
		return 0;
	}

	// set led status for blink requests:
	kinput->set_led(0x24, true);
	kinput->set_led(0x34, true);
	kinput->set_led(0x44, true);

	// set led for stop/play
	kinput->set_led(0x29, true);
	kinput->set_led(0x2A, true);

	// human motion connection:
	humotion::client::Client* client = new humotion::client::Client(argv[1], "ROS");

	// send values to human motion server
	float loop_delay = 1000.0 / 50.0; // run with 50Hz

	boost::system_time timeout = get_system_time() + posix_time::milliseconds(loop_delay);

	while (client->ok()) {
		// allow middleware to process data
		client->tick();

		// clear previous eyeblinks
		gaze_state.eyeblink_request_left = 0;
		gaze_state.eyeblink_request_right = 0;

		// handle slider data
		if (kinput->fetch_data() > 0) {
			// debug
			mouth_state.dump();
			gaze_state.dump();
			printf("\n");

			// transfer values to robot:
			slidervalues_to_mouthstate(kinput, &mouth_state);
			slidervalues_to_gazestate(kinput, &gaze_state);
		}

		// printf("BLINK: %d %d\n",gaze_state.eyeblink_request_left, gaze_state.eyeblink_request_right);

		// periodically send mouth & gaze target
		client->update_mouth_target(mouth_state);
		client->update_gaze_target(gaze_state);
		client->send_all();

		thread::sleep(timeout);
		timeout = get_system_time() + posix_time::milliseconds(loop_delay);
	}
}
