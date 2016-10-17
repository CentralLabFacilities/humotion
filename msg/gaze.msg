Header header

#type of pan/tilt/roll targets
time gaze_timestamp
int32 gaze_type
int32 GAZETYPE_ABSOLUTE=0
int32 GAZETYPE_RELATIVE=1

float32 pan
float32 tilt
float32 roll
float32 vergence

float32 pan_offset
float32 tilt_offset
float32 roll_offset

float32 eyelid_opening_upper
float32 eyelid_opening_lower

float32 eyebrow_left
float32 eyebrow_right

int32 eyeblink_request_left
int32 eyeblink_request_right
