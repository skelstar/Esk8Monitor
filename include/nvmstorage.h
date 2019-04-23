#ifndef Preferences_h
#include <Preferences.h>
// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.h
#endif

Preferences preferences;

void storeUInt8(const char* name, uint8_t value) {
	preferences.begin(STORE_NAMESPACE, false);	// r/w
	preferences.putUChar(name, value);
	preferences.end();
}

int recallUInt8(char* name)  {
	preferences.begin(STORE_NAMESPACE, false);	// r/w
	float result = preferences.getUChar(name, 0.0);
	preferences.end();
	return result;
}

void storeFloat(char* name, float value) {
	preferences.begin(STORE_NAMESPACE, false);	// r/w
	preferences.putFloat(name, value);
	preferences.end();
}

float recallFloat(char* name) {
	preferences.begin(STORE_NAMESPACE, false);	// r/w
	float result = preferences.getFloat(name, 0.0);
	preferences.end();
	return result;
}
