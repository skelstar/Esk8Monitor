

uint8_t getBatteryPercentage(float voltage) {
  float voltsLeft = voltage - BATTERY_VOLTAGE_CUTOFF_END;
  float voltsAvail = BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_CUTOFF_END;
  uint8_t percent = (voltsLeft /  voltsAvail) * 100;
  // Serial.printf("%.1f %d %.1f %.1f\n", 
  //   voltage, 
  //   percent, 
  //   voltsLeft, 
  //   voltsAvail);
	if (percent > 100) {
		percent = 100;
	}
  return percent;
}
