# MORK-1 Flight Computer - Code Improvements

## Critical Fixes Applied

### 1. **Fixed Flight State Machine Logic** ✅
**Problem:** Flight computer jumped directly to RECOVERY mode on ground
**Root Cause:** `calculateAltitude()` was called INSIDE validation check, and `updateFlightState()` was also inside, so states weren't updating properly
**Solution:**
- Moved `calculateAltitude()` to run BEFORE validation
- Moved `updateFlightState()` OUTSIDE the sensor read block so it runs every loop iteration
- Added sustained launch detection (requires 500ms of positive altitude + velocity to prevent false triggers)

### 2. **Enhanced Altitude Calculation**
**Improvements:**
- Added pressure sanity checks (300-1100 hPa range)
- Implemented low-pass filter on vertical speed (70% old + 30% new) to reduce noise
- Added delta-time validation to prevent division errors
- Better handling of uninitialized sensor states

### 3. **Improved Launch Detection**
**Old:** Single reading above 3m triggered launch
**New:** Requires:
- Altitude > 3m threshold
- Positive vertical velocity (>1 m/s)
- Sustained for 500ms (3 consecutive checks)
- Launch confirmation beep added

### 4. **Better Apogee Detection**
**Improvements:**
- Requires negative velocity AND altitude drop from peak
- Minimum 1 second flight time before apogee can trigger
- Logs maximum altitude achieved
- High-pitch beep on apogee detection

### 5. **Smarter Landing Detection**
**Improvements:**
- Requires low altitude (<3m) AND low velocity (<1 m/s)
- Minimum 2 seconds descent time
- Logs final altitude on landing
- Prevents premature recovery mode

### 6. **Power-Saving Servo Control**
**Changes:**
- Servos held at 90° (neutral) during GROUND and RECOVERY states
- Active stabilization only during ASCENT and DESCENT
- Reduces servo power draw when not needed

### 7. **Better Telemetry**
**Improvements:**
- Separated UDP send interval from sensor read interval
- Added flight state and max altitude to telemetry packet
- Reduced UDP spam (100ms intervals instead of 50ms)
- Format: `altitude,verticalSpeed,roll,pitch,temp,flightState,maxAltitude`

### 8. **Enhanced Serial Output**
**New Format:**
```
State: ASCENT | Alt: 45.23m | VelZ: 12.34m/s | Roll: -2.1° | Pitch: 3.4° | Temp: 24.5°C | Press: 1013.2 hPa
```
- Shows current flight state
- Formatted with units
- Precision-controlled (2 decimals for critical values)

### 9. **Improved Validation**
**Added:**
- Altitude sanity check (±10km absolute limit)
- Single temperature read per cycle (reduces I2C traffic)
- More descriptive error messages

### 10. **Better System Initialization**
**Improvements:**
- Startup delay for serial stabilization
- Banner message with system name
- UDP port explicitly opened
- Servos initialized to neutral position
- Clear status messages for each subsystem

## LED Status Reference

| LED Pattern | Meaning |
|------------|---------|
| Green solid | All systems OK (ground state) |
| Red solid | Component failure |
| Green fast blink (500ms burst) | Launch detected |
| Green blink (250ms) | Ascending |
| Red blink (250ms) | Descending |
| Green/Red alternate (500ms) | Recovery mode - landed |

## Power Supply Recommendations

### Your Current Issue
- **Problem:** ESP32 works on USB, but not from 3.7V battery via VIN
- **Likely Cause:** Voltage drop when servos draw current

### Solution Checklist
1. **Use proper 5V regulator rated for 3-5A minimum**
   - Recommended: UBEC/BEC module or Pololu 5V step-down
   - LM2596 modules work if properly sized
   
2. **Battery Configuration:**
   - If 1S (3.7V): Use boost converter to 5V
   - If 2S (7.4V): Use buck converter to 5V
   - **Never** connect 7.4V directly to ESP32 5V pin!

3. **Wiring:**
   ```
   Battery+ → Regulator IN+
   Battery- → Regulator IN- & Common GND
   Regulator OUT+ → ESP32 5V pin & Servo V+
   Regulator OUT- → Common GND
   ```

4. **Add Capacitors:**
   - 470-1000µF electrolytic near servos
   - 100nF ceramic near ESP32 VIN
   - Reduces voltage spikes from servo movement

5. **Wire Gauge:**
   - Use 20-22 AWG for power lines
   - Keep wires short to reduce voltage drop

### Testing Procedure
1. Connect battery to regulator, measure output with multimeter (should be 5.0V ±0.1V)
2. Connect ESP32 only, verify stable operation
3. Add servos, command movement, monitor voltage
4. If voltage dips below 4.6V during servo movement, upgrade regulator or add capacitors

## Flight Test Checklist

### Pre-Flight
- [ ] All LEDs flash 3 times after ground pressure calibration
- [ ] Green LED solid (all systems OK)
- [ ] Serial monitor shows "Flight computer ready!"
- [ ] SD card shows new flight_X.csv file created
- [ ] Servos respond to tilt (manual test)
- [ ] Place rocket on flat surface for 10 seconds (ensures ground state)

### Expected Behavior
1. **Ground:** Green LED solid, servos at 90°
2. **Launch:** Rapid green flashing, beep, then steady green blink
3. **Ascent:** Green blinks every 250ms, servos active
4. **Apogee:** High pitch beep (2000Hz), LED changes to red blink
5. **Descent:** Red blinks every 250ms, servos active
6. **Landing:** Alternating green/red, intermittent buzzer (800Hz)

### Post-Flight
- Check SD card for flight_X.csv data
- Review max altitude from serial log
- Verify all flight phases were detected

## Known Limitations

1. **BMP180 Altitude Resolution:** ±1m typical, ±3m worst case
2. **Vertical Speed Noise:** Filtered but can be jumpy at low speeds
3. **Launch Detection Delay:** 500ms intentional to prevent false triggers
4. **WiFi Range:** ~50-100m line of sight for telemetry
5. **Servo Response:** 50ms update rate may show slight lag in stabilization

## Future Enhancements (Optional)

- [ ] Add Kalman filter for altitude/velocity fusion
- [ ] Implement data replay from SD card
- [ ] Add parachute deployment trigger output
- [ ] Include battery voltage monitoring
- [ ] GPS integration for landing location
- [ ] Accelerometer-based launch detection (faster response)

## Changelog

**v2.0 (Current)**
- Fixed critical flight state machine bug
- Added sustained launch detection
- Improved apogee and landing detection
- Power-saving servo control
- Enhanced telemetry and logging
- Better error handling and validation

**v1.0 (Original)**
- Basic flight computer functionality
- BMP180, MPU6050, SD card integration
- WiFi telemetry
- LED status indicators
