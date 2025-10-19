# Quick Power Supply Fix Guide

## Your Problem
ESP32 works when connected to computer USB, but not from 3.7V battery via VIN when servos are connected.

## Why This Happens
1. **Voltage drop:** When servos move, they draw 0.5-2A each
2. **Insufficient current:** Battery or regulator can't supply enough current
3. **Ground issues:** Missing common ground between components

## Immediate Solution

### Option A: 2S Battery (7.4V) - RECOMMENDED
```
[7.4V Battery] → [5V Buck Regulator (3-5A)] → [ESP32 5V Pin]
                                            ↘ [Servo V+ (both)]
                                              
All GND pins connected together
```

**Parts needed:**
- 5V 3A+ buck converter (LM2596 module or UBEC)
- 470µF capacitor across 5V rail near servos
- Proper wire gauge (20-22 AWG)

### Option B: 1S Battery (3.7V)
```
[3.7V Battery] → [5V Boost Converter (3A)] → [ESP32 5V Pin]
                                           ↘ [Servo V+ (both)]
```

**Parts needed:**
- 5V 3A boost converter (MT3608 module)
- Same capacitors and wiring as Option A

## Critical Wiring Rules

1. **Common Ground = MANDATORY**
   ```
   Battery(-) → Regulator GND → ESP32 GND → Servo GND → USB GND (if connected)
   ```

2. **Power Input to ESP32:**
   - Use **5V pin** (not VIN if your board's VIN expects >5V)
   - OR use **VIN** only if your ESP32 board accepts 5V on VIN
   - Check your specific ESP32 dev board datasheet

3. **Capacitors:**
   - 470-1000µF electrolytic across servo power (close to servos)
   - 100nF ceramic near ESP32 power pins
   - Prevents voltage spikes

## Testing Steps

### Step 1: Voltage Check
1. Connect battery to regulator
2. Measure regulator output with multimeter: **must be 5.0V ±0.1V**
3. If not 5V, adjust regulator trimmer potentiometer

### Step 2: ESP32 Test
1. Connect ESP32 to regulated 5V
2. Do NOT connect servos yet
3. Power on and check:
   - Serial output appears
   - Green LED lights up
   - Measure voltage at ESP32 5V pin: **should be ~4.9-5.1V**

### Step 3: Servo Load Test
1. Connect both servos to same 5V rail
2. Power on
3. Command servo movement (tilt the board)
4. Watch multimeter at ESP32 5V pin:
   - **Good:** Voltage stays above 4.7V
   - **Bad:** Voltage drops below 4.5V → upgrade regulator or add bigger capacitor

### Step 4: Full System Test
1. If Step 3 is good, you're done!
2. Upload the improved code
3. Place on flat surface, wait for green LED
4. Tilt to test servo response

## Quick Diagnostics

| Symptom | Cause | Fix |
|---------|-------|-----|
| ESP32 resets when servo moves | Voltage sag | Bigger capacitor or stronger regulator |
| Nothing works at all | Wrong voltage or no common ground | Check wiring and ground connections |
| Works on USB, not battery | Regulator underpowered | Use 3-5A rated regulator |
| Servos jitter | Noise on power rail | Add 470µF cap near servos |
| ESP32 hot | Overvoltage | Check regulator output is 5V max |

## Recommended Parts (Amazon/AliExpress)

### Regulator Options:
1. **Best:** Pololu 5V 5A Step-Down Voltage Regulator D24V50F5
2. **Good:** UBEC 5V 3A (RC hobby BEC)
3. **Budget:** LM2596S DC-DC Buck Module (set to exactly 5.0V)

### Capacitors:
- 470µF 16V low-ESR electrolytic (×2)
- 100nF ceramic (×3-4)

### Wire:
- 22 AWG silicone wire (red/black)
- XT30 or XT60 connectors for battery

## Emergency Workaround (Testing Only)

If you need to test RIGHT NOW without proper parts:

1. **Power ESP32 from USB** (computer or powerbank)
2. **Power servos from separate 5V source** (could be battery through regulator)
3. **Connect ALL grounds together** - this is critical!
4. This separates ESP32 power from servo power

**Caution:** This is NOT suitable for flight - servos must be on same supply for reliable operation.

## After Fixing Power

Upload the improved code - it fixes the flight state bug where it jumped to RECOVERY mode immediately. Key fixes:

1. ✅ Flight state machine now works correctly
2. ✅ Launch detection requires sustained altitude gain
3. ✅ Servos disabled on ground to save power
4. ✅ Better telemetry and error handling
5. ✅ Improved serial output with flight state

Green LED will stay SOLID on ground now, only blinking after actual launch detection!
