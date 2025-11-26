
# MORK-1 Flight Computer

A compact, real-time flight computer for model rockets, built on the ESP32-C3 Super Mini. It features attitude stabilization using a PID controller, altitude measurement, and status indication via LEDs.

---

## 🚀 Features

- **Attitude stabilization** using MPU6050 gyro/accelerometer and PID control
- **Altitude measurement** with BMP180 barometric sensor
- **Servo control** for pitch and yaw correction
- **Status LEDs** for system health and alerts
- **Serial debug output** for real-time monitoring

---

## 🛠️ Hardware Components

- **ESP32-C3 Super Mini**
- **MPU6050** (Gyro/Accelerometer, I2C)
- **BMP180** (Barometric Pressure Sensor, I2C)
- **2x Servos** (Pitch & Yaw control)
- **Red LED** (Alert)
- **Green LED** (System OK)
- **Resistors:** 220Ω for each LED

---

## 📐 Pin Connections

| Component         | ESP32-C3 Pin | Notes                        |
|-------------------|--------------|------------------------------|
| MPU6050 VCC       | 3V3          | Power                        |
| MPU6050 GND       | GND          | Ground                       |
| MPU6050 SDA       | GPIO 5       | I2C Data                     |
| MPU6050 SCL       | GPIO 6       | I2C Clock                    |
| BMP180 VCC        | 3V3          | Power                        |
| BMP180 GND        | GND          | Ground                       |
| BMP180 SDA        | GPIO 5       | I2C Data (shared)            |
| BMP180 SCL        | GPIO 6       | I2C Clock (shared)           |
| Servo Pitch       | GPIO 7       | PWM output                   |
| Servo Yaw         | GPIO 10      | PWM output                   |
| Red LED Anode     | GPIO 2       | 220Ω resistor in series      |
| Red LED Cathode   | GND          | Ground                       |
| Green LED Anode   | GPIO 3       | 220Ω resistor in series      |
| Green LED Cathode | GND          | Ground                       |

---

## 🖼️ Wiring Diagram (Text-Based)

```
ESP32-C3
  3V3 --------+------ MPU6050 VCC
              |------ BMP180 VCC
  GND -------+------ MPU6050 GND
             |------ BMP180 GND
             |------ LEDs Cathode (via resistor)
 GPIO5 SDA --+------ MPU6050 SDA
             |------ BMP180 SDA
 GPIO6 SCL --+------ MPU6050 SCL
             |------ BMP180 SCL
 GPIO7 ------> Servo Pitch (PWM)
 GPIO10 -----> Servo Yaw (PWM)
 GPIO2 ------> Red LED Anode (via 220Ω)
 GPIO3 ------> Green LED Anode (via 220Ω)
```

---

## ⚡ Wiring Notes

- **I2C Bus:** Both sensors share SDA (GPIO 5) and SCL (GPIO 6).
- **LEDs:** Use 220Ω resistors in series with each LED anode.
- **Servos:** Connect signal wires to GPIO 7 (Pitch) and GPIO 10 (Yaw). Power from 3.3V or external supply if needed.
- **Ground:** Ensure all components share a common GND.

---

## 📦 Project Structure

- `flightcomputer.c++` – Main project source file

---

## 🚦 System Behavior

- **Green LED ON:** System initialized and sensors OK
- **Red LED ON:** Sensor error or excessive rotation detected
- **Servos:** Actively stabilize rocket using PID
- **Serial Output:** Real-time debug info (gyro, servo, altitude)

---

## 🚀 Getting Started

1. **Wire components** as per the diagram above.
2. **Flash `flightcomputer.c++`** to your ESP32-C3 Super Mini.
3. **Open Serial Monitor** at 115200 baud for debug output.
4. **Power up** and observe stabilization and sensor readings.

---

## 📝 License

MIT License. See [LICENSE](LICENSE) for details.

---

## 🙌 Credits

Project by [Your Name].

---

> **Tip:** For a graphical wiring diagram, use [Fritzing](https://fritzing.org/) or similar tools.
