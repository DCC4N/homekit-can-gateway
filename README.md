# HomeKit-CAN Gateway for Motorized Gate

Integration of a motorized gate with Apple HomeKit via CAN bus and ESP32.

## 📋 Description

This project implements a bidirectional gateway between Apple HomeKit and a motorized gate communicating via CAN bus. It allows you to:

- **Control the gate** from the Home app (iPhone/iPad/Mac)
- **View real-time status** (Closed, Opening, Open, Closing, Obstructed)
- **Automatically synchronize** the HomeKit interface with the actual gate state, even when commanded by external remote control
- **Detect obstructions** and report them in HomeKit

## ✨ Technical Features

- **Multi-core architecture**: HomeSpan on Core 0, CAN on Core 1 (FreeRTOS)
- **Thread-safe communication** with mutex
- **CAN bus monitoring timeout** (15s)
- **OTA support** (Over-The-Air updates)
- **Visual LED indicators** (opening/closing)
- **Configurable debug mode**

## 🔧 Required Hardware

### Main Components
- **ESP32** (Dev Kit v1 or equivalent)
- **CAN Transceiver**: MCP2562 or equivalent (5V tolerant)
- **LEDs** (optional): 2 LEDs + resistors for visual indication
- **Power Supply**: 5V for ESP32 and CAN transceiver

### Wiring Diagram

```
ESP32              MCP2562 (CAN Transceiver)
─────              ──────────────────────────
GPIO 23 ────────► Pin 1 (TXD)
GPIO 22 ◄──────── Pin 4 (RXD)
3.3V ───────────► Pin 3 (VDD)
GND ────────────► Pin 2 (VSS/GND)
                  Pin 6 (CANL) ──┐
                  Pin 7 (CANH) ──┤─► CAN Bus (to gate controller)
                                  │
                  120Ω Resistor (if termination needed)

GPIO 16 ────────► Red LED (Closing) + Resistor
GPIO 17 ────────► Green LED (Opening) + Resistor
```

### CAN Bus Configuration

- **Speed**: 125 kbps (configurable via `CAN_BITRATE`)
- **Format**: CAN 2.0A (11-bit identifiers)
- **Termination**: 120Ω if at bus end

## 📦 Required Libraries

Install via Arduino IDE Library Manager:

1. **HomeSpan** (version 1.8.0 or higher)
   ```
   https://github.com/HomeSpan/HomeSpan
   ```

2. **ACAN_ESP32** (CAN library for ESP32)
   ```
   https://github.com/pierremolinaro/acan-esp32
   ```

## 🚀 Installation

### 1. Arduino IDE Configuration

- Board: **ESP32 Dev Module**
- Partition Scheme: **Default 4MB with spiffs**
- Flash Frequency: **80MHz**
- Upload Speed: **921600**

### 2. WiFi and HomeKit Configuration

On first boot, the ESP32 creates a WiFi access point:

1. Connect to the **HomeSpan-Setup** network
2. Open your browser at `192.168.4.1`
3. Configure your WiFi credentials
4. Note the **HomeKit setup code** displayed in the serial monitor

### 3. Adding to Home App

1. Open the **Home** app on your iPhone
2. Tap **+** then **Add Accessory**
3. Scan the QR code or enter the 8-digit code
4. Follow the on-screen instructions

## 🔌 CAN Protocol

### Messages sent by gateway (commands)

| CAN ID  | Description    | Data[0] |
|---------|----------------|---------|
| `0x200` | OPEN command   | `0x01`  |
| `0x201` | CLOSE command  | `0x01`  |

### Messages received from gate (states)

| CAN ID  | Description | Data[0] | HomeKit State |
|---------|-------------|---------|---------------|
| `0x100` | CLOSED      | `0x01`  | CLOSED        |
| `0x100` | OPENING     | `0x02`  | OPENING       |
| `0x100` | OPEN        | `0x00`  | OPEN          |
| `0x100` | CLOSING     | `0x03`  | CLOSING       |
| `0x100` | OBSTRUCTED  | `0x04`  | STOPPED       |

> **Note**: These identifiers are configurable in the code (`CAN IDENTIFIERS` section)

## ⚙️ Custom Configuration

### Modify CAN Identifiers

```cpp
// Sent messages (commands)
#define CAN_ID_CMD_OUVRIR      0x200  // Modify according to your system
#define CAN_ID_CMD_FERMER      0x201

// Received messages (states)
#define CAN_ID_ETAT_PORTAIL    0x100
```

### Modify CAN States

```cpp
// Values in data[0] of state messages
#define ETAT_OUVERT     0x00  // Adapt to your protocol
#define ETAT_FERME      0x01
#define ETAT_OUVRANT    0x02
#define ETAT_FERMANT    0x03
#define ETAT_OBSTACLE   0x04
```

### Enable Debug Mode

```cpp
#define DEBUG_MODE true  // Display detailed CAN logs
```

### Modify GPIO Pins

```cpp
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_23;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_22;
static const gpio_num_t LED_CLOSE_PIN = GPIO_NUM_16;
static const gpio_num_t LED_OPEN_PIN = GPIO_NUM_17;
```

## 🐛 Troubleshooting

### Gate doesn't respond to commands

1. Check CAN wiring (CANH/CANL)
2. Verify bus speed (`CAN_BITRATE`)
3. Enable `DEBUG_MODE true` to see CAN frames
4. Verify your system's CAN identifiers

### Serial monitor shows "TIMEOUT: No CAN message for 15s"

- Gate is not sending periodic states
- Faulty CAN wiring
- CAN transceiver not powered or defective
- Missing CAN termination (120Ω)

### HomeKit shows "No Response"

- Check ESP32 WiFi connection
- Restart router and ESP32
- Remove and re-add accessory in Home app

### HomeKit switch doesn't synchronize

This issue was fixed in version 0.0.4. Verify you're using the latest code version that updates **both** `currentState` and `targetState`.

## 📊 Monitoring and Logs

### Serial Monitor (115200 baud)

The system displays:
- HomeKit connection status
- CAN messages sent and received (if `DEBUG_MODE`)
- Gate state changes
- Obstruction detection
- Communication timeouts

Example output:
```
┌── CAN STATE RECEIVED ───┐
  Gate state: OPEN
  → HomeKit CurrentState: 2 → 0
  → HomeKit TargetState: 1 → 0
```

## 🔐 Security

- ⚠️ **Do not expose** ESP32 directly to the Internet
- Use HomeKit via Apple **home hub** (iPad/HomePod/Apple TV)
- HomeKit protocol is **end-to-end encrypted**
- Change setup code if compromised

## 📈 Possible Enhancements

- [ ] Add temperature/humidity sensor
- [ ] Opening/closing history
- [ ] Push notifications on obstruction
- [ ] Web configuration interface
- [ ] Support for multiple gates
- [ ] Integration with home alarm system

## 🤝 Contributing

This project is shared for the community. Feel free to:
- Report bugs via Issues
- Propose improvements via Pull Requests
- Share your adaptations

## 📄 License

Open-source project free to use and modify.

## ✍️ Author

**Dominique**  
Contact: dominique@locoduino.org  
Website: [Locoduino.org](https://www.locoduino.org)

## 🙏 Acknowledgments

- [HomeSpan](https://github.com/HomeSpan/HomeSpan) by Gregg E. Berman
- [ACAN_ESP32](https://github.com/pierremolinaro/acan-esp32) by Pierre Molinaro
- Arduino and ESP32 community

## 📚 Resources

- [HomeSpan Documentation](https://github.com/HomeSpan/HomeSpan/blob/master/docs/Reference.md)
- [CAN 2.0 Specifications](https://www.can-cia.org/can-knowledge/can/can-specifications/)
- [HomeKit Accessory Protocol](https://developer.apple.com/homekit/)

---

**Version**: 0.0.4  
**Date**: October 2025  
**Tested with**: ESP32 DevKit v1, HomeSpan 1.8.0, iOS 17+
