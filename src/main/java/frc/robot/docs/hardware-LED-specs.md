# WS2811 12V COB RGB LED Strip — FRC Programming Reference

This document abstracts the key technical details of the LED strip for use in **FIRST Robotics Competition (FRC)** projects, particularly when writing **Java / WPILib** guides or student documentation.

---

## 1. LED Type & Control Protocol

- **LED Type:** WS2811 addressable RGB
- **Control Protocol:** WS2811 (single-wire serial data)
- **Data Direction:** One-way  
  - Indicated by **white arrow** printed on the PCB
- **Addressing Granularity:**
  - 1 WS2811 IC controls **36 physical LEDs**
  - **20 ICs per meter**
  - Effects are applied per IC group, **not per individual LED**

> ⚠️ Programming note: You are controlling **20 logical “pixels” per meter**, not 720 independent LEDs.

---

## 2. Electrical Characteristics

- **Operating Voltage:** 12 V DC
- **Logic Signal:** WS2811 data (typically 5 V logic; 3.3 V may work but is not guaranteed)
- **Grounding:** Controller ground **must be common** with LED strip ground
- **Power Considerations:**
  - Power injection recommended for longer runs
  - Do **not** power directly from RoboRIO 12V pins

---

## 3. Physical Characteristics

- **Strip Length:** 1 meter (3.2 ft)
- **LED Density:** 720 LEDs per meter
- **Addressable Density:** 20 WS2811 ICs per meter
- **Strip Width:** 12 mm
- **Construction:** COB (Chip-On-Board)
- **Substrate:** Flexible PCB (FPCB)

---

## 4. Wiring & Pinout

| Wire Color | Function |
|-----------|---------|
| **Red**   | +12 V Power |
| **White** | Ground (GND) |
| **Green** | Data In (DIN) |

- **Connector Type:** 3-pin JST-style
- **Important:** Data must enter the strip **in the direction of the arrow**

---

## 5. Timing & Protocol Notes

- Despite “SPI” appearing in marketing text, this strip uses **standard WS2811 single-wire signaling**
- Update rate is slower than WS2812 due to grouped LED control
- Animations should be designed around **segment-based control**

---

## 6. FRC Compatibility Notes

- ❌ **Not directly compatible** with WPILib `AddressableLED` (WS2812 / 5V)
- ✅ Requires an external controller:
  - Arduino / RP2040
  - CAN-based LED controller (e.g., CTRE CANdle with 12V WS2811 support)
- ⚠️ Level shifting may be required if driving data from RoboRIO GPIO

---

## 7. Programming Abstractions (Recommended)

Treat the strip as **segment-addressable**, not pixel-addressable.

- **Logical Pixels per Meter:** 20
- **LEDs per Logical Pixel:** 36

Example abstraction:
```java
int meters = 1;
int logicalLength = meters * 20;
```

Use logical pixels when:

- Setting alliance colors
- Displaying robot state
- Creating simple animations (chase, blink, pulse)

## 8. Power & Safety Guidelines (FRC)

- Power from PDH/PDP fused 12V output
- Recommended fuse: 3–5A per meter (depending on brightness)
- Add strain relief to JST connector
- Avoid sharp bends near solder joints

## 9. One-Sentence Summary

A 12V WS2811 COB RGB LED strip with 720 LEDs per meter, controlled as 20 addressable segments per meter (each segment = 36 LEDs), using a single-wire WS2811 protocol with red = +12V, white = GND, and green = data, requiring external control hardware for FRC Java use. It will be controlled with CTRE CANdle.