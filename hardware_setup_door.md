# Smart Door Hardware Setup Guide (Final)

This configuration provides **Hardware-Level Cutoff**, **Manual Override Support**, and **Reliable 2-State Sensing** using a single-wire trigger path.

## 1. Wiring Diagram (Grounded Cutoff)

```
[ Arduino Pin ] ----+---- [ Relay Trigger IN ]
                    |
[ Manual Switch ] --+---- [ Reed Switch (NC) ] ---- [ GND ]
```

### How it works:
1.  **Hardware Cutoff:** The Reed switch is the only path to Ground for the trigger signal. If the door opens the Reed switch, the Relay **cannot** turn on, regardless of the Arduino or Manual Switch state.
2.  **Series Protection:** Because the Manual Switch is wired "behind" the Reed switch, the limit switch protects the system even during manual operation.

---

## 2. Software Logic (The Sensing Trick)

To read the door state while idle, we treat the Trigger Pin as a Sensor Pin.

### Idle State (Monitoring):
```cpp
pinMode(REL_OP, INPUT_PULLUP);
if (digitalRead(REL_OP) == 0) {
  // Reed is CLOSED (Door is not at limit)
} else {
  // Reed is OPEN (Door is fully open)
}
```
*The weak current from the pull-up (~0.2mA) is enough for the Arduino to sense the Ground but too weak to activate the Relay LED/Optocoupler.*

### Moving State (Activating):
```cpp
pinMode(REL_OP, OUTPUT);
digitalWrite(REL_OP, LOW); // Trigger Relay ON
```

---

## 3. The "Closed" Sensor (3-Wire Mechanical)
Continue using the 3-wire mechanical switch as a dedicated Ground-pull for the `CL_SENS` pin.

*   **COM:** Ground
*   **NO (Normally Open):** Arduino `CL_SENS` pin (with `INPUT_PULLUP`).
*   **NC (Normally Closed):** In series with the `REL_CL` trigger (identical to the Reed setup above).

---

## 4. Key Advantages
*   **Antenna Effect:** By referencing everything to **Ground**, we eliminate the floating-voltage issues caused by long wires running near 220V motors.
*   **Fail-Safe:** If a wire breaks, the circuit opens, and the relay turns off (safety first).
*   **Minimal Pins:** You can actually sense and trigger using the **same wire**, saving Arduino pins for other features.
*   **Manual Logging:** The Arduino can detect if the manual switch is pressed (the pin goes to 0 while in monitoring mode), allowing you to log manual door operations.
