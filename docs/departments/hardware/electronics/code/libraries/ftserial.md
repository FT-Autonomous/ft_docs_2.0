# FTSerial

Custom Arduino library for **non-blocking** serial reading. Use this on Rigby instead of `Serial.readString()` (which blocks your control loop).

Source: `FT-Hardware/Code/Libraries/FTSerial/`

## Installation

1. Copy `FTSerial/` from `FT-Hardware/Code/Libraries/FTSerial/` into `~/Arduino/libraries/`
2. Restart the Arduino IDE
3. `#include <FTSerial.h>`

## Quick Start

```cpp
#include <FTSerial.h>

FTSerial ftSerial(Serial, 24); // serial port, buffer size in bytes

void setup() {
  Serial.begin(115200);
}

void loop() {
  float value;
  if (ftSerial.readFloat(value)) {
    // got a value
  }
  // control loop continues unblocked
}
```

## Function

```cpp
FTSerial(Stream &serial, byte bufferSize = 32)
```
Constructor. `bufferSize` should fit your longest expected message.

---

```cpp
String readUntilNewline()
```
Returns the next `\n`-terminated line, or `""` if not ready yet. Handles `\r\n`.

---

```cpp
String readWithMarkers(char startMarker = '<', char endMarker = '>')
```
Returns content between markers (e.g. `<steer,45.0>`), or `""` if not ready yet.

---

```cpp
bool readFloat(float &result)
```
Reads a newline-terminated float. Returns `true` and sets `result` on success. Note: invalid strings parse as `0.0`.
