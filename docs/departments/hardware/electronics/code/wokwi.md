# Wokwi - Online embedded systems emulator

[Wokwi](https://wokwi.com/) allows you to create circuits and test them online. It has compatibility with arduinos, esp32s, stm32s, and pi picos. It is useful to test code when you do not have the hardware at hand.

## How It Works

1. Log in with an account
2. Input code into sketch.ino. If you have multiple files that you used, you can press the downward-facing triangle (▼) next to the file tabs and add a new file there.
3. You can edit the diagram on the right, visual, panel and add components using the blue plus. You can then draw connections/wires by clicking between pads.
4. This will edit diagram.json, you can save this and push it to the repo if you want to share it with the rest of the team.
5. Press the play button, your program will compile and then automatically run.
6. You can interact with the components during the simulation (eg. press buttons, slide pots).
7. If the microcontroller prints something to serial, the serial monitor will appear and you can send it messages using the textbox at the bottom.

## Potential Pain Points

- I don't have all the components I need!
    - You are best off just approximating what the rest of the circuit does through some other form of visualisation, such as LEDs.
- I can't use custom libraries!
    - Contact Tommy Byrne on the slack for access to an account that is on the paid hobby plan (that allows custom libraries)
