# Simon Game for MSPM0

Embedded Simon game built in C for the MSPM0 board with LEDs, buzzer, buttons, and a bonus accelerometer mode.

## Description

This project was my midterm project for ELEC 327. The main goal was to build a full Simon memory game on embedded hardware and make it feel like a real little device instead of just a lab demo.

The game has a boot animation, random sequence generation, button-timed sound/light feedback, win/loss states, difficulty select, and a separate accelerometer mode. Most of the project logic is handled with a timer-driven state machine so the hardware behavior stays pretty predictable.

## Demo

Video Demo: https://www.youtube.com/watch?v=rrEui6MRfm0
![Image](image.png) 

## Tech Stack

- C
- TI MSPM0 microcontroller
- SPI for LEDs and accelerometer
- PWM buzzer output
- timer interrupt / state machine game loop
- Code Composer Studio

## Features

- classic Simon gameplay with 4 buttons, 4 tones, and 4 LEDs
- random sequence each new game
- difficulty selected from the startup button
- win, loss, boot, and record animations
- tracks fastest win during the current power cycle
- accelerometer mode that changes LED position and color based on tilt / movement

## Setup

Import the project into Code Composer Studio, build it, and flash it to the Simon board. On boot, press one of the 4 buttons to start a game. Hold `SW1 + SW4` during boot to enter accelerometer mode.

## Architecture

The code is split so the hardware drivers stay separate from the game rules.

```text
buttons / leds / buzzer / timing / accelerometer
                    |
                    v
          state_machine_logic.c
                    |
                    v
                 simon.c
```

`simon.c` does startup and runs the main loop. `state_machine_logic.c` handles the phases of the game, round progression, input checking, delays, and animations. The other modules handle specific hardware pieces.

## Future Improvements

- save best score across resets instead of only per power cycle
- add a cleaner debug mode for sensor bring-up
- make the accelerometer mode a little more game-like
- maybe use true hardware randomness instead of startup timing mix

## Challenges and What I Learned

The hardest part was making the game feel correct in real time. The logic itself was not too bad, but getting button debounce, hold behavior, timeouts, and sequence playback to all work together without weird edge cases took a lot more thought than I expected. I ended up learning that embedded projects are mostly about state control and timing, not just writing functions that work once.

I also learned a lot about splitting code into modules that actually make sense. Early on it was tempting to keep everything in one file, but separating the hardware drivers from the state machine made debugging a lot easier. The accelerometer mode also taught me that sensor features are messy in practice, especially when you have to filter readings, handle bad SPI reads, and still give useful output on limited hardware.

## Credits

Solo project built by Zane Hensley.
