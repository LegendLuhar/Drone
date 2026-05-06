Confirmed from the files in your repo; I could answer all listed items except one nuance (multi-button arbitration is not explicitly specified).
PA23/PA24/PA25/PA26 button mapping + active level
PA23 → BUTTON1_IO → physical SW1 (Green in lab notes)
PA24 → BUTTON2_IO → physical SW2 (Red)
PA25 → BUTTON3_IO → physical SW3 (Yellow)
PA26 → BUTTON4_IO → physical SW4 (Blue)
Inputs are active-low.
Sources:
Workspace/Lab5/docs/PROJ.md (states button GPIOs are PA23..PA26; maps Simon notes to SW1..SW4)
PCBs/Simon-2026-JLC/simon_io.kicad_sch (labels BUTTON1_IO..BUTTON4_IO on SW1..SW4)
PCBs/Simon-2026-JLC/Simon-2026-JLC.kicad_pcb (nets BUTTON1_IO..BUTTON4_IO on MCU pads 43..46)
PCBs/Simon-2026-JLC/Simon-2026-JLC.kicad_sch (MSPM0 symbol pin numbers: pad 43/44/45/46 are PA23/PA24/PA25/PA26)
Workspace/Lab5/hw_interface.c (pull-up enabled, comment: “switch connects to ground”)
Buzzer output (timer channel + mux)
Buzzer net is BUZZER_IO, connected to MCU pad 30, which is PA15.
Timer output used is TIMA1 CCP0 via pin mux IOMUX_PINCM37_PF_TIMA1_CCP0 (configured on PA15).
Sources:
Workspace/Lab5/docs/PROJ.md (“buzzer is connected to TIMA1 channel C0”)
Workspace/Lab5/hw_interface.c (IOMUX_PINCM37_PF_TIMA1_CCP0 // TIMA1-CCP0 on PA15)
PCBs/Simon-2026-JLC/Simon-2026-JLC.kicad_pcb (BUZZER_IO on MCU pad 30)
PCBs/Simon-2026-JLC/Simon-2026-JLC.kicad_sch (pad 30 is PA15_/_A1_0/DAC_OUT)
Timer math (LOAD, CC0)
Yes, for this lab setup the math is:
LOAD = (timer_clk / f) - 1
CC0 = (LOAD + 1) / 2 for 50% duty.
Sources:
Workspace/Lab5/docs/PROJ.md example: LOAD = 3999 with period LOAD+1, and CC_01[0] = (LOAD+1)/2
Workspace/Lab5/hw_interface.c same implementation/comments
Buzzer frequency constraints + octave recommendation
Hardware note says buzzer fundamental is roughly 4–8 kHz; lower notes are more nonlinear/mechanical.
Recommended cleaner set: two octaves higher (e.g., G6, E6, C6, G5).
Source: Workspace/Lab5/docs/PROJ.md
Behavior when multiple buttons are pressed
Explicitly specified:
any button press exits song mode
tones stop when all buttons released
each button corresponds to a distinct tone
Not explicitly specified: arbitration/priority/mixing when 2+ are held simultaneously.
Source: Workspace/Lab5/docs/PROJ.md
High-level Lab 5 behavior (song then button mode)
Confirmed: startup state plays looping “Mary Had a Little Lamb”; pressing any button transitions to button-tone behavior.
Source: Workspace/Lab5/docs/PROJ.md
Return to song mode without reset?
Lab text says song state is active upon reboot, and pressing any button transitions out of it; no return path is defined.
So expected behavior is do not return to song mode unless reset/reboot.
Source: Workspace/Lab5/docs/PROJ.md
Debounce recommendation/timing
Debounce is required (“not bouncing” / “debounce like Lab 3” in course notes).
No explicit fixed debounce duration is given in PROJ.md; only a general 10 ms tick context is mentioned for music timing.
Sources:
Workspace/Lab5/docs/PROJ.md (“pressed (not bouncing)”, mentions 10 ms interrupt ticks)
Workspace/Lab5/docs/random_guidelines.md (“debounce buttons like Lab 3”)

Chosen implementation constants for this project:
- tick period: 10 ms (state machine update cadence)
- debounce time: 30 ms (3 ticks)
- quarter note total duration: 700 ms (70 ticks)
- half note total duration: 1400 ms (140 ticks)
- whole note total duration: 2800 ms (280 ticks)
- internote gap (silence between notes): 80 ms (8 ticks)

Notes on these constants:
- note durations include the gap, so ratio requirements stay correct (h = 2q, w = 4q).
- multi-button arbitration is intentionally deferred for now per project decision.