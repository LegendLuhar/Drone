Midterm Bonus - Better Simon
[0.25 - 0.5 points] - Make nice Youtube video showing your Simon game in action. Make sure that “Rice” and “ELEC327” are in the tile or otherwise searchable. Your demo video should show you playing through to a win, running the “Game Over - Win” animation, and then pushing a button to restart and then playing through but making a mistake to generate the “Game Over - Loss” animation. More points will be given for higher production quality, humor, or other aesthetics.

[0.5 - 1 point each] - Possible improvements or modifications for Simon:

As described for Lab 6, implement a Simon that is driven by GPIO interrupts rather than just a simple timer tick.
Allow the user to select the level of difficulty - the timeout period and/or speed of initial sequence playing. You could implement this by which button is used to start the game after a reset.
Implement the reset functionality by requiring multiple buttons to be pressed simultaneously
Add double or triple button presses to the pattern
Do something interesting with the LEDs (i.e., using the color channels creatively or specifying interesting patterns)
Track fastest performance (i.e., how fast the entire game goes) and reward the player for beating their previous best
Use the PCB for something else in addition to Simon (e.g., a different game, a music sequencer) that is executed depending on some startup condtion.
[1-3 points] - Use the accelerometer on the Simon PCB to do something interesting. For example, have the LEDs signal the orientation of the PCB, so that if you hold it sideways, the LEDs always point up. Do something cool where you attach the PCB to a string and swing it in the air to create forces larger than 1g, with the LED denoting the intensity of acceleration.

