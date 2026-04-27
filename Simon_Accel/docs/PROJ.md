Rice ELEC327 - Digital Systems Laboratory
Bonus
Home
Parts/Notes
Syllabus
Labs
Midterm Project (2026): Creating “Simon”
There are three goals for this project:
To design and implement a somewhat complex embedded system that is evaluated by your peers based on principles like usability and enjoyment.
To construct, organize and implement a multi-module code project.
Due date: 3/27/2026
How will it be evaluated?
The code for your Simon game will be submitted via canvas. Please include only code files and a “README.txt” that describes (a) briefly your code architecture and approach, (b) the score that you think you should get based on the rubric below, and (c) how to access or run any special features.

In addition to peer grading, Prof. Kemere or the lab assistants may run your code on their boards for evaluation. Your score for the final project will be based on the rubric below.

Students from the class will be given the chance to evaluate based on a simpler rubric. These scores will not be used for grading, but course bonus will be assigned based on the final ranking of Simon programs. Additionally, a small bonus will be given for each of your peer’s Simon games you evaluate.

There is a hard deadline of Friday, March 27, 2026.

#### Rubric: (10 pts each, 150 total)

Power-on animation: When powered on, does the game display an animation that involves changing patterns of the lights?
Power-on animation: Does the power on animation also include changing sounds?
Power-on animation: Does the power-on animation transition to game play when a button is pressed?
Random-Gameplay: Is the first element displayed random, including between power cycles?
Gameplay-ButtonFeedback: When a button is pressed during play, does the light asssociated with that button track the button press, i.e., turn on when it is depressed and turn off when it is released?
Gameplay-ButtonFeedback: When a button is pressed during play, does the sound asssociated with that button track the button press, i.e., turn on when it is depressed and turn off when it is released?
Gameplay-Timeout: Does the game transition to a “loss” state if no button is pressed within some period during the response?
Gameplay-MisplayingError: Does the game transition to a “loss” state if a wrong button is pressed during a response?
Gameplay-Win Animation: If the player wins (correctly responds to a sequence of length 5), does the game display an animated sequence of lights and sounds? (This can be the same as the power on sequence but must be different than the gameplay-lose animation.)
Gameplay-Lose Animation: If the player loses (times out or responds incorrectly), does the game display an animated sequence of lights and sounds? (This can be the same as the power on sequence but must be different than the gameplay-win animation.)
Gameplay-Playability 1: Is there a pause and/or other meaningful transition between when the player releases a button to start the game (i.e., ending one of the animations) and when the first light/sound of the sequence is played (so that the game is clearly distinguished from the before-game animations)?
Gameplay-Playability 2: Is there a pause and/or other meaningful transition between when the player releases the last correct button of a sequence and when the subsequent sequence+1 is played (so that the game clearly distinguishes between the player’s button presses and the sequence)?
Gameplay-Difficulty: Can the length of a sequence required to win be changed by changing no more than one line of code?
Writeup and code- Are design choices well documented? Does the architecture make sense? Are there good comments?, functions (libraries?) used intelligently? - 20 pts
Background
The game “Simon” is a classic toy which tests the working memory of the player. For the final project, you will create a small, coin cell-operated Simon game. This will involve PCB design and fairly involved firmware to achieve an enjoyable user experience. In addition, you will be invited to add some sort of interesting feature beyond the Simon game. In the past, this has ranged from simple animations to a mode in which the device becomes a music sequencer.

Rules for the ELEC327 version of the Simon game
In our variant of Simon, the point of the game is to learn a sequence of M button presses. Each button corresponds to one of the LEDs and to one frequency of PWM tone. On the nth turn, the device plays the first n elements of the sequence. So one tone/LED on turn one, two on turn two, etc. The pattern of the n-1 elements of the sequence are the same as the previous turn. Each turn, after the sequence is played, the player must push the appropriate buttons to recreate the sequence. If they do it correctly, the game proceeds to the n+1th turn. If they make an error or wait too long (measured by the time since the last button press), they lose. Winning corresponds to playing the full M-element sequence. If the player loses, the device should play a “Game Over - Lost” animation that continues until it is reset (e.g., by power cycling or pressing a button). Similarly, if the player wins, the device should play a “Game Over - Won” animation until it is reset.

Details:

On initial boot, the devices should play either one of the “Game Over” animations or a third novel animation. Pushing any button should then start the game.
Sequences should be random each game (make sure your random routine produces a different value on startup!)
When a button is pressed, the appropriate LED should flash (and tone should sound).
The light and sound associated with the button should be displayed/played as long as the button is held down, and then should stop. (So not like in the example video below where they play for a constant length of time regardless of how long the button is pressed for.)
The timeout error should calculated relative from when a player depresses the previous button. A timeout of 1-2 s is appropriate.
An erroneous button press should result in a game over either when the button is depressed or when it is released.
It’s very helpful for playability if there’s a slight pause and/or additional animation (i) when the player releases the button to transition from the animations and (ii) after the player releases the last correct button press in a sequence to indicate the next presentation of sequence+1. Without these pauses, the distinction between the first light/sound of the sequence and the button response light/sound can often be hard to distinguish.
You should make the maximum sequence size adjustable by changing at most one line of code.
Helpful Tips
Much of Simon can be pieced together by using code developed from previous labs. To help you,

A template workspace has been provided
For your state machine logic, build off of your solution for Lab 6
Note that an example random number generator is given in the workspace which provides a rand() which generates random numbers in [0,3] and an srand(seed) which sets the LFSR state.
Remember that the LFSR random number generator can be used to both generate the button/light sequences and check the playback (if you store the seed used and start over during the button pressing)
Here’s a gist that shows how to turn on the TRNG module and grab a random number from it - https://gist.github.com/kemerelab/a8de90811a15cb982f5fb3d0e2410c70
Physical information is given in the ELEC327 Simon repository

