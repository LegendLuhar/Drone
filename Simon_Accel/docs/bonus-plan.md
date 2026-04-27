# bonus plan

## goal

This plan focuses on bonus items that add visible value fast and fit the current code structure with small to medium changes. The goal is to get a majority of the practical bonus items without turning the project into a large rewrite.

## planned bonus items

I am planning to do these bonus items:

1. difficulty select at startup
2. multi-button reset
3. more interesting LED behavior
4. fastest performance tracking
5. youtube demo video at the end

These give the best return for the amount of work because they are easy to show, easy to explain, and build on the current timer-driven state machine.

## excluded bonus items

I am planning to exclude these for now:

1. gpio interrupt driven Simon
2. double or triple button presses in the pattern
3. second non-Simon mode on startup
4. accelerometer feature

Reason for exclusion:

- gpio interrupt driven Simon is useful, but the current timer-loop version already works well and this would need a bigger control-flow refactor for limited visible payoff.
- double or triple button presses would change the core game model and increase input edge-case work a lot.
- a second mode is interesting, but it adds another product inside the project and is lower ROI than polishing the main game.
- the accelerometer feature is the biggest hardware and integration task, so it has the lowest ROI under time pressure.

## implementation plan

### 1. difficulty select at startup

Plan:

- During the boot animation, use the first pressed button to choose one of four presets.
- Map presets to sequence speed and timeout values.
- Keep the existing default behavior simple: press any button to start, but remember which button was used.

Why this is high ROI:

- Very little code change.
- Easy to demonstrate.
- Makes the game feel more complete.

Implementation notes:

- Add a small difficulty field to the main state.
- Add per-difficulty constants for playback on-time, playback off-time, and timeout.
- Reuse the existing `PHASE_WAIT_START_RELEASE` path so startup behavior stays clean.

### 2. multi-button reset

Plan:

- In win and loss animation states, require two buttons at the same time to restart.
- Keep single-button press behavior disabled in those states once this change is added.

Why this is high ROI:

- Small change.
- Matches the bonus spec directly.
- Avoids accidental restart during end animations.

Implementation notes:

- Add a helper that counts currently held buttons.
- In the win/loss animation states, only restart when the held count is at least two.

### 3. more interesting LED behavior

Plan:

- Improve the current animations so they look more polished and more obviously different.
- Add a richer boot animation and stronger win/loss color patterns.
- Add short LED transition flashes between rounds or on mistakes.

Why this is high ROI:

- Very visible improvement.
- No hardware changes needed.
- Helps both project quality and bonus value.

Implementation notes:

- Expand the animation tables in `state_machine_logic.c`.
- Add a few more LED frame patterns in `colors.c`.
- Keep the logic table-driven so the code stays simple.

### 4. fastest performance tracking

Plan:

- Measure total game time from the first playback start to win.
- If the player wins faster than the stored best, play a special short reward animation.
- Keep only one best score in RAM for the current power cycle unless nonvolatile storage is already available.

Why this is high ROI:

- Small to medium code change.
- Adds replay value.
- Easy to mention in a demo and writeup.

Implementation notes:

- Add `current_game_ticks` and `best_win_ticks` to the state.
- Increment `current_game_ticks` during active gameplay states.
- On win, compare and trigger either normal win or best-score win animation.

## optional stretch item

If time is still available after the items above, the next best extra would be the gpio interrupt driven version. It is not in the main plan because it is more of an architectural rewrite than a visible feature.

## suggested order

1. difficulty select
2. multi-button reset
3. LED improvements
4. fastest performance tracking
5. youtube demo video after code is stable

## files likely to change

- `state_machine_logic.h`
- `state_machine_logic.c`
- `colors.h`
- `colors.c`
- maybe `writeup.txt` later to document the finished bonus features
