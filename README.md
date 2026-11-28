# MyRobot
The template repository for the COGS 300 lab robots. You can edit this template as much as you want.

## Lab 07 – Line following (`src/Lab07_LineFollow/`)
Built on `docs/sketch_wallfollowing.ino` (same motor wiring and helpers). Adds IR sensors:

| IR sensor | Pin |
|-----------|-----|
| Left      | A0  |
| Center (optional) | A1 |
| Right     | A2  |

VCC → 5V, GND → GND, OUT → pin. Ultrasonics stay on 10–13, motors on 3,4,5,7,8,9.

Set `MODE` at the top of the sketch:
1. `MODE_SENSOR_TEST` – tune each pot so it reads **1 on white tape, 0 on floor** (Serial Monitor, 115200).
2. `MODE_START_STOP` – lab step 2 (`START_ON_LINE` picks stop-on-line vs. drive-until-tape-ends).
3. `MODE_PING_PONG` – lab step 3, bang-bang follower ping-ponging along the tape edges.
4. `MODE_BAYES` – lab step 4, Bayes filter over the line's position + P-control; remembers which side the line left from so it can recover on sharp corners.

Mount the centre sensor over the tape and the left/right sensors just outside its edges. Measure and set `SENSOR_SPACING_CM` (centre → side sensor, ~3–3.5 cm for 4.8 cm tape) and `TAPE_WIDTH_CM`.
