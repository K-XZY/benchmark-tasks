# Open a keypad home safe: key the code, turn the knob within the 5 s the green light gives, swing the door open

A SentrySafe X055 stands on the table with its door towards the arms. The right gripper keys 1-7-5-0-0 and # on twelve round keys (10 mm across, 16 mm apart, 1 mm of travel through a click), grips the 45 mm knob inside the 5.0 s the green light stays on, turns it a quarter turn and starts the door; the left gripper takes the knob and swings the door to 70 degrees. Then both let go and clear the safe.

## Run

From this folder, on a GPU machine:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,wrong,late,first,pull,shallow
```

The harness's instruction (Dream-1 run): `--instruction "Open the safe, leave its door open and take your hands away. The code is 1 7 5 0 0, then the # key."`

`KS_DX` (default 0 m) sets the whole safe that far out in x.

## Scoring, and the routes that score less

0.25: the code was accepted (green light). 0.5: the knob passed 80 degrees while the green light was on (bolts drawn). 0.75: the door reached 20 degrees. 1.0: the door at 70 degrees or more, every fingertip 3 cm clear of the safe, and all still for 1 s. 0 if the safe moves 2 cm or tips 5 degrees.

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | right keys 1-7-5-0-0-#, grips the knob and turns it (bolts drawn 3.4 s into the window), pulls the door to 20 degrees; left takes the knob and swings the door to 78 degrees; both clear | 1.0 (487 steps) |
| one_hand | the right hand alone does all of it, the door swung by the same arc | 1.0 (440 steps): one hand is enough, as the context says |
| wrong | the 4th digit wrong (17550), then # | 0 (yellow light, keys ignored for 60 s) |
| late | the code, then the knob only after the 5 s window has closed | 0.25 (the knob stays locked) |
| first | the knob is turned hard before any code is keyed, then the hands stop | 0 (the knob does not move, no code) |
| pull | the code, then the knob gripped and the door pulled without turning it | 0 (the bolts hold the door and the pull drags the safe 46 mm) |
| shallow | every key only touched 13 mm short, never pressed through its click | 0 |
| none | nothing | 0 |

The demo is 24 s of sim time; the run limit is 2.5 times that.

## Lock model

The lock's electronics are code (`Trial` in `task.py`): a key press is the key's own travel going past its 0.7 mm click (a 1 mm key, 2.4 N click), read every control step (20 Hz), so a finger must push the key through its click. After 1-7-5-0-0 and # the green light is on for 5.0 s and the knob joint is free; outside the window the knob is held where it is. Past 80 degrees of knob inside the window the bolts are drawn: the door joint is freed. Any other code before # gives the yellow light and the keys are ignored for 60 s. The bolts, the lock's solenoid and the lights are not separate parts. Magnetic or electronic behaviour beyond these documented conditions is not simulated.

## Real trial

Programme 1-7-5-0-0 as the user code (the manual: press the button in the battery compartment, then the code and #); stick a marker line across the knob's face and a tape mark at 80 degrees on the door; a protractor on the hinge. Hands-clear and stillness are read from the video.

## Sources

- SentrySafe X055 Digital Security Safe, Master Lock technical literature (https://solutions.masterlock.com/Technical_Literature/Sentry_Safe/Security_Safes/Digital/X055): exterior 13.8 in wide, 8.7 in high, 9.7 in deep (35.1 x 22.1 x 24.6 cm, modelled as listed); 14.2 lb (6.44 kg); '2 steel construction live-locking bolts'; digital lock with override key.
- Master Lock Security Safe Owner's Manual 402777 (https://cdn.masterlock.com/sentrysafe/resources/pdf/manuals/402777.pdf): factory code 1-7-5-0-0-#; 'Enter your factory code or user code followed by #. The green LED light will go and remain on for 5 seconds. Within 5 seconds, turn the knob clockwise and pull open the door.'; '5 beeps and a yellow LED light after entering your code. Invalid code. Wait one minute and retry your code.'; the safe is locked by turning the knob back, so the model has no return spring.
- Estimates, as the maker publishes no drawings or part figures: door size (33 x 19 x 1.8 cm), door mass (2.3 kg), the door set in the front face, keypad layout (12 round keys 10 mm across, 16 mm across and 14 mm down), key proud height (2 mm), travel (1 mm), spring and click force (1.0 N rest, 2.4 N click), knob size (45 mm across, 22 mm long), knob friction (0.1 N m), hinge side and friction (0.02 N m), light positions, steel on wood friction (0.4). The keypad and knob heights (15 and 10.5 cm) are placed within the wrists' reach.
- All meshes are built in `task.py` from boxes and cylinders; no third-party meshes are used.

SentrySafe and Master Lock are trademarks of Master Lock Company LLC, named for identification only; this task is not affiliated with them.

## Licence

MIT
