# SO-101 Leap Motion Teleop

**[Watch the demo on Instagram](https://www.instagram.com/reel/DfVRupJgW0j/)**

Control an SO-101 robot arm with your bare hand using a Leap Motion controller, my own inverse kinematics, and a live MuJoCo digital twin. Your palm moves the arm, your wrist tilts and rolls the gripper, and a pinch closes the gripper.

This repo covers the parts that are NOT in the official LeRobot docs: the Leap integration, the IK, and all the small tweaks that make it feel smooth and intuitive instead of jittery.

## What you need

**Hardware**

- SO-101 follower arm (print it yourself from the TheRobotStudio SO-ARM100 repo, or buy a kit)
- 6 x Feetech STS3215 servos, **12V version, 1/345 gear ratio**. I went with 12V over the 7.4V version for the extra torque. The shoulder holds most of the arm's weight, and it needs it.
- Waveshare Bus Servo Adapter (or the driver board that comes with your kit)
- 12V power supply, **at least 5A**. A small 2A brick is fine for setup and calibration, but not for the arm moving under load.
- Thicker power wire (18 to 20 gauge). Arduino jumper wires are too thin once the arm is actually working.
- Leap Motion controller. The original one from years ago works fine.
- A Mac or PC. I built this on an Apple Silicon Mac.

**Software**

- LeRobot (follow their install guide)
- MuJoCo (`python -m pip install mujoco`)
- Ultraleap tracking software (Hyperion / Gemini) for your OS
- Ultraleap's Python bindings: github.com/ultraleap/leapc-python-bindings
- The SO-ARM100 repo, cloned next to this one, for the simulation model

## Setup

### 1. Build and calibrate the arm with LeRobot

LeRobot's own docs already do this well, so follow them for assembly, motor ID setup, and calibration. Two things to watch:

- **Remember your calibration id.** Every script here uses `my_follower_arm`. If yours is different, change it everywhere, or LeRobot won't load your calibration and the joint numbers will be wrong.
- **Center the wrist roll before calibrating.** See the wrist roll section below. This saves you a big headache.

### 2. Install the Leap software

1. Download and install Ultraleap's hand tracking software (Leap Motion is now Ultraleap, and the software is called Hyperion, or Gemini on older pages). Ultraleap's guide to which version to get for your OS: \[which software do I need\](https://support.ultraleap.com/hc/en-us/articles/360005517598-What-software-should-I-use-if-I-am-on-a-Mac). Plug in the Leap and open the visualizer to check your hand shows up.
2. Clone the Python bindings and install them into the same environment as LeRobot, using `python -m pip`.
3. Run one of their examples and check that hand positions print in the terminal.

### 3. Folder layout

```
your-folder/
├── lerobot/
├── SO-ARM100/
├── leapc-python-bindings/
└── my_scripts/        (this repo's scripts)
```

Keep your scripts in their own folder, not loose next to the `lerobot` folder, or Python can grab the wrong thing on import.

### 4. Run it

```
mjpython leap_teleop.py
```

On a Mac, any script that opens the MuJoCo viewer must be run with `mjpython`, not `python`.

### Startup checklist for leap\_teleop.py

Do these in order, every time:

1. **Leap plugged in and Ultraleap tracking running.** The script doesn't talk to the sensor directly. It reads hand data from Ultraleap's tracking service through the Python bindings, so that service has to be running first. Quick check: open the Ultraleap visualizer and make sure it sees your hand.
2. **Robot powered on.** Plug in the USB to the driver board AND turn on the 12V supply (bench supply or the 12V adapter). USB alone only carries data. If the servos have no power, the script can't read any joint positions back and it will fail on startup with timeout errors.
3. **Run the script:** `mjpython leap_teleop.py`. The sim window opens, the arm holds its current pose, and it starts following your hand as soon as it sees it.

## How to use it

Hold one **hand** about 20 cm above the Leap, palm down, fingers relaxed.

Keep your other hand out of the frame. The script follows whichever hand the Leap sees first, so a second hand can take over. Also sit back a little: if you lean in close over the sensor, it can occasionally mistake your face for a hand.

| Your hand | What the arm does |
| --- | --- |
| Move your palm | The gripper follows, scaled down |
| Tilt your hand up or down | Wrist bends up or down |
| Rotate your hand | Wrist rolls |
| Pinch thumb and finger | Gripper closes |
| Make a fist, then open | Pause or resume |
| Pull your hand away | Arm freezes in place |

### It's relative, like a clutch

The arm doesn't jump to where your hand is. When your hand appears, the arm stays put, and from then on it copies how your hand **moves**. So you can start anywhere comfortable.

### The fist pause (this is the important one)

Make a fist and open it again to **pause**. The arm freezes. Now you can move your hand anywhere without the arm following. Make a fist again to **resume**, and it picks up from wherever your hand is now.

Use this whenever your hand runs out of comfortable range. Pause, reset your hand to the middle, resume. It's like lifting a mouse off the desk.

### Gripper lock

If you're holding something and you pause or pull your hand away, the gripper **keeps holding**. When you come back, it stays locked until your pinch catches up to how closed the gripper already is, then you take over smoothly. This stops it from dropping things the moment you open your hand.

## What makes it feel smooth

These are the tweaks that took it from twitchy to clean. All of them are settings near the top of `leap_teleop.py`.

### Smoothing

The Leap's readings jitter a little. Instead of jumping straight to each new hand position, the target moves part of the way there each frame. `SMOOTH = 0.15` is a good start. Lower is smoother but laggier, higher is snappier but shakier.

### Scaling

`SCALE = 0.5` means 10 cm of hand motion moves the arm 5 cm. That gives you finer control. Raise it if the arm feels slow to cover ground.

### Speed limit on the real arm

`MAX_STEP = 3.0` caps how many degrees any joint can move per update. If the tracking glitches for a frame, the arm can't lurch.

### Reach limit

The target can't leave the arm's reachable zone. If your hand goes past the edge, the target stops at the boundary, and the moment you move back in, it responds right away. No more pulling your hand out and back in to recover. There's also a floor so it won't drive into the table. Tune with `R_MAX`, `R_MIN` and `Z_MIN`.

### Split arm and wrist

The IK only uses the first three joints (base, shoulder, elbow) to place the **wrist** where your palm is. The wrist joints then copy your hand's tilt and roll directly. If the IK used all five joints, the wrist would be busy helping with position and couldn't follow your hand, so you couldn't do things like a wave.

### Joint limits

Wrist commands are clipped to the arm's real range, so they can't drift past a hard stop over time.

## Tuning the servos (read this if your arm feels weak)

Out of the box, LeRobot runs the servos gently. That's safe, but small moves often stall short and heavy joints sag. If your arm barely moves or only finishes a move when you push it, this is probably why.

- **Raise the P gain.** LeRobot sets `P_Coefficient` to 16. I use 32 on all joints and 48 on the shoulder, elbow and wrist roll. Above about 64, joints start to buzz and overshoot.
- **These writes only stick with torque off.** Wrap them in `with robot.bus.torque_disabled():`. If you write them while torque is on, the servo quietly ignores them. This one cost me hours.
- **Raise `Max_Torque_Limit` too, not just `Torque_Limit`.** `Torque_Limit` can't go above `Max_Torque_Limit` (680 by default), so setting only one does nothing.
- **Keep the gripper gentle.** I cap its torque at 700 so it can grip without crushing things or straining its gears.

A quick way to tell a tuning problem from a power problem: watch your supply's current. If the arm struggles but the current stays low (well under an amp), the servos aren't trying hard. That's gain, not power.

## The wrist roll fix

The roll joint can turn almost a full circle, but there's a physical bump that blocks a small section. LeRobot treats the roll as a full-turn joint and doesn't record that stop during calibration.

The problem: the roll reading wraps from -180 to +180 somewhere. If that wrap point is inside the arm's usable range, the servo can't cross it, so one direction gets cut short and the arm can jam against the bump.

The fix: during calibration, turn the roll to the middle of its usable range (halfway between the two sides of the bump) before you confirm the middle pose. That puts the wrap point inside the bump, where the arm never goes. Then set the roll limits a few degrees inside the real stops. Mine are -164 to 164.

If the roll ever stops responding after hitting the bump, the servo has probably tripped its overload protection. Power off, turn the wrist back to the middle by hand, let it cool for a minute, and power back on.

## Lining up the digital twin

Run `mirror.py` first. It turns torque off so you can move the real arm by hand and watch the sim copy it.

- If a joint turns the opposite way, flip its `SIGN` to -1.
- If it moves the right way but sits shifted, adjust its `OFFSET` in degrees.

Copy the same signs and offsets into every other script.

## What each script does

Every script is small and does one job, so you can read them in order and learn one idea at a time. Sim-only scripts don't need the arm plugged in.

| Script | Needs the arm? | What it does | What it teaches |
| --- | --- | --- | --- |
| `fk_test.py` | No | Drag the joint sliders in MuJoCo and watch the gripper's XYZ position print live | Forward kinematics: joint angles in, hand position out |
| `ik_test.py` | No | Solves for the joint angles that put the gripper on a red target ball, printing the error in mm as it converges | Inverse kinematics: the hot-and-cold loop, the Jacobian, damping |
| `mirror.py` | Yes | Torque off; move the real arm by hand and the sim copies it | The digital twin, and lining up signs and offsets |
| `keyboard_arm.py` | Yes | Arrow keys drive the base and wrist, showing target vs actual angles | Basic real-arm control, and spotting joints that stall |
| `wave.py` | Yes | Every joint swings in a smooth, offset wave, then returns home | Coordinated multi-joint motion and easing in and out |
| `goto_ik.py` | Yes | Arrow keys move a target ball in the sim; press Enter and the real arm glides there in one smooth move | IK on real hardware, plus smooth trajectories |
| `visuals.py` (FK mode) | Yes | Moves through preset joint angles and prints where the fingertip ends up | Forward kinematics, on the real arm |
| `visuals.py` (nudge mode) | Yes | Wiggles one joint at a time and draws a colored trail at the fingertip for each | The Jacobian you can see: how each joint moves the hand |
| `leap_teleop.py` | Yes | Full hand control with the Leap Motion | Everything combined |
| `check_power.py` | Yes | Reads voltage, load, temperature, gain and torque limits from a servo | Diagnosing weak or stalling joints |

**Suggested learning order:** `fk_test` → `ik_test` → `mirror` → `keyboard_arm` → `goto_ik` → `visuals` → `leap_teleop`.

### The nudge trails, explained

This is my favorite one to learn from. Each joint gets its own color: base red, shoulder orange, elbow green, wrist flex blue, wrist roll purple. As each joint wiggles, the fingertip draws a short stroke.

Each stroke is the answer to one question: if I nudge just this joint, which way does the hand move? Put all five strokes together and you're looking at the Jacobian, the exact thing the IK uses to decide which joints to move. Notice how the strokes change length and direction depending on the pose. Near full stretch, some directions shrink almost to nothing. That's a singularity.

Set `MODE` at the top of the file to `"fk"` or `"nudge"`, and adjust `TIP_OFFSET` until the yellow dot sits between the fingertips.

## Troubleshooting

**Joints barely move or stop short** See the servo tuning section. Make sure the gain writes are inside `torque_disabled()`.

**All six motors time out at once** The servos lost power or the board lost its connection. Check the supply is on and not in current limit, the power terminals are tight, and the USB is seated. Replug the USB and rerun `lerobot-find-port` if the port name changed.

**A LeRobot command hangs with no output (Mac)** If your project is in Documents or Desktop with iCloud syncing, iCloud may have offloaded files to the cloud and reads stall waiting for them. Right-click the project folder and choose Keep Downloaded, or move your projects out of iCloud-synced folders.

**"No module named lerobot"** You're in the wrong environment or running a script from next to the repo folder. Activate your environment and run from your scripts folder.

**Keyboard teleop says the process is not trusted (Mac)** Add your terminal or VS Code under System Settings, Privacy & Security, both Accessibility and Input Monitoring.

**Gripper won't close all the way** If the actual reading reaches your closed value but it's still open, recalibrate with the gripper squeezed fully shut. If the reading stalls a few points short, raise its torque a little.

**Hand movement goes the wrong direction** Your Leap is probably turned differently from mine. Flip the matching sign in `leap_to_robot`, or set `WRIST_GAIN` / `ROLL_GAIN` to negative.

## Safety

This arm can pinch fingers and knock things over. Keep the area clear, start with small movements, and keep a hand near the power switch the first few times you run anything new.

## Parts I used

| Part | Link | Notes |
| --- | --- | --- |
| STS3215 servos (12V) | [Amazon](https://link.amazon/B039jQd3p) | The official LeRobot vendors are usually cheaper, but this listing got to me the next day instead of waiting a week or two. |
| Bus servo driver board | [Amazon](https://link.amazon/B06JCpNYe) | Comes as a two-pack. I kept the spare in case I build a second arm. |

These are just the parts I used and can personally verify work. If you don't have a 3D printer, the official LeRobot vendors also sell complete SO-101 kits, including versions with the arm parts already printed. Check the LeRobot SO-101 docs for their recommended suppliers.

**You'll also need a power supply.** The official docs recommend a 12V supply rated for at least 5A. I didn't list one because I used my own variable bench supply, set to 12V with a 5A current limit. Either works, but don't skip it: USB only carries data, not power for the servos.

Some of these are affiliate links. If you buy through them I may earn a small commission, at no extra cost to you.

## Support this project

I build all of these projects on my own. If this helped you, you can support more of them here: [Buy Me a Coffee](https://buymeacoffee.com/jordainewisdom).
