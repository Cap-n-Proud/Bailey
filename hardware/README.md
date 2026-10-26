Hardware
========

OpenSCAD parts, schematics and photos of Bailey, plus the third-party parts below.

Bailey datasheet
----------------

The current robot (2026), designed in Fusion 360 (`bailey-PAR`). The simulator's robot (`Plant` in
`bailey_FW/test/sim.cpp`) and the firmware defaults (`bailey_FW/settings.ino`) are built from this
sheet. Update all three together.

### Frame (PLA, from the STEP export)

| Part | Count | Size (mm) |
|---|---|---|
| Side plate (`FrameExtension`) | 2 | 3 × 80 × 250 |
| Motor plate (`MotorPlate`), at the bottom | 1 | 156 × 80 × 25 |
| Shelf (`Shelve`), at 50, 100, 150 and 200 mm | 4 | 160 × 80 × 3 |
| Whole frame | | 163 wide × 80 deep × 263 high |

The PLA is about 200 cm³, so about 250 g at 100 % infill. 4 walls with 30-40 % gyroid is plenty for
3 mm parts. The plates are 80 mm deep in the pitch direction, so the frame is stiff where balancing
needs it. Side to side it can rack, and a diagonal brace or back panel on one floor fixes that.

**Fix in the model before printing:**
* every shelf has two identical bodies, which Fusion counts as double mass;
* the top shelf sits 3 mm off to the side compared with the others;
* shelves and side plates overlap slightly (0.1-0.3 cm³): give the slots about 0.2 mm clearance.

### Drive

| | |
|---|---|
| Motors | 2 × NEMA17 **17HD34008-22B** (Busheng, 34 mm long), 1.8°, about 0.3 N·m holding at 1.5 A, 2.3 Ω, 3 mH, about 220 g each |
| Drivers | 2 × BTT **TMC2209 V1.3** or **TMC2226**, in single-driver carrier boards, 1/8 microstepping (`stepsPerRev` 1600), about 1.0-1.2 A RMS |
| Wheels | **65 mm** rubber-tyred wheels (about 40 g each) on 5 mm shaft hubs |
| Supply | 3S 18650 pack (9.9-12.6 V) with BMS, straight to the drivers |

At 2.5 rev/s (`stepperMaxSpeed` 4000) the top speed is about 0.5 m/s. `stepperMaxAccel` 40000 gives
5.1 m/s², and the motors could give about 10 m/s².

Why 65 mm and not the 123 mm wheels: twice the wheel radius halves the force at the floor. With these
short motors that leaves about 4 m/s² to catch a fall. The big wheels also add about 0.3 kg and put
κ (2.46) right at the autotuner's limit (2.5).

### Electronics

| | |
|---|---|
| Controller | Waveshare ESP32-S3-Pico (`bailey_FW/config.h` for the pins) |
| IMU | MPU-6050 (SparkFun or GY-521), on the motor plate close to the axle |
| Computer | Raspberry Pi 4B in the case below, with the camera, on the top shelf |
| 5 V | Buck converter, 3 A or more, for the Pi and the ESP32 |

### Mass budget and model (estimates)

| Item | Mass | Height above the axle |
|---|---|---|
| PLA frame | 250 g | 73 mm (centre) |
| 2 motors | 440 g | 0 (at the axle) |
| 3S battery, shelf 1 | 190 g | 65 mm |
| ESP32, drivers, buck, wiring, shelf 2 | 80 g | 115 mm |
| Screws, standoffs, cables | 50 g | 130 mm |
| Pi 4 + case + camera, top shelf | 120 g | 220 mm |
| **Body** | **1.13 kg** | **centre of mass 64 mm** |
| 2 wheels | 80 g | |

Pitch inertia of the body about the axle: 4.6 g·m² (`bodyLength` 0.158 m). That gives a pendulum
length **L ≈ 0.24 m** and coupling **κ ≈ 1.56**, the firmware defaults. The heavy motors at the axle
keep the centre of mass low, so the robot behaves like a 24 cm pendulum although it is 26 cm tall.

The axle is taken at the bottom of the motor plate. For exact numbers, put the motors, wheels,
battery and Pi in the Fusion model with their real masses and read **Inspect → Properties**, then
update this table, `Plant` and the defaults. The autotuner measures L and κ on the real robot
anyway.

Starting settings on the real robot:

```
SCMD wheelDiameter 65
SCMD pendulumLength 240
SCMD pendulumCoupling 156
SCMD lqrAccelLimit 4.5
```

These are the firmware defaults, so they only matter if older settings were saved. Then run
`SCMD AUTOTUNE 1` and `SCMD SaveCfg 1`.

Third-party printed parts
-------------------------

These are other people's designs. Their licences don't allow sharing the files, so they are not in
this repository: download them from the links.

### Raspberry Pi 4B case

* **Model:** "Raspberry Pi 4 Case | Wall Mount | Options" by **i-BoxIt** on MakerWorld
  ([search link](https://makerworld.com/en/search/models?keyword=Raspberry%20Pi%204%20Case%20Wall%20Mount%20Options%20i-BoxIt)).
  Licence: MakerWorld Standard Digital File License.
* **Variant:** *Raspberry PI 4B Case Screw-able*, which takes **M2.5 × 10 mm** screws.
* **Printing:** PLA (or PETG), 0.2 mm layers, no supports.
* **On Bailey:** it goes on the top floor, together with the camera.
  * **Mounting:** use the case's screw holes or its wall-mount option to fix it to the shelf. Drill
    the shelf to match the case, and fit M2.5 heat-set inserts if the shelf is printed.
  * **Cables:** leave clearance for the USB cable to the ESP32 and for the 5 V supply.
  * **Cooling:** keep the heat-sink opening uncovered.
  * **Mass:** about 50 g for the Pi plus the printed case. Include it in the Fusion 360 mass
    properties used for the simulator.
