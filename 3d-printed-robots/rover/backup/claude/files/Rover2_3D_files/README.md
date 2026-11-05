# Rover 2 (Upgraded) – 3D printable 4WD rover with 4-DOF arm

Overall footprint is **236 × 178 mm**, matching the ~240 × 180 in the infographic. The chassis box is 210 × 120 mm.

Every printable part fits a **220 × 220 mm** bed, which covers Ender-3, Prusa and Bambu printers. All parts were checked to be watertight single solids. The assembly was checked for collisions in several arm poses.

## What's in this package

| Folder / file | Use it for |
|---|---|
| `STL/` | Slicing. Each part is already rotated into its print orientation. `_x4` and `_x2` in a name mean print that many. |
| `STEP/` | Editing in **FreeCAD**. These are real solids, not meshes. |
| `rover2_full_assembly.step` | The full assembly in FreeCAD, with colours and one object per part. Electronics are included as reference blocks. |
| `rover2_full_assembly.stl` | The assembled robot, printed parts only. |
| `rover2_full_assembly_with_electronics.stl` | The same, plus motors, servos and boards. |
| `rover2_cad.py` | The parametric source. Change the dimensions at the top and re-run (`pip install cadquery`, then `python rover2_cad.py`). |
| `preview/` | Renders of the assembly and a sheet showing every part. |

### Opening in FreeCAD

- **Single part:** use *File → Open* or *File → Import* on the `.step` file. You get a solid you can measure, or use as a base in Part Design with sketches, pockets and so on.
- **Converting an STL:** in the Part workbench use *Part → Create shape from mesh*, then *Convert to solid*. STEP is cleaner, so use that if you can.
- **Parametric edits:** the easiest route is to edit `rover2_cad.py` and re-export. The FreeCAD *CadQuery workbench* addon can also run it directly.

## Parts to print

| Part | Qty | Material | Notes |
|---|---|---|---|
| chassis_base | 1 | PLA/PETG | Has integrated L298N standoffs and an M3 accessory grid |
| top_plate | 1 | PLA/PETG | |
| side_panel | 2 | PLA/PETG | Prints wall-down |
| front_sensor_mount | 1 | PLA/PETG | Holds the HC-SR04 |
| rear_panel | 1 | PLA/PETG | KCD1 rocker switch, USB-C port and a spare Ø8 hole |
| wheel_mount | 4 | **PETG** preferred | TT motor bracket. All 4 are identical. |
| wheel_rim | 4 | PLA/PETG | Prints outer face down |
| wheel_tire_TPU | 4 | **TPU 95A** | PLA also works if you want hard wheels |
| arm_base_mount | 1 | PLA/PETG | Base servo deck and turntable support ring |
| turntable_retainer | 1 | PLA/PETG | Prints lip-down |
| turntable | 1 | PLA/PETG | |
| servo_bracket_MG996R | 2 | PETG | One for the shoulder, one for the elbow |
| arm_upper_link | 1 | PETG | Prints web-down |
| arm_forearm_link | 1 | PETG | Prints web-down |
| gripper_base | 1 | PLA/PETG | Prints flange-down. Enable *support from build plate only* for the gear pillar. |
| gripper_finger_drive | 1 | PLA | Mounts on the SG90 horn |
| gripper_finger_idle | 1 | PLA | |
| camera_mount | 1 | PLA | ESP32-CAM slides in from the top |
| electronics_holder | 1 | PLA | ESP32 DevKit tray. Has a snap lip and room for dupont wires underneath. |

**Print settings:** 0.2 mm layers, 4 walls, and 25–30 % gyroid infill. Use 40 % infill for the arm parts and wheel mounts. Only the gripper base needs supports. Total filament is about 650 g PLA/PETG plus about 200 g TPU.

## Hardware to buy

The electronics are the ones in the infographic:

- ESP32 DevKit
- ESP32-CAM
- HC-SR04 ultrasonic sensor
- L298N motor driver
- 4 × TT gear motors (yellow, straight shaft)
- 3 × **MG996R** servos (base, shoulder, elbow)
- 1 × **SG90 or MG90S** servo (gripper)

Fasteners:

- **M3 heat-set inserts**, 4 mm hole and about 5 mm long, ×11 (4 rail holes and 1 pivot per servo bracket, plus 1 in the gripper pillar). Each bracket has 8 rail holes, but only the 4 that match its mating part need inserts.
- **M3 bolts**: roughly 30 × M3×10 and 10 × M3×16, plus M3 nuts. You also need 4 × **M3×40** for the retainer.
- **8 × M3×30 countersunk** screws and nuts for the motors (2 per motor).
- 4 × M2×10 screws, one for each wheel hub.
- M2 self-tapping screws for the HC-SR04 and the horn screws. These usually come with the servos.

### Power: please read this

A single 18650 cell, as drawn in the infographic, won't drive this robot. The L298N drops about 2 V, and MG996R servos can each pull 2 A or more.

Use this setup instead:

- **2 × 18650 cells (2S, 7.4 V)** in a holder. The base plate has strap slots for a standard 2-cell holder.
- **A 2S BMS or charger board.** A TP4056 is for single cells only.
- **A separate 5–6 V buck converter rated at 5 A or more** (XL4015 or similar) for the servos.

Tie all grounds together.

## Assembly order

1. **Wheel mounts and motors.**
   - Put each TT motor between the bracket walls.
   - Fit two M3×30 countersunk screws from the wheel side, with nuts on the inside.
   - Bolt each bracket under the chassis base with 2 × M3×10.
   - Route the motor wires up through the Ø10 holes.
2. **Inside the base.** Mount the L298N on the built-in standoffs, strap the battery holder down, then fit the buck converter and BMS on the M3 grid.
3. **Panels.**
   - Bolt the side, front and rear panels to the base through their bottom flanges.
   - Press the HC-SR04 through the front mount.
   - Fit the rocker switch in the rear panel.
4. **Top plate prep.** Fit the ESP32 holder and camera mount *before* closing the top. Their screws also pass through the front panel flange.
5. **Base servo.**
   - Drop the MG996R through the deck cutout so it hangs from its tabs.
   - Screw it down, then bolt the deck to the top plate.
6. **Turntable.**
   - Screw the round horn into the turntable's underside recess.
   - Bolt the shoulder bracket on from below, into heat-set inserts.
   - Press the turntable onto the servo spline. Grease the support ring.
   - Fit the retainer with 4 × M3×40 through the deck and top plate. The retainer holds the turntable down and takes the tipping load off the servo.
7. **Shoulder and elbow.** Each joint works the same way:
   - The servo sits in a `servo_bracket` with its tabs on the wall.
   - The U-link's horn leg screws onto the horn.
   - The idler leg pivots on an M3 bolt into the insert in the bracket's back boss. Snug it, but don't clamp it; a drop of threadlocker helps.
   - The elbow bracket bolts onto the upper link's web.
8. **Gripper.**
   - Screw the SG90 onto the plate.
   - Attach the single-arm horn to the drive finger's underside pocket with 2 small screws, then press it onto the spline.
   - The idle finger pivots on an M3 bolt into the insert in the pillar.
   - Centre both servos at 90° before mounting the horns.

The arm can reach the floor about 45 mm ahead of the front wheels.

## Measure these first and change them in `rover2_cad.py` if needed

These parts vary between suppliers:

- **TT motor:** the mounting hole spacing (`TT_HOLE_DX`, `TT_HOLE_DZ`) and gearbox size.
- **Servo horn screw circle** (`HORN_SCREW_PCD`, 14 mm). The 25 mm round recess also centres the horn, so the screws are only there to hold it on.
- **MG996R** tab height and shaft offset, which differ slightly between clones.
- **Lens position on your ESP32-CAM.** The window is a generous 20 × 28 mm.

Print a single `wheel_mount` and one `servo_bracket` first to check the fit before you print everything.

The 5-DOF and 6-DOF arm options from the infographic aren't modelled. Adding them means one more servo bracket and U-link pair, and the existing parts share a common interface designed for that.
