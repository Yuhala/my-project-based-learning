# Mini tank: 157 × 108 mm, 2 × N20, ESP32-S3, pan camera

**Size:** 157 mm long, 108 mm wide, 43 mm to the deck and 97 mm with the camera head.

**Why 108 mm wide, not 100:** two N20 motors placed end to end inside the hull need about 64 mm inside. Adding two 18 mm tracks gives 108 mm. Going narrower would mean offsetting the motors and adding a gear stage.

**Drive:** each N20 drives its sprocket directly. The N20 already has its own gearbox, so the sprocket is the only "gear" needed.

## Files
| File | Use |
|---|---|
| `STL/` | Slicing. Parts are already turned onto their print face. `_xN` = print N. |
| `STEP/` | FreeCAD (File → Open). These are real solids you can edit. |
| `tank_full_assembly.step` | Full assembly with colours. Motors, servo, battery and camera are reference shapes. |
| `tank_full_assembly*.stl` | The assembled tank, printed parts only or with electronics. |
| `tank_cad.py` | Parametric source (`pip install cadquery scipy`, then `python tank_cad.py`). |

## Printed parts
| Part | Qty | Material / settings |
|---|---|---|
| hull | 1 | PLA or PETG, 3 walls, 20% infill. No supports. |
| deck | 1 | PLA/PETG, 3 walls |
| motor_cap | 2 | PLA/PETG. Flip one for the left side; it's symmetric. |
| sprocket | 2 | **PETG**, 5 walls, 50%+ infill (it takes the drive torque) |
| road_wheel | 6 | PLA/PETG |
| idler | 2 | PLA/PETG |
| axle_sleeve | 8 | PETG, 100% infill (wheel bearing surface) |
| axle_spacer | 8 | PLA/PETG, 100% infill |
| track_link | 68 (34 per side, + spares) | **PETG** preferred; 0.12–0.16 mm layers for clean pin holes |
| camera_head | 1 | PLA |

Total is about 220 g printed solid, about 130 g at normal infill.

**Track pins:** 68 pieces of **1.75 mm filament**, each about 18 mm long. Push one through the knuckles, then flatten both ends gently with a soldering iron or a lighter-heated nail. If a pin is tight, run a 2 mm drill through the holes by hand first.

## Screws (only from your kits)
| Where | Screw | Qty | Also needed |
|---|---|---|---|
| Motor caps | **M2×6 socket** | 8 | 8 × **M2 heat-set inserts** in the motor-box walls |
| Sprocket on N20 shaft | **M2×4 socket** (set screw onto the shaft flat) | 2 | 2 × **M2 insert** pressed into the sprocket hub |
| Road wheels + idlers | **M3×25 countersunk** | 8 | 8 × M3 nut + 8 × M3 washer inside the hull |
| Deck | **M3×8 socket** | 4 | 4 × **M3 insert** in the hull posts |
| Pan servo | **M2×6 socket** | 2 | 2 × **M2 insert** in the deck bosses. Drill the servo's tab holes to 2.2 mm if needed. |
| Camera head → servo horn | the small screw supplied with the servo | 1–3 | Or drill the horn holes to 2.2 mm and use M2×4 |

The M3×40, M4 and M5 screws aren't needed.

**Screw length checks:**
- **M3×25:** goes through the flanged sleeve (countersunk flush), wheel, spacer, wall, washer and nut, leaving 3 mm of thread past the nut.
- **M2×6 caps:** 3.5 mm goes into the insert.
- **M2×4 set screw:** reaches the shaft flat.

## Electronics (suggested)
- **Motors:** 2 × N20 gear motors. The **6 V, 150–300 rpm** versions are best; 300 rpm gives about 0.6 m/s on these sprockets.
- **Motor driver:** a **DRV8833** or **TB6612FNG** module, mounted with a zip tie through the deck slots or with foam tape.
- **Controller:** ESP32-S3 dev board.
- **Battery:** **2 × 18650 (2S, 7.4 V)** in a holder strapped down through the floor slots. Use a **5 V buck converter, 2 A or more**, for the ESP32-S3 and the servo. If your N20s are 6 V types, cap the PWM duty at about 80%.
- **Pan servo:** SG90 or MG90S in the deck pocket.
- **Camera:** the head holds a board up to **27 × 40.5 mm** (ESP32-CAM / ESP32-S3-CAM size), tilted 10° up. If your camera is different, change `CAM_W/CAM_H/CAM_T` in the script and re-run.
- **Power switch:** a Ø12 mm round switch in the rear wall. The front wall has a Ø6 hole for an LED or sensor.

## Assembly
1. **Inserts:** press all heat-set inserts with the soldering iron:
   - 8 × M2 in the motor-box walls;
   - 4 × M3 in the deck posts;
   - 2 × M2 in the deck servo bosses;
   - 1 × M2 radially into each sprocket hub.
2. **Motors:**
   - Seat each N20 in its box, gearbox face against the wall, shaft through the wall hole.
   - Fit the cap with 4 × M2×6. The box walls sit 0.15 mm below the motor, so the cap clamps it.
3. **Sprockets:** push each sprocket onto its shaft and tighten the M2×4 set screw onto the **flat** of the D-shaft.
4. **Wheels:** for each road wheel and idler, stack:
   - countersunk M3×25 → flanged sleeve → wheel → spacer → hull wall → washer → nut.
   - The wheel must spin freely; the screw clamps the sleeve, not the wheel.
   - Leave the idler nuts loose for now.
5. **Tracks:** pin 34 links into a loop per side, guide horns facing inward. Wrap the track over the wheels, then **slide the idler forward in its slot** to tension (±4 mm of travel) and tighten its nut. Aim for slight sag on the top run, not drum-tight.
6. **Electronics:** fit the battery (strap through the floor slots), driver, ESP32-S3, buck converter and switch.
7. **Deck and camera:** mount the servo in the deck (2 × M2×6). **Centre the servo before fitting the horn.** Attach the camera head to the horn, then fit the deck with 4 × M3×8.

## Checks already done in CAD
- All parts are watertight single solids.
- No interference anywhere in the assembly. All 68 links articulate with zero overlap.
- The road wheels clear the sprocket.
- Sprocket teeth engage the knuckles, with a tooth window in each link. The only contact is a 0.1 mm graze on the link entering or leaving the sprocket.
- The 34-link loop closes at exactly 10 mm pitch, with the idler at x = 59.8 mm, centred in its ±4 mm tensioning slot.

## Tips
- **Print one test set first:** 2 links, 1 sprocket, 1 motor cap. Check the pin fit, the motor fit, and the D-shaft fit (`N20_D` / `N20_FLAT`, `PIN_D`).
- **Clone N20s vary slightly.** If yours is longer than 24 mm (e.g. with an encoder), raise `N20_LEN` and re-run.

## Disclaimer
- Most of these parts for the 3D model (STL files and images) were created with the help of AI. 