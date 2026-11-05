# Mini FPV Racing Drone (3-inch), v2: ESP32-S3 flight controller

The motors are 150 mm apart diagonally, and the frame takes 3-inch props.

The printed parts weigh about 67 g printed solid, so the finished drone should be **about 195–205 g** with a 3S 450 mAh battery. That's under the 250 g limit.

The drone uses only the screws from your kits. There are no metal standoffs, no M2×8, M2×25 or M3×14 screws, and no nylock nuts.

## Files
| File | Use |
|---|---|
| `STL/` | Slicing. Parts are already turned onto their print face. `_x4` means print 4. |
| `STEP/` | FreeCAD (File → Open). These are real solids you can edit. |
| `fpv3_full_assembly.step` | The full assembly with colours. Bought parts are shown as reference shapes. |
| `fpv3_full_assembly*.stl` | The assembled drone, printed parts only or with electronics. |
| `fpv3_cad.py` | The parametric source (`pip install cadquery`, then `python fpv3_cad.py`). |

## Printed parts
| Part | Qty | Material | Settings |
|---|---|---|---|
| arm | 4 (+ spares) | **PA-CF / nylon best**, PETG OK. **Not PLA.** | 100% infill, or 6 walls |
| bottom_plate | 1 | PA-CF / PETG | 5 walls, 40% infill |
| top_plate | 1 | PA-CF / PETG | 4 walls, 30% infill |
| spacer_M3 | 4 | PETG / PA-CF | **100% infill**. These replace the metal standoffs. |
| stack_spacer_M2 | 4 | PETG | 100% infill |
| fc_carrier_ESP32S3 | 1 | PETG | 4 walls |
| camera_mount | 1 | PETG or TPU | 4 walls |
| motor_guard_TPU | 4 | TPU 95A | 3 walls |
| antenna_mount_TPU | 1 | TPU 95A | 3 walls |

No supports are needed.

## Screws: only from your kits
| Where | Screw | Qty | Also needed | Kit |
|---|---|---|---|---|
| Main stack: bottom plate, arm, spacer, top plate | **M3×40 socket head** | 4 | M3 lock washer under the head, M3 hex nut pressed into the hex trap under the bottom plate | uxcell + Besitu |
| Arm lock (outer arm hole) | **M3×12 countersunk**, from below | 4 | M3 hex nut on top of the arm, plus a drop of blue threadlocker | Phillips kit + Besitu |
| Camera mount feet | **M3×10 countersunk**, from below | 2 | M3 hex nut + threadlocker | Phillips kit + Besitu |
| Motors | **M2×10 socket head** | 16 | — | Besitu |
| FC/ESC stack | **M2×12 socket head** | 4 | **M2 heat-set inserts** in the bottom plate bosses | Besitu + Preciva |
| Camera pivot | **M2×4 socket head** | 2 | — | Besitu |

**Screw length checks:**
- **M3×40:** the frame is 39 mm thick, so with the lock washer the bolt tip ends flush in the nut. Nothing sticks out of the belly.
- **Countersunk screws:** these sit flush underneath. There's 3 mm of thread for the nut. A lock washer also fits, which leaves 2.2 mm (4 threads) in the nut, but threadlocker is the better choice.
- **M2×10 motor screws:** 2.5 mm goes into the motor. **Before fitting, check that a screw in your motor's hole doesn't touch the windings.** If it's too long, add an M2 washer from the Besitu kit.
- **M2×12 stack screws:** 2.4 mm goes into the insert. The hole for the insert is 3.2 mm; change `M2_INSERT` in the script if your inserts need a different size.

## Electronics
- **Flight controller:** an **ESP32-S3 SuperMini** (a Seeed XIAO ESP32-S3 also fits the cradle) running **ESP-FC** (github.com/rtlopez/esp-fc). It's open-source, lists the ESP32-S3 as a recommended target, and you set it up in **Betaflight Configurator 10.10**.
- **Gyro:** a **GY-521 (MPU6050, I2C)** sits in the centre of the carrier on 1–2 mm foam tape. For a higher loop rate, use an SPI gyro board such as a GY-91 (MPU9250); it fits the same spot.
- **ESC:** a 20×20 **4-in-1 ESC**, 20–35 A, BLHeli_S or BLHeli_32, driven with DShot300 or DShot600.
- **5 V supply:** ESP-FC needs a **5 V BEC**. Pick an ESC with a 5 V output, or add a mini 5 V BEC.
- **Receiver:** an **ELRS receiver** (CRSF, connected to one UART). ESP-FC also supports SBUS, PPM, and ESP-NOW from a second ESP32.
- **Battery monitor:** a voltage divider to an ADC pin (e.g. 10 kΩ / 1 kΩ) lets the flight controller read the 3S battery voltage.
- **Video:** a 19 mm micro camera and a 5.8 GHz VTX on foam tape. The ESP32-S3 has no analog OSD chip, so you won't get Betaflight's on-screen display unless you add a separate OSD board.
- **Motors, props, battery:** 1404–1408 motors (3000–4500 KV), 3-inch props, and a 3S 450–650 mAh battery with an XT30 connector.

Pin assignments for motors, I2C, the receiver UART and the voltage sensor are set in the ESP-FC CLI to match your wiring. Follow its wiring guide and setup docs.

## Assembly
1. **Inserts:** press 4 **M2 heat-set inserts** into the bosses on the bottom plate with a soldering iron.
2. **Nuts:** press **M3 nuts** into the 4 hex traps under the bottom plate.
3. **Arms:**
   - Lay each arm on the bottom plate.
   - Fit the arm-lock **M3×12 countersunk** screw from below, with a nut on top and threadlocker.
4. **Motors:** put the TPU guard under each arm tip and fit 4 × **M2×10** per motor.
5. **ESC:** set the ESC on the bosses. Add the printed M2 spacers, then the FC carrier, then 4 × **M2×12** down into the inserts.
6. **Boards:**
   - Stick the gyro in the centre of the carrier with foam tape, its arrow pointing forward.
   - Strap or zip-tie the ESP32-S3 into its cradle, USB-C facing the rear.
7. **Camera:** fit the camera mount with 2 × **M3×10 countersunk** screws from below, plus nuts. Fit the camera with 2 × **M2×4** and tilt it to 20–30°.
8. **Top plate:**
   - Stand the 4 printed spacers on the arm roots and clip the antenna mount over the rear two.
   - Fit the top plate, then the **M3×40** bolts with lock washers from the top, down into the nut traps.
9. **Flash and set up:**
   - Flash ESP-FC with the web ESP Tool, set the pins in the CLI, then connect Betaflight Configurator.
   - Check the motor order and direction **with the props off**, and calibrate the accelerometer.
10. **Props:** fit the props, strap on the battery, and test-hover low in an open area.

## Checks already done in CAD
- All parts are watertight single solids.
- There's no interference between parts, including the ESP32-S3, gyro, M2 screw heads and bolt heads.
- Prop discs (+3 mm radius and ±4 mm flex margin) are clear.
- The camera clears both plates from 0–40° tilt.
- The M3×40 bolt length works out exactly for the 39 mm frame.

## Safety
- Props off on the bench.
- Use a LiPo-safe bag when charging.
- **Canada:** under 250 g needs no registration, but you still can't fly near people, airports or aircraft. Check Transport Canada's rules.
