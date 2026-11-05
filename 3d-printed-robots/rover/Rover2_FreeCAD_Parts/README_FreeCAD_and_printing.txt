# Rover 2 printable parts — prototype pack

This pack contains STL files for slicing, STEP solids for importing into FreeCAD, and a FreeCAD macro that opens all the parts in one document. Dimensions are in millimetres.

## Included
- 240 × 180 mm rectangular chassis plate, no suspension springs
- 2 side rails
- 4x-style universal TT motor mount (print four copies; verify motor body/shaft geometry)
- HC-SR04 ultrasonic sensor mount
- generic camera mount and 45 mm riser
- 80 mm robot-arm base plate
- standard-size servo cradle prototype
- 100 mm and 70 mm arm links
- 2 gripper fingers
- electronics tray

## Open in FreeCAD
1. Extract this ZIP to a normal folder.
2. Open FreeCAD. Choose **Macro → Macros… → Execute** and select `Open_Rover2_parts_in_FreeCAD.FCMacro`.
3. The macro imports the STEP files into a single document. Save as `.FCStd` if desired. The imported shapes are solids; they are not fully feature-history-parametric. To change dimensions, edit the STEP in a CAD workflow or modify the included design script if supplied.
4. For individual parts, use **File → Open** and select a `.step` file.

## Printing notes
- Prototype dimensions only. Test-fit critical electronics, motors, servo horns, camera board and sensor modules before printing the full set.
- Recommended starting settings: 0.2 mm layer height, 4 walls, 25–35% infill for chassis/arm parts; PETG is a good starting material. Use more perimeters or higher infill around arm pivots and mounting holes.
- Print flat plates flat on the bed. Print the sensor/camera brackets so their largest flat face is on the bed where possible.
- Use M3 hardware for general mounting and M5 hardware for the arm-base-to-chassis pattern. Add heat-set inserts or nuts where appropriate.
- The black rock-crawler wheels are intentionally not included as printable parts: use purchased wheels that match your motors/hubs.
- The TT motor mount is a generic starting point and may not fit your exact motors. The arm parts are prototype geometry, not a tested load-rated arm.
- For safety, power servos from a suitable separate 5–6 V supply with common ground to the ESP32; do not power arm servos from the ESP32 3.3 V pin.

## Important fit checks before final build
Please confirm the motor type (TT yellow gearmotor or another), wheel hub/shaft type, chosen servos (SG90/MG90S/MG996R etc.), camera board dimensions, battery, and printer bed size. These parts are a starting CAD kit, not a guaranteed drop-in assembly.
