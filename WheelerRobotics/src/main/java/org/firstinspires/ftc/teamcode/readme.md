# So here's how this all works

## File and Code Organization
In this repository all the code you're gonna be concerned with is in the org.firstinspires.ftc.teamcode.comp package. 
The code is organized into the following directories:
- auto: autonomous opmodes.
- chassis: classes which control any chassis, for example, the mecanum drivetrain has the Meccanum class which any robot can use.
- controller: the classes that dictate controller mappings.
- dev: things being tested or in development, maybe a class to test out a new feature.
- helpers: classes that help with things like math, the PID class is in here, and more math is coming.
- notes: notes on how to do things or how things work (feel free to add to this).
- robot: classes which are specific to each robot, this could contain a function that moves Brokey's arm or smth.
- teleop: teleop opmodes.
- tests: tests for test driven development (haven't quite figured out how to do this yet).
- utility: opmodes used for debugging, these are classes like the MotorTest class which runs one motor at a time so we could diagnose a motor problem.
- vision: classes that relate to vision
  - pipelines: OpenCVPipeline instance classes which are used to process images in certain ways.

As stated above, the `Robot` classes are for robot specific things while the `Chassis` classes are for things that are generic to all robots of some drivetrain.
If there is something all robots or chassis share you can add that to the `Robot` interface or the `Chassis` interface respectively.

## Hardware and Config
On the driver hub (black cube running android), there are config settings for setting up devices. 
Some devices should have specific names including:
   - `motorFrontLeft`: the front left motor.
   - `motorFrontRight`: the front right motor.
   - `motorBackLeft`: the back left motor.
   - `motorBackRight`: the back right motor.
   - `distanceLeft`: the left distance sensor.
   - `distanceRight`: the right distance sensor.
   - `distanceFront`: the front distance sensor.
   - `distanceBack`: the back distance sensor.
   - `imu`: the internal imu.

All other stuff can be called whatever as long as it matches up in code.

## Stylistic Conventions
Here's how your variables should look:
  - general variables should be in `camelCase`.
  - methods should be in `camelCase`.
  - classes and opmodes should be in `PascalCase`.
  - constants should be in `UPPER_CASE_SNAKE_CASE`.
  - config names should be in `snake_case`.
  - branch names should be in `kebab-case`.
