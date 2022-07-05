# FRC Simulation

## About

This repository contains various examples of different frc simulation systems, mainly to explore possible uses for each one.

## Summary

**CTRE Simulations** are useful for verifying logic before programmers are given access to a physical robot, as well as for simulating physics. However, because of the boilerplate classes and code (ex. SimCollection) and its lack of compatibility with existing methods of controlling motors (ex. MotionMagic, built-in PID that does not use the PIDController class), I believe there is not a strong reason to integrate CTRE Simulations with a codebase besides for the reasons mentioned above. A good use case for CTRE simulations include anything that may be logic heavy, including a swerve drivebase.

## Single Motor Arm Simulation

Please see the following documentation which was referenced in the making of this simulation:
* [CTRE Javadoc](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/SingleJointedArmSim.html)
* [WPILib sample](https://github.com/mcm001/allwpilib/tree/state-space-v2/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation)

### Getting Started

1. To run, open the command palette (Ctrl+Shift+P) and select WPILib: Simulate Robot Code.
2. Select Sim GUI
3. Click Teleoperated in the Robot State window (top left corner)
4. Use the WS keys to move the arm forwards and backwards, and view simulated encoder values under NetworkTables > SmartDashboard

## Differential Drive Simulation

A majority of this code was found on the [CTRE Github](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages).

### Getting Started

1. To run, open the command palette (Ctrl+Shift+P) and select WPILib: Simulate Robot Code.
2. Select Sim GUI
3. Click Teleoperated in the Robot State window (top left corner)
4. Use WASD to control the robot on the field, or view values under NetworkTables > SmartDashboard 

