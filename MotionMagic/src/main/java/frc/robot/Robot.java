package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.PS4ControllerSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  PS4Controller ps4Controller = new PS4Controller(0);
  PS4ControllerSim ps4ControllerSim = new PS4ControllerSim(ps4Controller);

  WPI_TalonFX arm = new WPI_TalonFX(0);

  // Create a SimCollection class for simulated inputs into the Talon
  TalonFXSimCollection armSim = arm.getSimCollection();

  final double gearing = 1;
  final double jKgMetersSquared = 0.14753;
  final double armLengthMeters = 0.3048; // 1 ft
  final double minAngleRads = 0;
  final double maxAngleRads = 3.14; // 180 degrees
  final double armMassKg = 1.588; // 3.5 lbs

  SingleJointedArmSim singleJointedArmSim = new SingleJointedArmSim(
    DCMotor.getFalcon500(1), 
    gearing,
    jKgMetersSquared,
    armLengthMeters, 
    minAngleRads, 
    maxAngleRads, 
    armMassKg, 
    true);

  @Override
  public void robotInit() {
    arm.configFactoryDefault();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Position", arm.getSelectedSensorPosition());
  }

  @Override
  public void teleopPeriodic() {

    // Control the arm with percent output and ps4 controller
    double movement = ps4Controller.getLeftY();
    arm.set(ControlMode.PercentOutput, movement);
    SmartDashboard.putNumber("PS4 Y-axis", movement);
  }

  @Override
  public void simulationPeriodic() {

    // Pass battery voltage to simulated arm
    armSim.setBusVoltage(RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Arm Output Voltage", armSim.getMotorOutputLeadVoltage());

    // TODO: Update sensors in SingleJointedArmSim here
  }

}
