package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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

  // Arbitrary arm length, gearing, and angle limits.
  // Arm mass based off AndyMark Climber in a Box
  // These values were created for demonstration purposes, as I do not have access to a physical arm.
  final double gearing = 1;
  final double armLengthMeters = 0.3048; // 1 ft
  final double minAngleRads = 0;
  final double maxAngleRads = 3.14; // 180 degrees
  final double armMassKg = 1.588; // 3.5 lbs
  final double jKgMetersSquared = SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg);

  SingleJointedArmSim singleJointedArmSim = new SingleJointedArmSim(
    DCMotor.getFalcon500(1), 
    gearing,
    jKgMetersSquared,
    armLengthMeters, 
    minAngleRads, 
    maxAngleRads, 
    armMassKg, 
    false);

  @Override
  public void robotInit() {
    arm.configFactoryDefault();
    arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
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

    singleJointedArmSim.setInputVoltage(armSim.getMotorOutputLeadVoltage());

    // Update model using standard loop time (20 ms)
    singleJointedArmSim.update(0.02);

    // Set simulated sensor position and velocity
    armSim.setIntegratedSensorRawPosition(
      radiansToNativeUnits(singleJointedArmSim.getAngleRads())
    );

    armSim.setIntegratedSensorVelocity(
      velocityToNativeUnits(singleJointedArmSim.getVelocityRadPerSec())
    );
  }

  private int radiansToNativeUnits(double radians) {
    double ticks = (radians / (2 *  Math.PI)) * 2048;
    return (int)ticks;
  }

  // Converts to native units per 100 ms
  private int velocityToNativeUnits(double radPerSec) {
    double radPer100ms = radPerSec / 1000;
    return radiansToNativeUnits(radPer100ms);
  }

}
