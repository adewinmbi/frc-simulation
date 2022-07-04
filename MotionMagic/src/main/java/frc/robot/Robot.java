package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.PS4ControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  PS4Controller ps4Controller = new PS4Controller(0);
  PS4ControllerSim ps4ControllerSim = new PS4ControllerSim(ps4Controller);

  WPI_TalonFX arm = new WPI_TalonFX(0);

  // Create a SimCollection class for simulated inputs into the Talon
  TalonFXSimCollection armSim = arm.getSimCollection();

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
