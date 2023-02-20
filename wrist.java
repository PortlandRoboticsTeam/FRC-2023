// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

public class wrist extends PIDSubsystem {
  public int position =0;
  CANSparkMax wrist;
  PIDController wristController; 
  Encoder wristEncoder;
  public HashMap <Integer, Double> positionsMap = new HashMap<Integer, Double>();
  

  public static double wristSetpoint;
  private final SimpleMotorFeedforward m_wristFeedforward =
      new SimpleMotorFeedforward(
          wVolts, wVoltSecondsPerRotation);

  public boolean atHome = true;

  public wrist() {
    super(new PIDController(wristP, wristI, wristD));
    getController().setTolerance(wristToleranceRPS);
    wristEncoder.setDistancePerPulse(wristEncoderDistancePerPulse);
    setSetpoint(wristTargetRPS);
  }

  public void setPos(double wristPoint){
    wristSetpoint = wristPoint;
  }

  public void setAtHome(boolean AtHome){
    atHome = AtHome;
  }
  //return the encoder value
  public int getPosition(){
    return position;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    wrist.setVoltage(output + m_wristFeedforward.calculate(setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wrist.set(wristController.calculate(wristEncoder.getDistance(),wristSetpoint));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public double getMeasurement() {
    return wristEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}
