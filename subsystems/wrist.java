// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
//import java.util.HashMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

public class wrist extends PIDSubsystem {
  public int position =0;
  CANSparkMax wrist = new CANSparkMax(wristMotorPortNum, MotorType.kBrushless);
  static PIDController wristController; 
  CANCoder wristEncoder = new CANCoder(wristEncoderPortNum);
  public static double wristSetpoint;
  private final SimpleMotorFeedforward m_wristFeedforward =
      new SimpleMotorFeedforward(
          wVolts, wVoltSecondsPerRotation);

  public boolean atHome = true;

  public wrist() {  
    super(wristController = new PIDController(wristP, wristI, wristD), wristSetpoint); 
    getController().setTolerance(wristToleranceRPS); 
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
    wrist.set(wristController.calculate(wristEncoder.getAbsolutePosition(),wristSetpoint));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public double getMeasurement() {
    return wristEncoder.getAbsolutePosition();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}