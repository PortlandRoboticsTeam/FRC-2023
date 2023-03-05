// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

//import java.util.HashMap;

public class elbow extends PIDSubsystem {
  public int position =0;
  CANSparkMax elbow = new CANSparkMax(elbowMotorPortNum, MotorType.kBrushless);
  static PIDController elbowController;
  static CANCoder elbowEncoder = new CANCoder(elbowEncoderPortNum);
  double elbowSetpoint;
  boolean disabled = false;
  private final SimpleMotorFeedforward m_elbowFeedforward =
      new SimpleMotorFeedforward(
          eVolts, eVoltSecondsPerRotation);

  public boolean atHome = true;
  
  
  public elbow() {
    super(elbowController = new PIDController(elbowP, elbowI, elbowD));
    getController().setTolerance(elbowToleranceRPS);
    setSetpoint(elbowTargetRPS);
  }

  public void setPos(double elbowPoint){
    elbowSetpoint = elbowPoint;
  }
  public int getPosition(){
    return position;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    elbow.setVoltage(output + m_elbowFeedforward.calculate(setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elbow.set(elbowController.calculate(elbowEncoder.getAbsolutePosition(),elbowSetpoint));
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public double getMeasurement() {
    return elbowEncoder.getAbsolutePosition();
  }
  
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}
