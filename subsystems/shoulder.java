// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

//import java.util.HashMap;

public class shoulder extends PIDSubsystem {
  public int position = 0;
  CANSparkMax shoulder = new CANSparkMax(shoulderMotorPortNum, MotorType.kBrushless);
  static PIDController shoulderController;
  CANCoder shoulderEncoder = new CANCoder(shoulderEncoderPortNum);
  double shoulderSetpoint;
  private final SimpleMotorFeedforward m_shoulderFeedforward =
      new SimpleMotorFeedforward(
          sVolts, sVoltSecondsPerRotation);
  public boolean atHome = true;
  

  public shoulder() {
    super(shoulderController = new PIDController(shoulderP, shoulderI, shoulderD));
    getController().setTolerance(shoulderToleranceRPS);
    setSetpoint(shoulderSetpoint);
    enable();
  }

  public void setPos(double shoulderPoint){
    shoulderSetpoint = shoulderPoint;
  }
  public int getPosition(){
    return position;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    shoulder.setVoltage(output + -1*m_shoulderFeedforward.calculate(setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("sholder position", getMeasurement());
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public double getMeasurement() {
    return shoulderEncoder.getAbsolutePosition();
  }
  
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}