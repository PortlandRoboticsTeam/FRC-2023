// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

//import java.util.HashMap;

public class elbow extends PIDSubsystem {
  public int position = 0;
  CANSparkMax elbow = new CANSparkMax(elbowMotorPortNum, MotorType.kBrushless);
  static PIDController elbowController;
  CANCoder elbowEncoder = new CANCoder(elbowEncoderPortNum);
  double elbowSetpoint;
  // private final SimpleMotorFeedforward m_elbowFeedforward =
  //     new SimpleMotorFeedforward(
  //         sVolts, sVoltSecondsPerRotation);
  public boolean atHome = true;
  double optimised;

  public elbow() {
    super(elbowController = new PIDController(elbowP, elbowI, elbowD));
    getController().setTolerance(elbowToleranceRPS);
    setSetpoint(elbowSetpoint);
    disable();
    elbow.setSmartCurrentLimit(20);
  }

  public void setPos(double elbowPoint){
    elbowSetpoint = elbowPoint;
  }
  public int getPosition(){
    return position;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    elbow.setVoltage(optimised =optimise(getMeasurement(),-output));//+ -1*m_elbowFeedforward.calculate(setpoint));
    SmartDashboard.putNumber("optimised", optimised);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("elbow position", getMeasurement());
    SmartDashboard.putNumber("ePosition number", getPosition());
    super.periodic();
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

  public static double optimise(double current ,double desired){
    var delta = desired - current;
    if (Math.abs(delta)> Math.PI/2){
      return desired - Math.PI;
    }else{
      return desired;
    }
  }

  public void wSetVolatge(){
    elbow.setVoltage(2.);
  }

}
