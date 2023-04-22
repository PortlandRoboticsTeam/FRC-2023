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

public class wrist extends PIDSubsystem {
  public int position = 0;
  CANSparkMax wrist = new CANSparkMax(wristMotorPortNum, MotorType.kBrushless);
  static PIDController wristController;
  CANCoder wristEncoder = new CANCoder(wristEncoderPortNum);
  double wristSetpoint;
  // private final SimpleMotorFeedforward m_wristFeedforward =
  //     new SimpleMotorFeedforward(
  //         sVolts, sVoltSecondsPerRotation);
  public boolean atHome = true;
  double optimised;

  public wrist() {
    super(wristController = new PIDController(wristP, wristI, wristD));
    getController().setTolerance(wristToleranceRPS);
    setSetpoint(wristSetpoint);
    disable();
    wrist.setSmartCurrentLimit(20);
  }

  public void setPos(double wristPoint){
    wristSetpoint = wristPoint;
  }
  public int getPosition(){
    return position;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    wrist.setVoltage(optimised =optimise(getMeasurement(),-output));//+ -1*m_wristFeedforward.calculate(setpoint));
    SmartDashboard.putNumber("optimised", optimised);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
SmartDashboard.putNumber("wrist position", getMeasurement());
SmartDashboard.putNumber("wPosition number", getPosition());
    super.periodic();
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

  public static double optimise(double current ,double desired){
    var delta = desired - current;
    if (Math.abs(delta)> Math.PI/2){
      return desired-Math.PI;
    }else{
      return desired;
    }
  }
  public void wSetVolatge(){
    wrist.setVoltage(2.);
  }

}


