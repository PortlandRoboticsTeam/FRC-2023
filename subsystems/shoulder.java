// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

//import java.util.HashMap;

public class shoulder extends PIDSubsystem {
  public int position = 0;
  CANSparkMax shoulder = new CANSparkMax(shoulderMotorPortNum, MotorType.kBrushless);
  static PIDController shoulderController;
  CANCoder shoulderEncoder = new CANCoder(shoulderEncoderPortNum);
  double shoulderSetpoint;
  // private final SimpleMotorFeedforward m_shoulderFeedforward =
  //     new SimpleMotorFeedforward(
  //         sVolts, sVoltSecondsPerRotation);
  public boolean atHome = true;
  double optimised;
  private GenericHID m_Joystick = new GenericHID(0);
  

  //all value commands
  // Command sChangePos0 = new InstantCommand(()-> setPos(0));
  // Command sChangePos1 = new InstantCommand(()-> setPos(1));
  // Command sChangePos2 = new InstantCommand(()-> setPos(2));
  // Command sChangePos3 = new InstantCommand(()-> setPos(3));
  // Command sChangePos4 = new InstantCommand(()-> setPos(4));
  // Command sChangePos5 = new InstantCommand(()-> setPos(5));
  JoystickButton b1 = new JoystickButton(m_Joystick, 1);
  JoystickButton b2 = new JoystickButton(m_Joystick, 2);
  JoystickButton b3 = new JoystickButton(m_Joystick, 3);
  JoystickButton b4 = new JoystickButton(m_Joystick, 4);
  JoystickButton b5 = new JoystickButton(m_Joystick, 5);
  JoystickButton b6 = new JoystickButton(m_Joystick, 6);
  JoystickButton b7 = new JoystickButton(m_Joystick, 7);
  JoystickButton b8 = new JoystickButton(m_Joystick, 8);
  JoystickButton b12 = new JoystickButton(m_Joystick, 12);

  public shoulder() {
    super(shoulderController = new PIDController(shoulderP, shoulderI, shoulderD));
    getController().setTolerance(shoulderToleranceRPS);
    setSetpoint(shoulderSetpoint);
    enable();
    shoulder.setSmartCurrentLimit(20);
    
  }

  public void setPos(double shoulderPoint){
    shoulderSetpoint = shoulderPoint;
  }
  public int getPosition(){
    return position;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    shoulder.setVoltage(optimised =optimise(getMeasurement(),-output)); //+ -1*m_shoulderFeedforward.calculate(setpoint));
    // SmartDashboard.putNumber("optimised", optimised);
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    // if (b1.getAsBoolean()){
    //   position = 0;
    // }else if (b2.getAsBoolean()){
    //   position = 1;
    // }else if (b3.getAsBoolean()){
    //   position = 2;
    // }else if (b4.getAsBoolean()){
    //   position = 3;
    // }else if (b5.getAsBoolean()){
    //   position = 4;
    // }else if (b6.getAsBoolean()){
    //   position = 5;
    // }else if (b7.getAsBoolean()){
    //   position = 6;
    // }else if (b8.getAsBoolean()){
    //   position = 7;
    // }

    SmartDashboard.putNumber("sholder position", getMeasurement());
    SmartDashboard.putNumber("sPosition number", getPosition());
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

  public static double optimise(double current ,double desired){
    var delta = desired - current;
    if (Math.abs(delta)> Math.PI/2){
      return desired-Math.PI;
    }else{
      return desired;
    }
  }

}


