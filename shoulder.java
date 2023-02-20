// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

import java.util.HashMap;

public class shoulder extends PIDSubsystem {
  public int position =0;
  CANSparkMax shoulder;
  PIDController shoulderController;
  Encoder shoulderEncoder;
  double shoulderSetpoint;
  private final SimpleMotorFeedforward m_shoulderFeedforward =
      new SimpleMotorFeedforward(
          sVolts, sVoltSecondsPerRotation);

  public static boolean atHome = true;
  public HashMap <Integer, Double> spositionsMap = new HashMap<Integer, Double>();

  public shoulder() {
    super(new PIDController(shoulderP, shoulderI, shoulderD));
    getController().setTolerance(shoulderToleranceRPS);
    shoulderEncoder.setDistancePerPulse(shoulderEncoderDistancePerPulse);
    setSetpoint(shoulderTargetRPS);
  }

  public void setPos(double shoulderPoint){
    shoulderSetpoint = shoulderPoint;
  }
  public int getPosition(){
    return position;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    shoulder.setVoltage(output + m_shoulderFeedforward.calculate(setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulder.set(shoulderController.calculate(shoulderEncoder.getDistance(),shoulderSetpoint));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public double getMeasurement() {
    return shoulderEncoder.getRate();
  }
  
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}
