// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

   CANSparkMax wrist;
   CANSparkMax shoulder;
   CANSparkMax elbow;
   pidStuff wristPID;
   pidStuff shoulderPID;
   pidStuff elbowPID;
   
  static boolean atHome = true;
  public arm() {
    atHome = true;
    this.wrist = new CANSparkMax(wristMotorPortNum, MotorType.kBrushless);
    this.shoulder = new CANSparkMax(shoulderMotorPortNum, MotorType.kBrushless);
    this.elbow = new CANSparkMax(elbowMotorPortNum, MotorType.kBrushless);
    this.wristPID = new pidStuff(wristMotorPortNum, wristP, wristI, wristD, wristIz, wristFF, wristMaxOutput, wristMinOutput);
    this.elbowPID = new pidStuff(elbowMotorPortNum, elbowP, elbowI, elbowD, elbowIz, elbowFF, elbowMaxOutput, elbowMinOutput);
    this.shoulderPID = new pidStuff(shoulderMotorPortNum, shoulderP, shoulderI, shoulderD, shoulderIz, shoulderFF, shoulderMaxOutput, shoulderMinOutput);
  }

  public void moveArm(double shoulderpos,double elbowpos,double wristpos){
    wrist.set(wristpos);
    shoulder.set(shoulderpos);
    elbow.set(elbowpos);
  }




  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wristPID.updatePID();
    elbowPID.updatePID();
    shoulderPID.updatePID();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
