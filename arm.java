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
    this.wristPID = new pidStuff(wristMotorPortNum);
    this.elbowPID = new pidStuff(elbowMotorPortNum);
    this.shoulderPID = new pidStuff(shoulderMotorPortNum);
  }

  public void setPos(double wristpos){
    wristPID.setRotations(wristpos);
    
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
