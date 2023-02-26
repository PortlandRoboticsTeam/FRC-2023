// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shoulder;
//import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.wrist;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class sSetPos extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public sSetPos(shoulder m_shoulder) {
    super(
      //PIDControler
      new PIDController(Constants.shoulderP,Constants.shoulderI,Constants.shoulderD),
      ()->m_shoulder.getMeasurement(),
      //give it set point
      Constants.SAngels[m_shoulder.position],
      //send output to motor
      output->m_shoulder.useOutput(output,Constants.SAngels[m_shoulder.position]),
      //required subsystem
      m_shoulder);
      
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.wTurnToleranceDeg, Constants.wTurnRateToleranceDegPerS);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    getController().atSetpoint();
    
    
    return false;
  }
}
