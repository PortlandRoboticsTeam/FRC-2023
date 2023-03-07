// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.shoulder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class sPidComLess extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final shoulder m_shoulder;

    
    public sPidComLess(shoulder shoulder) {
      m_shoulder = shoulder;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(shoulder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_shoulder.getController().disableContinuousInput();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shoulder.useOutput(shoulder.optimise(m_shoulder.getMeasurement(),  m_shoulder.getController().calculate(m_shoulder.getMeasurement(), Constants.SAngels[m_shoulder.position])), Constants.SAngels[m_shoulder.position]);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}