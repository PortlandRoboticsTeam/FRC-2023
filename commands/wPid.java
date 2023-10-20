// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.wrist;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class wPid extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final wrist m_wrist;

    
    public wPid(wrist wrist) {
      m_wrist = wrist;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_wrist.getController().enableContinuousInput(-180, 180);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_wrist.useOutput(wrist.optimise(m_wrist.getMeasurement(),m_wrist.getController().calculate(m_wrist.getMeasurement(), Constants.WAngels[m_wrist.position])), Constants.WAngels[m_wrist.position]);
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