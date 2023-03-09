// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.elbow;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ePidComLess extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final elbow m_elbow;

    
    public ePidComLess(elbow elbow) {
      m_elbow = elbow;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(elbow);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_elbow.getController().enableContinuousInput(-180, 180);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elbow.useOutput(elbow.optimise(m_elbow.getMeasurement(),m_elbow.getController().calculate(m_elbow.getMeasurement(), Constants.EAngels[m_elbow.position])), Constants.EAngels[m_elbow.position]);
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