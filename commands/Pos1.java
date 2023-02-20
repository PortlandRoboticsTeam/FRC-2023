// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.elbow;
import frc.robot.subsystems.shoulder;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Pos1 extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final wrist m_wrist;
    private final elbow m_elbow;
    private final shoulder m_shoulder;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Pos1(wrist wrist,shoulder shoulder,elbow elbow) {
      m_wrist = wrist;
      m_elbow = elbow;
      m_shoulder = shoulder;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(wrist,shoulder,elbow);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_wrist.position = 1;
        m_elbow.position = 1;
        m_shoulder.position = 1;}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
}