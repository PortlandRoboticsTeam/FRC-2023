// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.simulation.SolenoidSim;

public class claw extends SubsystemBase {
    boolean closed = false;
    Solenoid m_Solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    Compressor m_Compressor = new Compressor(PneumaticsModuleType.REVPH);
  /** Creates a new ExampleSubsystem. */
  public claw() {
    m_Solenoid.set(closed);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean getClosed() {
    // Query some boolean state, such as a digital sensor.
    return closed;
  }

  public void openClose(){
    closed = !closed;
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}