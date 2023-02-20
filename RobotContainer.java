// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.driving;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.*;
import frc.robot.subsystems.driveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final driveTrain m_driveTrain = new driveTrain();
  private final wrist m_wrist = new wrist(); 
  private final elbow m_elbow = new elbow(); 
  private final shoulder m_Shoulder = new shoulder();
  public  final driving m_driving ;
  public final wSetPos m_WSetPos = new wSetPos(m_wrist);
  public final sSetPos m_sSetPos = new sSetPos(m_Shoulder);
  public final eSetPos m_eSetPos = new eSetPos(m_elbow);
  public final Pos1 m_Pos1 = new Pos1(m_wrist, m_Shoulder, m_elbow);

  public final Joystick m_Joystick = new Joystick(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final PS4Controller ps4 = new PS4Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @param m_driving */
  public RobotContainer() {
    m_driveTrain.setDefaultCommand( m_driving = new driving(
      m_driveTrain,
      ()->-m_driverController.getRawAxis(Constants.driverxAxis),
      ()->m_driverController.getRawAxis(Constants.driverYAxis),
      ()->m_driverController.getRawAxis(Constants.driverRotAxis)
      ));

    //makes the arm constinly update positions  
    m_wrist.setDefaultCommand(m_WSetPos);
    m_elbow.setDefaultCommand(m_eSetPos);
    m_Shoulder.setDefaultCommand(m_sSetPos);
    // Configure the trigger bindings
    configureBindings();
    
  }
  public void setAllPos(int pos){
    m_wrist.position =pos;
    m_elbow.position =pos;
    m_Shoulder.position =pos;
  }
  public boolean getAllAtHome(){
    if (m_wrist.atHome&&m_elbow.atHome&&m_Shoulder.atHome){
      return true;
    }else{
      return false;
    }
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
     Trigger xButton = m_driverController.cross();
     Trigger oButton = m_driverController.circle();
     Trigger triButton = m_driverController.triangle();
     Trigger sqrButton  = m_driverController.square();
    if ()
     else if (m_Joystick.getRawButton(0)){


    }
    
    //  oButton.onTrue(m_wPickUpPos.andThen(m_ePickUpPos.andThen(m_sPickUpPos)));
    //  triButton.onTrue(m_wmiddleConePos.andThen(m_emiddleConePos.andThen(m_smiddleConePos)));
    //  sqrButton.onTrue(m_whumanPlayerPos.andThen(m_ehumanPlayerPos.andThen(m_shumanPlayerPos)));
    
  }

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
