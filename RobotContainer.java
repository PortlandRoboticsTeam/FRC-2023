// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.commands.driving;

//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.*;

import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PS4Controller;
//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveShaninigans m_DrivetrainSubsystem = new SwerveShaninigans();
  private final wrist m_wrist = new wrist(); 
  private final elbow m_elbow = new elbow(); 
  private final shoulder m_Shoulder = new shoulder();
  //public  final driving m_driving ;
  public final claw m_Claw = new claw();
  //public final wSetPos m_WSetPos = new wSetPos(m_wrist);
  //public final sSetPos m_sSetPos = new sSetPos(m_Shoulder);
  //public final eSetPos m_eSetPos = new eSetPos(m_elbow);
  public final ZeroGyro m_ZeroGyro = new ZeroGyro(m_DrivetrainSubsystem);
  public final GenericHID m_Joystick = new GenericHID(OperatorConstants.kDriverControllerPort);
  public final sPidComLess m_sPidComLess = new sPidComLess(m_Shoulder);
  public final ePidComLess m_ePidComLess = new ePidComLess(m_elbow);
  public final wPidComLess m_wPidComLess = new wPidComLess(m_wrist);
  public final deactivateArms m_DeactivateArms = new deactivateArms(m_Shoulder,m_elbow,m_wrist);

  public final InstantCommand pos1 = new InstantCommand(()->setPos(0),m_Shoulder);//,m_elbow,m_wrist);
  public final InstantCommand pos2 = new InstantCommand(()->setPos(1),m_Shoulder);//,m_elbow,m_wrist);
  public final InstantCommand pos3 = new InstantCommand(()->setPos(2),m_Shoulder);//,m_elbow,m_wrist);
  public final InstantCommand pos4 = new InstantCommand(()->setPos(3),m_Shoulder);//,m_elbow,m_wrist);
  public final InstantCommand pos5 = new InstantCommand(()->setPos(4),m_Shoulder);//,m_elbow,m_wrist);
  public final InstantCommand pos6 = new InstantCommand(()->setPos(5),m_Shoulder);//,m_elbow,m_wrist);
  public final InstantCommand pos7 = new InstantCommand(()->setPos(6),m_Shoulder);//,m_elbow,m_wrist);
  // public final InstantCommand pos = new InstantCommand(()->setPos(0),m_Shoulder,m_elbow,m_wrist);
  // public final InstantCommand pos = new InstantCommand(()->setPos(0),m_Shoulder,m_elbow,m_wrist);
  public final InstantCommand openClose = new InstantCommand(()->openClose() ,m_Claw);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  //private final PS4Controller ps4 = new PS4Controller(0);

  public double smoothLogisticInput(double input, Boolean drive) {
    if(drive){
      if (input > 0.1) {
          return 1/(1 + Math.exp(-10*Math.abs(input)+5));
      } else if (input < -0.1) {
          return -(1/(1 + Math.exp(-10*Math.abs(input)+5)));
      } 
      return 0;
    }
    else{
      if (input > 0.4) {
        return 1/(1 + Math.exp(-15*Math.abs(input)+10));
    } else if (input < -0.4) {
        return -(1/(1 + Math.exp(-15*Math.abs(input)+10)));
    } 
    return 0;
    }
  }
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    m_DrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_DrivetrainSubsystem,
      //For PS4/5 controller
      // () -> -(smoothLogisticInput(m_driverController.getLeftX(), true) * SwerveShaninigans.MAX_VELOCITY_METERS_PER_SECOND * Constants.speedReductionConst),
      // () -> (smoothLogisticInput(m_driverController.getLeftY(), true) * SwerveShaninigans.MAX_VELOCITY_METERS_PER_SECOND * Constants.speedReductionConst),
      // () -> (smoothLogisticInput(m_driverController.getRightX(), true) * SwerveShaninigans.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.speedReductionConst)

      //For Joystick
      () -> -(smoothLogisticInput(m_Joystick.getRawAxis(0), true) * SwerveShaninigans.MAX_VELOCITY_METERS_PER_SECOND * Constants.speedReductionConst),
      () -> (smoothLogisticInput(m_Joystick.getRawAxis(1), true) * SwerveShaninigans.MAX_VELOCITY_METERS_PER_SECOND * Constants.speedReductionConst),
      () -> (smoothLogisticInput(m_Joystick.getRawAxis(2), true) * SwerveShaninigans.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.speedReductionConst)
));
    // Configure the trigger bindings
    configureBindings();
  m_Shoulder.setDefaultCommand(m_sPidComLess);
  m_elbow.setDefaultCommand(m_ePidComLess);
  m_wrist.setDefaultCommand(m_wPidComLess);

  }

  

  //set the positons that all the motors go to
  public void setAllPos(int pos){
    m_wrist.position =pos;
    m_elbow.position =pos;
    m_Shoulder.position =pos;
  }
  public boolean getAllAtHome(){
    //if (m_Shoulder.atHome&&m_elbow.atHome){
    if (m_wrist.atHome&&m_elbow.atHome&&m_Shoulder.atHome){
      return true;
    }else{
      return false;
    }
  }

  public void setAllAtHome(boolean atHome){
    m_wrist.atHome = atHome;
    m_elbow.atHome = atHome;
    m_Shoulder.atHome = atHome;
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
    
    JoystickButton b1 = new JoystickButton(m_Joystick, 1);
    JoystickButton b2 = new JoystickButton(m_Joystick, 2);
    JoystickButton b3 = new JoystickButton(m_Joystick, 3);
    JoystickButton b4 = new JoystickButton(m_Joystick, 4);
    JoystickButton b5 = new JoystickButton(m_Joystick, 5);
    JoystickButton b6 = new JoystickButton(m_Joystick, 6);
    JoystickButton b7 = new JoystickButton(m_Joystick, 7);
    JoystickButton b8 = new JoystickButton(m_Joystick, 8);
    JoystickButton b9 = new JoystickButton(m_Joystick, 9);
    JoystickButton b10 = new JoystickButton(m_Joystick, 10);
    JoystickButton b11 = new JoystickButton(m_Joystick, 11);
    JoystickButton b12 = new JoystickButton(m_Joystick, 12);
    JoystickButton b13 = new JoystickButton(m_Joystick, 13);
    JoystickButton b16 = new JoystickButton(m_Joystick, 16);

    b1.onTrue(openClose);
    //zero gyro
    b2.onTrue(m_ZeroGyro);
    //home
    b5.onTrue(pos1);
    //start
    b6.onTrue(pos5);
    //player
    b7.onTrue(pos6);

    //low mid high
    b10.onTrue(pos2);
    b9.onTrue(pos3);
    b8.onTrue(pos4);

    //deactivate arms
    //b16.onTrue(m_DeactivateArms);

    
  
        
      // Trigger triButton = m_driverController.triangle();
      // Trigger sqrButton  = m_driverController.square();
    //xButton.onTrue(m_ZeroGyro);
    //sqrButton.onTrue(m_ZeroGyro);
    
  }
  public void setPos(int position){
    // if (position == 0){
    //   setAllAtHome(true);
    //   setAllPos(position);
    // }else if (getAllAtHome()){
    //   setAllAtHome(false);
    //   setAllPos(position);
    // }
    setAllPos(position); 
    
  }

  public void openClose(){
    m_Claw.openClose();
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