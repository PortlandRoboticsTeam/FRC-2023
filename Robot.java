// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  static autoBalance mAutoBalance = new autoBalance();
  static double forwardSpeed;
  static double rotationSpeed;
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20.5);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(1.1875);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3.833);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("Arducam_IMX477_HQ_Camera");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.05;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.01;
  final double ANGULAR_D = 0.1;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  PS4Controller xboxController = new PS4Controller(0);

  // Drive motors


  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("tilt", mAutoBalance.getTilt());
    
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    RobotContainer.m_wrist.setDefaultCommand(m_robotContainer.m_wPidComLess);
    RobotContainer.m_elbow.setDefaultCommand(m_robotContainer.m_ePidComLess);
    RobotContainer.m_Shoulder.setDefaultCommand(m_robotContainer.m_sPidComLess);
    // double speed = mAutoBalance.autoBalanceRoutine();
    // m_robotContainer.m_DrivetrainSubsystem.drive(new ChassisSpeeds(0,speed,0));
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   

    if (xboxController.getSquareButton()) {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
            if ((-forwardController.calculate(range, GOAL_RANGE_METERS)) >= 1){
              forwardSpeed = 1;
              }
            if ((-forwardController.calculate(range, GOAL_RANGE_METERS)) < 1){
              forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
              }
            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            //rotationSpeed = rotation;
            if ((-turnController.calculate(result.getBestTarget().getYaw())) >= 0.9){
              rotationSpeed = 0.9;
              }
            if ((-turnController.calculate(result.getBestTarget().getYaw())) < 0.9){
              rotationSpeed = ((-turnController.calculate(result.getBestTarget().getYaw())));
              }
        } else {
            // If we have no targets, stay still.
            forwardSpeed = 0;
            rotationSpeed = 0;
        }
    } else {
        // Manual Driver Mode
        
    }

    if (xboxController.getCircleButton()) {
      forwardSpeed = 0;
      rotationSpeed = 0;
    }
    // Use our forward/turn speeds to control the drivetrain
    SmartDashboard.putNumber("Forward Speed", forwardSpeed);
    SmartDashboard.putNumber("Rotation Speed",rotationSpeed);
  }
 
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
