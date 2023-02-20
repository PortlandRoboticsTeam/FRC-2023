// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driveTrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import static frc.robot.Constants.*;

/** An example command that uses an example subsystem. */
public class driving extends CommandBase {
  private final driveTrain m_DriveTrain;
  private final Supplier<Double> xSpdFunction,ySpdFunction,turningSpdFunction;
  // private final Supplier<Boolean>fealdOreantedFunction;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;
  private SlewRateLimiter xLimiter;
  public driving(driveTrain driveTrain, Supplier<Double> ySpdFunction, Supplier<Double> xSpdFunction,
      Supplier<Double> turningSpdFunction) {
        
    this.m_DriveTrain = driveTrain; 
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;

    
    //limiters
    this.xLimiter = new SlewRateLimiter(Constants.maxExeleration);
    this.yLimiter = new SlewRateLimiter(Constants.maxExeleration);
    this.turningLimiter = new SlewRateLimiter(Constants.maxAngulerExeleration);
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    //deadband
    xSpeed = Math.abs(xSpeed)>Constants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed)>Constants.deadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed)>Constants.deadband ? turningSpeed : 0.0;

    //smoothing
    xSpeed = xLimiter.calculate(xSpeed)*teleopMaxSpeedMetersPerSec;
    ySpeed = yLimiter.calculate(ySpeed)*teleopMaxSpeedMetersPerSec;
    turningSpeed = turningLimiter.calculate(turningSpeed)*teleopMaxAngulerSpeedRaidensPerSec;
    //to chassySpeed
    ChassisSpeeds chassisSpeeds;
    //to field
    // fealdOreantedFunction.get();
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turningSpeed,driveTrain.getRotation2d());
    
    SwerveModuleState[] moduleStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    driveTrain.setModualeState(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
