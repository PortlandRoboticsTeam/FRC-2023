// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
//import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
//import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
//import com.swervedrivespecialties.swervelib.SwerveModule;

//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;  
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import static frc.robot.Constants.*;
//import frc.robot.subsystems.SModule;

public class driveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final static SModule leftFrontModule = new SModule(
    Constants.leftFrontDrivePortNum,
    Constants.leftFrontTurnPortNum,
    Constants.leftFrontDriveEncoderReversed,
    Constants.leftFrontTurnEncoderReversed,
    Constants.leftFrontEncoderPortNum,
    Constants.leftFrontAbsoluteEncoderOff,
    Constants.leftFrontAbsoluteEncoderReversed
  );
  private final static SModule rightFrontModule = new SModule(
    Constants.rightFrontDrivePortNum,
    Constants.rightFrontTurnPortNum,
    Constants.rightFrontDriveEncoderReversed,
    Constants.rightFrontTurnEncoderReversed,
    Constants.rightFrontEncoderPortNum,
    Constants.rightFrontAbsoluteEncoderOff,
    Constants.rightFrontAbsoluteEncoderReversed
  );
  private final static SModule leftBackModule = new SModule(
    Constants.leftBackDrivePortNum,
    Constants.leftBackTurnPortNum,
    Constants.leftBackDriveEncoderReversed,
    Constants.leftBackTurnEncoderReversed,
    Constants.leftBackEncoderPortNum,
    Constants.leftBackAbsoluteEncoderOff,
    Constants.leftBackAbsoluteEncoderReversed
  );
  private final static SModule rightBackModule = new SModule(
    Constants.rightBackDrivePortNum,
    Constants.rightBackTurnPortNum,
    Constants.rightBackDriveEncoderReversed,
    Constants.rightBackTurnEncoderReversed,
    Constants.rightBackEncoderPortNum,
    Constants.leftBackAbsoluteEncoderOff,
    Constants.rightBackAbsoluteEncoderReversed
  );

  private static AHRS gyro = new AHRS(SPI.Port.kMXP);
  // private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.DRIVE_KINEMATICS,
  //           new Rotation2d(0),);

  public driveTrain() {
    new Thread(()->{
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
        // handle exception
      }
    }).start();
  }
  public void zeroHeading(){
    gyro.reset();
  }
  public static double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  public static Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("robot Heading", getHeading());
  }

  public static void stopModules(){
    leftFrontModule.stop();
    rightFrontModule.stop();
    leftBackModule.stop();
    rightBackModule.stop();
  }

  public static void setModualeState(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MaxSpeedMetersPerSecond);
    leftFrontModule.setDesiredState(desiredStates[0]);
    rightFrontModule.setDesiredState(desiredStates[1]);
    leftBackModule.setDesiredState(desiredStates[2]);
    rightBackModule.setDesiredState(desiredStates[3]);
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
