// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.time.temporal.WeekFields;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //port nums
  public static final int wristMotorPortNum = 10;
  public static final int elbowMotorPortNum = 11;
  public static final int shoulderMotorPortNum = 12;
  public static final int wristEncoderPortNum = 5;
  public static final int elbowEncoderPortNum = 6;
  public static final int shoulderEncoderPortNum = 7;
  // public static final int leftFrontDrivePortNum = 1;
  // public static final int rightFrontDrivePortNum = 3;
  // public static final int leftBackDrivePortNum = 5;
  // public static final int rightBackDrivePortNum = 7;
  // public static final int leftFrontTurnPortNum = 2;
  // public static final int rightFrontTurnPortNum = 4;
  // public static final int leftBackTurnPortNum = 6;
  // public static final int rightBackTurnPortNum = 8;
  // public static final int leftFrontEncoderPortNum = 1;
  // public static final int rightFrontEncoderPortNum = 2;
  // public static final int leftBackEncoderPortNum = 3;
  // public static final int rightBackEncoderPortNum = 4;

  // public static final boolean [] DriveEncoderReversed = {false,false,false,false};
  // public static final boolean [] TurnEncoderReversed = {false,false,false,false};
  // public static final boolean leftFrontAbsoluteEncoderReversed = false;
  // public static final boolean rightFrontAbsoluteEncoderReversed = false;
  // public static final boolean leftBackAbsoluteEncoderReversed = false;
  // public static final boolean rightBackAbsoluteEncoderReversed = false;

  // public static final double leftFrontAbsoluteEncoderOff = -1.856117;//Math.toRadians(286.347);
  // public static final double rightFrontAbsoluteEncoderOff = -0.655010;//Math.toRadians(217.002);;
  // public static final double leftBackAbsoluteEncoderOff = -1.030835;//Math.toRadians(27.686);
  // public static final double rightBackAbsoluteEncoderOff = -2.686001;//Math.toRadians(256.113);

  // public static final double MaxSpeedMetersPerSecond = 5880 / 60;

  // public static final double drivingGearRatio = 6.75/1;
  // public static final double turningGearRatio = 150/7;

  // public static final double driveEncoderRotationToMeter = 3.0*42*drivingGearRatio;
  // public static final double driveEncoderRPM2MetersPerSecend = driveEncoderRotationToMeter/60;
  // public static final double turnEncoderRotationToRaiden = turningGearRatio*2*Math.PI;
  // public static final double turnEncoderRPM2RadPerSecend = turnEncoderRotationToRaiden/60;

  // public static final double pTurning = 0.5;
  // public static final double iTurning = 0.0;
  // public static final double dTurning = 0.0;

  // public static final double phisicalMaxSpeedMetersPerSec = 1.0;

  // public static final double deadband = 0.05;

  // public static final double maxExeleration = 0.5;
  // public static final double maxAngulerExeleration = 0.1;

  // public static final double teleopMaxSpeedMetersPerSec = 0.5;
  // public static final double teleopMaxAngulerSpeedRaidensPerSec = 2*Math.PI;

  public static final double trackWidth = 0.5207;
  public static final double weelbase = 0.4699;
  // public static final SwerveDriveKinematics DRIVE_KINEMATICS =  new SwerveDriveKinematics(
  //   new Translation2d(weelbase/2,-trackWidth/2),
  //   new Translation2d(weelbase/2,trackWidth/2),
  //   new Translation2d(-weelbase/2,-trackWidth/2),
  //   new Translation2d(-weelbase/2,trackWidth/2));

  // //public static final int fieldOrentedButtonIndex;

  public static final int driverxAxis = 1;
  public static final int driverYAxis = 0;
  public static final int driverRotAxis = 4;

  //pid stuff
  public static final double wristP = 1.0;
  public static final double wristI = 1.0; 
  public static final double wristD = 1.0; 

  public static final double elbowP = 1.0;
  public static final double elbowI = 1.0; 
  public static final double elbowD = 1.0;
  
  public static final double shoulderP = 1.0;
  public static final double shoulderI = 1.0; 
  public static final double shoulderD = 1.0; 

  public static final double wristToleranceRPS = 0.5;
  public static final double elbowToleranceRPS = 0.5;
  public static final double shoulderToleranceRPS = 0.5;

  public static final double wristEncoderDistancePerPulse = 0.348;
  public static final double elbowEncoderDistancePerPulse = 0.348;
  public static final double shoulderEncoderDistancePerPulse = 0.348;

  public static final double wristTargetRPS = 1.0;
  public static final double elbowTargetRPS = 1.0;
  public static final double shoulderTargetRPS = 1.0;

  public static final double wVolts =12;
  public static final double eVolts =12;
  public static final double sVolts =12;

  public static final double wVoltSecondsPerRotation = 0.1;
  public static final double eVoltSecondsPerRotation = 0.1;
  public static final double sVoltSecondsPerRotation = 0.1;

  public static final double wTurnToleranceDeg = 0.1;
  public static final double eTurnToleranceDeg = 0.1;
  public static final double sTurnToleranceDeg = 0.1;

  public static final double wTurnRateToleranceDegPerS = 0.1;
  public static final double eTurnRateToleranceDegPerS = 0.1;
  public static final double sTurnRateToleranceDegPerS = 0.1;

  public static final double zero = 0; 
  //pid angles
  public static final double[] WAngels = {0,0,0,0,0,0,0};
  public static final double[] EAngels = {0,0,0,0,0,0,0};
  public static final double[] SAngels = {0,0,0,0,0,0,0};
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort =0 ;

    

  }


  //Controller Port Nums
  public static final int m_controllerPortNum = 0;

  //Motor Port Nums
  public static final int lowerConveyorMotorPortNum = 13;
  public static final int higherConveyorMotorPortNum = 5;
  public static final int ballLaunchingMotorPortNum = 10;
  public static final int rightArmMotorPortNum = 11;
  public static final int leftArmMotorPortNum = 12;

  //Other Ports
  public static final int ultrasonicPortNum = 0;
  public static final int pcmPortNum = 1;

  //random things
  public static final double speedReductionConst = 0.25;
  public static final Boolean driveRelativeToField = false;
  public static final double launchSpeed = 0.9;
  public static final double lowerConveyorSpeed = 0.8;
  public static final double higherConveyorSpeed = 0.6;
  public static final double minShootDistance = 103+10;
  public static final double maxShootDistance = 103-10;
  public static final double extendSpeed = 1;
  public static final double retractSpeed = .9;
  public static final double maxArmLength = 430;
  public static final double minArmLength = 0;


  //All code below is swerve drive constants.
  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = trackWidth;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = weelbase;

  public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; // FIXME Set front left steer encoder ID
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = (5*Math.PI/4);//Math.toRadians(286.4355); // FIXME Measure and set front left steer offset

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front right drive motor ID
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2; // FIXME Set front right steer encoder ID
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -(Math.PI/4);//Math.toRadians(37.52930); // FIXME Measure and set front right steer offset

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3; // FIXME Set back left steer encoder ID
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -(Math.PI/4);//-Math.toRadians(27.5097); // FIXME Measure and set back left steer offset

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set back right steer motor ID
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4; // FIXME Set back right steer encoder ID
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = (Math.PI/4); // FIXME Measure and set back right steer offset
  

}

