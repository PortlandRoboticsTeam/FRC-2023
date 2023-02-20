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
  public static final int leftFrontDrivePortNum = 1;
  public static final int rightFrontDrivePortNum = 3;
  public static final int leftBackDrivePortNum = 5;
  public static final int rightBackDrivePortNum = 7;
  public static final int leftFrontTurnPortNum = 2;
  public static final int rightFrontTurnPortNum = 4;
  public static final int leftBackTurnPortNum = 6;
  public static final int rightBackTurnPortNum = 8;
  public static final int leftFrontEncoderPortNum = 1;
  public static final int rightFrontEncoderPortNum = 2;
  public static final int leftBackEncoderPortNum = 3;
  public static final int rightBackEncoderPortNum = 4;

  public static final boolean leftFrontDriveEncoderReversed = false;
  public static final boolean rightFrontDriveEncoderReversed = false;
  public static final boolean leftBackDriveEncoderReversed = false;
  public static final boolean rightBackDriveEncoderReversed = false;
  public static final boolean leftFrontTurnEncoderReversed = false;
  public static final boolean rightFrontTurnEncoderReversed = false;
  public static final boolean leftBackTurnEncoderReversed = false;
  public static final boolean rightBackTurnEncoderReversed = false;
  public static final boolean leftFrontAbsoluteEncoderReversed = false;
  public static final boolean rightFrontAbsoluteEncoderReversed = false;
  public static final boolean leftBackAbsoluteEncoderReversed = false;
  public static final boolean rightBackAbsoluteEncoderReversed = false;

  public static final double leftFrontAbsoluteEncoderOff = -Math.toRadians(286.347);
  public static final double rightFrontAbsoluteEncoderOff = -Math.toRadians(217.002);;
  public static final double leftBackAbsoluteEncoderOff = -Math.toRadians(27.686);
  public static final double rightBacktAbsoluteEncoderOff = -Math.toRadians(256.113);

  public static final double MaxSpeedMetersPerSecond = 0.5;

  public static final double driveEncoderRotationToMeter = 1.0;
  public static final double driveEncoderRPM2MetersPerSecend =10;
  public static final double turnEncoderRotationToMeter = 1.0;
  public static final double turnEncoderRPM2MetersPerSecend = 1.0;

  public static final double pTurning = 0.5;

  public static final double phisicalMaxSpeedMetersPerSec = 1.0;

  public static final double deadband = 0.05;

  public static final double maxExeleration = 0.5;
  public static final double maxAngulerExeleration = 0.5;

  public static final double teleopMaxSpeedMetersPerSec = 0.5;
  public static final double teleopMaxAngulerSpeedRaidensPerSec = .5*Math.PI;

  public static final double trackWidth = 0.5207;
  public static final double weelbase = 0.5969;
  public static final SwerveDriveKinematics DRIVE_KINEMATICS =  new SwerveDriveKinematics(
    new Translation2d(weelbase/2,-trackWidth/2),
    new Translation2d(weelbase/2,trackWidth/2),
    new Translation2d(-weelbase/2,-trackWidth/2),
    new Translation2d(-weelbase/2,trackWidth/2));

  //public static final int fieldOrentedButtonIndex;

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

  public static final double wHomePos = 0;
  public static final double eHomePos = 0;
  public static final double sHomePos = 0;

  public static final double WPickUpPos = 0;
  public static final double EPickUpPos = 0;
  public static final double SPickUpPos = 0;

  public static final double WMiddleConePos = 0;
  public static final double EMiddleConePos = 0;
  public static final double SMiddleConePos = 0;

  public static final double WMiddleCubePos = 0;
  public static final double EMiddleCubePos = 0;
  public static final double SMiddleCubePos = 0;

  public static final double WHeighConePos = 0;
  public static final double EHeighConePos = 0;
  public static final double SHeighConePos = 0;

  public static final double WHeighCubePos = 0;
  public static final double EHeighCubePos = 0;
  public static final double SHeighCubePos = 0;

  public static final double WHumanPlayerPos = 0;
  public static final double EHumanPlayerPos = 0;
  public static final double SHumanPlayerPos = 0;

  
  public static class OperatorConstants {
    public static final int kDriverControllerPort =0 ;
    

  }
}
