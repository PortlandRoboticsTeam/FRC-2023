// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.temporal.WeekFields;

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
  public static final int wristMotorPortNum = 15;
  public static final int elbowMotorPortNum = 12;
  public static final int shoulderMotorPortNum = 13;
  public static final int leftFrontDrivePortNum = 0;
  public static final int rightFrontDrivePortNum = 1;
  public static final int leftBackDrivePortNum = 2;
  public static final int rightBackDrivePortNum = 3;
  public static final int leftFrontTurnPortNum = 4;
  public static final int rightFrontTurnPortNum = 5;
  public static final int leftBackTurnPortNum = 6;
  public static final int rightBackTurnPortNum = 7;
  public static final int leftFrontEncoderPortNum = 8;
  public static final int rightFrontEncoderPortNum = 9;
  public static final int leftBackEncoderPortNum = 10;
  public static final int rightBackEncoderPortNum = 11;

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

  public static final double leftFrontAbsoluteEncoderOff = 0.0;
  public static final double rightFrontAbsoluteEncoderOff = 0.0;
  public static final double leftBackAbsoluteEncoderOff = 0.0;
  public static final double rightBacktAbsoluteEncoderOff = 0.0;

  public static final double MaxSpeedMetersPerSecond = 0;

  public static final double driveEncoderRotationToMeter = 0;
  public static final double driveEncoderRPM2MetersPerSecend = 0;
  public static final double turnEncoderRotationToMeter = 0;
  public static final double turnEncoderRPM2MetersPerSecend = 0;

  public static final double pTurning = 0;

  public static final double phisicalMaxSpeedMetersPerSec = 0;

  public static final double deadband = 0;

  public static final double maxExeleration = 0;
  public static final double maxAngulerExeleration = 0;

  public static final double teleopMaxSpeedMetersPerSec = 0;
  public static final double teleopMaxAngulerSpeedRaidensPerSec = 0;

  public static final double trackWidth = 0;
  public static final double weelbase = 0;
  public static final SwerveDriveKinematics DRIVE_KINEMATICS =  new SwerveDriveKinematics(
    new Translation2d(weelbase/2,-trackWidth/2),
    new Translation2d(weelbase/2,trackWidth/2),
    new Translation2d(-weelbase/2,-trackWidth/2),
    new Translation2d(-weelbase/2,trackWidth/2));

  public static final int driverYAxis = 0;
  public static final int driverxAxis = 0;
  public static final int driverRotAxis = 0;
  public static final int fieldOrentedButtonIndex = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 14;
    

  }
}
