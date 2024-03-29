// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.time.temporal.WeekFields;




/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 public static final double output = 24;



  //port nums
  public static final int wristMotorPortNum = 10;
  public static final int elbowMotorPortNum = 11;
  public static final int shoulderMotorPortNum = 12;
  public static final int wristEncoderPortNum = 7;
  public static final int elbowEncoderPortNum = 6;
  public static final int shoulderEncoderPortNum = 5;

  public static final double trackWidth = 0.5207;
  public static final double weelbase = 0.4699;

  public static final int driverxAxis = 1;
  public static final int driverYAxis = 0;
  public static final int driverRotAxis = 4;

  //pid stuff
  public static final double wristP = 0.06;//09
  public static final double wristI = 0.0; 
  public static final double wristD = 0.01;//007 009

  public static final double elbowP = 0.1;
  public static final double elbowI = 0.0; 
  public static final double elbowD = 0.01;
  
  public static final double shoulderP = 0.1;
  public static final double shoulderI = 0.0; 
  public static final double shoulderD = 0.01; 

  public static final double wristToleranceRPS = 0.05;
  public static final double elbowToleranceRPS = 0.05;
  public static final double shoulderToleranceRPS = 0.05;

  public static final double wristEncoderDistancePerPulse = 0.348;
  public static final double elbowEncoderDistancePerPulse = 0.348;
  public static final double shoulderEncoderDistancePerPulse = 0.348;

  public static final double wristTargetRPS = 1.0;
  public static final double elbowTargetRPS = 1.0;
  public static final double shoulderTargetRPS = 1.0;

  public static final double wVolts =12;
  public static final double eVolts =12;
  public static final double sVolts =12;

  public static final double wVoltSecondsPerRotation = 0.05;
  public static final double eVoltSecondsPerRotation = 0.05;
  public static final double sVoltSecondsPerRotation = 0.05;

  public static final double wTurnToleranceDeg = 0.1;
  public static final double eTurnToleranceDeg = 0.05;
  public static final double sTurnToleranceDeg = 0.05;

  public static final double wTurnRateToleranceDegPerS = 0.05;
  public static final double eTurnRateToleranceDegPerS = 0.05;
  public static final double sTurnRateToleranceDegPerS = 0.05;

  public static final double zero = 0; 

  //home,0
  //pid angles
  public static final double wo = 336;
  public static final double[] WAngels = {wo,wo+12,45,55,45,wo+23,45,wo};

  //elbow offset
  public static final double eo = 260;
  public static final double[] EAngels = {eo-50,eo-51,eo+10,eo+74,eo-17,eo+10,eo};

  //shoulder offset
  private static final double so = 0;
  public static final double[] SAngels = {so-19,so+30,so+40,so+90,so,so+40,so};//116s
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0 ;
    public static final int kArmControllerPort = 1 ;

    

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
  //needs a pi off
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = (7*Math.PI/4);//Math.toRadians(37.52930); // FIXME Measure and set front right steer offset

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3; // FIXME Set back left steer encoder ID
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = (7*Math.PI/4);//-Math.toRadians(27.5097); // FIXME Measure and set back left steer offset

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set back right steer motor ID
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4; // FIXME Set back right steer encoder ID
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = (1*Math.PI/4); // FIXME Measure and set back right steer offset
  

}