// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//most of this code is based on canden's a graduated member's code
package frc.robot.subsystems;

// import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SPI;


import static frc.robot.Constants.*;

public class SwerveShaninigans extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
//   private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
 private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
        
  private final Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();


  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // BuiltInAccelerometer mRioAccel = new BuiltInAccelerometer();
  // private int state = 0;
  // private int debounceCount = 0;

  //       /**********
  //        * CONFIG *
  //        **********/
  //       // Speed the robot drived while scoring/approaching station, default = 0.4
  //       private double robotSpeedFast = 0.4;

  //       // Speed the robot drives while balancing itself on the charge station.
  //       // Should be roughly half the fast speed, to make the robot more accurate,
  //       // default = 0.2
  //       private double robotSpeedSlow = 0.2;

  //       // Angle where the robot knows it is on the charge station, default = 13.0
  //       private double onChargeStationDegree = 13.0;

  //       // Angle where the robot can assume it is level on the charging station
  //       // Used for exiting the drive forward sequence as well as for auto balancing,
  //       // default = 6.0
  //       private double levelDegree = 6.0;

  //       // Amount of time a sensor condition needs to be met before changing states in
  //       // seconds
  //       // Reduces the impact of sensor noice, but too high can make the auto run
  //       // slower, default = 0.2
  //       private double debounceTime = 0.2;

  //       // Amount of time to drive towards to scoring target when trying to bump the
  //       // game piece off
  //       // Time it takes to go from starting position to hit the scoring target
  //       private double singleTapTime = 0.4;

  //       // Amount of time to drive away from knocked over gamepiece before the second
  //       // tap
  //       private double scoringBackUpTime = 0.2;

  //       // Amount of time to drive forward to secure the scoring of the gamepiece
  //       private double doubleTapTime = 0.3;

  // public double getPitch() {
  //   return Math.atan2((-mRioAccel.getX()),
  //     Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
  //   }
  
  // public double getRoll() {
  //   return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
  // }
  // public double getTilt() {
  //   double pitch = getPitch();
  //   double roll = getRoll();
  //   if ((pitch + roll) >= 0) {
  //       return Math.sqrt(pitch * pitch + roll * roll);
  //   } else {
  //       return -Math.sqrt(pitch * pitch + roll * roll);
  //   }
  // }

  // public int secondsToTicks(double time) {
  //   return (int) (time * 50);
  // }

  public SwerveShaninigans() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    config.setDriveCurrentLimit(30);
    config.setSteerCurrentLimit(20);
    
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(config, Mk4iSwerveModuleHelper.GearRatio.L2, 
    FRONT_LEFT_MODULE_DRIVE_MOTOR,
    FRONT_LEFT_MODULE_STEER_MOTOR,
    FRONT_LEFT_MODULE_STEER_ENCODER,
    FRONT_LEFT_MODULE_STEER_OFFSET);
        
    m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(config,
        Mk4iSwerveModuleHelper.GearRatio.L2,
        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        FRONT_RIGHT_MODULE_STEER_MOTOR,
        FRONT_RIGHT_MODULE_STEER_ENCODER,
        FRONT_RIGHT_MODULE_STEER_OFFSET
        );
    m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(config,
        Mk4iSwerveModuleHelper.GearRatio.L2,
        BACK_LEFT_MODULE_DRIVE_MOTOR,
        BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER,
        BACK_LEFT_MODULE_STEER_OFFSET
        );

    m_backRightModule = Mk4iSwerveModuleHelper.createNeo(config,
        Mk4iSwerveModuleHelper.GearRatio.L2,
        BACK_RIGHT_MODULE_DRIVE_MOTOR,
        BACK_RIGHT_MODULE_STEER_MOTOR,
        BACK_RIGHT_MODULE_STEER_ENCODER,
        BACK_RIGHT_MODULE_STEER_OFFSET
        );
        
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
    // m_pigeon.setFusedHeading(0.0);

    // FIXME Uncomment if you are using a NavX
    m_navx.zeroYaw();
  }


  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    //fl is fl /fr is bl /bl is fr/br is br
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    //wrong inverted positions
    SmartDashboard.putNumber("fl", m_backRightModule.getSteerAngle());
    SmartDashboard.putNumber("fr", m_backLeftModule.getSteerAngle());
    SmartDashboard.putNumber("bl", m_frontRightModule.getSteerAngle());
    SmartDashboard.putNumber("br", m_frontLeftModule.getSteerAngle()); 
  }

  public Rotation2d getGyroscopeRotation() {
        return m_navx.getRotation2d();
  };

  // public  getOdomentery(){
  //   return
  // }
}