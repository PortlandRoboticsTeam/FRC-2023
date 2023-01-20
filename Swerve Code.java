package frc.robot.subsystems;


import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class moduales {
    
    private final RelativeEncoder driveCanCoder;
    private final RelativeEncoder turnCanCoder;

    private final CANSparkMax driveCanSparkMax;
    private final CANSparkMax turnCanSparkMax;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean analogReversed;
    private final double analogOffsetAngle;
    public moduales(int driveMoterID, int turnMoterID, boolean driveMoterReversed, boolean turnMoterReversed, boolean analogReversed,double analogOffset, int analogID){
        this.analogOffsetAngle = analogOffset;
        this.analogReversed = analogReversed;
        AnalogInput absoluteEncoder = CANSparkMax.getAnalog(SparkMaxAnalogSensor());
        CANSparkMax driveMoter = new CANSparkMax(driveMoterID, MotorType.kBrushless);
        CANSparkMax turnMoter = new CANSparkMax(driveMoterID, MotorType.kBrushless);

        driveMoter.setInverted(driveMoterReversed);
        turnMoter.setInverted(turnMoterReversed);

        driveCanCoder = driveMoter.getEncoder();
        turnCanCoder = driveMoter.getEncoder();

        //todo fill arguments
        driveCanCoder.setPositionConversionFactor();
        driveCanCoder.setVelocityConversionFactor();

        turnCanCoder.setPositionConversionFactor();
        turnCanCoder.setVelocityConversionFactor();

        turningPidController = new PIDController(, , );
        turningPidController.enableContinuousInput(-Math.PI,Math.PI );

        resetEncoders();
    }
    public double getDrivePosition(){
        return driveCanCoder.getPosition();
    }
    public double getDriveVolocity() {
        return turnCanCoder.getVelocity();
    }
    public double getTurnPosition() {
        return  driveCanCoder.getPosition();
    }
    public double getTurnVolocity() {
        return turnCanCoder.getVelocity();
    }
    public double getAnologEncoderRad() {
        double angle = absoluteEncoder.getVelocity/RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= analogOffsetAngle;
        return angle * (analogReversed ? -1.0 : 1.0);
    }
    
    public void resetEncoders(){
        driveCanCoder.setPosition(0);
        turnCanCoder.setPosition(getAnologEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVolocity(),new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond)< 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMoter.set(state.speedMetersPerSecond / );
        turnmoter.set(turningPidController.calculate(getTurnPosition(),state.angle.getRadians()));
        SmartDashboard.putString("swerve["+AbsoluteEncoder.getChannel()+"] state", state.toString());

    }

    public  void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
