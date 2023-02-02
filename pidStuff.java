package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class pidStuff {
  double defRotations = 16.6;
  int deviceID = 1;
  CANSparkMax m_motor;
  SparkMaxPIDController m_pidController;
  RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  double rotations;
  public pidStuff(int motorPortNum){
// initialize motor
m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

/**
 * The restoreFactoryDefaults method can be used to reset the configuration parameters
 * in the SPARK MAX to their factory default state. If no argument is passed, these
 * parameters will not persist between power cycles
 */
m_motor.restoreFactoryDefaults();

/**
 * In order to use PID functionality for a controller, a SparkMaxPIDController object
 * is constructed by calling the getPIDController() method on an existing
 * CANSparkMax object
 */
m_pidController = m_motor.getPIDController();

// Encoder object created to display position values
m_encoder = m_motor.getEncoder();

// PID coefficients
kP = 0.1; 
kI = 1e-4;
kD = 1; 
kIz = 0; 
kFF = 0; 
kMaxOutput = 1; 
kMinOutput = -1;

// set PID coefficients
m_pidController.setP(kP);
m_pidController.setI(kI);
m_pidController.setD(kD);
m_pidController.setIZone(kIz);
m_pidController.setFF(kFF);
m_pidController.setOutputRange(kMinOutput, kMaxOutput);

// display PID coefficients on SmartDashboard
SmartDashboard.putNumber("P Gain", kP);
SmartDashboard.putNumber("I Gain", kI);
SmartDashboard.putNumber("D Gain", kD);
SmartDashboard.putNumber("I Zone", kIz);
SmartDashboard.putNumber("Feed Forward", kFF);
SmartDashboard.putNumber("Max Output", kMaxOutput);
SmartDashboard.putNumber("Min Output", kMinOutput);
SmartDashboard.putNumber("Set Rotations", defRotations);
    }
    public void updatePID(){
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      rotations = SmartDashboard.getNumber("Set Rotations", 0);
  
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { m_pidController.setP(p); kP = p; }
      if((i != kI)) { m_pidController.setI(i); kI = i; }
      if((d != kD)) { m_pidController.setD(d); kD = d; }
      if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
  
      /**
       * PIDController objects are commanded to a set point using the 
       * SetReference() method.
       * 
       * The first parameter is the value of the set point, whose units vary
       * depending on the control type set in the second parameter.
       * 
       * The second parameter is the control type can be set to one of four 
       * parameters:
       *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
       *  com.revrobotics.CANSparkMax.ControlType.kPosition
       *  com.revrobotics.CANSparkMax.ControlType.kVelocity
       *  com.revrobotics.CANSparkMax.ControlType.kVoltage
       */
      m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
      
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }
  public void setRotations( double rotations){
    this.rotations = rotations;
  }
}
