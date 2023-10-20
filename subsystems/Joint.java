package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joint extends PIDSubsystem{
        public int position = 0;
        CANSparkMax moter;
        static PIDController elbowController;
        CANCoder encoder;
        double elbowSetpoint;
        // private final SimpleMotorFeedforward m_elbowFeedforward =
        //     new SimpleMotorFeedforward(
        //         sVolts, sVoltSecondsPerRotation);
        public boolean atHome = true;
        double optimised;
      
        public Joint(int CANCoderID,int moterID,double p, double i, double d,double tolerance) {
          super(elbowController = new PIDController(p, i, d));
          moter = new CANSparkMax(moterID, MotorType.kBrushless);
          encoder = new CANCoder(CANCoderID);
          getController().setTolerance(tolerance);
          setSetpoint(elbowSetpoint);
          disable();
          moter.setSmartCurrentLimit(20);
        }
      
        public void setPos(double elbowPoint){
          elbowSetpoint = elbowPoint;
        }
        public int getPosition(){
          return position;
        }
      
        @Override
        public void useOutput(double output, double setpoint) {
          moter.setVoltage(optimised =optimise(getMeasurement(),-output));//+ -1*m_elbowFeedforward.calculate(setpoint));
          SmartDashboard.putNumber("optimised", optimised);
        }
      
        @Override
        public void periodic() {
          // This method will be called once per scheduler run
          
          SmartDashboard.putNumber("elbow position", getMeasurement());
          SmartDashboard.putNumber("ePosition number", getPosition());
          super.periodic();
        }
      
        @Override
        public void simulationPeriodic() {
          // This method will be called once per scheduler run during simulation
        }
      
        @Override
        public double getMeasurement() {
          return encoder.getAbsolutePosition();
        }
        
        public boolean atSetpoint() {
          return m_controller.atSetpoint();
        }
      
        public static double optimise(double current ,double desired){
          var delta = desired - current;
          if (Math.abs(delta)> Math.PI/2){
            return desired - Math.PI;
          }else{
            return desired;
          }
        }
      
        public void wSetVolatge(double volts){
          moter.setVoltage(volts);
        } 
}
