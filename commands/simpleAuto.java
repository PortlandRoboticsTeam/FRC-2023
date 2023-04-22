package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveShaninigans;

public class simpleAuto extends CommandBase{
    private final SwerveShaninigans driveTrain;
    private final Timer timer = new Timer();
    private boolean done;
    private double ySpeed;

    public simpleAuto(SwerveShaninigans swerve){
        this.driveTrain = swerve;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize(){
        System.out.print("started");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("timer", timer.get());
        if(timer.get()<=3.4){
            driveTrain.drive(new ChassisSpeeds(0, .9, 0));
        //driveTrain
        
        }else{
            driveTrain.drive(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished(){
        return done;
    }

    @Override
    public void end(boolean interupted){
        timer.stop();
        timer.reset();
    }
}
