// package frc.robot.commands;

// import edu.wpi.first.wpilibj.BuiltInAccelerometer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.SwerveShaninigans;;

// public class balanceAuto extends CommandBase {
//     private BuiltInAccelerometer mRioAccel;
//     private int state;
//     private int debounceCount;
//     private double robotSpeedSlow;
//     private double robotSpeedFast;
//     private double onChargeStationDegree;
//     private double levelDegree;
//     private double debounceTime;
//     private double singleTapTime;
//     private double scoringBackUpTime;
//     private double doubleTapTime;
//     private SwerveShaninigans drivetrain;
//     private boolean done;

//     public balanceAuto(SwerveShaninigans swerve) {
//         this.drivetrain = swerve;
//         addRequirements(swerve);
        

//     }
//     @Override
//     public void initialize(){
       
//     }
//     @Override
//     public void execute(){

//     }

//     @Override
//     public boolean isFinished(){
//         return done;
//     }

//     @Override
//     public void end(boolean interupted){

//     }

//     // returns the magnititude of the robot's tilt calculated by the root of
//     // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    

//     // routine for automatically driving onto and engaging the charge station.
//     // returns a value from -1.0 to 1.0, which left and right motors should be set
//     // to.
//     public double autoBalanceRoutine() {
//         switch (state) {
//             // drive forwards to approach station, exit when tilt is detected
//             case 0:
//                 if (drivetrain.getTilt() > onChargeStationDegree) {
//                     debounceCount++;
//                 }
//                 if (debounceCount > drivetrain.secondsToTicks(debounceTime)) {
//                     state = 1;
//                     debounceCount = 0;
//                     return robotSpeedSlow;
//                 }
//                 return robotSpeedFast;
//             // driving up charge station, drive slower, stopping when level
//             case 1:
//                 if (drivetrain.getTilt() < levelDegree) {
//                     debounceCount++;
//                 }
//                 if (debounceCount > drivetrain.secondsToTicks(debounceTime)) {
//                     state = 2;
//                     debounceCount = 0;
//                     return 0;
//                 }
//                 return robotSpeedSlow;
//             // on charge station, stop motors and wait for end of auto
//             case 2:
//                 if (Math.abs(drivetrain.getTilt()) <= levelDegree / 2) {
//                     debounceCount++;
//                 }
//                 if (debounceCount > drivetrain.secondsToTicks(debounceTime)) {
//                     state = 4;
//                     debounceCount = 0;
//                     return 0;
//                 }
//                 if (drivetrain.getTilt() >= levelDegree) {
//                     return 0.1;
//                 } else if (drivetrain.getTilt() <= -levelDegree) {
//                     return -0.1;
//                 }
//             case 3:
//                 return 0;
//         }
//         return 0;
//     }

//     // Same as auto balance above, but starts auto period by scoring
//     // a game piece on the back bumper of the robot
//     public double scoreAndBalance() {
//         switch (state) {
//             // drive back, then forwards, then back again to knock off and score game piece
//             case 0:
//                 debounceCount++;
//                 if (debounceCount < drivetrain.secondsToTicks(singleTapTime)) {
//                     return -robotSpeedFast;
//                 } else if (debounceCount < drivetrain.secondsToTicks(singleTapTime + scoringBackUpTime)) {
//                     return robotSpeedFast;
//                 } else if (debounceCount < drivetrain.secondsToTicks(singleTapTime + scoringBackUpTime + doubleTapTime)) {
//                     return -robotSpeedFast;
//                 } else {
//                     debounceCount = 0;
//                     state = 1;
//                     return 0;
//                 }
//                 // drive forwards until on charge station
//             case 1:
//                 if (drivetrain.getTilt() > onChargeStationDegree) {
//                     debounceCount++;
//                 }
//                 if (debounceCount > drivetrain.secondsToTicks(debounceTime)) {
//                     state = 2;
//                     debounceCount = 0;
//                     return robotSpeedSlow;
//                 }
//                 return robotSpeedFast;
//             // driving up charge station, drive slower, stopping when level
//             case 2:
//                 if (drivetrain.getTilt() < levelDegree) {
//                     debounceCount++;
//                 }
//                 if (debounceCount > drivetrain.secondsToTicks(debounceTime)) {
//                     state = 3;
//                     debounceCount = 0;
//                     return 0;
//                 }
//                 return robotSpeedSlow;
//             // on charge station, ensure robot is flat, then end auto
//             case 3:
//                 if (Math.abs(drivetrain.getTilt()) <= levelDegree / 2) {
//                     debounceCount++;
//                 }
//                 if (debounceCount > drivetrain.secondsToTicks(debounceTime)) {
//                     state = 4;
//                     debounceCount = 0;
//                     return 0;
//                 }
//                 if (drivetrain.getTilt() >= levelDegree) {
//                     return robotSpeedSlow / 2;
//                 } else if (drivetrain.getTilt() <= -levelDegree) {
//                     return -robotSpeedSlow / 2;
//                 }
//             case 4:
//                 return 0;
//         }
//         return 0;
//     }
// }