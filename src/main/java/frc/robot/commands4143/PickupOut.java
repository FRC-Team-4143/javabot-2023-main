package frc.robot.commands4143;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer4143;
import frc.robot.Constants.gamePiece;
import frc.robot.subsystems4143.Arm;
import frc.robot.subsystems4143.PickupSubsystem;

public class PickupOut extends CommandBase{
    private PickupSubsystem pickupSubsystem;
    private int count;
    private Arm arm;
    public RobotContainer4143 container;

    public PickupOut(PickupSubsystem pickupSubsystem, Arm arm, RobotContainer4143 container){
        this.container = container;
        this.pickupSubsystem = pickupSubsystem;
        this.arm =arm;
        count=0;
    }

    @Override
    public void initialize() {
        count = 0;
        arm.elevatorPickup();
        addRequirements(pickupSubsystem);
        pickupSubsystem.solenoidExtend();
    }

    @Override
    public void execute() {
        if(container.currentMode == gamePiece.Cube) {
            pickupSubsystem.rollersSetSlow(-0.5);
        } else {
            
            pickupSubsystem.rollersSet(-0.80);
        }
        
        count+=1;
        if(count > 20) {
            pickupSubsystem.dump();
        }
        else{
            pickupSubsystem.solenoidExtend();
        }

    }

    @Override
    public void end(boolean interrupted){
        if(!DriverStation.isAutonomous()){
            pickupSubsystem.solenoidRetract();
            pickupSubsystem.rollersSet(0.0);
        }

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
