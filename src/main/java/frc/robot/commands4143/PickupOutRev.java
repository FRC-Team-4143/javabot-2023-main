package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems4143.PickupSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer4143;
import frc.robot.Constants.gamePiece;
import frc.robot.subsystems4143.Arm;


public class PickupOutRev extends CommandBase{
    private PickupSubsystem pickupSubsystem;
    private Arm arm;
    private int count;
    public RobotContainer4143 container;

    public PickupOutRev(PickupSubsystem pickupSubsystem, Arm arm, RobotContainer4143 container){
        this.container = container;
        this.pickupSubsystem = pickupSubsystem;
        this.arm = arm;
        count=0;
    }

    @Override
    public void initialize() {
        count = 0;
        arm.elevatorPickup();
    }

    @Override
    public void execute() {
        if(container.currentMode == gamePiece.Cube) {
            pickupSubsystem.solenoidExtend();
            pickupSubsystem.rollersSet(0.35);
        } else {
            pickupSubsystem.solenoidExtend();
            pickupSubsystem.rollersSet(1);
        }
        
        count+=1;

    }

    @Override
    public void end(boolean interrupted){
        pickupSubsystem.solenoidRetract();
        pickupSubsystem.rollersSet(0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
