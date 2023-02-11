package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldPositionSubsystem extends SubsystemBase{
    private int xPosition = 0;
    private int yPosition = 0;
    public FieldPositionSubsystem(){
        SmartDashboard.putData("Location 1", location1());
        SmartDashboard.putData("Location 2", location2());
        SmartDashboard.putData("Location 3", location3());
        SmartDashboard.putData("Location 4", location4());
        SmartDashboard.putData("Location 5", location5());
        SmartDashboard.putData("Location 6", location6());
        SmartDashboard.putData("Location 7", location7());
        SmartDashboard.putData("Location 8", location8());
        SmartDashboard.putData("Location 9", location9());
        SmartDashboard.putData("Location 10", location10());
        SmartDashboard.putData("Location 11", location11());
    }

    public int getXPosition() {
        return xPosition;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("position", xPosition);
    }

    public CommandBase location1() {
        return run(() -> {
            xPosition = 1;           
        }).ignoringDisable(true);
    }
   
    public CommandBase location2() {
        return run(() -> {
            xPosition = 2;           
        }).ignoringDisable(true);
    }

    public CommandBase location3() {
        return run(() -> {
            xPosition = 3;           
        }).ignoringDisable(true);
    }
    
    public CommandBase location4() {
        return run(() -> {
            xPosition = 4;           
        }).ignoringDisable(true);
    }
    
    public CommandBase location5() {
        return run(() -> {
            xPosition = 5;           
        }).ignoringDisable(true);
    }
    
    public CommandBase location6() {
        return run(() -> {
            xPosition = 6;           
        }).ignoringDisable(true);
    }
    
    public CommandBase location7() {
        return run(() -> {
            xPosition = 7;           
        }).ignoringDisable(true);
    }
    
    public CommandBase location8() {
        return run(() -> {
            xPosition = 8;           
        }).ignoringDisable(true);
    }
    
    public CommandBase location9() {
        return run(() -> {
            xPosition = 9;           
        }).ignoringDisable(true);
    }
    
    public CommandBase location10() {
        return run(() -> {
            xPosition = 10;           
        }).ignoringDisable(true);
    }

    public CommandBase location11() {
        return run(() -> {
            xPosition = 11;           
        }).ignoringDisable(true);
    }

}
