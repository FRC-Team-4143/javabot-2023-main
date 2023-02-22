package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {System.out.println("start of main");}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
