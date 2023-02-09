package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Arm extends SubsystemBase {
    // private SparkMaxPIDController m_pidController;
    // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private ProfiledPIDController elevatorMotorController;
    private ProfiledPIDController rotatorMotorController;
    private CANSparkMax clawMotor;
    private CANSparkMax elevatorMotor;
    private CANSparkMax rotatorMotor; 
    //private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_elevatorEncoder;
    private RelativeEncoder m_rotatorEncoder;
    public double elevatorkP, rotatorkP, elevatorkI, rotatorkI, elevatorkD, rotatorkD, kMaxOutput, kMinOutput;
    private double distance;
    private double angle;
    private static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(84, 84);
    private static final TrapezoidProfile.Constraints rotatorConstraints = new TrapezoidProfile.Constraints(20, 20);
    private Mechanism2d mechanism = new Mechanism2d(4, 4);
    private MechanismRoot2d root = mechanism.getRoot("Arm", 2, 2);
    private MechanismLigament2d arm1;
    private MechanismLigament2d arm2;
    public static final Rotation2d arm1StartingAngle = Rotation2d.fromDegrees(45);
    public static final Rotation2d arm2StartingAngle = Rotation2d.fromDegrees(-135);


    public Arm(){ 
        clawMotor = new CANSparkMax(3, MotorType.kBrushed);
        elevatorMotor = new CANSparkMax(10, MotorType.kBrushless);
        rotatorMotor = new CANSparkMax(5, MotorType.kBrushless);
        clawMotor.setSmartCurrentLimit(10);
        elevatorMotor.setSmartCurrentLimit(20);
        rotatorMotor.setSmartCurrentLimit(20);
        

    // Encoder object created to display position values
    m_elevatorEncoder = elevatorMotor.getEncoder();
    m_rotatorEncoder = rotatorMotor.getEncoder();
    m_elevatorEncoder.setPositionConversionFactor(22*0.25/(22/16*3*3)*(25.4/1000)); //Conversion from revolutions to meters, 0.01129
    m_rotatorEncoder.setPosition(360/100); //Conversion from revolutions to degrees, 3.6

    arm1 = root.append(
                new MechanismLigament2d("Arm 1", 2.0, arm1StartingAngle.getDegrees()));
    arm2 = arm1.append(
                new MechanismLigament2d("Arm 2", 0.5, arm2StartingAngle.getDegrees()));

    arm1.setLineWeight(5);
    arm2.setLineWeight(5);

    // PID coefficients
    elevatorkP = 18; 
    elevatorkI = 0;
    elevatorkD = 0;
    rotatorkP = 0.03; 
    rotatorkI = 0;
    rotatorkD = 0;
    kMaxOutput = 0.25; 
    kMinOutput = -0.25;
    distance = 0;
    angle = 0;

    // set PID coefficients
    elevatorMotorController = new ProfiledPIDController(elevatorkP, elevatorkI, elevatorkD, elevatorConstraints);
    rotatorMotorController = new ProfiledPIDController(rotatorkP, rotatorkI, rotatorkD, rotatorConstraints);


    }

    
    public void setClawSpeed(double clawSpeed){
        clawMotor.set(clawSpeed);
    }

    public void setRotateSpeed(double rotateSpeed){ //Make this two functional
        angle += rotateSpeed;
        if(angle>0){
            angle=0;
           }
           if(angle<-151){
            angle=-151;
           }
    }



    public CommandBase setHighPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() > -0.44 ){
                distance = -0.9;
            } else {
                distance = -0.9;
                angle = -151;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -151 && distance == -0.9){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setMidPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() > -0.44 ){
                distance = -0.56;
            } else {
                distance = -0.56;
                angle = -122;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -122 && distance == -0.56){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setHomePosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_rotatorEncoder.getPosition() < -5.4 ){
                angle = -3.6;
            } else {
                distance = -0.01129;
                angle = -3.6;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -3.6 && distance == -0.01129){
                return true;
            }else{
                return false;
            }
            
        });
    }

    public CommandBase set0Arm() {
        return runOnce(() -> {
            m_elevatorEncoder.setPosition(0);
            m_rotatorEncoder.setPosition(0);
        }).ignoringDisable(true);
    }

    public void elevatorMove(double elevateSpeed){
       distance +=elevateSpeed;
       if(distance>0){
        distance=0;
       }
       if(distance<-0.95){
        distance=-0.95;
       }
    }

    public void setPosition(){
        distance = m_elevatorEncoder.getPosition();
        angle = m_rotatorEncoder.getPosition();
        elevatorMotorController.setGoal(distance);
        rotatorMotorController.setGoal(angle);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm distance", m_elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Arm angle", m_rotatorEncoder.getPosition());
        SmartDashboard.putData("Arm Mechanism", mechanism);
        SmartDashboard.putNumber("Distance Setpoint", distance);
        SmartDashboard.putNumber("Angle Setpoint", angle);
        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Voltage", elevatorMotor.getAppliedOutput());

        elevatorMotorController.setGoal(distance);
        elevatorMotor.set(elevatorMotorController.calculate(m_elevatorEncoder.getPosition()));
        rotatorMotorController.setGoal(angle);
        rotatorMotor.set(rotatorMotorController.calculate(m_rotatorEncoder.getPosition()));
        //m_pidController.setReference(distance, CANSparkMax.ControlType.kPosition);
    }
}
