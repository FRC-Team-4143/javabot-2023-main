package frc.robot.subsystems4143;

import com.ctre.phoenixpro.Utils;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import javax.naming.directory.DirContext;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.GlobalConstants;
import frc.lib.swerve.*;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Preferences;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private WPI_TalonFX angleMotor;
    //private WPI_TalonFX driveMotor;
    private TalonFX driveMotor;
    private AnalogEncoder analogEncoder;
    private double lastAngle;
    private final DutyCycleOut driveOut = new DutyCycleOut(0,true,false);
    private final VelocityDutyCycle driveVelOut = new VelocityDutyCycle(0,true,0,0,false);
    
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.angleKS, Constants.SwerveConstants.angleKV, Constants.SwerveConstants.angleKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = Preferences.getDouble("Module"+moduleNumber, 0);
       
        /* Angle Encoder Config */
        // angleEncoder = moduleConstants.canivoreName.isEmpty()
        //         ? new WPI_CANCoder(moduleConstants.cancoderID)
        //         : new WPI_CANCoder(moduleConstants.cancoderID, moduleConstants.canivoreName.get());
        // configAngleEncoder();

        analogEncoder = new AnalogEncoder(moduleConstants.cancoderID);

        /* Angle Motor Config */
        angleMotor = moduleConstants.canivoreName.isEmpty()
                ? new WPI_TalonFX(moduleConstants.angleMotorID, "CANivore")
                : new WPI_TalonFX(moduleConstants.angleMotorID, moduleConstants.canivoreName.get());
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.driveMotorID, "CANivore")
                : new TalonFX(moduleConstants.driveMotorID, moduleConstants.canivoreName.get());
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }
    public void setWheelOffsets(){
        angleOffset = analogEncoder.getAbsolutePosition()*360.0;
        Preferences.setDouble("Module"+moduleNumber, angleOffset);
        resetToAbsolute();
    }
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        setDesiredState(desiredState, isOpenLoop, false);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isSecondOrder) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller, which CTRE is not
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            //driveMotor.set(ControlMode.PercentOutput, percentOutput);
            driveOut.Output = percentOutput;
            driveMotor.setControl(driveOut);
        } else {
            double velocity = Conversions.MPSToFalconPro(
                    desiredState.speedMetersPerSecond,
                    Constants.SwerveConstants.wheelCircumference,
                    Constants.SwerveConstants.driveGearRatio);
            driveVelOut.Velocity = velocity;
            driveVelOut.FeedForward = driveFeedforward.calculate(desiredState.speedMetersPerSecond) / GlobalConstants.targetVoltage;
            driveMotor.setControl(driveVelOut);
            /*driveMotor.set(
                    ControlMode.Velocity,
                    velocity,
                    DemandType.ArbitraryFeedForward,
                    driveFeedforward.calculate(desiredState.speedMetersPerSecond));*/
        }

        // Determine the angle to set the module to
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle
                        .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        // Account for the velocity of the angle motor if in second order mode
        if (isSecondOrder && desiredState instanceof SecondOrderSwerveModuleState) {
            angleMotor.set(
                    ControlMode.Position,
                    Conversions.degreesToFalcon(0, Constants.SwerveConstants.angleGearRatio),
                    DemandType.ArbitraryFeedForward,
                    angleFeedforward.calculate(
                            ((SecondOrderSwerveModuleState) desiredState).angularVelocityRadiansPerSecond));
        } else {
            angleMotor.set(
                    ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio));
        }

        lastAngle = angle;
    }

    public void setDesiredAngleOnly(Rotation2d desiredAngle, boolean optimized) {
        // Set the module to face forwards
        if (optimized) {
            desiredAngle = CTREModuleState.optimize(new SwerveModuleState(1, desiredAngle), getState().angle).angle;
        }

        angleMotor.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(desiredAngle.getDegrees(), Constants.SwerveConstants.angleGearRatio));

        lastAngle = 0;

        // Set the drive motor to the specified voltage
        driveMotor.stopMotor();
    }
    
    public void setDriveCharacterizationVoltage(double voltage) {
        // Set the module to face forwards
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(0, Constants.SwerveConstants.angleGearRatio));

        lastAngle = 0;

        // Set the drive motor to the specified voltage
        driveOut.Output = voltage / Constants.GlobalConstants.targetVoltage;
        driveMotor.setControl(driveOut);
        //driveMotor.set(ControlMode.PercentOutput, voltage / Constants.GlobalConstants.targetVoltage);
    }

    public void setAngleCharacterizationVoltage(double voltage) {
        // Set the module to face forwards
        angleMotor.set(ControlMode.PercentOutput, voltage / Constants.GlobalConstants.targetVoltage);

        lastAngle = 0;

        // Set the drive motor to just enough to overcome static friction
        driveOut.Output = 1.1 * Constants.SwerveConstants.driveKS;
        driveMotor.setControl(driveOut);
        //driveMotor.set(ControlMode.PercentOutput, 1.1 * Constants.SwerveConstants.driveKS);
    }

    public Rotation2d getCanCoder(){
        return getAnalogEncoder();
    }

    public Rotation2d getAnalogEncoder(){
        return Rotation2d.fromDegrees(analogEncoder.getAbsolutePosition()*360.0);
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getAnalogEncoder().getDegrees() - angleOffset, Constants.SwerveConstants.angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);

        System.out.println("Analog encoder value: " + getAnalogEncoder().getDegrees() + " ID: " + moduleNumber);
    }

    // private void configAngleEncoder() {
    //     angleEncoder.configFactoryDefault();
    //     angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    // }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        angleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        angleMotor.enableVoltageCompensation(true);
        //resetToAbsolute();
    }

    private void configDriveMotor() {
        var driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.MotorOutput.Inverted = Constants.SwerveConstants.driveEncoderInvert 
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfiguration.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode == NeutralMode.Coast 
                        ? NeutralModeValue.Coast
                        : NeutralModeValue.Brake;
        driveConfiguration.Voltage.PeakForwardVoltage = Constants.GlobalConstants.targetVoltage;
        driveConfiguration.Voltage.PeakReverseVoltage = -Constants.GlobalConstants.targetVoltage;
        driveConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        driveConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        driveConfiguration.ClosedLoopGeneral.ContinuousWrap = false;
        driveConfiguration.Slot0.kP = Constants.SwerveConstants.driveKP;
        driveConfiguration.Slot0.kI = Constants.SwerveConstants.driveKI;
        driveConfiguration.Slot0.kD = Constants.SwerveConstants.driveKD;
        driveConfiguration.Slot0.kV = 0.0;
        //driveConfiguration.Slot0.kF = Constants.SwerveConstants.driveKF;
        driveConfiguration.CurrentLimits.StatorCurrentLimit = Constants.SwerveConstants.driveContinuousCurrentLimit;
        driveConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.driveContinuousCurrentLimit;
        driveConfiguration.CurrentLimits.StatorCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
        driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
        
        driveMotor.getConfigurator().apply(driveConfiguration);
        driveMotor.setRotorPosition(0);
        driveMotor.setSafetyEnabled(true);
        driveOut.EnableFOC = true;
        
        /*
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSensorPhase(Constants.SwerveConstants.driveEncoderInvert);
        driveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert); */
    }

    // public Rotation2d getCanCoder() {
    //     return Rotation2d.fromDegrees(analogEncoder.getAbsolutePosition());
    // }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconProToMPS(
                driveMotor.getVelocity().getValue(),
                //driveMotor.getSelectedSensorVelocity(),
                Constants.SwerveConstants.wheelCircumference,
                Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public double getAngularVelocity() {
        return Conversions.falconToRPM(angleMotor.getSelectedSensorVelocity(), Constants.SwerveConstants.angleGearRatio)
                / 60
                * 2
                * Math.PI;
    }

    public SwerveModulePosition getPosition() {
        double encoder = Conversions.falconProToMPS(
                        driveMotor.getPosition().getValue(),
                        //driveMotor.getSelectedSensorPosition(),
                        Constants.SwerveConstants.wheelCircumference,
                        Constants.SwerveConstants.driveGearRatio);
                /// 10.0; // Compensate for Talon measuring in 100 ms units
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
        return new SwerveModulePosition(encoder, angle);
    }

    public double getDriveTemperature() {
        return driveMotor.getDeviceTemp().getValue();
        //return driveMotor.getTemperature();
    }

    public double getAngleTemperature() {
        return angleMotor.getTemperature();
    }

    public double getDriveVoltage() {
        return driveMotor.getBridgeOuput().getValue().value;
        //return driveMotor.getMotorOutputVoltage();
    }

    public double getAngleVoltage() {
        return angleMotor.getMotorOutputVoltage();
    }

    public double getDriveCurrent() {
        return driveMotor.getSupplyCurrent().getValue();
        //return driveMotor.getSupplyCurrent();
    }

    public double getAngleCurrent() {
        return angleMotor.getSupplyCurrent();
    }
}
