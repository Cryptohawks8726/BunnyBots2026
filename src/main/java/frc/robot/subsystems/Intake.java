package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class Intake extends StatefulSubsystem {
    // Motors
    private final SparkMax armMotor;
    private final SparkMax rollerMotor;

    double armOutput;

    // Encoder
    private final RelativeEncoder armEncoder;

    // PID
    private final PIDController armController;
    private final BangBangController rollerController;

    public Intake() {
        super("Intake Subsystem");
        // PID
        armController = new PIDController(IntakeConstants.arm_kP, IntakeConstants.arm_kI,
                IntakeConstants.arm_kD);
        armController.reset();
        armController.setSetpoint(IntakeConstants.armRaisedPosition);

        // Motors
        armMotor = new SparkMax(IntakeConstants.armMotorID, MotorType.kBrushless);
        rollerMotor = new SparkMax(IntakeConstants.rollerMotorID, MotorType.kBrushless);

        // Arm encoder
        armEncoder = armMotor.getEncoder();

        // Configures
        EncoderConfig armEncoderConfig = new SparkMaxConfig().encoder
                .positionConversionFactor(IntakeConstants.armConversionFactor);

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.inverted(false);
        armConfig.apply(armEncoderConfig);
        armConfig.idleMode(IdleMode.kBrake);
        armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        armController.setTolerance(IntakeConstants.armTolerance);

        EncoderConfig rollerEncoderConfig = new SparkMaxConfig().encoder
                .positionConversionFactor(IntakeConstants.rollerVelocityConversionFactor);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        rollerConfig.idleMode(IdleMode.kCoast)
                .apply(rollerEncoderConfig)
                .inverted(false);
        rollerMotor.configure(rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        armController.setTolerance(IntakeConstants.armTolerance);

        armEncoder.setPosition(IntakeConstants.armRaisedPosition);

        rollerController = new BangBangController(IntakeConstants.acceptedRollerError);
        rollerController.setSetpoint(0);

        putOnDashboard();
    }

    @Override
    public void periodic() {
        // Arm Profiled PID control
        double armPosition = getArmPosition();
        double armSetpoint = armController.getSetpoint();

        // Roller
        double armOutput = IntakeConstants.arm_kS + (IntakeConstants.arm_kG * Math.cos(armPosition)) + armController
                .calculate(armPosition, armSetpoint);

        armMotor.setVoltage(armOutput);

        double rollerOutput = rollerController.calculate(rollerMotor.getEncoder().getVelocity(),
                IntakeConstants.desiredRollerRPM);
        rollerMotor.setVoltage(rollerOutput * IntakeConstants.rollerVoltage);
    }

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    // SETTING FUNCTIONS
    public void setIntakeState(double position, double rollerVolts) {
        rollerMotor.setVoltage(rollerVolts);

        armController.reset();
        armController.setSetpoint(position);
    }

    public void stopArm() {
        armMotor.setVoltage(0);
    }

    public void stopRollers() {
        rollerMotor.setVoltage(0);
    }

    // BOOLEANS
    public boolean armAtSetpoint() {
        return armController.atSetpoint();
    }

    public boolean rollersAtSetpoint() {
        return Math.abs(rollerMotor.getEncoder().getVelocity()
                - IntakeConstants.desiredRollerRPM) < IntakeConstants.acceptedRollerError;
    }

    // RUN NEXT COMMAND FUNCS
    public void IntakeArmUp() {
        runNextCommand(new InstantCommand(() -> {
            setIntakeState(IntakeConstants.armRaisedPosition, 0);
        }).withName("Intake Up"), false);
    }

    public void IntakeArmDown() {
        runNextCommand(new InstantCommand(() -> {
            setIntakeState(IntakeConstants.armRaisedPosition, IntakeConstants.desiredRollerRPM);
        }).withName("Intake Down"), false);
    }

    public void IntakeReverseRollers() {
        runNextCommand(new InstantCommand(() -> {
            setIntakeState(IntakeConstants.armPartialRaisedPosition, IntakeConstants.desiredRollerRPM);
        }).withName("Intake Down"), false);
    }

    public double getArmOutput() {
        return armOutput;
    }

    public double getArmPos() {
        return armEncoder.getPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        SmartDashboard.putData("Subsystems/" + getName() +
                "/MutableValues/IntakeArmPID", armController);
        SmartDashboard.putData("Subsystems/" + getName() +
                "/MutableValues/IntakeRollersBBC", rollerController);

        builder.addDoubleProperty("Arm Output", this::getArmOutput, null);
        builder.addDoubleProperty("Arm Position", this::getArmPos, null);

    }

}