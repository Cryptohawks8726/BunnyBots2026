package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.Constants;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class Intake extends StatefulSubsystem {
    // Motors
    private final SparkMax armMotor;
    private final SparkMax rollerMotor;

    // Encoder
    private final RelativeEncoder armEncoder;

    // PID
    private final PIDController armController;

    public Intake() {
        super("IntakeSubsystem");
        // PID
        armController = new PIDController(Constants.IntakeConstants.arm_kP, Constants.IntakeConstants.arm_kI,
                Constants.IntakeConstants.arm_kD);
        armController.reset();
        armController.setSetpoint(IntakeConstants.armRaisedPosition);

        // Motors
        armMotor = new SparkMax(Constants.IntakeConstants.armMotorID, MotorType.kBrushless);
        rollerMotor = new SparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);

        // Arm encoder
        armEncoder = armMotor.getEncoder();

        // Configures
        AlternateEncoderConfig armEncoderConfig = new AlternateEncoderConfig()
                .positionConversionFactor(IntakeConstants.armConversionFactor)
                .setSparkMaxDataPortConfig();
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.inverted(false);
        armConfig.apply(armEncoderConfig);
        armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        armController.setTolerance(Constants.IntakeConstants.armTolerance);

        putOnDashboard();
    }

    @Override
    public void periodic() {
        // Arm Profiled PID control
        double armPosition = getArmPosition();
        double armSetpoint = armController.getSetpoint();

        // Roller
        double armOutput = IntakeConstants.arm_kG * IntakeConstants.arm_kS * Math.cos(armPosition);

        if (Math.abs(armSetpoint - armPosition) > 0.1)
            armOutput += armController.calculate(armPosition, armSetpoint);

        // SmartDashboard updates
        SmartDashboard.putNumber("Roller Setpoint", Constants.IntakeConstants.rollerSetpoint);
        SmartDashboard.putNumber("Arm Position", armPosition);
        SmartDashboard.putNumber("Arm Setpoint", armSetpoint);
        SmartDashboard.putNumber("Arm Output", armOutput);

        armMotor.setVoltage(armOutput);
    }

    public double getArmPosition() {
        return armEncoder.getPosition() + IntakeConstants.armOffset;
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
                - Constants.IntakeConstants.rollerSetpoint) < Constants.IntakeConstants.acceptedRollerError;
    }

    // RUN NEXT COMMAND FUNCS
    public void IntakeArmUp() {
        runNextCommand(new InstantCommand(() -> {
            setIntakeState(IntakeConstants.armRaisedPosition, 0);
        }).withName("Intake Up"), false);
    }

    public void IntakeArmDown() {
        runNextCommand(new InstantCommand(() -> {
            setIntakeState(IntakeConstants.armRaisedPosition, IntakeConstants.rollerSetpoint);
        }).withName("Intake Down"), false);
    }

}
