package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class Intake extends SubsystemBase {
    // Motors
    private final SparkMax armMotor = new SparkMax(Constants.IntakeConstants.armMotorID, MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);

    // Encoder
    private final RelativeEncoder armEncoder;

    // PID
    private final PIDController pidController;

    // Setpoint for roller
    private double setpoint = 0.0;

    public Intake() {
        // Initialize the PID
        pidController = new PIDController(Constants.IntakeConstants.arm_kP, Constants.IntakeConstants.arm_kI,
                Constants.IntakeConstants.arm_kD);
        pidController.reset();
        pidController.setSetpoint(IntakeConstants.armRaisedPosition);

        armEncoder = armMotor.getEncoder();

        AlternateEncoderConfig armEncoderConfig = new AlternateEncoderConfig()
                .positionConversionFactor(IntakeConstants.armConversionFactor)
                .setSparkMaxDataPortConfig();
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.inverted(false);
        armConfig.apply(armEncoderConfig);
        armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        pidController.setTolerance(Constants.IntakeConstants.armTolerance);
    }

    @Override
    public void periodic() {
        // Arm Profiled PID control
        double armPosition = getArmPosition();
        double armSetpoint = pidController.getSetpoint();

        double armOutput = IntakeConstants.arm_kG * Math.cos(armPosition);

        if (Math.abs(armSetpoint - armPosition) > 0.1)
            armOutput += pidController.calculate(armPosition, armSetpoint);

        // SmartDashboard updates
        SmartDashboard.putNumber("Roller Setpoint", setpoint);
        SmartDashboard.putNumber("Arm Position", armPosition);
        SmartDashboard.putNumber("Arm Setpoint", armSetpoint);
        SmartDashboard.putNumber("Arm Output", armOutput);

        armMotor.setVoltage(0);
    }

    public void setIntakeState(double position, double rollerVolts) {
        intakeMotor.setVoltage(rollerVolts);

        pidController.reset();
        pidController.setSetpoint(position);
    }

    public void stopArm() {
        armMotor.setVoltage(0);
    }

    public double getArmPosition() {
        return armEncoder.getPosition() + IntakeConstants.armOffset;
    }

    public boolean armAtSetpoint() {
        return pidController.atSetpoint();
    }

    public boolean rollersAtSetpoint() {
        return Math.abs(intakeMotor.getEncoder().getVelocity() - setpoint) < 25;
    }

}
