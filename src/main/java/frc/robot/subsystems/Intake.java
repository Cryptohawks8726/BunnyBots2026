package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Intake extends SubsystemBase {
    // Motors
    private final SparkMax armMotor = new SparkMax(Constants.IntakeConstants.armMotorID, MotorType.kBrushless); // the
                                                                                                                // device
                                                                                                                // ID
                                                                                                                // will
                                                                                                                // be
                                                                                                                // changed;
    private final SparkMax rollerMotor = new SparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless); // the
                                                                                                                      // device
                                                                                                                      // ID
                                                                                                                      // will
                                                                                                                      // be
                                                                                                                      // changed;

    // Encoder
    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    // PID
    public final PIDController pidController = new PIDController(1, 0, 0); // Values will be tuned
    private double desiredAngleRadians;

    public Intake() {
        // SparkMaxConfig motorConfig = new SparkMaxConfig();

        rollerMotor.setVoltage(10); // Change based on testing
    }

    @Override
    public void periodic() {
        double measuredAngle = getAngleRadians();
        double feedbackVolts = pidController.calculate(measuredAngle, desiredAngleRadians);
        armMotor.setVoltage(feedbackVolts);
    }

    public void SetSetpoint(double val) {
        pidController.setSetpoint(val);
    }

    public void setDesiredAngleRadians(double angle) {
        desiredAngleRadians = angle;
    }

    public double getAngleRadians() {
        return (armEncoder.getPosition()) * (2 * Math.PI) / Constants.IntakeConstants.gearRatio; // The constant might
                                                                                                 // be changed
    }

    public double getDesiredAngleRadians() {
        return desiredAngleRadians;
    }

}
