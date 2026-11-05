package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Indexer {
    private final SparkMax shooterIndexerMotor = new SparkMax(0, MotorType.kBrushless); // this device ID will be
                                                                                        // changed
    private final SparkMax indexerMotorOne = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax indexerMotorTwo = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax indexerMotorThree = new SparkMax(3, MotorType.kBrushless);
    private final BangBangController bangbang = new BangBangController();

    public Indexer() {

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        bangbang.setSetpoint(0.0);

        motorConfig.inverted(false);
        motorConfig.smartCurrentLimit(0); // change this later
        motorConfig.idleMode(IdleMode.kBrake);
        shooterIndexerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // change

    }

    public void setVoltage() {
        bangbang.setSetpoint(0.0);
    }

    public void indexerOn(double maxSpeed) {

    }

}
