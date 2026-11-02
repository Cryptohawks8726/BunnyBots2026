package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Indexer {
    private final SparkMax shooterIndexerMotor;
    private final SparkMax indexerMotorOne;
    private final SparkMax indexerMotorTwo;
    private final SparkMax indexerMotorThree;

    public Indexer() {
        shooterIndexerMotor = new SparkMax(0, MotorType.kBrushless); // the device ID will be changed
        indexerMotorOne = new SparkMax(1, MotorType.kBrushless); // the device ID will be changed
        indexerMotorTwo = new SparkMax(2, MotorType.kBrushless); // the device ID will be changed
        indexerMotorThree = new SparkMax(3, MotorType.kBrushless); // the devicde ID will be chanegd
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.inverted(false);
        motorConfig.smartCurrentLimit(0); // change this later
        motorConfig.idleMode(IdleMode.kBrake);

        shooterIndexerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // change

    }

    public void indexerOn(double maxSpeed) {

    }

}
