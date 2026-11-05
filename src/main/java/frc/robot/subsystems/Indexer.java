package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import au.grapplerobotics.LaserCan;

public class Indexer extends SubsystemBase {

    // Motors
    private final SparkMax shooterIndexerMotor = new SparkMax(Constants.IndexerConstants.shooterIndexerMotorID,
            MotorType.kBrushless); // the device ID will be changed;
    private final SparkMax indexerMotorOne = new SparkMax(Constants.IndexerConstants.indexerMotorOneID,
            MotorType.kBrushless); // the device ID will be changed;
    private final SparkMax indexerMotorTwo = new SparkMax(Constants.IndexerConstants.indexerMotorTwoID,
            MotorType.kBrushless);
    private final SparkMax indexerMotorThree = new SparkMax(Constants.IndexerConstants.indexerMotorThreeID,
            MotorType.kBrushless); // the devicde ID will be chanegd;

    // Encoders
    private final RelativeEncoder shooterEncoder = shooterIndexerMotor.getEncoder();
    private final RelativeEncoder indexerEncoderOne = indexerMotorOne.getEncoder();
    private final RelativeEncoder indexerEncoderTwo = indexerMotorTwo.getEncoder();
    private final RelativeEncoder indexerEncoderThree = indexerMotorThree.getEncoder();

    // LIDARs
    private final LaserCan lidarOne = new LaserCan(Constants.IndexerConstants.lidarOneID);
    private final LaserCan lidarTwo = new LaserCan(Constants.IndexerConstants.lidarTwoID);
    private final LaserCan lidarThree = new LaserCan(Constants.IndexerConstants.lidarThreeID);

    private BangBangController bangBang;

    private double rollerVoltage = Constants.IndexerConstants.rollerVoltage;

    public Indexer() { // the device ID will be changed
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        bangBang.setSetpoint(0.0);

        motorConfig.inverted(false);
        motorConfig.smartCurrentLimit(0); // change this later
        motorConfig.idleMode(IdleMode.kBrake);

        shooterIndexerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // change
        indexerMotorOne.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        indexerMotorTwo.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        indexerMotorThree.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    @Override
    public void periodic() {
        return;
    }

    public void indexerOff() {
        shooterIndexerMotor.setVoltage(0);
        indexerMotorOne.setVoltage(0);
        indexerMotorTwo.setVoltage(0);
        indexerMotorThree.setVoltage(0);
    }

    public void shoot() {
        double currentRPMSE = shooterEncoder.getVelocity();
        double currentRPMIM1 = indexerEncoderOne.getVelocity();
        double currentRPMIM2 = indexerEncoderTwo.getVelocity();
        double currentRPMIM3 = indexerEncoderThree.getVelocity();

        bangBang.setSetpoint(Constants.IndexerConstants.rollerTargetRPM);

        shooterIndexerMotor.setVoltage(bangBang.calculate(currentRPMSE) * rollerVoltage);
        indexerMotorOne.setVoltage(bangBang.calculate(currentRPMIM1) * rollerVoltage);
        indexerMotorTwo.setVoltage(bangBang.calculate(currentRPMIM2) * rollerVoltage);
        indexerMotorThree.setVoltage(bangBang.calculate(currentRPMIM3) * rollerVoltage);
    }

    public void indexerOn(double maxSpeed) {

    }

}

// extends subsystem base

// get encoders
// should use RPM for speed
// Make a new constant flyWheelVoltage, can put it in util/Constants later
//
// have wheels move in periodic -> if flywheel's target rpm is above 0 move
// wheels
//
// do LIDAR stuff later