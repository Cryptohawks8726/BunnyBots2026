package frc.robot.subsystems;

import frc.robot.util.Constants;
import frc.robot.util.stateStuff.StatefulSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.*;

public class Indexer extends StatefulSubsystem {

    // Motors
    private final SparkMax shooterIndexerMotor = new SparkMax(Constants.IndexerConstants.shooterIndexerMotorID,
            MotorType.kBrushless); // the device ID will be changed;
    private final SparkMax indexerMotorOne = new SparkMax(Constants.IndexerConstants.indexerMotorOneID,
            MotorType.kBrushless);
    private final SparkMax indexerMotorTwo = new SparkMax(Constants.IndexerConstants.indexerMotorTwoID,
            MotorType.kBrushless);
    private final SparkMax indexerMotorThree = new SparkMax(Constants.IndexerConstants.indexerMotorThreeID,
            MotorType.kBrushless); // the devicde ID will be chanegd;
    private int counter = 0;
    // Encoders
    private final RelativeEncoder shooterEncoder = shooterIndexerMotor.getEncoder();
    private final RelativeEncoder indexerEncoderOne = indexerMotorOne.getEncoder();
    private final RelativeEncoder indexerEncoderTwo = indexerMotorTwo.getEncoder();
    private final RelativeEncoder indexerEncoderThree = indexerMotorThree.getEncoder();

    // LIDARs
    private final LaserCan lidarOne;
    private final LaserCan lidarTwo = new LaserCan(Constants.IndexerConstants.lidarTwoID);

    private BangBangController bangBang = new BangBangController();

    private double rollerVoltage = Constants.IndexerConstants.rollerVoltage;

    public Indexer() { // the device ID will be changed
        super("Indexer Subsystem");
        lidarOne = new LaserCan(Constants.IndexerConstants.lidarOneID);
        putOnDashboard();
    }

    public void indexerOff() {
        runNextCommand(new InstantCommand(() -> {
            shooterIndexerMotor.setVoltage(0);
            indexerMotorOne.setVoltage(0);
            indexerMotorTwo.setVoltage(0);
            indexerMotorThree.setVoltage(0);
        }).withName("Indexer Stop"), false);

    }

    public void shoot() {
        runNextCommand(run(() -> {
            boolean onLidarTwo = true;
            double currentRPMSE = shooterEncoder.getVelocity();
            double currentRPMIM1 = indexerEncoderOne.getVelocity();
            double currentRPMIM2 = indexerEncoderTwo.getVelocity();
            double currentRPMIM3 = indexerEncoderThree.getVelocity();

            bangBang.setSetpoint(Constants.IndexerConstants.rollerTargetRPM);

            shooterIndexerMotor.setVoltage(bangBang.calculate(currentRPMSE) * rollerVoltage);
            indexerMotorOne.setVoltage(bangBang.calculate(currentRPMIM1) * rollerVoltage);
            indexerMotorTwo.setVoltage(bangBang.calculate(currentRPMIM2) * rollerVoltage);
            indexerMotorThree.setVoltage(bangBang.calculate(currentRPMIM3) * rollerVoltage);

            if (onLidarTwo == false) {
                counter -= 1;
            }
        }).until(() -> counter == 0).andThen(new WaitCommand(5)).andThen(new InstantCommand(() -> {
            indexerOff();
        })),
                false);
    }

    public void primeFirstCarrot() {
        runNextCommand(run(() -> {

        }).until(() -> lidarTwoSeesCarrot()).andThen(new InstantCommand(() -> {
            indexerOff();
        })), false);
    }

    public boolean lidarTwoSeesCarrot() {
        LaserCan.Measurement lidarTwoMeasurement = lidarTwo.getMeasurement();
        if (lidarTwoMeasurement != null && lidarTwoMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {

        }
    }

    public void indexerOn(double maxSpeed) {

    }
}