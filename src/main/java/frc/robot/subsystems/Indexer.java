package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.IndexerConstants;
import frc.robot.util.stateStuff.StatefulSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

public class Indexer extends StatefulSubsystem {
    // Motors
    private final SparkMax shooterMotor;
    private final SparkMax indexerMotorOne;
    private final SparkMax indexerMotorTwo;
    private final SparkMax indexerMotorThree;

    // Lidars
    private final LaserCan lidarOne;
    private final LaserCan lidarTwo;

    // bang bang
    private final BangBangController bangbang = new BangBangController();

    // Encoders
    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder indexerOneEncoder;
    private final RelativeEncoder indexerTwoEncoder;
    private final RelativeEncoder indexerThreeEncoder;

    // Variables
    private int counter = 0;
    private boolean wasPieceLidar;
    private double rollerVoltage = IndexerConstants.rollerVoltage;
    private double tempIncr;

    public Indexer() {
        super("Indexer");
        shooterMotor = new SparkMax(IndexerConstants.shooterMotorID, MotorType.kBrushless);
        indexerMotorOne = new SparkMax(IndexerConstants.indexerMotorOneID, MotorType.kBrushless);
        indexerMotorTwo = new SparkMax(IndexerConstants.indexerMotorTwoID, MotorType.kBrushless);
        indexerMotorThree = new SparkMax(IndexerConstants.indexerMotorThreeID, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        bangbang.setSetpoint(0.0);

        motorConfig.inverted(false);
        motorConfig.smartCurrentLimit(0); // change this later
        motorConfig.idleMode(IdleMode.kBrake);
        shooterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // change

        lidarOne = new LaserCan(IndexerConstants.lidarOneID);
        lidarTwo = new LaserCan(IndexerConstants.lidarTwoID);

        shooterEncoder = shooterMotor.getEncoder();
        indexerOneEncoder = indexerMotorOne.getEncoder();
        indexerTwoEncoder = indexerMotorTwo.getEncoder();
        indexerThreeEncoder = indexerMotorThree.getEncoder();

        putOnDashboard();
    }

    public void shooter() {
        runNextCommand(new FunctionalCommand(() -> {
            // as soon as command gets called
            wasPieceLidar = true;

        }, () -> {
            // runs periodically (every 0.2 seconds)
            double voltageShooterOutput = bangbang.calculate(shooterEncoder.getVelocity(),
                    IndexerConstants.rollerTargetRPM);
            double voltageOneOutput = bangbang.calculate(indexerOneEncoder.getVelocity(),
                    IndexerConstants.rollerTargetRPM);
            double voltageTwoOutput = bangbang.calculate(indexerTwoEncoder.getVelocity(),
                    IndexerConstants.rollerTargetRPM);
            double voltageThreeOutput = bangbang.calculate(indexerThreeEncoder.getVelocity(),
                    IndexerConstants.rollerTargetRPM);

            shooterMotor.setVoltage(voltageShooterOutput * rollerVoltage);
            indexerMotorOne.setVoltage(voltageOneOutput * rollerVoltage);
            indexerMotorTwo.setVoltage(voltageTwoOutput * rollerVoltage);
            indexerMotorThree.setVoltage(voltageThreeOutput * rollerVoltage);

            // decrementing carrot counter for shooter
            if (!lidarTwoDetects() && wasPieceLidar == true) {
                counter -= 1;
                wasPieceLidar = false;
            } else if (lidarTwoDetects()) {
                wasPieceLidar = true;
            }

        }, resetMotors -> {
            // runs when the command is finished
            shooterMotor.setVoltage(0);
            indexerMotorOne.setVoltage(0);
            indexerMotorTwo.setVoltage(0);
            indexerMotorThree.setVoltage(0);
        }, () -> counter == 0 // provides when the command stops
        ), false);
    }

    public void indexerOn() {

        runNextCommand(new FunctionalCommand(() -> {
            tempIncr = 0;
        }, () -> {

            double voltageOneOutput = bangbang.calculate(indexerOneEncoder.getVelocity(),
                    IndexerConstants.rollerTargetRPM);
            double voltageTwoOutput = bangbang.calculate(indexerTwoEncoder.getVelocity(),
                    IndexerConstants.rollerTargetRPM);
            double voltageThreeOutput = bangbang.calculate(indexerThreeEncoder.getVelocity(),
                    IndexerConstants.rollerTargetRPM);

            // increments?
            if (lidarOneDetects()) {
                tempIncr += 1;
            } else if (tempIncr > 0 && counter < 1 && lidarTwoDetects()) {
                counter += 1;
                tempIncr = 0;
            } else if (lidarOneDetects() && counter == 1) {
                counter += 1;
            } else if (lidarOneDetects() && counter == 2) {
                counter += 1;
            }

            // motor logic
            if (counter < 1) {
                indexerMotorOne.setVoltage(voltageOneOutput * rollerVoltage);
                indexerMotorTwo.setVoltage(voltageTwoOutput * rollerVoltage);
                indexerMotorThree.setVoltage(voltageThreeOutput * rollerVoltage);
            } else if (counter == 1) {
                indexerMotorOne.setVoltage(voltageOneOutput * rollerVoltage);
                indexerMotorTwo.setVoltage(voltageTwoOutput * rollerVoltage);
                indexerMotorThree.setVoltage(0);
            } else if (counter == 2) {
                indexerMotorOne.setVoltage(voltageOneOutput * rollerVoltage);
                indexerMotorTwo.setVoltage(0);
                indexerMotorThree.setVoltage(0);
            } else if (counter == 3) {
                indexerMotorOne.setVoltage(0);
                indexerMotorTwo.setVoltage(0);
                indexerMotorThree.setVoltage(0);
            }

        }, resetMotors -> {
            indexerMotorOne.setVoltage(0);
            indexerMotorTwo.setVoltage(0);
            indexerMotorThree.setVoltage(0);
        },
                () -> counter == 3), false);
    }

    /*
     * Returns true if lidar detects carrot
     */
    public boolean lidarOneDetects() {
        LaserCan.Measurement lidarMeasurement = lidarOne.getMeasurement();
        if(lidarMeasurement == null) return false;
        if (lidarMeasurement.distance_mm <= IndexerConstants.lidarDetection) {
            return true;
        }
        return false;
    }

    /*
     * Returns true if lidar detects carrot
     */
    public boolean lidarTwoDetects() {
        LaserCan.Measurement lidarMeasurement = lidarTwo.getMeasurement();
        if(lidarMeasurement == null) return false;
        if (lidarMeasurement.distance_mm <= IndexerConstants.lidarDetection) {
            return true;
        }
        return false;
    }

    public int getLocalCounter()
    {
        return counter;
    }

    public void setLocalCounter(long newCounter)
    {
        counter = (int)newCounter;
    }

    // ismail will have his funsies later
    /*
     * public Runnable resetMotors() {
     * // runs when the command is finished
     * return () -> {
     * shooterMotor.setVoltage(0);
     * indexerMotorOne.setVoltage(0);
     * indexerMotorTwo.setVoltage(0);
     * indexerMotorThree.setVoltage(0);
     * }
     * }
     */
    
     @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        SmartDashboard.putData("Subsystems/" + getName() +
        "/MutableValues/RollerBangBang", bangbang);

        builder.addBooleanProperty("Lidar 1 Detects", this::lidarOneDetects, null);
        builder.addBooleanProperty("Lidar 2 Detects", this::lidarTwoDetects, null);

        builder.addIntegerProperty("Local Counter", this::getLocalCounter, (num)->setLocalCounter(num));
    }
}