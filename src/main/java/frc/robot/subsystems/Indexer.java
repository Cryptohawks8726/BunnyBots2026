package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IndexerConstants;
import frc.robot.util.stateStuff.StatefulSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

public class Indexer extends StatefulSubsystem {
    // Motors 
    private final SparkMax shooterIndexerMotor;
    private final SparkMax indexerMotorOne;
    private final SparkMax indexerMotorTwo;
    private final SparkMax indexerMotorThree;

    // Lidars
    private final LaserCan lidarOne = new LaserCan(IndexerConstants.lidarOneID);
    private final LaserCan lidarTwo = new LaserCan(IndexerConstants.lidarTwoID);
    private final BangBangController bangbang = new BangBangController();

    // Encoders
    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder motorOneEncoder;
    private final RelativeEncoder motorTwoEncoder;
    private final RelativeEncoder motorThreeEncoder;

    // Variables

    private int counter = 0;
    private boolean wasPieceLidar;
    public Indexer() {
        super("Indexer Subsystem");
        shooterIndexerMotor = new SparkMax(IndexerConstants.shooterIndexerMotorID, MotorType.kBrushless);
        indexerMotorOne = new SparkMax(IndexerConstants.indexerMotorOneID, MotorType.kBrushless);
        indexerMotorTwo = new SparkMax(IndexerConstants.indexerMotorTwoID, MotorType.kBrushless);
        indexerMotorThree = new SparkMax(IndexerConstants.indexerMotorThreeID, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        bangbang.setSetpoint(0.0);

        motorConfig.inverted(false);
        motorConfig.smartCurrentLimit(0); // change this later
        motorConfig.idleMode(IdleMode.kBrake);
        shooterIndexerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // change
        putOnDashboard();
    }

    public void shooter(){
        runNextCommand(new FunctionalCommand(()->{
            // as soon as command gets called
            wasPieceLidar = true;

        },()->{
            // runs periodically (every 0.2 seconds)
            double voltageShooterOutput = bangbang.calculate(shooterEncoder.getVelocity(), IndexerConstants.setpoint);
            double voltageOneOutput = bangbang.calculate(motorOneEncoder.getVelocity(), IndexerConstants.setpoint);
            double voltageTwoOutput = bangbang.calculate(motorTwoEncoder.getVelocity(), IndexerConstants.setpoint);
            double voltageThreeOutput = bangbang.calculate(motorThreeEncoder.getVelocity(), IndexerConstants.setpoint);
            
            shooterIndexerMotor.setVoltage(voltageShooterOutput * 12);
            indexerMotorOne.setVoltage(voltageOneOutput * 12);
            indexerMotorTwo.setVoltage(voltageTwoOutput * 12 );
            indexerMotorThree.setVoltage(voltageThreeOutput * 12);
            
            if (!lidarTwoDetects() && wasPieceLidar == true){
                counter-=1;
                wasPieceLidar = false;
            }
            else if (lidarTwoDetects()){
                wasPieceLidar = true;
            }


        },johnBoolean->{
            // runs when the command is finished
            shooterIndexerMotor.setVoltage(0);
            indexerMotorOne.setVoltage(0);
            indexerMotorTwo.setVoltage(0);
            indexerMotorThree.setVoltage(0);      
        },()->counter == 0 // provides when the command stops
        ), false);
    }


    public void indexerOn(double maxSpeed) {

    }
    /*
     * Returns true if lidar detects carrot
     */
    public boolean lidarOneDetects(){
        LaserCan.Measurement lidarMeasurement = lidarOne.getMeasurement();
        if (lidarMeasurement.distance_mm <= IndexerConstants.lidarDetection){
            return true;
        }
        return false;
    }
    public boolean lidarTwoDetects(){
        LaserCan.Measurement lidarMeasurement = lidarTwo.getMeasurement();
        if (lidarMeasurement.distance_mm <= IndexerConstants.lidarDetection){
            return true;
        }
        return false;
    }
}
