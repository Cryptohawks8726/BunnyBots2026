package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.IntakeConstants.OdometryConstants;
import frc.robot.util.IntakeConstants.SwerveConstants;
import frc.robot.util.stateStuff.StateBase;

//Provide static classes representing robot-wide states here
public class State {
    /**
     * Variables which multiple states or subsystems need to access go here
     */
    public class StateVariables {
        // Put random variables here and make the codebase look crazy again
        public int carrotCount = 2;

    }

    public static class Intake extends StateBase {
        public RobotContainer robot;

        public Intake(RobotContainer newRobot) {
            robot = newRobot;

            // The name of this state will be treated as the robot's state's name in
            // dashboard outputs
            setName("Intake");
        }

        @Override
        public void onStateEnter() {
            // john johnson
            robot.swerve.setDefaultCommand(robot.swerveCommander.slowDriveFieldOriented);
            robot.intakeSubsystem.IntakeArmDown();
        }

        @Override
        public void periodic() {
            if (controller.getYButtonPressed()) {

            }

            if (controller.getXButtonPressed()) {
                robot.swerve.driveToPose8726(OdometryConstants.ovenScoringPos, SwerveConstants.maxSpeed);
            }

            if (controller.getXButtonPressed()) {
            }
        }

        @Override
        public void onStateExit() {
            // do funny stuff
            robot.intakeSubsystem.IntakeArmUp();
        }
    }

    public static class IdleOutake extends StateBase {
        public RobotContainer robot;

        public IdleOutake(RobotContainer container) {
            robot = container;

            setName("IdleOutake");
        }

        @Override
        public void onStateEnter() {
            robot.swerve.setDefaultCommand(robot.swerveCommander.driveFieldOriented);

        }

        @Override
        public void periodic() {
            if (controller.getYButtonPressed()) {

            }

            if (controller.getAButtonPressed()) {
            }

            if (controller.getBButtonPressed() && robot.swerve.getCurrentCommand() != null) {
                robot.swerve.getCurrentCommand().cancel();
                robot.swerve.setDefaultCommand(robot.swerveCommander.slowDriveFieldOriented); // Ermm this should cancel
                                                                                              // (in
                // theory)
            }
        }

        @Override
        public void onStateExit() {
        }
    }
}
