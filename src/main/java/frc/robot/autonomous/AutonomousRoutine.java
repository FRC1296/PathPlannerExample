// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FMJRobot;
import frc.robot.drivesystem.CommandSwerveDrivetrain;

/**
 * This is a generic class for an autonomous routine used during Autonomous Period of match
 */
public class AutonomousRoutine extends SequentialCommandGroup implements IAuto {

    protected FMJRobot theRobot;
    protected double maxVelocity;
    protected double maxAcceleration;
    protected boolean redPath;
    protected Pose2d initialPose;

    /** Contructor */
    public AutonomousRoutine(FMJRobot robot, double velocity, double acceleration, boolean isRedAlliance) {
        this.theRobot = robot;
        this.maxVelocity = velocity;
        this.maxAcceleration = acceleration;
        this.redPath = isRedAlliance;
    }

    /**
     * Constructor with multiple paths to drive
     * @param robot
     * @param velocity
     * @param acceleration
     * @param driveCommands
     */
    public AutonomousRoutine(FMJRobot robot, double velocity, double acceleration, Command... driveCommands) {
        this.theRobot = robot;
        this.maxVelocity = velocity;
        this.maxAcceleration = acceleration;

        CommandSwerveDrivetrain drivetrain = robot.getDriveSubsystem();
        this.addRequirements(drivetrain);
        
        if (driveCommands != null && driveCommands.length > 0) {
            for(int i=0; i < driveCommands.length; i++) {
                addCommands(driveCommands[i]);
            }
        }
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(Commands.print("Auton Complete."));
    }

    /**
     * This method will return the initial pose for our robot. It will most likely come from the
     * trajectory information.
     * 
     * @return Pose2d representation of initial position of our robot
     */
    public Pose2d getInitialPose() {
        return initialPose;
    }
}
