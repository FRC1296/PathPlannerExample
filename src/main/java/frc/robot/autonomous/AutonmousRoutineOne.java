// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.FMJRobot;
import frc.robot.drivesystem.CommandSwerveDrivetrain;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.FeederSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.wrist.WristSubsystem;

/** Add your docs here. */
public class AutonmousRoutineOne extends AutonomousRoutine {

    public AutonmousRoutineOne(FMJRobot robot, double velocity, double acceleration, boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDriveSubsystem();
        ShooterSubsystem shooter = robot.getShooterSubsystem();
        FeederSubsystem feeder = robot.getFeederSubsystem();
        IntakeSubsystem intake = robot.getIntakeSubsystem();
        WristSubsystem wrist = robot.getWristSubsystem();

        PathPlannerPath firstPath = PathPlannerPath.fromPathFile("Auton 1 Blue");
        this.initialPose = firstPath.getPreviewStartingHolonomicPose();
        PathPlannerPath secondPath = PathPlannerPath.fromPathFile("Auton 1 Blue Pt2");

        addCommands(
            Commands.runOnce(shooter::runShooter, shooter),
            Commands.waitSeconds(1.0),
            Commands.runOnce(feeder::runFeeder, feeder),
            Commands.waitSeconds(0.6),
            Commands.runOnce(feeder::stopFeeder, feeder),
            new ParallelCommandGroup(
                new IntakeCommand(intake, feeder, wrist),
                drivetrain.getAutoPath(firstPath)
            ),
            Commands.waitSeconds(0.6),
            Commands.runOnce(feeder::runFeeder, feeder),
            Commands.waitSeconds(0.6),
            Commands.runOnce(shooter::stopShooter, shooter),
            Commands.runOnce(feeder::stopFeeder, feeder),
            drivetrain.getAutoPath(secondPath),
            Commands.runOnce(drivetrain::seedFieldRelative, drivetrain)
        );
    }
    
}
