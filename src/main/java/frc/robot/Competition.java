// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Competition extends TimedRobot {
    private Command m_autonomousCommand;

    private FMJRobot robot;

    private final boolean UseLimelight = false;

    @Override
    public void robotInit() {
        boolean isCompetitionBot = true;

        // Get the Alliance Color, default to Blue if not present
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean isBlueAlliance = (Alliance.Blue == alliance);

        // TODO: send isCompetitionBot as parameter rather than default to false
        // TODO: test that alliance is being properly set
        robot = new FMJRobot(false, isBlueAlliance);

        robot.drivetrain.getDaqThread().setThreadPriority(99);

        // Start WPILib Data Log
        //DataLogManager.start(); // stores logs in either USB(diretory logs) or roborio drive(/home/lvuser/logs)
        //DriverStation.startDataLog(DataLogManager.getLog());

        // Start CTRE Data Log - logging will automatically start for FRC match
        //SignalLogger.setPath("/media/sda1/ctre-logs"); // we need to valid this location
        //SignalLogger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (UseLimelight) {
            var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

            Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

            if (lastResult.valid) {
                robot.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            }
        }

        robot.robotPeriodic();
    }

    @Override
    public void disabledInit() {
        StatusCode status = FMJRobot.FMJOrchestra.loadMusic("FlightOfTheBumblebee.chrp");
        //FMJRobot.FMJOrchestra.play();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        FMJRobot.FMJOrchestra.pause();
    }

    @Override
    public void autonomousInit() {
        robot.autonomousInit();
        m_autonomousCommand = robot.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        robot.autonomousPeriodic();
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        robot.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        robot.teleopPeriodic();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
