// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.shooter.FeederSubsystem;
import frc.robot.wrist.WristSubsystem;

public class IntakeCommand extends Command {
    private BooleanSubscriber intakeBeambreak;
    private IntakeSubsystem intakeSS;
    private FeederSubsystem feederSS;
    private WristSubsystem wristSS;

    private int isTimeout = -1;
    private int counter = 0;
    private int COUNT_TO = 0;

    /** Creates a new IntakeCommand. */
    public IntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, WristSubsystem wrist, int timeout) {
        isTimeout = timeout;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake, feeder, wrist);
        intakeSS = intake;
        feederSS = feeder;
        wristSS = wrist;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);

        BooleanTopic beamTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_BEAM_BROKEN);
        intakeBeambreak = beamTopic.subscribe(false);
    }
    
    /** Creates a new IntakeCommand. */
    public IntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, WristSubsystem wrist) {
        this(intake, feeder, wrist, -1);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intakeBeambreak.get() == false) {
            intakeSS.runIntake();
            feederSS.runIntake();
            // TODO: test keeping wrist down during intake
            wristSS.holdWristDownForIntake();
        }
        counter++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSS.stopIntake();
        feederSS.stopFeeder();
        wristSS.stopWrist();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean result = intakeBeambreak.get();
        if (isTimeout != -1 && result == false && counter >= isTimeout) {
            result = true;
        }
        return result;
    }
}
