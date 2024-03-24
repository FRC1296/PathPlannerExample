// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FMJRobot;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intakeMotor;
    private double INTAKE_SPEED = -1.0;

    private DutyCycleOut dcOut = new DutyCycleOut(0);

    private BooleanPublisher haveNotePublisher;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.CAN_INTAKE);
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration().withAudio(Constants.ORCHESTRA_CONFIGS));

        FMJRobot.FMJOrchestra.addInstrument(intakeMotor);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);
        BooleanTopic statorTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_HAS_NOTE);
        haveNotePublisher = statorTopic.publish();
        haveNotePublisher.setDefault(false);
    }

    @Override
    public void periodic() {
        Double currVal = intakeMotor.getStatorCurrent().getValue();
        if ( currVal > 11.0) {
            haveNotePublisher.set(true);
        } else {
            haveNotePublisher.set(false);
        }
    }

    public void runIntake() {
        intakeMotor.setControl(dcOut.withOutput(INTAKE_SPEED));
    }

    public void stopIntake() {
        intakeMotor.setControl(dcOut.withOutput(0.0));
    }

    public void reverseIntake() {
        intakeMotor.setControl(dcOut.withOutput(-INTAKE_SPEED));
    }
}
