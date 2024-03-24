// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// SHOOTER RUNS VELOCITY CONTROL

package frc.robot.shooter;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */
    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private VelocityDutyCycle velocityDC = new VelocityDutyCycle(0, 0, true, 0, 0, false, false, false);
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private VoltageOut sysidOut = new VoltageOut(0);

    private TalonFX upperShooter;
    private TalonFX lowerShooter;

    private final double SHOOT_SPEED = 30.0; // rotations / sec
    private final double AMP_SPEED = 15;
    private final double SHOOT_SPEED_PCT = 0.9; // percentage power
  
    // Creates a SysIdRoutine
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            (state)->SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> upperShooter.setControl(sysidOut.withOutput(volts.in(Volts))), 
            null, 
            this, 
            "shooter"
        )
    );

    public ShooterSubsystem() {
        /*
         * Documentation from CTRE
            In a Velocity closed loop, the gains should be configured as follows:
        
            kS - output to overcome static friction (output)
            kV - output per unit of requested velocity (output/rps)
            kA - unused, as there is no target acceleration
            kP - output per unit of error in velocity (output/rps)
            kI - output per unit of integrated error in velocity (output/rotation)
            kD - output per unit of error derivative in velocity (output/(rps/s))
        
            Example:
            slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
            slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
            slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
            slot0Configs.kI = 0; // no output for integrated error
            slot0Configs.kD = 0; // no output for error derivative
         */
        /* sysid values
         * Feedforward Analysis:
         *  kS = 0.17612
         *  kV = 0.12121
         *  kA = 0.0082089
         *  
         * Velocity:
         *  kP = 0.17982
         *  kD = 0.0
         */
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.17578125;
        slot0Configs.kV = 0.032;
        slot0Configs.kP = 0.15;
        slot0Configs.kA = 0.017418;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicAcceleration = 180.0;
        mmConfigs.MotionMagicCruiseVelocity = 90.0;

        ClosedLoopRampsConfigs clrConfigs = new ClosedLoopRampsConfigs();
        clrConfigs.DutyCycleClosedLoopRampPeriod = 0.5;

        TalonFXConfiguration talonConfig = new TalonFXConfiguration()
            .withSlot0(slot0Configs)
            .withMotionMagic(mmConfigs)
            .withClosedLoopRamps(clrConfigs);

        upperShooter = new TalonFX(Constants.CAN_UPPERSHOOTER);
        upperShooter.getConfigurator().apply(talonConfig);

        lowerShooter = new TalonFX(Constants.CAN_LOWERSHOOTER);
        lowerShooter.getConfigurator().apply(talonConfig);

        // Added master-slave relationship so the lowershooter will always follow uppershooter.
        //lowerShooter.setControl(new Follower(upperShooter.getDeviceID(), false).withUpdateFreqHz(500));

        velocityDC.Slot = 0;
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void runShooter() {
        //upperShooter.setControl(dcOut.withOutput(SHOOT_SPEED_PCT));
        upperShooter.setControl(velocityDC.withVelocity(SHOOT_SPEED));
        lowerShooter.setControl(velocityDC.withVelocity(SHOOT_SPEED));
    }

    public void stopShooter() {
        upperShooter.setControl(dcOut.withOutput(0.0));
        lowerShooter.setControl(dcOut.withOutput(0.0));
    }

    public void setAmpSpeed() {
        upperShooter.setControl(velocityDC.withVelocity(AMP_SPEED));
        lowerShooter.setControl(velocityDC.withVelocity(AMP_SPEED));
    }

    public void runAutonShoot() {
        upperShooter.setControl(velocityDC.withVelocity(SHOOT_SPEED));
        lowerShooter.setControl(velocityDC.withVelocity(SHOOT_SPEED));
    }

}
