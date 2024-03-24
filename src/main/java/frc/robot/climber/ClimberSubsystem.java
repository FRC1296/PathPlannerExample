// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import org.opencv.core.Mat.Tuple2;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FMJRobot;

public class ClimberSubsystem extends SubsystemBase {
    /** Creates a new ClimberSubsystem. */
    private TalonFX climberLeft;
    private TalonFX climberRight;
    private double CLIMBER_DOWNSPEED = 0.3;
    private double CLIMBER_UPSPEED = -0.3;
    private double AMP_HEIGHT = -50.0;
    private double TRAP_HEIGHT = -230;
    private double ZERO_HEIGHT = 0;

    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private PositionDutyCycle pOut = new PositionDutyCycle(0);
    private MotionMagicDutyCycle mmOut = new MotionMagicDutyCycle(0);

    CurrentLimitsConfigs statorLimitConfig = new CurrentLimitsConfigs();

    public ClimberSubsystem() {
        climberRight = new TalonFX(Constants.CAN_CLIMBERRIGHT);
        MotorOutputConfigs rightMotorConfig = new MotorOutputConfigs();
        rightMotorConfig.NeutralMode = NeutralModeValue.Brake;
        // HardwareLimitSwitchConfigs lefthwLimitConfig = new HardwareLimitSwitchConfigs();
        // lefthwLimitConfig.ReverseLimitEnable = false;
        // lefthwLimitConfig.ReverseLimitAutosetPositionEnable = false;
        // lefthwLimitConfig.ReverseLimitAutosetPositionValue = 0.0;
        // lefthwLimitConfig.ForwardLimitEnable = true;
        // lefthwLimitConfig.ForwardLimitAutosetPositionEnable = true;
        // lefthwLimitConfig.ForwardLimitAutosetPositionValue = 0.0;
        // HardwareLimitSwitchConfigs righthwLimitSwitch = new HardwareLimitSwitchConfigs();
        // righthwLimitSwitch.ReverseLimitEnable = true;
        // righthwLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        // righthwLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
        // righthwLimitSwitch.ForwardLimitEnable = false;
        // righthwLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        // righthwLimitSwitch.ForwardLimitAutosetPositionValue = 0.0;
        statorLimitConfig.StatorCurrentLimitEnable = true;
        statorLimitConfig.StatorCurrentLimit = 14;
        SoftwareLimitSwitchConfigs swLimitConfig = new SoftwareLimitSwitchConfigs();
        swLimitConfig.ReverseSoftLimitEnable = true;
        swLimitConfig.ReverseSoftLimitThreshold = -245;
        swLimitConfig.ForwardSoftLimitEnable = true;
        swLimitConfig.ForwardSoftLimitThreshold = 0.0;
        
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.1;
        slot0Configs.kD = 0.01;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicAcceleration = 50.0;
        mmConfigs.MotionMagicCruiseVelocity = 100;

        climberLeft = new TalonFX(Constants.CAN_CLIMBERLEFT);
        MotorOutputConfigs leftMotorConfig = new MotorOutputConfigs();
        leftMotorConfig.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration rightConfig =
                new TalonFXConfiguration()
                    .withMotorOutput(rightMotorConfig)
                    .withCurrentLimits(statorLimitConfig)
                    .withAudio(Constants.ORCHESTRA_CONFIGS);
        climberRight.getConfigurator().apply(rightConfig);

        TalonFXConfiguration leftConfig =
                new TalonFXConfiguration()
                    .withMotorOutput(leftMotorConfig)
                    .withSlot0(slot0Configs)
                    .withMotionMagic(mmConfigs)
                    .withSoftwareLimitSwitch(swLimitConfig)
                    .withCurrentLimits(statorLimitConfig)
                    .withAudio(Constants.ORCHESTRA_CONFIGS);
        climberLeft.getConfigurator().apply(leftConfig);
        climberLeft.setPosition(0.0);

        climberRight.setControl(new Follower(climberLeft.getDeviceID(), true));
        pOut.Slot = 0;
        mmOut.Slot = 0;

        FMJRobot.FMJOrchestra.addInstrument(climberLeft);
        FMJRobot.FMJOrchestra.addInstrument(climberRight);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void climberUp() {
        climberLeft.setControl(dcOut.withOutput(CLIMBER_UPSPEED));
    }

    public void stopClimber() {
        climberLeft.setControl(dcOut.withOutput(0.0));
    }

    public void climberDown() {
        climberLeft.setControl(dcOut.withOutput(CLIMBER_DOWNSPEED));
    }

    public void moveToAmpHeight() {
        //climberLeft.setControl(pOut.withPosition(AMP_HEIGHT));
        climberLeft.setControl(mmOut.withPosition(AMP_HEIGHT));
    }

    public void moveToTrapHeight() {
        climberLeft.setControl(pOut.withPosition(TRAP_HEIGHT));
        //climberLeft.setControl(mmOut.withPosition(TRAP_HEIGHT));
    }

    public void moveToZero(){
        //climberLeft.setControl(pOut.withPosition(ZERO_HEIGHT));
        climberLeft.setControl(mmOut.withPosition(ZERO_HEIGHT));
    }

    public void setToClimbMode() {
        CurrentLimitsConfigs clc = new CurrentLimitsConfigs();
        clc.StatorCurrentLimitEnable = false;
        climberRight.getConfigurator().apply(clc);
        climberLeft.getConfigurator().apply(clc);
    }

    public void setToShootMode() {
        climberRight.getConfigurator().apply(statorLimitConfig);
        climberLeft.getConfigurator().apply(statorLimitConfig);
    }

    public void resetPosition() {
        climberLeft.setPosition(0.0);
    }
}

