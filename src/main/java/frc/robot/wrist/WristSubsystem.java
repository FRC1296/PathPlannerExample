// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// WRIST RUNS POSTITIONAL CONTROL

package frc.robot.wrist;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FMJRobot;

public class WristSubsystem extends SubsystemBase {
    /** Creates a new WristSubsystem. */
    private TalonFX wristMotor;

    private double WRISTUP_SPEED = -0.2;  
    private double WRISTDOWN_SPEED = 0.10;
    private double STATOR_DOWN = 6;
    private double STATOR_CONTINOUS_DOWN = 8;

    private double AMP_POSITION = -21.0;

    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private MotionMagicDutyCycle mmOut = new MotionMagicDutyCycle(0);

    CurrentLimitsConfigs statorLimitConfigUp;
    CurrentLimitsConfigs statorLimitConfigDown;
    CurrentLimitsConfigs statorLimitConfigHold;
    private DutyCycleEncoder shaftEncoder;
    private double MIN_ABSOLUTE_POSITION = 0;
    private double MAX_ABSOLUTE_POSITION = 100;

    public WristSubsystem() {
        /*
         * Documentation from CTRE
            In a Position closed loop, the gains should be configured as follows:

            kS - unused, as there is no target velocity
            kV - unused, as there is no target velocity
            kA - unused, as there is no target acceleration
            kP - output per unit of error in position (output/rotation)
            kI - output per unit of integrated error in position (output/(rotation*s))
            kD - output per unit of error derivative in position (output/rps)
         */
        wristMotor = new TalonFX(Constants.CAN_WRIST);

        MotorOutputConfigs currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        currentConfigs.NeutralMode = NeutralModeValue.Brake;

        /* HardwareLimitSwitchConfigs hwLimitConfig = new HardwareLimitSwitchConfigs();
        hwLimitConfig.ReverseLimitAutosetPositionEnable = true;
        hwLimitConfig.ReverseLimitAutosetPositionValue = 0.0;
        hwLimitConfig.ForwardLimitAutosetPositionEnable = true;
        hwLimitConfig.ForwardLimitAutosetPositionValue = 0.0; */
        SoftwareLimitSwitchConfigs swLimitConfig = new SoftwareLimitSwitchConfigs();
        swLimitConfig.ForwardSoftLimitEnable = true;
        swLimitConfig.ForwardSoftLimitThreshold = 0;
        swLimitConfig.ReverseSoftLimitEnable = true;
        swLimitConfig.ReverseSoftLimitThreshold = -21;
        statorLimitConfigUp = new CurrentLimitsConfigs();
        statorLimitConfigUp.StatorCurrentLimitEnable = true;
        statorLimitConfigUp.StatorCurrentLimit = 12;  
        statorLimitConfigDown = new CurrentLimitsConfigs();
        statorLimitConfigDown.StatorCurrentLimitEnable = true;
        statorLimitConfigDown.StatorCurrentLimit = STATOR_DOWN;
        statorLimitConfigHold = new CurrentLimitsConfigs();
        statorLimitConfigHold.StatorCurrentLimitEnable = true;
        statorLimitConfigHold.StatorCurrentLimit = STATOR_CONTINOUS_DOWN;

        
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.16;
        slot0Configs.kS = 0.1; //TODO: why are we setting this
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicAcceleration = 50.0;
        mmConfigs.MotionMagicCruiseVelocity = 100;

        TalonFXConfiguration talonConfig = new TalonFXConfiguration()
            .withMotorOutput(currentConfigs)
            //.withHardwareLimitSwitch(hwLimitConfig)
            .withSlot0(slot0Configs)
            .withMotionMagic(mmConfigs)
            .withSoftwareLimitSwitch(swLimitConfig)
            .withCurrentLimits(statorLimitConfigUp)
            .withAudio(Constants.ORCHESTRA_CONFIGS);

        wristMotor.getConfigurator().apply(talonConfig);
        wristMotor.setPosition(0.0);

        mmOut.Slot = 0;

        FMJRobot.FMJOrchestra.addInstrument(wristMotor);

        configureShaftEncoder();
    }

    private void configureShaftEncoder() {
        shaftEncoder = new DutyCycleEncoder(Constants.DP_WRIST_ENCODER_1);
        if (shaftEncoder.isConnected()) {
            shaftEncoder.reset();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        shaftEncoder.initSendable(builder);
    }

    @Override
    public void periodic() {
        if (shaftEncoder.isConnected()) {
            double position = shaftEncoder.getAbsolutePosition();
            if (position <= MIN_ABSOLUTE_POSITION || position >= MAX_ABSOLUTE_POSITION) {
                this.stopWrist();
            }
        } else {
            //System.out.println("Shaft Encoder  is not plugged into the Roborio!!!!");
        }
    }

    public void wristUp() {
        wristMotor.getConfigurator().apply(statorLimitConfigUp);
        wristMotor.setControl(dcOut.withOutput(WRISTUP_SPEED));   
    }

    public void stopWrist() {
        wristMotor.setControl(dcOut.withOutput(0.0));
    }

    public void wristDown() {
        wristMotor.getConfigurator().apply(statorLimitConfigDown);

        wristMotor.setControl(dcOut.withOutput(WRISTDOWN_SPEED));   
    }

    public void holdWristDownForIntake() {
        wristMotor.getConfigurator().apply(statorLimitConfigHold);

        wristMotor.setControl(dcOut.withOutput(0.20));   
    }

    public void moveToAmpHeight() {
        wristMotor.getConfigurator().apply(statorLimitConfigUp);
        wristMotor.setControl(mmOut.withPosition(AMP_POSITION));
    }

    public void moveToZero() {
        wristMotor.getConfigurator().apply(statorLimitConfigDown);
        // Moving to Positon 2, this will cause wrist to move past the zero point 
        // thus insuring that we trip the limit switch
        wristMotor.setControl(mmOut.withPosition(0));
    }

    public double getPos() {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public void moveToPosNeg5() {
        wristMotor.getConfigurator().apply(statorLimitConfigUp);
        wristMotor.setControl(mmOut.withPosition(-5));
    }

    public void resetPosition() {
        wristMotor.setPosition(0.0);
    }
}
