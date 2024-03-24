package frc.robot.led;

import java.util.Random;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
    private BooleanSubscriber intakeBeambreak;
    private BooleanSubscriber intakeHasNote;
    private AddressableLED ledStrip1;
    private AddressableLEDBuffer buffer1;

    private final int ledCount = 26; // 26 each side    

    Random rand = new Random();

    public LedSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);
        BooleanTopic beamTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_BEAM_BROKEN);
        intakeBeambreak = beamTopic.subscribe(false);
        BooleanTopic statorTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_HAS_NOTE);
        intakeHasNote = statorTopic.subscribe(false);

        ledStrip1 = new AddressableLED(Constants.PWM_LED1);
        buffer1 = new AddressableLEDBuffer(ledCount);
        ledStrip1.setLength(buffer1.getLength());

        // Set all leds to yellow for startup
        for (int i = 0; i < buffer1.getLength(); i++) {
            buffer1.setHSV(i, 30, 80, 90);
        }

        ledStrip1.setData(buffer1);
        ledStrip1.start();
    }

    public void noteStatus() {
        int red = 255;
        int green = 255;
        int blue = 255;

        if (intakeBeambreak.get() == true  || intakeHasNote.get() == true) {
            red = 0;
            green = 255;
            blue = 0;
        }
        for (int i = 0; i < ledCount; i++) {
            buffer1.setRGB(i, red, green, blue);
        }
        ledStrip1.setData(buffer1);
    }

    @Override
    public void periodic() {
        // If robot is disabled then display a flame
        if (DriverStation.isDisabled()) {
            // Orange Flame: r=226, g=121, b=35
            // Bottom 3 set always to be on
            int red = 226;
            int green = 121;
            int blue = 35;
            for (int i = 0; i < ledCount; i++) {
                int rValue = red;
                int gValue = green;
                int bValue = blue;
                if (i > 3) {
                    int flicker = rand.nextInt(55); // random integer between 0(inclusive) and 55(exclusive)
                    rValue = Math.max(red - flicker, 0);
                    gValue = Math.max(green - flicker, 0);
                    bValue = Math.max(blue - flicker, 0);
                }
                buffer1.setRGB(i, rValue, gValue, bValue);
            }
            ledStrip1.setData(buffer1);
            // If robot is not disabled then display status information
        } else {
            noteStatus();
        }

    }

    /**
     * This method is responsible for configuration of the SubSystem for test
     * operation
     */
    public void testInit() {

    }

    /**
     * This method is responsible for configuration of the SubSystem for
     * teleoperative operation
     */
    public void teleopInit() {

    }

    /**
     * This method is responsible for configuration of the SubSystem for autonomous
     * operation
     */
    public void autonomousInit() {}

    /**
     * This method is responsible for running all tests related to the subsystem
     */
    public void runSubSystemTest() {

    }

    /**
     * This method is called when robot is disabled
     */
    public void disable() {

    }
}
