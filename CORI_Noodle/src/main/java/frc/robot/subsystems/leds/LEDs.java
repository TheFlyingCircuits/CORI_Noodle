package frc.robot.subsystems.leds;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class LEDs extends SubsystemBase {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private static boolean putSwitcher = true;

    private Timer ledTimer= new Timer();

    int rainbowNumber;
    int[] hues;

    public LEDs() {
        leds = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(60); // I don't know the length yet

        leds.setLength(buffer.getLength());
        
        leds.setData(buffer);
        leds.start();

        int rainbowNumber = 0;
        int[] hues = new int[buffer.getLength()];

        ledTimer.restart();
        for (int i = 0; i < buffer.getLength(); i++) {
            int color = i * 3;
            hues[i] = color;
        }
    }
    public void resetTimer() {
        ledTimer.reset();;
    }
    public void setHSVFunction(int hue, int s, int v) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, hue, s, v);
        }
        leds.setData(buffer);
    }
    public void lowHighHSV() {
        // reset timer
        // 117 
        // 87
        double ledNumber = ledTimer.get() * 30;
        if (ledTimer.get() < 1 ) {
            setHSVFunction(117 - (int)ledNumber, 255, 255);
        } else if ((ledTimer.get() >= 1) && (ledTimer.get() < 2) ) {
            setHSVFunction(87 + ((int)ledNumber - 30), 255, 255);
        } else {
            resetTimer();
        }
    }
    public void zoomingHSV() {
        if (ledTimer.get() * 40 > 59) {
            resetTimer();
        }
        double ledSpot = ledTimer.get() * 40;
        for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setHSV(i, 0,10, 10);
        }
        buffer.setHSV((int)ledSpot, 50, 204, 232);
        leds.setData(buffer);
    }
    public void rainbowHSV() {
        for (int i = 0; i < buffer.getLength(); i++) {
           buffer.setHSV(i, hues[rainbowNumber], 255, 255);
           rainbowNumber = rainbowNumber + 1;
        if (rainbowNumber >= 60) {
            rainbowNumber -= 60;
          }
        }
        rainbowNumber = rainbowNumber + 1;
        if (rainbowNumber >= 60) {
            rainbowNumber -= 60;
        }
        leds.setData(buffer);
      }
    public Command playIntakeAnimationCommand(Supplier<Boolean> intakeCamSeesNote) {
        return this.run(
            () -> {
                if (intakeCamSeesNote.get().booleanValue()) {
                    rainbowHSV();
                } else {
                    zoomingHSV();
                }
            }
        );
    }
}