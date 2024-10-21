package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VendorWrappers.Neo;

public class Arm extends SubsystemBase {
    
    private Neo leftPivot;
    private Neo rightPivot;

    public Arm() {
        leftPivot = new Neo(99);
        rightPivot = new Neo(99);
    }


    @Override
    public void periodic() {
        Logger.recordOutput("");
    }

}
