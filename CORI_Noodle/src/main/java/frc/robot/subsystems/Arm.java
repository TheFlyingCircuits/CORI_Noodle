package frc.robot.subsystems;

public class Arm extends SubsystemBase {
    
    private Neo leftPivot;
    private Neo rightPivot;

    public Arm() {
        leftPivot = new Neo(99);
        rightPivot = new Neo(99);
    }


    @Override
    public void periodic() {
        Logger.recordOutput()
    }

}
