package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VendorWrappers.Neo;

public class Intake extends SubsystemBase {
    private Neo bottomIntakeNeo;
    private Neo topIntakeNeo;
    private Neo indexerNeo;
    public Intake() {
        bottomIntakeNeo = new Neo(IntakeConstants.bottomIntakeNeoID);
        topIntakeNeo = new Neo(IntakeConstants.topIntakeNeoID);
        indexerNeo = new Neo(IntakeConstants.topIntakeNeoID);
    }
    public Command runIntakeCommand(double bottomIntakeVolts, double topIntakeVolts, double indexerIntakeVolts) {
        // we can use this for intake, stop intake, and eject
        return this.run(
            () -> {
                bottomIntakeNeo.setVoltage(bottomIntakeVolts);
                topIntakeNeo.setVoltage(topIntakeVolts);
                indexerNeo.setVoltage(indexerIntakeVolts);
            }
        );
    }
    @Override
    public void periodic() {
        Logger.recordOutput("bottomIntakeNeoVelocity,RPM?", bottomIntakeNeo.getVelocity());
        Logger.recordOutput("topIntakeNeoVelocity,RPM?", topIntakeNeo.getVelocity());
        Logger.recordOutput("indexerNeoVelocity,RPM?", indexerNeo.getVelocity());
    }
}
