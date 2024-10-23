package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VendorWrappers.Neo;

public class Intake extends SubsystemBase {
    private CANSparkMax bottomIntakeNeo;
    private CANSparkMax topIntakeNeo;
    private CANSparkMax indexerNeo;
    public Intake() {
        bottomIntakeNeo = new CANSparkMax(IntakeConstants.bottomIntakeNeoID, MotorType.kBrushless);
        topIntakeNeo = new CANSparkMax(IntakeConstants.topIntakeNeoID, MotorType.kBrushless);
        indexerNeo = new CANSparkMax(IntakeConstants.indexerNeoID, MotorType.kBrushless);
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
        // Logger.recordOutput("bottomIntakeNeoVelocity,RPM?", bottomIntakeNeo.getVelocity());
        // Logger.recordOutput("topIntakeNeoVelocity,RPM?", topIntakeNeo.getVelocity());
        // Logger.recordOutput("indexerNeoVelocity,RPM?", indexerNeo.getVelocity());
    }
}
