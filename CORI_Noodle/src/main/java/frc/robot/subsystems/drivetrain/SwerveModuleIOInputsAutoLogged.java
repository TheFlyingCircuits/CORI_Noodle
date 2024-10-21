package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs implements LoggableInputs, Cloneable{
    @Override
    public void toLog(LogTable table) {
        table.put("DrivePositionMeters", drivePositionMeters);
        table.put("DriveVelocityMetersPerSecond", driveVelocityMetersPerSecond);
        table.put("AngleAbsolutePositionDegrees", angleAbsolutePositionDegrees);
        table.put("DriveAppliedVoltage", driveAppliedVoltage);
        table.put("DriveCurrent", driveCurrent);
    }

    @Override
    public void fromLog(LogTable table) {
        drivePositionMeters = table.get("DrivePositionMeters", drivePositionMeters);
        driveVelocityMetersPerSecond = table.get("DriveVelocityMetersPerSecond", driveVelocityMetersPerSecond);
        angleAbsolutePositionDegrees = table.get("AngleAbsolutePositionDegrees", angleAbsolutePositionDegrees);
        driveAppliedVoltage = table.get("DriveAppliedVoltage", driveAppliedVoltage);
        driveCurrent = table.get("DriveCurrent", driveCurrent);
    }

    public SwerveModuleIOInputsAutoLogged clone() {    
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.drivePositionMeters = this.drivePositionMeters;
    copy.driveVelocityMetersPerSecond = this.driveVelocityMetersPerSecond;
    copy.angleAbsolutePositionDegrees = this.angleAbsolutePositionDegrees;
    copy.driveAppliedVoltage = this.driveAppliedVoltage;
    copy.driveCurrent = this.driveCurrent;
    return copy;
    }
}
