package frc.robot.subsystems.drivetrain;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveModuleConstants;


/*   ids from my summer noodle code
 Pigeon2 pigeon = new Pigeon2(device id 0, "rio");
    Swerve frontR = new Swerve(driveID 3, steerID 4,cancoder id 2);
    Swerve backR = new Swerve(7,8,0);
    Swerve frontL = new Swerve(1,2,1);
    Swerve backL = new Swerve(5,6,3); */

public class SwerveModule {
    PIDController pid;
    public int moduleIndex;
    private SwerveModuleIOInputsAutoLogged inputs;
    private SwerveModuleIO io;
    private static PIDController drivePID;
    private static PIDController anglePID;
    private SimpleMotorFeedforward driveFeedforward;

    public SwerveModule(SwerveModuleIO io, int moduleIndex) {
        this.moduleIndex = moduleIndex;

      inputs = new SwerveModuleIOInputsAutoLogged();
        
        drivePID = new PIDController(
            SwerveModuleConstants.drivekPVoltsPerMeterPerSecond, 
            SwerveModuleConstants.drivekIVoltsPerMeter, 
            SwerveModuleConstants.drivekDVoltsPerMeterPerSecondSquared);
        anglePID = new PIDController(
            SwerveModuleConstants.anglekPVoltsPerDegree,
            SwerveModuleConstants.anglekIVoltsPerDegreeSeconds,
            SwerveModuleConstants.anglekDVoltsPerDegreePerSecond);

        driveFeedforward = new SimpleMotorFeedforward(
            SwerveModuleConstants.drivekSVolts, 
            SwerveModuleConstants.drivekVVoltsSecondsPerMeter, 
            SwerveModuleConstants.drivekAVoltsSecondsSquaredPerMeter);

        anglePID.enableContinuousInput(-180, 180);
    }
    public void periodic() {
        Logger.processInputs("module" + Integer.toString(moduleIndex), inputs);
    }
    private SwerveModuleState constrainState(SwerveModuleState toConstrain) {
        double originalDegrees = toConstrain.angle.getDegrees();
        double constrainedDegrees = MathUtil.inputModulus(originalDegrees, -180, 180);
        return new SwerveModuleState(toConstrain.speedMetersPerSecond, Rotation2d.fromDegrees(constrainedDegrees));
    }
    public void setDesiredState(SwerveModuleState desiredState, boolean closedLoop) {
        // input an angle in the range 0f {-180, 180}, so it can be compaired to the angle
        // from the CanCoder, constrainState() needs to be called both before 
        // and after SwerveModuleState.optimize().
        desiredState = constrainState(desiredState);
        desiredState = SwerveModuleState.optimize(
            desiredState,
            Rotation2d.fromDegrees(inputs.angleAbsolutePositionDegrees)
        
        );

        setDesiredStateNoOptimize(desiredState, closedLoop);
    }
    public void setDesiredStateNoOptimize(SwerveModuleState desiredState, boolean closedLoop) {
        desiredState = constrainState(desiredState); // constrain one more time after optimization

        if (closedLoop) {
            // Conversion factor is already set below to convert rpm of motor to m/s of wheel.
            double wheelMetersPerSecond = inputs.driveVelocityMetersPerSecond;
            double feedforward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            double pidCorrection = drivePID.calculate(wheelMetersPerSecond);
            double outputVolts = MathUtil.clamp(feedforward + pidCorrection, -12, 12);
            
            io.setDriveVoltage(outputVolts);
        }
        else {
            io.setDriveVoltage(driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }
        double targetWheelAngleDegrees = desiredState.angle.getDegrees();
        double currentEncoderAngleDegrees = inputs.angleAbsolutePositionDegrees;

        io.setAngleVoltage(anglePID.calculate(currentEncoderAngleDegrees, targetWheelAngleDegrees));

        
    }
    public SwerveModuleState getState() {
    
    return new SwerveModuleState(
        inputs.driveVelocityMetersPerSecond,
        Rotation2d.fromDegrees(
            //  use the absolut encoder
            inputs.angleAbsolutePositionDegrees)
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionMeters,
            // use the absolute encoder.
            Rotation2d.fromDegrees(inputs.angleAbsolutePositionDegrees)
        );
    }
}