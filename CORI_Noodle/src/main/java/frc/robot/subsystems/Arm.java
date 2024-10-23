package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FlyingCircuitUtils;
import frc.robot.Constants.ArmConstants;
import frc.robot.FlyingCircuitUtils.ArmPDController;
import frc.robot.VendorWrappers.Neo;

public class Arm extends SubsystemBase {
    
    private Neo leftPivot;
    private Neo rightPivot;

    private CANcoder pivotEncoder;

    double pivotAngleDegrees;
    double pivotVelocityDegreesPerSecond;
    
    double prevTargetAngleDegrees;
    double targetAngleDegrees;

    Timer setpointVelocityTimer;
    double setpointVelocityDegreesPerSecond;

    TrapezoidProfile profile;
    TrapezoidProfile.State initState;
    Timer trapezoidProfileTimer;

    ArmFeedforward armFeedforward;
    ArmPDController armPD;

    public Arm() {
        leftPivot = new Neo(12);
        rightPivot = new Neo(11);

        configMotors();

        pivotEncoder = new CANcoder(4);

        configCANcoder();

        armFeedforward = new ArmFeedforward(
            ArmConstants.kSArmVolts,
            ArmConstants.kGArmVolts,
            ArmConstants.kVArmVoltsSecondsPerRadian,
            ArmConstants.kAArmVoltsSecondsSquaredPerRadian
        );

        armPD = new ArmPDController(ArmConstants.kPArmVoltsPerDegree, ArmConstants.kDArmVoltsSecondsPerDegree);

        setpointVelocityTimer = new Timer();


        profile = new TrapezoidProfile(ArmConstants.constraints);
        trapezoidProfileTimer = new Timer();
    }

    private void configMotors() {
        int ampLimit = 30;
        rightPivot.restoreFactoryDefaults();
        rightPivot.setSmartCurrentLimit(ampLimit);
        rightPivot.setInverted(false);
        rightPivot.setIdleMode(IdleMode.kCoast);
        rightPivot.burnFlash();

        leftPivot.restoreFactoryDefaults();
        leftPivot.setSmartCurrentLimit(ampLimit);
        leftPivot.setInverted(true);
        leftPivot.setIdleMode(IdleMode.kCoast);
        leftPivot.burnFlash();
    }

    private void configCANcoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCoderConfig.MagnetSensor.MagnetOffset = ArmConstants.canCoderOffset;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        pivotEncoder.getConfigurator().apply(canCoderConfig);
    
    
    }

    private void setArmMotorVolts(double volts) {
        leftPivot.setVoltage(volts);
        rightPivot.setVoltage(volts);
    }
    
    private boolean atLowerLimit() {
        return pivotAngleDegrees <= ArmConstants.armMinAngleDegrees;
    }

    private boolean atUpperLimit() {
        return pivotAngleDegrees >= ArmConstants.armMaxAngleDegrees;
    }


    //This method is in here because future commands which want to move the arm will be more easily written.
    //Rather than having to motion profile within each command, it can be just one function call.
    /**
     * Sets the desired position for the arm's motion profile to follow.
     * @param targetDegrees - Target angle in degrees for the arm.
     */
    public void setDesiredDegrees(double targetAngleDegrees) {
        // Save the last target before updating for this iteration
        prevTargetAngleDegrees = this.targetAngleDegrees;
        this.targetAngleDegrees = MathUtil.clamp(targetAngleDegrees, ArmConstants.armMinAngleDegrees, ArmConstants.armMaxAngleDegrees);
        

        //calculate setpoint velocity
        double timeSinceLastSetpoint = setpointVelocityTimer.get();
        if (timeSinceLastSetpoint > 0) {
            this.setpointVelocityDegreesPerSecond = (this.targetAngleDegrees - prevTargetAngleDegrees) / timeSinceLastSetpoint;
        }
        setpointVelocityTimer.restart();
        

        //If we need a big movement, generate a trapezoidal profile
        if (Math.abs(this.targetAngleDegrees - prevTargetAngleDegrees) >= 2) {
            initState = new TrapezoidProfile.State(pivotAngleDegrees, pivotVelocityDegreesPerSecond);
            trapezoidProfileTimer.restart();
            System.out.println("AAAAAAA");

            profile.calculate( //calculate to start trapezoid profile
                trapezoidProfileTimer.get(),
                initState,
                new TrapezoidProfile.State(targetAngleDegrees, 0)
            );

            Logger.recordOutput("isMovingToTarget", true);
        }



        double totalOutputVolts;

        //If there's no trapezoidal profile active, just use PID
        if (profile.isFinished(trapezoidProfileTimer.get())) {

            Logger.recordOutput("isMovingToTarget", false);

            double feedforwardOutputVolts = armFeedforward.calculate(
                Math.toRadians(targetAngleDegrees),
                0);
            double pidOutputVolts = armPD.calculate(
                pivotAngleDegrees,
                pivotVelocityDegreesPerSecond,
                targetAngleDegrees,
                0.
            );

            totalOutputVolts = feedforwardOutputVolts + pidOutputVolts;
        }

        //otherwise, follow trapezoid profile
        else {

            
            Logger.recordOutput("isMovingToTarget", true);

            TrapezoidProfile.State desiredState = profile.calculate(
                trapezoidProfileTimer.get(),
                initState,
                new TrapezoidProfile.State(targetAngleDegrees, 0)
            );

            double feedforwardOutputVolts = armFeedforward.calculate(
                Math.toRadians(pivotAngleDegrees),
                Math.toRadians(desiredState.velocity)
            );

            double pidOutputVolts = armPD.calculate(
                pivotAngleDegrees,
                pivotVelocityDegreesPerSecond,
                desiredState.position,
                desiredState.velocity
            );

            totalOutputVolts = feedforwardOutputVolts + pidOutputVolts;

            Logger.recordOutput("arm/trapezoidProfilePosition", desiredState.position);
        }


        totalOutputVolts = MathUtil.clamp(totalOutputVolts, -12, 12);

        if ((atLowerLimit() && totalOutputVolts < 0) || (atUpperLimit() && totalOutputVolts > 0))
            totalOutputVolts = 0;

        Logger.recordOutput("arm/totalOutputVolts", totalOutputVolts);

        setArmMotorVolts(totalOutputVolts);

    }

    public Command setDesiredDegreesCommand(double targetAngleDegrees) {
        return this.run(() -> {
            this.setDesiredDegrees(targetAngleDegrees);});
    }

    public Command holdCurrentPositionCommand() {
        return this.run(() -> {this.setDesiredDegrees(pivotAngleDegrees);});
    }

    public double getErrorDegrees() {
        return this.targetAngleDegrees - pivotAngleDegrees;
    }

    public boolean isCloseToTarget() {
        // TODO: pick a non-arbitrary value based on sensor resolution?
        boolean errorIsSmall = Math.abs(getErrorDegrees()) < 1.0;
        boolean isKeepingUp = Math.abs(pivotVelocityDegreesPerSecond - setpointVelocityDegreesPerSecond) < 1.0;
        return errorIsSmall && isKeepingUp;
    }

    @Override
    public void periodic() {
        this.pivotAngleDegrees = pivotEncoder.getAbsolutePosition().getValueAsDouble()*360;
        this.pivotVelocityDegreesPerSecond = pivotEncoder.getVelocity().getValueAsDouble()*360;

        Logger.recordOutput("arm/pivotAngleDegrees", pivotAngleDegrees);
        Logger.recordOutput("arm/targetAngleDegrees", targetAngleDegrees);
        Logger.recordOutput("arm/prevTargetAngleDegrees", prevTargetAngleDegrees);
        Logger.recordOutput("arm/trapezoidProfileTimer", trapezoidProfileTimer.get());

        Logger.recordOutput("arm/leftMotor/appliedOutput", leftPivot.getAppliedOutput());
        Logger.recordOutput("arm/rightMotor/appliedOutput", rightPivot.getAppliedOutput());
        Logger.recordOutput("arm/leftMotor/current", leftPivot.getOutputCurrent());
        Logger.recordOutput("arm/rightMotor/current", rightPivot.getOutputCurrent());
    }
}
