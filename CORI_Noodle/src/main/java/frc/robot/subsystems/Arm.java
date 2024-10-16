package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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

    TrapezoidProfile.State initState;
    Timer trapezoidProfileTimer;

    ArmFeedforward armFeedforward;
    PIDController armPD;

    public Arm() {
        leftPivot = new Neo(99);
        rightPivot = new Neo(99);

        pivotEncoder = new CANcoder(99);
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
        
        double timeSinceLastSetpoint = setpointVelocityTimer.get();
        if (timeSinceLastSetpoint > 0) {
            this.setpointVelocityDegreesPerSecond = (this.targetAngleDegrees - prevTargetAngleDegrees) / timeSinceLastSetpoint;
        }
        setpointVelocityTimer.restart();
        

        //If we need a big movement, generate a trapezoidal profile
        if (Math.abs(this.targetAngleDegrees - prevTargetAngleDegrees) >= 2) {
            initState = new TrapezoidProfile.State(pivotAngleDegrees, pivotVelocityDegreesPerSecond);
            
            trapezoidProfileTimer.restart();
        }

        // No need to generate a new profile if the requested
        // target is close to the current target. PID should get us
        // there on its own.






    }



    private void followTrapezoidProfile() {
        //Hold the current position if there's no trapezoidal profile active. 
        //I think? that generating new trapezoidal profiles for an inactive arm causes some oscillations.
        if (!isMovingToTarget) {

            double feedforwardOutputVolts = armFeedforward.calculate(
                Math.toRadians(targetAngleDegrees),
                0);
            double pidOutputVolts = armPD.calculate(
                inputs.armAngleDegrees,
                inputs.armVelocityDegreesPerSecond,
                targetAngleDegrees,
                0
            );


            // double outputVolts = MathUtil.clamp(
            //     feedforwardOutputVolts + pidOutputVolts,
            //     -maxDesiredVoltageDown,
            //     maxDesiredVoltageUp);

            double outputVolts = feedforwardOutputVolts + pidOutputVolts;

            io.setArmMotorVolts(feedforwardOutputVolts + pidOutputVolts);

            Logger.recordOutput("arm/totalOutputVolts", feedforwardOutputVolts + pidOutputVolts);

            return;
        }

        

        TrapezoidProfile.State desiredState = profile.calculate(
            timer.get(),
            initState,
            new TrapezoidProfile.State(targetAngleDegrees, 0)
        );

        double feedforwardOutputVolts = armFeedforward.calculate(
            Math.toRadians(inputs.armAngleDegrees),
            Math.toRadians(desiredState.velocity)
        );
        double pidOutputVolts = armPD.calculate(
            inputs.armAngleDegrees,
            inputs.armVelocityDegreesPerSecond,
            desiredState.position,
            desiredState.velocity
        );

        double totalOutputVolts = feedforwardOutputVolts + pidOutputVolts;

        totalOutputVolts = MathUtil.clamp(totalOutputVolts, -12, 12);

        if ((inputs.atLowerLimit && totalOutputVolts < 0) || (inputs.atUpperLimit && totalOutputVolts > 0))
            totalOutputVolts = 0;

        Logger.recordOutput("arm/trapezoidProfilePosition", desiredState.position);
        Logger.recordOutput("arm/totalOutputVolts", totalOutputVolts);

        

        io.setArmMotorVolts(totalOutputVolts);

        //profile.calculate() must be called before this line in order for isFinished() to function properly
        if (profile.isFinished(timer.get())) {
            isMovingToTarget = false;
        }
    }

    @Override
    public void periodic() {
        this.pivotAngleDegrees = pivotEncoder.getPosition().getValueAsDouble()*360;
        this.pivotVelocityDegreesPerSecond = pivotEncoder.getVelocity().getValueAsDouble()*360;

        Logger.recordOutput("pivotAngleDegrees", pivotEncoder.getPosition().getValueAsDouble());
    }
}
