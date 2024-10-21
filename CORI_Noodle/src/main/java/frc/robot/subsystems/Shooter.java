package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.VendorWrappers.Neo;
import main.java.frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;


/**
 * IMPORTANT NOTE TO BENJI:
 * 1. MOTORS HAVE NO OVERHEAT PROTECTION, VOLTAGE WILL NOT STOP BEING APPLIED IF MOTORS GET DANGEOUSLY HOT
 * not so important notes to benji:
 * 2. inversions may be wrong in configMotors(), elaboration in method
 * 3. auto commands were not replicated here
 * 4. idk what the spring control thing was but i did not add that either
 * 5. shooterconstraints i believe are for krakens so PID could be off
*/


public class Shooter extends SubsystemBase {
    
    /**
     * Linear surface speed of the left set of flywheels.
     */
    public double leftFlywheelsMetersPerSecond = 0.0;
    /**
     * Linear surface speed of the right set of flywheels.
     */
    public double rightFlywheelsMetersPerSecond = 0.0;
    /**
     * Current voltage being applied to the shooter's left motor
     */
    public double leftMotorAppliedVoltage = 0.0;
    /**
     * Current voltage being applied to the shooter's right motor
     */
    public double rightMotorAppliedVoltage = 0.0;
    
    public double leftMotorOutputCurrent = 0.0;
    public double rightMotorOutputCurrent = 0.0;

    public double leftFlywheelRadians = 0.0;
    public double rightFlywheelRadians = 0.0;

    public double leftFlywheelRadiansPerSecond = 0.0;
    public double rightFlywheelRadiansPerSecond = 0.0;

    public double leftFlywheelRadiansPerSecondSquared = 0.0;
    public double rightFlywheelRadiansPerSecondSquared = 0.0;


    public Neo leftMotor;
    public Neo rightMotor;
    /** PID controller for the left motor and flywheels.
     * The PID constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's PIDController class.
     */
    private PIDController leftFlywheelsPID;
    /** PID controller for the right motor and flywheels.
     * The PID constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's PIDController class.
     */
    private PIDController rightFlywheelsPID;

    /** Feedforward model for an individual set of flywheels. 
     * This assumes that the shooter's flywheel physics is 
     * symmetric on the left and right sides.
     * The feedforward constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's SimpleMotorFeedforward class.
     */
    private SimpleMotorFeedforward flywheelsFeedforward;



    public Shooter() {

        leftMotor = new Neo(99);

        rightMotor = new Neo(99);

        configMotors();

        leftFlywheelsPID = new PIDController(
            ShooterConstraints.kPFlywheelsVoltsSecondsPerMeter,
            ShooterConstants.kIFlywheelsVoltsPerMeter,
            ShooterConstants.kDFlywheelsVoltsSecondsSquaredPerMeter
        );

        rightFlywheelsPID = new PIDController(
            ShooterConstants.kPFlywheelsVoltsSecondsPerMeter, 
            ShooterConstants.kIFlywheelsVoltsPerMeter, 
            ShooterConstants.kDFlywheelsVoltsSecondsSquaredPerMeter
        );

        flywheelsFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.kSFlywheelsVolts,
            ShooterConstants.kVFlywheelsVoltsSecondsPerMeter,
            ShooterConstants.kAFlywheelsVoltsSecondsSquaredPerMeter
        );

        leftFlywheelsPID.setTolerance(1.0);
        rightFlywheelsPID.setTolerance(1.0);
    }

    private void configMotors() {
        leftMotor.setInverted(false); //needs to be clockwise positive
        leftMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setSmartCurrentLimit(120);
        
        rightMotor.setInverted(true); //neds to be counterclockwise positive
        rightMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setSmartCurrentLimit(120);
    }

    private void setLeftMotorVolts(double volts) {
        leftMotor.setVoltage(volts);
    }

    private void setRightMotorVolts(double volts) {
        rightMotor.setVoltage(volts);
    }

    /**
     * Sets the surface speed of the left set of flywheels.
     * This is theoretically the speed that this side of the note will have when exiting the shooter.
     * @param metersPerSecond - Desired surface speed in meters per second.
     */
    public void setLeftFlywheelsMetersPerSecond(double metersPerSecond) {
        double feedforwardOutput = flywheelsFeedforward.calculate(metersPerSecond);
        double pidOutput = leftFlywheelsPID.calculate(leftFlywheelsMetersPerSecond, metersPerSecond);
        setLeftMotorVolts(feedforwardOutput + pidOutput);
    }

    /**
     * @return - The shooter's left flywheel surface speed in meters per second
     */
    public double getLeftFlywheelsMetersPerSecond() {
        return leftFlywheelsMetersPerSecond;
    }

    /**
     * Sets the surface speed of the right set of flywheels.
     * This is theoretically the speed that this side of the note will have when exiting the shooter.
     * @param metersPerSecond - Desired surface speed in meters per second.
     */
    public void setRightFlywheelsMetersPerSecond(double metersPerSecond) {
        double feedforwardOutput = flywheelsFeedforward.calculate(metersPerSecond);
        double pidOutput = rightFlywheelsPID.calculate(rightFlywheelsMetersPerSecond, metersPerSecond);
        setRightMotorVolts(feedforwardOutput + pidOutput);
    }

    /**
     * @return - The shooter's right flywheel surface speed in meters per second
     */
    public double getRightFlywheelsMetersPerSecond() {
        return rightFlywheelsMetersPerSecond;
    }

    public void setBothFlywheelsMetersPerSecond(double metersPerSecond) {
        setLeftFlywheelsMetersPerSecond(metersPerSecond);
        setRightFlywheelsMetersPerSecond(metersPerSecond);
    }

    /**
     * Returns true if both flywheels are spinning within some threshold of their target speeds.
     */
    public boolean flywheelsAtSetpoints() {
        return leftFlywheelsPID.atSetpoint() && rightFlywheelsPID.atSetpoint();
    }

    public double getWorstError() {
        double errorLeft = leftFlywheelsPID.getPositionError();
        double errorRight = rightFlywheelsPID.getPositionError();

        if (Math.abs(errorLeft) > Math.abs(errorRight)) {
            return errorLeft;
        }
        return errorRight;
    }

    @Override
    public void periodic() {
        leftFlywheelsMetersPerSecond = leftMotor.getVelocity(); // need to multiply by flywheel circumference in meters
        rightFlywheelsMetersPerSecond = rightMotor.getVelocity(); // need to mutliply by flywheel circumference in meters

        leftMotorAppliedVoltage = leftMotor.getBusVoltage();
        rightMotorAppliedVoltage = rightMotor.getBusVoltage();

        Logger.recordOutput("shooter/leftFlywheelMetersPerSecond", leftFlywheelsMetersPerSecond);
        Logger.recordOutput("rightFlywheelMetersPerSecond", rightFlywheelsMetersPerSecond);
        Logger.recordOutput("leftMotorAppliedVoltage", leftMotorAppliedVoltage);
        Logger.recordOutput("rightMotorAppliedVoltage", rightMotorAppliedVoltage);
    }

    
}
