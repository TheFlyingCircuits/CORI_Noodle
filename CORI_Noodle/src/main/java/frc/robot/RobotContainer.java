// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PrepShot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.vision.VisionIOPhotonLib;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public Arm arm;
    public Shooter shooter;
    public Drivetrain drivetrain;
    public Intake intake;
    public LEDs leds;


    public final HumanDriver driver = new HumanDriver(0);




    /**** INITIALIZE SUBSYSTEMS ****/
    public RobotContainer() {
        if (RobotBase.isReal()) {
        drivetrain = new Drivetrain( // fr 0.092041015625, br , 0.0419921875, fl -0.178955078125, bl -0.332763671875
            new GyroIOPigeon(),
            new SwerveModuleIONeo(7, 8, -0.184814453125, 0), 
            new SwerveModuleIONeo(5, 6, 0.044677734375, 3),
            new SwerveModuleIONeo(3, 4, -0.3349609375, 2),
            new SwerveModuleIONeo(1, 2,  0.088134765625, 1),
            new VisionIOPhotonLib()
            );
        }

        leds = new LEDs();
        intake = new Intake();
        arm = new Arm();
        shooter = new Shooter();

        arm.setDefaultCommand(arm.holdCurrentPositionCommand());
        shooter.setDefaultCommand(shooter.setFlywheelSurfaceSpeedCommand(0));
        drivetrain.setDefaultCommand(drivetrain.run(() -> {drivetrain.fieldOrientedDrive(driver.getRequestedFieldOrientedVelocity(), true);}));
        intake.setDefaultCommand(intake.runIntakeCommand(0, 0, 0));

        // Configure the trigger bindings
        realBindings();
    }

    private void realBindings() {
        CommandXboxController controller = driver.getXboxController();

        controller.rightTrigger()
            .whileTrue(
                //intake after note if on other side of the field
                intakeTowardsNote(driver::getRequestedFieldOrientedVelocity)
            ).onFalse(
                positionNote()
            );
        
        // controller.a().onTrue(arm.setDesiredDegreesCommand(ArmConstants.armMinAngleDegrees));
        // controller.x().onTrue(arm.setDesiredDegreesCommand(30));
        // controller.y().onTrue(arm.setDesiredDegreesCommand(60));
        // controller.b().onTrue(arm.setDesiredDegreesCommand(90));

        Trigger inSpeakerShotRange = new Trigger(drivetrain::inSpeakerShotRange);
        controller.rightBumper().and(inSpeakerShotRange)
            .onTrue(
                this.speakerShot()
                .andThen(new ScheduleCommand(this.resetShooter()))
            );

        controller.b().onTrue(shart().andThen(new ScheduleCommand(resetShooter())));
        controller.x().onTrue(resetShooter());


        controller.leftTrigger().whileTrue(reverseIntake());
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()).repeatedly().until(drivetrain::seesTag));
    }



    private Command runIntake() {
        return intake.runIntakeCommand(5,5,5);
    }

    private Command reverseIntake() {
        return intake.runIntakeCommand(-6,-6,-6);
    }
    private Command intakeNote() {
        return new ScheduleCommand(leds.playIntakeAnimationCommand(() -> {return drivetrain.getBestNoteLocationFieldFrame().isPresent();}).withName("intake animation"))
            .alongWith(this.runIntake());
    }

    private Command positionNote() {
        //reverse intake to position note and also to eject double intakes
        return intake.runIntakeCommand(-4,-4,-4).alongWith(
                shooter.setFlywheelSurfaceSpeedCommand(-3)).withTimeout(0.6);
    }

    private Command fireNote() {
        return intake.runIntakeCommand(0, 0, 5).withTimeout(0.3);
    }

    //////// ARM/SHOOTER /////////////

    /** Resets the angle and speed of the shooter back to its default idle position. */
    private Command resetShooter() {
        double desiredAngle = ArmConstants.armMinAngleDegrees + 2; // puts the arm at min height to pass under stage

        return arm.setDesiredDegreesCommand(desiredAngle)
                .alongWith(shooter.setFlywheelSurfaceSpeedCommand(0));
    }

    private Command speakerShot() {
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, driver::getRequestedFieldOrientedVelocity, leds, FieldElement.SPEAKER);
        Command waitForAlignment = new WaitUntilCommand(() -> {return drivetrain.hasRecentSpeakerTagMeasurement(0.25);})
                                    .withTimeout(1)
                                    .andThen(new WaitUntilCommand(aim::readyToShoot));  // wait for vision to stabilize
        Command fire = fireNote();
        return aim.raceWith(waitForAlignment.andThen(fire));
    }
    
    private Command shart() {
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, null, leds, FieldElement.CARPET);
        Command waitForAlignment = new WaitUntilCommand(aim::readyToShoot);
        Command fire = fireNote();
        return aim.raceWith(waitForAlignment.andThen(fire));
    }

    private Command prepLobShot() {
        return new PrepShot(drivetrain, arm, shooter, driver::getRequestedFieldOrientedVelocity, leds, FieldElement.LOB_TARGET);
    }

    /**
     * @param howToDriveWhenNoNoteDetected let's driver have control if the noteCam doesn't see a note
     * @return
     */
    private Command intakeTowardsNote(Supplier<ChassisSpeeds> howToDriveWhenNoNoteDetected) {
        return intakeNote().raceWith(drivetrain.run(() -> {

            // have charlie stay in control when the noteCam doesn't see a note
            if (drivetrain.getBestNoteLocationFieldFrame().isEmpty()) {
                drivetrain.fieldOrientedDrive(howToDriveWhenNoNoteDetected.get(), true);
                return;
            }

            // drive towards the note when the noteCam does see a note.
            drivetrain.driveTowardsNote(drivetrain.getBestNoteLocationFieldFrame().get());
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

}
