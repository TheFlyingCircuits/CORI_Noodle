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

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
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

    public final HumanDriver driver = new HumanDriver(0);
    private boolean goodPickup;




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

        //lob shot (prep while holding, release to fire)
        Command prepLobShot = prepLobShot(); // <- grab a single instance so we can check if it's cancelled.
        controller.rightBumper().and(inSpeakerShotRange.negate())
            .onTrue(prepLobShot)
            .onFalse(this.fireNote()
                            .andThen(new ScheduleCommand(this.resetShooter()))
                        .unless(() -> {return !prepLobShot.isScheduled();})
                        // only fire if the lob shot wasn't cancelled
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
        return this.runIntake();
    }

    /**
     * @param howToDriveWhenNoNoteDetected let's charlie have control if the noteCam doesn't see a note
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

    private Command positionNote() {
        //reverse intake to position note and also to eject double intakes
        return intake.runIntakeCommand(-6,-9,-4).alongWith(
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
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, driver::getRequestedFieldOrientedVelocity, FieldElement.SPEAKER);
        Command waitForAlignment = new WaitUntilCommand(() -> {return drivetrain.hasRecentSpeakerTagMeasurement(0.25);})
                                    .withTimeout(1)
                                    .andThen(new WaitUntilCommand(aim::readyToShoot));  // wait for vision to stabilize
        Command fire = fireNote();
        return aim.raceWith(waitForAlignment.andThen(fire));
    }
    
    private Command shart() {
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, null, FieldElement.CARPET);
        Command waitForAlignment = new WaitUntilCommand(aim::readyToShoot);
        Command fire = fireNote();
        return aim.raceWith(waitForAlignment.andThen(fire));
    }

    private Command prepLobShot() {
        return new PrepShot(drivetrain, arm, shooter, driver::getRequestedFieldOrientedVelocity, FieldElement.LOB_TARGET);
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return new SequentialCommandGroup(
        //     speakerShot(),
        //     resetShooter().alongWith(
        //         drivetrain.run(
        //             () -> {
        //                 drivetrain.robotOrientedDrive(
        //                     new ChassisSpeeds(-2, 0, 0),
        //                     true
        //                 );
        //             }
        //         ).withTimeout(2.0)   
        //     )
        // );

        FieldElement[] notesToGoFor = {FieldElement.NOTE_4, FieldElement.NOTE_5, FieldElement.NOTE_6};
        return sideAuto(notesToGoFor, FieldElement.AMP);
    }


    /**** AUTO ****///////////////////////////////////////////////////////////////////////

    /**
     * Aims the flywheels and arm at the speaker, while also setting the Pathplanner rotation override.
     * This command never finishes.
     */
    private Command prepAutoSpeakerShot() {
        return new InstantCommand(() -> {drivetrain.isTrackingSpeakerInAuto = true;})
                .andThen(new PrepShot(drivetrain, arm, shooter, null, FieldElement.SPEAKER))
                .finallyDo(() -> {drivetrain.isTrackingSpeakerInAuto = false;});
    }

    private Command autoIntakeTowardsNote(FieldElement note) {
        return new SequentialCommandGroup(
            // Assume the pickup didn't work until proven otherwise
            new InstantCommand(() -> {this.goodPickup = false;}),

            // Drive towards the note if we see it, or drive to where it should be if we don't see it.
            intakeNote().raceWith(drivetrain.run(() -> {

                Translation2d noteLocation = note.getLocation().toTranslation2d();
                if (drivetrain.getBestNoteLocationFieldFrame().isPresent()) {
                    noteLocation = drivetrain.getBestNoteLocationFieldFrame().get();
                }

                drivetrain.driveTowardsNote(noteLocation);
            }))
            // Set goodPickup to true if the intake sequence finishies normally
            // (i.e. intakeNote() wins the ParallelRaceGroup because the intake sensor triggered)
            .finallyDo(
                (boolean interrupted) -> {this.goodPickup = !interrupted;}
            )
            // Interrupt the intake sequence if it's determined that the current target note is a lost cause
            // (already taken by the opposing alliance, or too risky to pickup because its so far over the midline that
            // we might get fouls by going for it)
            .until(() -> {return noteIsLostCauseInAuto(note);}).withTimeout(4.0)
        ).withTimeout(2);
    }

    /**
     * TODO: documentation
     * @param note
     * @return
     */
    private boolean noteIsLostCauseInAuto(FieldElement note) {

        boolean canSeeNote = drivetrain.getBestNoteLocationFieldFrame().isPresent();
        Translation2d noteLocation = note.getLocation().toTranslation2d();
        if (canSeeNote) {
            noteLocation = drivetrain.getBestNoteLocationFieldFrame().get();
        }

        boolean noteIsGone = shouldSeeNote(note) && !canSeeNote;
        boolean pickupTooRisky = noteIsTooRiskyForPickupInAuto(noteLocation);

        // if the first note is too far away, then that's the note we'll likely see on the next frame
        // even if we're starting to turn away. This is why we need the additional shouldSeeNote() check.
        pickupTooRisky = pickupTooRisky && shouldSeeNote(note);

        Logger.recordOutput("auto/targetNote", note.name());
        Logger.recordOutput("auto/noteIsGone", noteIsGone);
        Logger.recordOutput("auto/pickupTooRisky", pickupTooRisky);

        return noteIsGone || pickupTooRisky;
    }


    private boolean shouldSeeNote(FieldElement note) {
        double pickupRadius = 0.75; // want this to be large for early exit.
        double fovConeDegrees = 3; // need this so we don't abort too quickly.
        Translation2d noteToRobot = drivetrain.getPoseMeters().getTranslation().minus(note.getLocation().toTranslation2d());
        boolean closeEnough = noteToRobot.getNorm() <= pickupRadius;
        boolean noteInFov = Math.abs(drivetrain.getPoseMeters().getRotation().minus(noteToRobot.getAngle()).getDegrees()) <= (fovConeDegrees / 2.);
        return closeEnough && noteInFov;
    }

    private boolean noteIsTooRiskyForPickupInAuto(Translation2d noteLocation) {
        // We'll risk getting fowls if the note is too far into enemy territory.
        double midlineX = FieldElement.MID_FIELD.getX();
        double overshootAllowanceMeters = 2.0; // TODO: tune me!

        boolean tooRiskyForBlue = noteLocation.getX() > (midlineX + overshootAllowanceMeters);
        boolean tooRiskyForRed  = noteLocation.getX() < (midlineX - overshootAllowanceMeters);

        boolean onBlueAlliance = FieldElement.SPEAKER.getLocation().getX() < midlineX;

        return (onBlueAlliance && tooRiskyForBlue) || (!onBlueAlliance && tooRiskyForRed);
    }


    /**
     * Navigates from a scoring location to the pickup spot for the next note.
     * @param note - the note you want to pickup.
     * @param startLocation 
     */
    private Command navigatePickup(FieldElement note, String startLocation) {
        char noteNuber = note.name().charAt(note.name().length()-1);
        String pathName = startLocation+" to Ring "+noteNuber;

        return new ParallelDeadlineGroup(
                FlyingCircuitUtils.followPath(pathName),
                resetShooter(),
                new PrintCommand("Driving from "+startLocation+" to "+ note.name())
        );
    }

    /**
     * Navigates from a note to a scoring location after a pickup.
     * @param note - the note you are navigating from.
     */
    private Command scoreRingAfterPickup(FieldElement note, String endLocation) {
        char noteNuber = note.name().charAt(note.name().length()-1);
        String pathName = "Ring "+noteNuber+" to "+endLocation;

        // Don't aim on the way if we're going under the stage
        boolean aimOnTheWay = !endLocation.contains("Center Side");

        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                FlyingCircuitUtils.followPath(pathName),
                new ConditionalCommand(
                    prepAutoSpeakerShot(), 
                    new InstantCommand(),
                    () -> {return aimOnTheWay;}),
                new PrintCommand("Scoring " + note.name())
            ),
            speakerShot()
        );
    }



    public Command sideAuto(FieldElement[] notesToGoFor, FieldElement startLocation) {
        FieldElement highPriorityNote = notesToGoFor[0];
        FieldElement midPriorityNote = notesToGoFor[1];
        FieldElement lowPriorityNote = notesToGoFor[2];

        String side = "";
        if (startLocation == FieldElement.AMP) {
            side = "Amp";
        }
        if (startLocation == FieldElement.SOURCE) {
            side = "Source";
        }
        String shotName = side+" Shot";

        Command auto = new SequentialCommandGroup(
            speakerShot(),
            navigatePickup(highPriorityNote, "Starting Line ("+side+" Side)"),
            autoIntakeTowardsNote(highPriorityNote),
            scoreRingAfterPickup(highPriorityNote, shotName).andThen(navigatePickup(midPriorityNote, shotName)),
            autoIntakeTowardsNote(midPriorityNote),
            scoreRingAfterPickup(midPriorityNote, shotName).andThen(navigatePickup(lowPriorityNote, shotName)),
            autoIntakeTowardsNote(lowPriorityNote),
            scoreRingAfterPickup(lowPriorityNote, shotName).onlyIf(() -> {return this.goodPickup;})
        );

        // Name the auto so it looks nice on the dashboard.
        char highPriorityAsChar = highPriorityNote.name().charAt(highPriorityNote.name().length()-1);
        char midPriorityAsChar = midPriorityNote.name().charAt(midPriorityNote.name().length()-1);
        char lowPriorityAsChar = lowPriorityNote.name().charAt(lowPriorityNote.name().length()-1);
        String autoName = "("+highPriorityAsChar+", "+midPriorityAsChar+", "+lowPriorityAsChar+") HyperChad "+side+" Side";
        return auto.withName(autoName);
    }

}
