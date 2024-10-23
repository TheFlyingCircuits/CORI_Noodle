// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
    public static final boolean atCompetition = false;
    public static final boolean isDemoMode = false;

    public static class ArmConstants {

        public final static double canCoderOffset = -0.272;

        //TODO: FILL
        /** Distance from the floor to the center of the pivot. This is used for angle calculations for shoot from anywhere. */
        public final static double pivotHeightMeters = Units.inchesToMeters(15);
        
        /** Horizontal distance from the robot center to the pivot center */
        //POSITIVE IF BEHIND THE CENTER
        public final static double pivotOffsetMeters = Units.inchesToMeters(13);

        public final static double armLengthMeters = Units.inchesToMeters(17.73);



        public static final double armMinAngleDegrees = 7;
        public static final double armMaxAngleDegrees = 111.5;

        public final static double armMaxVelDegreesPerSecond = 500.;
        public final static double armMaxAccelDegreesPerSecondSquared = 1000.;

        public static final TrapezoidProfile.Constraints constraints = new Constraints(
            armMaxVelDegreesPerSecond,
            armMaxAccelDegreesPerSecondSquared
        );


        /***** REAL CONSTANTS ******/
        public final static double kSArmVolts = 0.00;
        public final static double kGArmVolts = 0.25;
        public final static double kVArmVoltsSecondsPerRadian = 2.25; //3.1;
        public final static double kAArmVoltsSecondsSquaredPerRadian = 0;

        public final static double kPArmVoltsPerDegree = 0.01;
        public final static double kDArmVoltsSecondsPerDegree = 0.;
    }

    public final static class ShooterConstants {
        /**Rotations of the motor per rotations of the wheel; a number greater than 1 represents a reduction. */
        public final static double flywheelGearReduction = 1.;
        public final static double flywheelCircumferenceMeters = Units.inchesToMeters(4)*Math.PI;

        /**Meters per second of the flywheel per RPM that the motor reads, multiply this number by RPM to get MPS. */
        public final static double motorRPMToFlywheelMPS = flywheelCircumferenceMeters * flywheelGearReduction / 60.;
    
        public final static double kPFlywheelsVoltsSecondsPerMeter = .1;

        public final static double kSFlywheelsVolts = 0.2735; //copied from ronnie
        public final static double kVFlywheelsVoltsSecondsPerMeter = 0.38;
        public final static double kAFlywheelsVoltsSecondsSquaredPerMeter = 0.;

        public final static int currentLimitAmps = 100;

        public static final double motorMaxTempCelsius = 70;
    }


    public final static class IntakeConstants {
        public static final int bottomIntakeNeoID = 10;
        public static final int topIntakeNeoID = 9;
        public static final int indexerNeoID = 13;
    }

    public final static class SwerveModuleConstants {
      /** Rotations of the drive wheel per rotations of the drive motor. */
      public static final double driveGearReduction = (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

      /** Rotations of the steering column per rotations of the angle motor. */
      public static final double steerGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

      // The wheels have a 2 inch radius, but sink into the capet about (1/16) of an inch.
      // As an estimate, the wheel radius is Units.inchesToMeters(2.-1./16.), or 0.0492m
      public static final double wheelRadiusMeters = 0.04946; //use MeasureWheelDiameter for this!
      public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters; // ~0.31

      public static final double drivekPVoltsPerMeterPerSecond = 0.1;
      public static final double drivekIVoltsPerMeter = 0.;
      public static final double drivekDVoltsPerMeterPerSecondSquared = 0.;

      // PID for angle motors.
      public static final double anglekPVoltsPerDegree = 0.08;
      public static final double anglekIVoltsPerDegreeSeconds = 0.; // this might be the wrong unit idk 
      public static final double anglekDVoltsPerDegreePerSecond = 0.;

      public static final double drivekSVolts = 0.2383;
      public static final double drivekVVoltsSecondsPerMeter = 2.2859; // TODO: add desmos link
      public static final double drivekAVoltsSecondsSquaredPerMeter = 0.;
    }

    public final static class DrivetrainConstants {
        // KINEMATICS CONSTANTS

        /**
         * Distance between the center point of the left wheels and the center point of the right wheels.
         */        /**
         * Distance between the center point of the front wheels and the center point of the back wheels.
         */
        /**
         * Distance from the center of the robot to each swerve module.
         */  //0.4177

        public static final double trackwidthMeters = Units.inchesToMeters(22.75);
        /**
         * Distance between the center point of the front wheels and the center point of
         * the back wheels.
         */
        public static final double wheelbaseMeters = Units.inchesToMeters(22.75);

        public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelDiamaterMeters = 0.10033;
        public static final double wheelCircumferenceMeters = wheelDiamaterMeters * Math.PI;
        public static final double drivetrainRadiusMeters = Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
                new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
                new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
                new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0));
        public static final double maxDesiredTeleopAngularVelocityRadiansPerSecond = Units.rotationsToRadians(0.85);
         
        public static final double maxAchievableVelocityMetersPerSecond = 5880.0 / 60.0 *
        driveReduction *
        wheelDiamaterMeters * Math.PI;
        public static final double maxDesiredTeleopVelocityMetersPerSecond = 4.3;
        public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond /
                Math.hypot(trackwidthMeters / 2.0, wheelbaseMeters / 2.0);
        public static final double maxDesiredAngularVelocityRadiansPerSecond = 5.4;

        public static final double maxDesiredDriverAccel = 27.27;
        public static final PathConstraints pathfindingConstraints = new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(360), Units.degreesToRadians(360));
    }

    public final static class VisionConstants {

    

        public static AprilTagFieldLayout getPracticeFieldTagLayout() {
            AprilTagFieldLayout officialLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

            List<AprilTag> practiceFieldTags = new ArrayList<AprilTag>(officialLayout.getTags());

            for (AprilTag tag : practiceFieldTags) {
                // Our red speaker tags are in line with the red alliance wall,
                // but they are recessed 1.5 inches behind the wall on an official field.
                if (tag.ID == 3 || tag.ID == 4) {
                    Translation3d officialLocation = tag.pose.getTranslation();
                    double x = officialLocation.getX() - Units.inchesToMeters(1.5);
                    double y = officialLocation.getY();
                    double z = officialLocation.getZ();
                    tag.pose = new Pose3d(new Translation3d(x, y, z), tag.pose.getRotation());
                }

                // set the tag on the pit to be the red amp when in the basement for temporary testing.
                boolean inBasement = false;
                if (inBasement && tag.ID == 12) {
                    tag.pose = officialLayout.getTagPose(5).get();
                }
            }

            return new AprilTagFieldLayout(practiceFieldTags, officialLayout.getFieldLength(), officialLayout.getFieldWidth());
        }
    

        public final static AprilTagFieldLayout aprilTagFieldLayout = atCompetition ? AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
          : getPracticeFieldTagLayout();



        
        public final static Transform3d robotToNoteCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-17.75), 0, Units.inchesToMeters(13.75)),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))
        );

        public final static Transform3d robotToShooterCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.125), 0, Units.inchesToMeters(11)),
            new Rotation3d(0, Math.toRadians(-28), 0)
        );

        public final static String[] cameraNames = {
            "shooterCamera",
        };

        public final static Transform3d tagCameraTransforms[] = {
            robotToShooterCamera,
        };


    }
    public static enum FieldElement {
        SPEAKER(4, 7), AMP(5, 6), 
        STAGE_LEFT(11, 15), STAGE_RIGHT(12, 16), CENTER_STAGE(13, 14), 
        LOB_TARGET(new Pose3d(AMP.redPose.interpolate(SPEAKER.redPose, 0.2).toPose2d()),
                   new Pose3d(AMP.bluePose.interpolate(SPEAKER.bluePose, 0.2).toPose2d())
        ), 
        CARPET(), 
        NOTE_3(new Translation3d(FieldConstants.maxX - FieldConstants.metersFromAllianceWallToFrontlineNotes, FieldConstants.maxY / 2.0, 0),
               new Translation3d(FieldConstants.metersFromAllianceWallToFrontlineNotes, FieldConstants.maxY / 2.0, 0)
        ),
        NOTE_2(NOTE_3.redPose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenFrontlineNotes, 0)),
               NOTE_3.bluePose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenFrontlineNotes, 0))
        ),
        NOTE_1(NOTE_2.redPose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenFrontlineNotes, 0)),
               NOTE_2.bluePose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenFrontlineNotes, 0))
        ),
        NOTE_6(new Translation3d(FieldConstants.maxX / 2.0, FieldConstants.maxY / 2.0, 0)),
        NOTE_5(NOTE_6.redPose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenMidlineNotes, 0))),
        NOTE_4(NOTE_5.redPose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenMidlineNotes, 0))),
        NOTE_7(NOTE_6.redPose.getTranslation().plus(new Translation3d(0, -FieldConstants.metersBetweenMidlineNotes, 0))),
        NOTE_8(NOTE_7.redPose.getTranslation().plus(new Translation3d(0, -FieldConstants.metersBetweenMidlineNotes, 0))),
        MID_FIELD(new Translation3d(FieldConstants.maxX / 2.0, FieldConstants.maxY / 2.0, 0)),
        WING(new Translation3d(FieldConstants.maxX - FieldConstants.metersFromAllianceWallToWing, FieldConstants.maxY / 2.0, 0),
             new Translation3d(FieldConstants.metersFromAllianceWallToWing, FieldConstants.maxY / 2.0, 0)),
        SOURCE(9, 2);

        private static class FieldConstants {
            public static final double maxX = VisionConstants.aprilTagFieldLayout.getFieldLength();
            public static final double maxY = VisionConstants.aprilTagFieldLayout.getFieldWidth();
            public static final double metersBetweenFrontlineNotes = Units.inchesToMeters(57);
            public static final double metersBetweenMidlineNotes = Units.inchesToMeters(66);
            public static final double metersFromAllianceWallToFrontlineNotes = Units.inchesToMeters(114);
            public static final double metersFromAllianceWallToWing = Units.inchesToMeters(231.20);
        }

        /* End of Enum Instances */

        private Pose3d redPose;
        private Pose3d bluePose;
        public static Translation3d demoTargetLocation;

        private FieldElement(int redTagID, int blueTagID) {
            this.redPose = VisionConstants.aprilTagFieldLayout.getTagPose(redTagID).get();
            this.bluePose = VisionConstants.aprilTagFieldLayout.getTagPose(blueTagID).get();

            /* Target the opening of the speaker, rather than the speaker tag */
            if (redTagID == 4 || blueTagID == 7) {
                double speakerUpperLipHeightMeters = Units.inchesToMeters(82.90);
                double speakerLowerLipHeightMeters = Units.inchesToMeters(78.13);
                double speakerDepthIntoFieldMeters = Units.inchesToMeters(18.11);
                double speakerHeightMeters = (speakerUpperLipHeightMeters + speakerLowerLipHeightMeters) / 2.;

                double speakerY = Units.inchesToMeters(218.42);
                double redSpeakerX = Units.inchesToMeters(652.73-1.5) - (speakerDepthIntoFieldMeters / 2.);
                double blueSpeakerX = 0 + (speakerDepthIntoFieldMeters / 2.);

                Translation3d redLocation = new Translation3d(redSpeakerX, speakerY, speakerHeightMeters);
                Rotation3d redOrientation = new Rotation3d(0 , 0, Math.toRadians(180));
                Translation3d blueLocation = new Translation3d(blueSpeakerX, speakerY, speakerHeightMeters);
                Rotation3d blueOrientation = new Rotation3d(0, 0, 0);

                redPose = new Pose3d(redLocation, redOrientation);
                bluePose = new Pose3d(blueLocation, blueOrientation);
            }

            // use the middle of the source as the pose
            if (redTagID == 9 || redTagID == 10 || blueTagID == 1 || blueTagID == 2) {
                Pose3d firstRedPose = VisionConstants.aprilTagFieldLayout.getTagPose(9).get();
                Pose3d secondRedPose = VisionConstants.aprilTagFieldLayout.getTagPose(10).get();

                Pose3d firstBluePose = VisionConstants.aprilTagFieldLayout.getTagPose(1).get();
                Pose3d secondBluePose = VisionConstants.aprilTagFieldLayout.getTagPose(2).get();

                this.redPose = firstRedPose.interpolate(secondRedPose, 0.5);
                this.bluePose = firstBluePose.interpolate(secondBluePose, 0.5);
            }
        }

        private FieldElement(Translation3d location) {
            this.redPose = new Pose3d(location, new Rotation3d());
            this.bluePose = new Pose3d(location, new Rotation3d());
        }

        private FieldElement(Translation3d redLocation, Translation3d blueLocation) {
            this.redPose = new Pose3d(redLocation, new Rotation3d());
            this.bluePose = new Pose3d(blueLocation, new Rotation3d());
        }

        private FieldElement(Pose3d redPose, Pose3d bluePose) {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }

        private FieldElement() {
            // ONLY TO BE USED FOR FieldElement.CARPET,
            // because the location of the shart depends on the location of the robot.
            this.redPose = null;
            this.bluePose = null;
        }

        public Pose3d getPose() {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    return redPose;
                }
                if (alliance.get() == Alliance.Blue) {
                    return bluePose;
                }
            }
            // Should never get to this point as long as we're connected to the driver station.
            return new Pose3d();
        }

        public Translation3d getLocation() {
            return getPose().getTranslation();
        }

        public Rotation3d getOrientation() {
            return getPose().getRotation();
        }

        public double getX() {
            return getLocation().getX();
        }

        public double getY() {
            return getLocation().getY();
        }

        public double getZ() {
            return getLocation().getZ();
        }

        public static Pose2d getClosestTrap(Pose2d yourPoseOnTheField) {
            Pose2d[] trapLocations = {STAGE_LEFT.getPose().toPose2d(),
                                      STAGE_RIGHT.getPose().toPose2d(),
                                      CENTER_STAGE.getPose().toPose2d()};

            return yourPoseOnTheField.nearest(Arrays.asList(trapLocations));
        }

        public static int getSpeakerTagID() {

            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    return 4;
                }
                if (alliance.get() == Alliance.Blue) {
                    return 7;
                }
            }
            // Should never get to this point as long as we're connected to the driver station.
            return -1;
        }
    }
        public final static class GyroConstants {
        public static final int pigeonID = 0;


        //Follow the mount calibration process in Phoenix Tuner to determine these
        public static final double mountPoseYawDegrees = 0.7333114147186279; // NEED TO UPDATE THIS FOR NOODLES PIGEON
        public static final double mountPosePitchDegrees = -0.11852765083312988;
        public static final double mountPoseRollDegrees = -1.0425487756729126;
    }
    public final static class MotorConstants {
        public static final int universalCurrentLimitAmps = 50;

        // Motor configs
        public static final int angleContinuousCurrentLimit = 50;
        public static final boolean angleInvert = true;
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        
        public static final int driveContinuousCurrentLimit = 60;
        public static final boolean driveInvert = true;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    }
    public final static class ControllerConstants {
        public static final double controllerDeadzone = 0.175;
        public static final double maxThrottle = 1.0;
    }

  }

