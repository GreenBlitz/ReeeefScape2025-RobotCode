package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import java.util.function.Function;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.swervestatehelpers.AimAssistUtils.getRotationAssistedSpeeds;

public class SwerveStateHandler {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Swerve swerve;
    private final SwerveConstants swerveConstants;
    private final Supplier<Translation2d> noteTranslationSupplier;

    public SwerveStateHandler(Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> noteTranslationSupplier, Swerve swerve) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.swerve = swerve;
        this.swerveConstants = swerve.getConstants();
        this.noteTranslationSupplier = noteTranslationSupplier;
    }

    public ChassisSpeeds applyStateOnInputsSpeeds(AimAssist aimAssistState, ChassisSpeeds inputSpeeds) {
        return switch (aimAssistState) {
            case SPEAKER -> handleSpeakerState(
                    inputSpeeds,
                    robotPoseSupplier,
                    robotPose -> FieldConstants.getSpeaker().toTranslation2d().minus(robotPose.getTranslation()).getAngle(),
                    swerveConstants
            );
            case AMP -> handleAmpState(
                    inputSpeeds,
                    robotPoseSupplier,
                    (robotPose -> Rotation2d.fromDegrees(90)),
                    swerveConstants
            );
            case NOTE -> handleNoteAssistState(
                    inputSpeeds,
                    robotPoseSupplier,
                    noteTranslationSupplier
            );
            case NONE -> inputSpeeds;
            default -> inputSpeeds;
        };
    }


    private ChassisSpeeds handleNoteAssistState(ChassisSpeeds inputSpeeds, Supplier<Pose2d> robotPoseSupplier,
            Supplier<Translation2d> noteTranslationSupplier) {


        Rotation2d robotAngle = robotPoseSupplier.get().getRotation();
        Pose2d robotSpeeds = new Pose2d(
                inputSpeeds.vxMetersPerSecond,
                inputSpeeds.vyMetersPerSecond,
                Rotation2d.fromRadians(inputSpeeds.omegaRadiansPerSecond)
        );
        Translation2d rotatedNoteTranslation =
                noteTranslationSupplier.get().minus(
                        robotPoseSupplier.get().getTranslation()
                ).rotateBy(robotPoseSupplier.get().getRotation());


        Logger.recordOutput("note position", noteTranslationSupplier.get());
        Logger.recordOutput("rotated note position", rotatedNoteTranslation);

        robotSpeeds.rotateBy(robotAngle);


        return inputSpeeds

    }

    private ChassisSpeeds handleAmpState(ChassisSpeeds inputSpeeds, Supplier<Pose2d> robotPoseSupplier,
            Function<Pose2d, Rotation2d> targetRotationSupplier, SwerveConstants swerveConstants) {
        return getRotationAssistedSpeeds(
                inputSpeeds,
                swerve.getRobotRelativeVelocity(),
                robotPoseSupplier,
                targetRotationSupplier,
                swerveConstants
        );
    }

    private ChassisSpeeds handleSpeakerState(ChassisSpeeds inputSpeeds,
            Supplier<Pose2d> robotPoseSupplier,
            Function<Pose2d, Rotation2d> targetRotationSupplier, SwerveConstants swerveConstants) {
        return getRotationAssistedSpeeds(
                inputSpeeds,
                swerve.getRobotRelativeVelocity(),
                robotPoseSupplier,
                targetRotationSupplier,
                swerveConstants
        );
    }

}
