package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.util.Optional;

public class PathToTag extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private Command pathCommand;

    private final int[] allowedTags = {10, 26};

    public PathToTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    private boolean isAllowedTag(int tid){ 
        for (int tag : allowedTags) {
            if (tag == tid) return true;
        }
        return false;
    }

    @Override
    public void initialize() {
        System.out.println("PathToTag started");

        // Check for target
        if (!LimelightHelpers.getTV("limelight")) {
            System.out.println("No target");
            cancel();
            return;
        }

        int tid = (int) LimelightHelpers.getFiducialID("limelight");

        if (!isAllowedTag(tid)) {
            System.out.println("Tag not allowed: " + tid);
            cancel();
            return;
        }

        // Start pose from drivetrain (more reliable than vision)
        Pose2d startPose = drivetrain.getState().Pose;

        // Calculate target pose (FIXED)
        Pose2d targetPose = calculateTargetPose(tid);

        // Get current speed for smoother path start
        double currentSpeed = Math.hypot(
            drivetrain.getState().Speeds.vxMetersPerSecond,
            drivetrain.getState().Speeds.vyMetersPerSecond
        );

        // Generate path
        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(startPose, targetPose),
            new PathConstraints(2.0, 2.0, Math.PI, Math.PI),
            new IdealStartingState(currentSpeed, startPose.getRotation()),
            new GoalEndState(0.0, targetPose.getRotation())
        );

        // Prevent unwanted flipping
        path.preventFlipping = true;

        // Follow path
        pathCommand = AutoBuilder.followPath(path);
        pathCommand.schedule();
    }

    private Pose2d calculateTargetPose(int tid) {
        try {
            // Load field layout
            AprilTagFieldLayout layout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

            Optional<Pose3d> tagPose3d = layout.getTagPose(tid);

            if (tagPose3d.isEmpty()) {
                System.out.println("Tag pose not found");
                return drivetrain.getState().Pose;
            }

            Pose2d tagPose = tagPose3d.get().toPose2d();

            // Adjustable shooting distance
            double offsetMeters = 1.0;

            // Offset forward from tag, facing it
            Transform2d offset = new Transform2d(
                offsetMeters,
                0.0,
                Rotation2d.fromDegrees(180)
            );

            return tagPose.plus(offset);

        } catch (Exception e) {
            System.out.println("Error calculating target pose");
            return drivetrain.getState().Pose;
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PathToTag ended");
    }
}
