package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


import com.ctre.phoenix6.swerve.SwerveRequest;



public class AutoAlignToTag extends Command {

    // assists with debug informatiion to Elastic
    /*
    private final NetworkTable autoAlignTable =
    NetworkTableInstance.getDefault().getTable("AutoAlign");

    private final NetworkTableEntry activeEntry = autoAlignTable.getEntry("active");
    private final NetworkTableEntry tagAllowedEntry = autoAlignTable.getEntry("tagAllowed");
    private final NetworkTableEntry txErrorEntry = autoAlignTable.getEntry("txError");
    private final NetworkTableEntry tyErrorEntry = autoAlignTable.getEntry("tyError");
    private final NetworkTableEntry driveOutputEntry = autoAlignTable.getEntry("driveOutput");
    private final NetworkTableEntry turnOutputEntry = autoAlignTable.getEntry("turnOutput");
    private final NetworkTableEntry tagIdEntry = autoAlignTable.getEntry("tagID");
    private final NetworkTableEntry hasTargetEntry = autoAlignTable.getEntry("hasTarget");
    */
    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    int[] allowedTags = {10,26};

    @Override
    public void initialize() {
        System.out.println("AutoAlign started");
        // activeEntry.setBoolean(true);
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight", allowedTags);
    }

    private void stopAutoAlign() {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    private boolean isAllowedTag(int tid){ 
        for (int tag : allowedTags) {
            if (tag == tid)
                return true;
        }
        return false;
    }

    // 0.1 - 0.4, make negative
    double kP_turn = -0.06;
    // 0.03- 0.08
    double kP_drive = 0.07;
    // This needs to be replaced with the value given from the limelight exaclty
    // where we want it placed when shooting.
    // this will give a direct shot of the target regardless of where we are on the field.
    double targetTY = 10.3;


    public AutoAlignToTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);             
    }

    @Override
    public void execute() {

        System.out.println("AutoAlign Running");

        int tid = (int) LimelightHelpers.getFiducialID("limelight");

        if (!LimelightHelpers.getTV("limelight") || !isAllowedTag(tid)) {
            stopAutoAlign();
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        double drive = -(ty - targetTY) * kP_drive;
        double turn = tx * kP_turn;

        // Deadzone for aiming adjustment
        if (Math.abs(tx) < 0.15)
            turn = 0;

        // Deadzone for distance adjustment
        if (Math.abs(ty - targetTY) < 0.07)
            drive = 0;

        // turn and drive the robot whilst allowing strafing (ideally)
        drivetrain.setControl(
            driveRequest
                .withVelocityX(drive)
                .withVelocityY(0)
                .withRotationalRate(turn));

        // NEW debug information sent directly to Elastic (can be dragged out as widgets, also currently broken)
        /*
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        boolean tagAllowed = isAllowedTag(tid);

        hasTargetEntry.setBoolean(hasTarget);
        tagAllowedEntry.setBoolean(tagAllowed);

        tagIdEntry.setDouble(tid);

        txErrorEntry.setDouble(tx);
        tyErrorEntry.setDouble(ty - targetTY);

        autoAlignTable.getEntry("targetTY").setDouble(targetTY);

        driveOutputEntry.setDouble(drive);
        turnOutputEntry.setDouble(turn);
        */
        
        // debug
        System.out.println("tx: " + tx);
        System.out.println("ty: " + ty);
        System.out.println("Aligning to tag: " + tid);

    }

    @Override
    public void end(boolean interrupted) {
        //LimelightHelpers.SetFiducialIDFiltersOverride("limelight", null);
        stopAutoAlign();
        drivetrain.setControl(new SwerveRequest.Idle());
        System.out.println("AutoAlign ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
