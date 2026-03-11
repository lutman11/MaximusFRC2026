package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoAlignToTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    //0.1 - 0.4
    double kP_turn = -0.02;
    //0.3 - 0.7
    double kP_drive = 0.05;
    //This needs to be replaced with the value given from the limelight exaclty where we want it placed when shooting.
    //this will give a direct shot of the target regardless of where we are on the field. 
    double targetTY = 0.0;

    public AutoAlignToTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        if (!LimelightHelpers.getTV("limelight")) {
            drivetrain.setControl(
                driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
            return;
        }

        double tx = LimelightHelpers.getTX("limelight");

        double turn = tx * kP_turn;

        if (Math.abs(tx) < 1) turn = 0;

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(turn)
        );

        System.out.println("tx: " + tx);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;

        
    }
    
}
