package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Subsystem that handles shooting mechanism:
 * - Dragger motor
 * - Pull-up motor
 * - Flywheel motor
 */
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX dragger;
    private final TalonFX pullUp;
    private final TalonFX flyWheel;

    private static final int CAN_ID_DRAGGER = 31;
    private static final int CAN_ID_PULLUP = 32;
    private static final int CAN_ID_FLYWHEEL = 33;

    private static final double DRAGGER_SPEED = 0.2;
    private static final double PULLUP_SPEED = -0.3;
    private static final double FLYWHEEL_SPEED = -0.4;

    private static final double DRAGGER_SPEED_R = -0.2;
    private static final double PULLUP_SPEED_R = 0.3;
    private static final double FLYWHEEL_SPEED_R = 0.4; 

    public ShooterSubsystem() {
        dragger = new TalonFX(CAN_ID_DRAGGER);
        pullUp = new TalonFX(CAN_ID_PULLUP);
        flyWheel = new TalonFX(CAN_ID_FLYWHEEL);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        dragger.getConfigurator().apply(config);
        pullUp.getConfigurator().apply(config);
        flyWheel.getConfigurator().apply(config);
    }

    /** Starts the shooter motors */
    public void startShooter() {
        dragger.set(DRAGGER_SPEED);
        pullUp.set(PULLUP_SPEED);
        flyWheel.set(FLYWHEEL_SPEED);
    }

    /** Stops the shooter motors */
    public void stopShooter() {
        dragger.set(0);
        pullUp.set(0);
        flyWheel.set(0);
    }

    public void stopDragger() {
        dragger.set(0);
     }

        public void draggerReverse() {
            dragger.set(DRAGGER_SPEED_R);
    }

    /** Returns a command that runs the shooter while held */
    public Command getShootCommand() {
    return Commands.run(() -> startShooter(), this)
                    .until(() -> false)
                   .finallyDo(interrupted -> stopShooter());
    }


  public Command getRDCommand() {
    return Commands.run(() -> draggerReverse(), this)
                    .until(() -> false)
                   .finallyDo(interrupted -> stopDragger());
    }

}
