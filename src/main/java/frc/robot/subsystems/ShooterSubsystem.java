package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Subsystem that handles shooting mechanism:
 * - Dragger motor
 * - Pull-up motor
 * - Flywheel motors 1 & 2
 */
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX dragger;
    private final TalonFX pullUp;
    private final TalonFX flyWheel1;
    private final TalonFX flyWheel2;

    private static final int CAN_ID_DRAGGER = 19;
    private static final int CAN_ID_PULLUP = 32;
    private static final int CAN_ID_FLYWHEEL1 = 33;
    private static final int CAN_ID_FLYWHEEL2 = 35;

    private static final double DRAGGER_SPEED = 0.15;
    private static final double PULLUP_SPEED = -0.3;
    private static final double FLYWHEEL_SPEED = -0.8;

    private static final double DRAGGER_SPEED_R = -0.15;
    private static final double PULLUP_SPEED_R = 0.3;
    private static final double FLYWHEEL_SPEED_R = 0.4; 

    public ShooterSubsystem() {
        dragger = new TalonFX(CAN_ID_DRAGGER);
        pullUp = new TalonFX(CAN_ID_PULLUP);
        flyWheel1 = new TalonFX(CAN_ID_FLYWHEEL1);
        flyWheel2 = new TalonFX(CAN_ID_FLYWHEEL2);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        dragger.getConfigurator().apply(config);
        pullUp.getConfigurator().apply(config);
        flyWheel1.getConfigurator().apply(config);
        flyWheel2.getConfigurator().apply(config);
    }

    /** Starts flywheel motors immediately */
    public void startFlywheels() {
        flyWheel1.set(-FLYWHEEL_SPEED);
        flyWheel2.set(FLYWHEEL_SPEED);
    }

    /** Starts dragger + pullUp feeder */
    public void startFeeder() {
        dragger.set(DRAGGER_SPEED);
        pullUp.set(PULLUP_SPEED);
    }

    /** Stops the shooter motors */
    public void stopShooter() {
        dragger.set(0);
        pullUp.set(0);
        flyWheel1.set(0);
        flyWheel2.set(0);
    }

    public void feederReverse(){
        dragger.set(DRAGGER_SPEED_R);
        pullUp.set(PULLUP_SPEED_R);}
    public void flywheelReverse(){
        flyWheel1.set(-FLYWHEEL_SPEED_R);
        flyWheel2.set(FLYWHEEL_SPEED_R);
        
    }
    public void draggerReverse() {
        dragger.set(DRAGGER_SPEED_R);
    }
    public void stopDragger() {
        dragger.set(0);
    }

    /** Returns a command that runs the shooter while held */
    public Command getShootCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> startFlywheels()),
            new WaitCommand(1.5),
            Commands.run(() -> startFeeder(), this)
        ).finallyDo(interrupted -> stopShooter());
    }

    public Command reverseShootCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> flywheelReverse())
        ).finallyDo(interrupted -> stopShooter());

    }

    public Command getRDCommand(){
        return Commands.run(() -> draggerReverse(), this)
                      .until(() -> false)
                      .finallyDo(interrupted -> stopDragger());
    }
}
