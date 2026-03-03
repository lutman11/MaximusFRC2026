package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Subsystem that handles chain mechanism:
 * - Chain motor
 */

public class ChainSubsystem extends SubsystemBase

// New Kraken motor
private final TalonFX chain;
private static final int CAN_ID_SLOW = 34;   // (change to actual CAN ID) 

private static final double SLOW_FORWARD = 0.05;
private static final double SLOW_REVERSE = -0.05;

 private static final double CURRENT_LIMIT = 40.0;   // amps (tune this)
    private static final double VELOCITY_THRESHOLD = 1.0; // near zero velocity
    private static final double STALL_DELAY = 0.25; // seconds

    private final Timer stallTimer = new Timer();
    private boolean stallDetected = false;

public ChainSubsystem() { 

chain = new TalonFX(CAN_ID_CHAIN);

TalonFXConfiguration slowConfig = new TalonFXConfiguration();
slowConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
slowConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

chain.getConfigurator().apply(slowConfig);

    private void runWithProtection(double speed) {

        double current = chainMotor.getStatorCurrent().getValueAsDouble();
        double velocity = Math.abs(chainMotor.getVelocity().getValueAsDouble());

        boolean possibleStall = current > CURRENT_LIMIT && velocity < VELOCITY_THRESHOLD;

        if (possibleStall) {

            if (!stallDetected) {
                stallTimer.reset();
                stallTimer.start();
                stallDetected = true;
            }

            if (stallTimer.hasElapsed(STALL_DELAY)) {
                chainMotor.set(0); // Stop motor after delay
                return;
            }

        } else {
            stallDetected = false;
            stallTimer.stop();
            stallTimer.reset();
        }

        chainMotor.set(speed);
    }

    public void moveForward() {
        runWithProtection(CHAIN_FORWARD);
    }

    public void moveReverse() {
        runWithProtection(CHAIN_REVERSE);
    }

    public void stop() {
        chainMotor.set(0);
        stallDetected = false;
        stallTimer.stop();
        stallTimer.reset();

     // Command to run chain forward with protection
    public Command intakeDrop() {
    return Commands.run(
        () -> moveForward(),
        this
    ).finallyDo(interrupted -> stop());
}

// Command to run chain in reverse with protection
public Command intakeLift() {
    return Commands.run(
        () -> moveReverse(),
        this
    ).finallyDo(interrupted -> stop());
}
    }
}
