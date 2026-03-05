// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.NamedCommands;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.LinearServo;
import frc.robot.subsystems.ChainSubsystem;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final ShooterSubsystem shooter = new ShooterSubsystem();

    private final ShooterSubsystem reverseDragger = new ShooterSubsystem();

    //private final LinearServo linearServo;

    private final ChainSubsystem battleBus = new ChainSubsystem();
    

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

     // Fuel Intake game variables
     private final TalonFX fuelIntake;
     private static final int CAN_IDFI = 30;
     private static final double INTAKE_MOTOR_SPEED = -0.3; // Change this value to adjust the motor speed
     private static final double REVERSE_INTAKE_MOTOR_SPEED = .125;
     private static final double INTAKE_AUTO = -0.25;
     private static final double INTAKE_STOP = 0;
     private static final double SHOOTER_AUTO = 0.25;
     private static final double SHOOTER_STOP = 0;

     // endgame variables
     private final SparkMax endGame;
     private static final int CAN_IDblank = 100000;
     private static final double ENDGAME_MOTOR_SPEED = 0.55; // Change this value to adjust the motor speed
 
     // slowMode variables
     private boolean slowMode = false;
     private boolean fastMode = false;
 
    public RobotContainer() {

        fuelIntake = new TalonFX(CAN_IDFI);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        fuelIntake.getConfigurator().apply(config);

        endGame = new SparkMax(CAN_IDblank, MotorType.kBrushless);
        SparkMaxConfig neoConfig2 = new SparkMaxConfig();
        neoConfig2.inverted(false).idleMode(IdleMode.kBrake);

        NamedCommands.registerCommand("scooperOn", Commands.runOnce(()->{
            fuelIntake.set(INTAKE_AUTO);
        }));
        
        NamedCommands.registerCommand("scooperStop", Commands.runOnce(()->{
            fuelIntake.set(INTAKE_STOP);
        }));
         
         NamedCommands.registerCommand("shooterOn", Commands.runOnce(()->{
            fuelIntake.set(SHOOTER_AUTO);
        }));

        NamedCommands.registerCommand("shooterStop", Commands.runOnce(()->{
            fuelIntake.set(SHOOTER_STOP);
        }));
        NamedCommands.registerCommand("limelightAuto", Commands.runOnce(()->{
            drivetrain.driveToGoal();
        }, drivetrain
        ));
        

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        //  Linear Servo Constants
         //linearServo = new LinearServo(3, 70, 0);
    }

    private void configureBindings() {

            joystick.povLeft().onTrue(Commands.runOnce(() -> {  // Use onTrue() to toggle slowMode
            slowMode = !slowMode; // Toggle slow mode
            fastMode = false;
            updateDriveModeDashboard();
        }, drivetrain));

        joystick.povRight().onTrue(Commands.runOnce(() -> {
            fastMode = !fastMode; // toggle fast mode
            slowMode = false;
            updateDriveModeDashboard();
        }, drivetrain));
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
             drivetrain.applyRequest(() -> {
                double speedFactor;
                if(slowMode){
                    speedFactor = 0.2;
                }
                else if(fastMode){
                    speedFactor = 1.0;
                }
                else{
                    speedFactor = 0.75;
                }
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed* 0.5 * speedFactor) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5 * speedFactor) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.5 * speedFactor); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.leftBumper().whileTrue(
            Commands.parallel(
            Commands.run(() -> fuelIntake.set(REVERSE_INTAKE_MOTOR_SPEED), drivetrain),
            shooter.getRDCommand())
                ).whileFalse(
            Commands.run(() -> fuelIntake.set(0), drivetrain)
        );

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on start press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Reset the field-centric heading on start press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

    /*
        // Joystick control Controlling Servo
        joystick.x().onTrue(Commands.runOnce(() -> {
            linearServo.setPosition(70.0); // extend
        }));

        joystick.y().onTrue(Commands.runOnce(() -> {
            linearServo.setPosition(0.0); // retract
        }));

          Commands.run(() -> linearServo.updateCurPos()).schedule();
    */
        
         // Intake is a motor controller that is controlled by the right bumper

        Trigger leftJoystickMoved = new Trigger(() ->
        Math.abs(joystick.getLeftX()) > 0.10 || Math.abs(joystick.getLeftY()) > 0.10);

        Trigger intakeTrigger = joystick.rightBumper().or(leftJoystickMoved.and(joystick.rightBumper()));

        intakeTrigger.whileTrue(Commands.run(() -> fuelIntake.set(INTAKE_MOTOR_SPEED)))
            .whileFalse(Commands.run(() -> fuelIntake.set(0)));

        joystick.rightTrigger()
        .whileTrue(shooter.getShootCommand());

        joystick.leftTrigger(0.7)
        .onTrue(drivetrain.driveToGoalCommand());

        joystick.rightStick()
        .onTrue(battleBus.intakeDrop());

        joystick.leftStick()
        .onTrue(battleBus.intakeLift());
        
    }

   /*    
       // Climber goes up
        joystick.povDown().whileTrue(Commands.run(() -> {
           endGame.set(ENDGAME_MOTOR_SPEED);
        }, drivetrain)).whileFalse(Commands.run(() -> {
            endGame.set(0);
        }, drivetrain));
        // Climber goes down
        joystick.povUp().whileTrue(Commands.run(() -> {
            endGame.set(-ENDGAME_MOTOR_SPEED);
        }, drivetrain)).whileFalse(Commands.run(() -> {
            endGame.set(0);
        }, drivetrain));
    }
*/

    private void updateDriveModeDashboard() {
        if (slowMode) {
        SmartDashboard.putString("Drive Mode", "Slow");
        } else if (fastMode) {
        SmartDashboard.putString("Drive Mode", "Fast");
        } else {
        SmartDashboard.putString("Drive Mode", "Normal"); // Or "Default", or whatever you want to call it
        }
     }
    
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
