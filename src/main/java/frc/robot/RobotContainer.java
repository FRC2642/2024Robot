package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.DriveCommands.TurnTowardsGamePieceCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.VectorR;


public class RobotContainer {
  public final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  public final Joystick auxButtonBoard = new Joystick(Constants.AUX_BUTTON_BOARD_PORT);

  public final DriveSubsystem drive = new DriveSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ElevatorSubsystem elevator = new ElevatorSubsystem();
  //public final LimelightSubsystem shooterLimelight = new LimelightSubsystem("limelight-shooter");
  public final LimelightSubsystem intakeLimelight = new LimelightSubsystem("limelight-intake");


  public final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public static boolean DEBUG = false;


  public RobotContainer() {

    SmartDashboard.putNumber("DEBUG MODE", 0);

    // Default commands

    // Auto options
    autoChooser.setDefaultOption("NO AUTO SELECTED!", new WaitCommand(5));
    
    SmartDashboard.putData(autoChooser);

    
    SmartDashboard.putData(new ResetDisplacementCommand(new VectorR()));

  }

  public void autonomousInit() {
    drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
  }

  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    
    if (!DEBUG) {
      CommandScheduler.getInstance().schedule(new PresetSelectorCommand(shooter, elevator, intake, auxButtonBoard));
      drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));
      shooter.setDefaultCommand(new ManualShooterCommand(shooter, mainControl));
      elevator.setDefaultCommand(new ManualElevatorCommand(elevator, mainControl));
      intake.setDefaultCommand(new ManualIntakeCommand(intake, mainControl));

      
      //Reset Gyro D-Pad
      new POVButton(mainControl, 0).onTrue(new ResetGyroCommand(180).andThen(new ResetDisplacementCommand(new VectorR())));
      
      //Ground Note Detection
      //new Trigger(()-> mainControl.getRightTriggerAxis() >= 0.2).onTrue(new TurnTowardsGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, mainControl));

    } 
    
    
    //DEBUG MODE
    else {
      //leds.setDefaultCommand(new RunCommand(() -> LEDs.animateLEDs(LEDPattern.STROBE_BLUE), leds));
      drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
      //ADD SUBSYSTEMS HERE
      
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}