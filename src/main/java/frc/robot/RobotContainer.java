package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.DriveCommands.TurnTowardsGamePieceCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.VectorR;
import frc.robot.utils.Easings.Functions;


public class RobotContainer {
  public final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  public final XboxController auxControl = new XboxController(Constants.AUX_CONTROL_PORT);
  public final Joystick auxButtonBoard = new Joystick(Constants.AUX_BUTTON_BOARD_PORT);

  //public final DriveSubsystem drive = new DriveSubsystem();
  //public final LimelightSubsystem shooterLimelight = new LimelightSubsystem("limelight-shooter");
  public final LimelightSubsystem intakeLimelight = new LimelightSubsystem("limelight-intake");

  //public final LEDs leds = new LEDs();


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
  //drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
  }

  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    
    if (!DEBUG) {
      //CommandScheduler.getInstance().schedule(new TurnTowardsGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, mainControl));
      //drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));
      
      //leds.setDefaultCommand(new SetLEDsCommand(leds, mainControl, auxControl));

      // BUTTONS

      
      //Reset Gyro D-Pad
      new POVButton(mainControl, 0).onTrue(new ResetGyroCommand(180).andThen(new ResetDisplacementCommand(new VectorR())));
      
      //Ground Note Detection
      //new Trigger(()-> mainControl.getRightTriggerAxis() >= 0.2).onTrue(new TurnTowardsGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, mainControl));

    } 
    
    
    //DEBUG MODE
    else {
      //leds.setDefaultCommand(new RunCommand(() -> LEDs.animateLEDs(LEDPattern.STROBE_BLUE), leds));
      //drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
      
    }
  }

  public void testInit() {

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}