package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.auto.fullAutos.AmpFive;
import frc.robot.commands.auto.fullAutos.ChargeAmpN2Pc;
import frc.robot.commands.auto.fullAutos.HPFive;
import frc.robot.commands.auto.fullAutos.HPSixAndHalf;
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.commands.auto.positionable.SetShooterCommand;
import frc.robot.commands.teleop.ManualElevatorCommand;
import frc.robot.commands.teleop.ManualIntakeCommand;
import frc.robot.commands.teleop.ManualShooterCommand;
import frc.robot.commands.teleop.PresetSelectorCommand;
import frc.robot.commands.teleop.RobotPresetCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickTurnSpeedDriveCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterPosition;
import frc.robot.utils.VectorR;


public class RobotContainer {
  public final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  //public final XboxController auxControl = new XboxController(1);
  public final Joystick auxButtonBoard = new Joystick(Constants.AUX_BUTTON_BOARD_PORT);

  public final DriveSubsystem drive = new DriveSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public final LimelightSubsystem shooterLimelight = new LimelightSubsystem("limelight-shooter");
  //public final LimelightSubsystem intakeLimelight = new LimelightSubsystem("limelight-intake");


  public final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public static boolean DEBUG = false;


  public RobotContainer() {
    
  
    SmartDashboard.putNumber("DEBUG MODE", 0);

    // Default commands

    // Auto options
    /*autoChooser.setDefaultOption("NO AUTO SELECTED!", new WaitCommand(5));
    
    SmartDashboard.putData(autoChooser);

    
    SmartDashboard.putData(new ResetDisplacementCommand(new VectorR()));

  
    autoChooser.setDefaultOption("NO AUTO SELECTED!", new WaitCommand(5));
    /*
    autoChooser.addOption("6.5 Pc HP Side", new HPSixAndHalf(drive, shooter, intake, intakeLimelight, elevator));
    autoChooser.addOption("5 Pc HP Side", new HPFive(drive, shooter, intake, intakeLimelight, elevator));
    autoChooser.addOption("5 Pc Amp Side", new AmpFive(drive, shooter, intake, intakeLimelight, elevator));
    autoChooser.addOption("Charged Amp and 2 Pc Amp Side", new ChargeAmpN2Pc(drive, shooter, intake, intakeLimelight, elevator));
    */
    //SmartDashboard.putData(autoChooser);
  }

  public void autonomousInit() {
    //drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
    //shooter.setDefaultCommand(new RunCommand(()-> shooter.set(0.0), shooter));
    //elevator.setDefaultCommand(new RunCommand(()-> elevator.set(0.0), elevator));
    //intake.setDefaultCommand(new RunCommand(()-> intake.set(0.0), intake));
  }

  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    
    if (!DEBUG) {
      CommandScheduler.getInstance().schedule(new PresetSelectorCommand(mainControl, auxButtonBoard));
      CommandScheduler.getInstance().schedule(new RobotPresetCommand(drive, shooter, elevator, intake, shooterLimelight, mainControl, auxButtonBoard));
      //CommandScheduler.getInstance().schedule(new RobotPresetCommand(drive, shooter, elevator, intake, intakeLimelight, intakeLimelight, mainControl, auxButtonBoard));
      //drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, auxControl));
      //shooter.setDefaultCommand(new InstantCommand());
      //elevator.setDefaultCommand(new ManualElevatorCommand(elevator, mainControl));
      //intake.setDefaultCommand(new ManualIntakeCommand(intake, mainControl));
      //shooter.setDefaultCommand(new ManualShooterCommand(shooter, mainControl));
      //CommandScheduler.getInstance().schedule(new SetIntakeCommand(intake, () ->IntakePosition.RETRACTED));
      
      //Reset Gyro D-Pad
      new POVButton(mainControl, 0).onTrue(new ResetGyroCommand(0).andThen(new ResetDisplacementCommand(new VectorR())));
      //new POVButton(mainControl, 90).onTrue(new SetShooterCommand(shooter, ()->ShooterPosition.TRAVEL));
      //new POVButton(mainControl, 180).onTrue(new SetShooterCommand(shooter, ()->ShooterPosition.TRAP));
      //new POVButton(mainControl, 90).onTrue(new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED));
      //new POVButton(mainControl, 180).onTrue(new SetIntakeCommand(intake, ()->IntakePosition.RETRACTED));
       new POVButton(mainControl, 180).onTrue(new InstantCommand(()->{
        ElevatorSubsystem.resetEncoder();
      }));
      
      new POVButton(mainControl, 270).onTrue(new InstantCommand(()->{
        System.out.println("interrupt");
      }, shooter, intake));

      //Ground Note Detection
      //new Trigger(()-> mainControl.getRightTriggerAxis() >= 0.2).onTrue(new TurnTowardsGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, mainControl));

    } 
    
    
    //DEBUG MODE
    else {
      //leds.setDefaultCommand(new RunCommand(() -> LEDs.animateLEDs(LEDPattern.STROBE_BLUE), leds));
 
      
      //drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
      
      //shooter.setDefaultCommand(new ManualShooterCommand(shooter, mainControl));
      /*
      elevator.setDefaultCommand(new ManualElevatorCommand(elevator, mainControl));
      intake.setDefaultCommand(new ManualIntakeCommand(intake, mainControl));
      */
      
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}