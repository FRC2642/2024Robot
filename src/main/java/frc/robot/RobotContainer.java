package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.auto.drive.StopCommand;
import frc.robot.commands.auto.fullAutos.AmpSideThreePiece;
import frc.robot.commands.auto.fullAutos.CloseFourPiece;
import frc.robot.commands.auto.fullAutos.DisruptorAuto;
import frc.robot.commands.auto.fullAutos.FivePiece;
import frc.robot.commands.auto.fullAutos.FourPieceFromMiddleCommand;
import frc.robot.commands.auto.fullAutos.FourPieceToMiddleCommand;
import frc.robot.commands.auto.fullAutos.MiddleNotesCommand;
import frc.robot.commands.auto.fullAutos.MoveCommand;
import frc.robot.commands.auto.fullAutos.OnePieceCommand;
import frc.robot.commands.auto.fullAutos.OnePiecePath;
import frc.robot.commands.teleop.LedCommand;
import frc.robot.commands.teleop.ManualIntakeCommand;
import frc.robot.commands.teleop.ManualShooterCommand;
import frc.robot.commands.teleop.PresetSelectorCommand;
import frc.robot.commands.teleop.RobotPresetCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LedSubsystem.LEDColor;
import frc.robot.utils.VectorR;


public class RobotContainer {
  public final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  public final Joystick auxButtonBoard = new Joystick(Constants.AUX_BUTTON_BOARD_PORT);

  public final DriveSubsystem drive = new DriveSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final LimelightSubsystem shooterLimelight = new LimelightSubsystem("limelight-shooter");
  public final LimelightSubsystem intakeLimelight = new LimelightSubsystem("limelight-intake");
  public final LedSubsystem leds = new LedSubsystem();

  
  public final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public static boolean DEBUG = false;
  public static double ANGLE = 0;
  public static double OFFSET = 0;


  public RobotContainer() {
    LedSubsystem.setColor(LEDColor.GREEN);

    SmartDashboard.putNumber("DEBUG MODE", 0);
    SmartDashboard.putNumber("ANGLE", 0);
    
    
    // Auto options
    autoChooser.setDefaultOption("NO AUTO SELECTED!", new WaitCommand(15));
    
    autoChooser.addOption("1 Piece Stop", new OnePieceCommand(drive, shooter, shooterLimelight));
    autoChooser.addOption("1 Piece Move", new OnePiecePath(drive, shooter, shooterLimelight));
    autoChooser.addOption("Close 4 Piece", new CloseFourPiece(drive, shooter, intake, shooterLimelight, intakeLimelight));
    autoChooser.addOption("5 Piece", new FivePiece(drive, shooter, intake, shooterLimelight, intakeLimelight));
    autoChooser.addOption("Middle Notes", new MiddleNotesCommand(drive, shooter, intake, shooterLimelight, intakeLimelight));
    autoChooser.addOption("Disruptor", new DisruptorAuto(drive, shooter, intake, shooterLimelight, intakeLimelight));
    autoChooser.addOption("FourPieceToMiddle", new FourPieceToMiddleCommand(drive, shooter, intake, shooterLimelight, intakeLimelight));
    autoChooser.addOption("FourPieceFromMiddle", new FourPieceFromMiddleCommand(drive, shooter, intake, shooterLimelight, intakeLimelight));

    SmartDashboard.putData(autoChooser);
  }
  

  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    drive.setDefaultCommand(new RunCommand(()->drive.stop(), drive));
    shooter.setDefaultCommand(new RunCommand(()-> shooter.setManual(0.0), shooter));
    intake.setDefaultCommand(new RunCommand(()-> intake.set(0.0), intake));
  }

  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    
    
    if (!DEBUG) {
      CommandScheduler.getInstance().schedule(new PresetSelectorCommand(mainControl, auxButtonBoard));
      CommandScheduler.getInstance().schedule(new RobotPresetCommand(drive, shooter, intake, shooterLimelight, intakeLimelight, mainControl, auxButtonBoard));
      CommandScheduler.getInstance().schedule(new LedCommand(leds));

      //Reset Gyro and Displacement D-Pad
      new POVButton(mainControl, 0).onTrue(new ResetGyroCommand(0).andThen(new ResetDisplacementCommand(new VectorR())));
      //Adjust shooter angle up
      new POVButton(mainControl, 90).onTrue(new InstantCommand(()->OFFSET+=0.5));
      //Adjust shooter angle down
      new POVButton(mainControl, 270).onTrue(new InstantCommand(()->OFFSET-=0.5));

      //Data button
      /*new JoystickButton(mainControl, 7).onTrue(new InstantCommand(()->{
        ShooterSubsystem.dataArray.add(new Double[]{ANGLE, shooterLimelight.a});
        for (int i = 0; i < ShooterSubsystem.dataArray.size(); i++){
          System.out.println(ShooterSubsystem.dataArray.get(i)[0] + ","+ShooterSubsystem.dataArray.get(i)[1]);          
        }
        System.out.println("");
      }));*/





    } 
    
    //DEBUG MODE
    else {
      drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, mainControl));
      intake.setDefaultCommand(new ManualIntakeCommand(intake, mainControl));
      shooter.setDefaultCommand(new ManualShooterCommand(shooter, mainControl));     
      
      new JoystickButton(mainControl, 7).onTrue(new InstantCommand(()->{
        ShooterSubsystem.dataArray.add(new Double[]{ANGLE, shooterLimelight.a});
        for (int i = 0; i < ShooterSubsystem.dataArray.size(); i++){
          System.out.println(ShooterSubsystem.dataArray.get(i)[0] + ","+ShooterSubsystem.dataArray.get(i)[1]);          
        }
        System.out.println("");
      }));

    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}