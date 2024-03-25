package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.auto.fullAutos.FourPieceCommand;
import frc.robot.commands.auto.fullAutos.FrontThreePiece;
import frc.robot.commands.auto.fullAutos.FrontTwoPiece;
import frc.robot.commands.auto.fullAutos.MoveCommand;
import frc.robot.commands.auto.fullAutos.OnePieceCommand;
import frc.robot.commands.auto.fullAutos.OnePiecePath;
import frc.robot.commands.auto.fullAutos.OptimizedThreePiece;
import frc.robot.commands.auto.fullAutos.SideTwoPiece;
import frc.robot.commands.teleop.ManualIntakeCommand;
import frc.robot.commands.teleop.ManualShooterCommand;
import frc.robot.commands.teleop.PresetSelectorCommand;
import frc.robot.commands.teleop.RobotPresetCommand;
import frc.robot.commands.teleop.DriveCommands.JoystickOrientedDriveCommand;
import frc.robot.commands.teleop.resetters.ResetDisplacementCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.VectorR;


public class RobotContainer {
  public final XboxController mainControl = new XboxController(Constants.DRIVE_CONTROL_PORT);
  public final Joystick auxButtonBoard = new Joystick(Constants.AUX_BUTTON_BOARD_PORT);

  public final DriveSubsystem drive = new DriveSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final LimelightSubsystem shooterLimelight = new LimelightSubsystem("limelight-shooter");
  public final LimelightSubsystem intakeLimelight = new LimelightSubsystem("limelight");


  public final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public static boolean DEBUG = false;
  public static double ANGLE = 0;


  public RobotContainer() {
    SmartDashboard.putNumber("DEBUG MODE", 0);
    SmartDashboard.putNumber("ANGLE", 0);
    
    
    // Auto options
    autoChooser.setDefaultOption("NO AUTO SELECTED!", new WaitCommand(15));
    
    /*autoChooser.addOption("1 Piece Stop", new OnePieceCommand(drive, shooter, shooterLimelight));
    autoChooser.addOption("1 Piece Move", new OnePiecePath(drive, shooter, shooterLimelight));
    autoChooser.addOption("2 Piece", new FrontTwoPiece(drive, shooter, intake, shooterLimelight));
    autoChooser.addOption("3 Piece", new FrontThreePiece(drive, shooter, intake, shooterLimelight));
    autoChooser.addOption("3 Piece Optimized", new OptimizedThreePiece(drive, shooter, intake, shooterLimelight));
    autoChooser.addOption("4 Piece", new FourPieceCommand(drive, shooter, intake, shooterLimelight, intakeLimelight));
    autoChooser.addOption("Side 2 Piece", new SideTwoPiece(drive, shooter, intake, shooterLimelight));
    autoChooser.addOption("Move", new MoveCommand(drive, intakeLimelight, intake));
    */
    
    SmartDashboard.putData(autoChooser);
  }
  

  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    drive.setDefaultCommand(new StopCommand(drive));
    shooter.setDefaultCommand(new RunCommand(()-> shooter.setShooter(0.0), shooter));
    intake.setDefaultCommand(new RunCommand(()-> intake.set(0.0), intake));
  }

  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    
    if (!DEBUG) {
      CommandScheduler.getInstance().schedule(new PresetSelectorCommand(mainControl, auxButtonBoard));
      CommandScheduler.getInstance().schedule(new RobotPresetCommand(drive, shooter, intake, shooterLimelight, intakeLimelight, mainControl, auxButtonBoard));
  

      //Reset Gyro D-Pad
      new POVButton(mainControl, 0).onTrue(new ResetGyroCommand(0).andThen(new ResetDisplacementCommand(new VectorR())));
      
      new JoystickButton(mainControl, 7).onTrue(new InstantCommand(()->{
        ShooterSubsystem.dataArray.add(new Double[]{ANGLE, shooterLimelight.a});
        for (int i = 0; i < ShooterSubsystem.dataArray.size(); i++){
          System.out.println(ShooterSubsystem.dataArray.get(i)[0] + ","+ShooterSubsystem.dataArray.get(i)[1]);          
        }
        System.out.println("");
      }));

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