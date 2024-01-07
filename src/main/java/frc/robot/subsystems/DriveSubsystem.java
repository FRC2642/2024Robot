// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModules;
import frc.robot.utils.TimedVectorDerivative;
import frc.robot.utils.VectorR;

public class DriveSubsystem extends SubsystemBase {

  // COMPONENTS
  public final SwerveModules modules;
  private static AHRS gyro;

  // POSITION TRACKING
  private static VectorR increment;
  private static VectorR displacement;
  private static VectorR velocity;
  private static TimedVectorDerivative acceleration;
  private static TimedVectorDerivative jerk;

  // OTHER
  private boolean defensiveMode = true;
  private static double yawOffsetDegrees = 0;

  public DriveSubsystem() {
    modules = new SwerveModules(
        new SwerveModule(Constants.FRONT_RIGHT), new SwerveModule(Constants.FRONT_LEFT),
        new SwerveModule(Constants.BACK_RIGHT), new SwerveModule(Constants.BACK_LEFT));

    gyro = new AHRS();
    gyro.calibrate();

    increment = new VectorR();
    displacement = new VectorR();
    velocity = new VectorR();
    acceleration = new TimedVectorDerivative(velocity);
    jerk = new TimedVectorDerivative(acceleration);
  }

  /*
   * SYSTEM STANDARD FOLLOWS COORDINATE PLANE STANDARD
   * positive (+) = forwards/left/left turn CCW
   * negative (-) = backwards/right/right turn CW
   * velocity magnitude (0-1) 1:fastest 0:stopped
   * turn (0-1)
   * NOTE: the speed of any wheel can reach a maximum of turn + |velocity|
   */
  public void move(VectorR directionalSpeed, double turnSpeed) {

    velocity.setFromCartesian(0, 0);
    increment.setFromCartesian(0, 0);
    
    VectorR directionalPull = directionalSpeed.clone();
    directionalPull.rotate(-getYawDegrees());

    for (SwerveModule module : modules) {

      VectorR rotationalPull = VectorR.fromPolar(turnSpeed, module.info.MODULE_TANGENT_DEG);
      VectorR wheelPull = VectorR.addVectors(directionalPull, rotationalPull);

      module.update(wheelPull.getMagnitude(), wheelPull.getAngle());

      // position tracking
      var inc = module.getPositionIncrement();
      
      inc.mult(1d / 4d);
      inc.rotate(getYawDegrees());
      displacement.add(inc);
      increment.add(inc);

      var velocityMeasured = module.getVelocity();
      velocityMeasured.mult(1d / 4d);
      velocityMeasured.rotate(getYawDegrees());
      velocity.add(velocityMeasured);
    }
    acceleration.update();
    jerk.update();
  }
  
  public void stop() {
    for (SwerveModule module : modules) {
      if (defensiveMode)
        module.stopDefensively();
      else
        module.stop();
    }

    velocity.setFromCartesian(0, 0);
    acceleration.update();
    jerk.update();
  }

  /*
   * public void debugWheelDirections(double angle) {
   * modules.frontRight.update(0.25, angle);
   * modules.frontLeft.update(0.25, angle);
   * modules.backRight.update(0.25, angle);
   * modules.backLeft.update(0.25, angle);
   * }
   */

  public void setDefensiveMode(boolean activated) {
    defensiveMode = activated;
  }
  public boolean getDefensiveMode() {
    return defensiveMode;
  }

  public static void resetDisplacement(VectorR v) {
    displacement.setFromCartesian(v.getX(), v.getY());
  }



  // POSITION DATA
  public static VectorR getRelativeIncrement() {
    return increment.clone();
  }
  
  public static VectorR getRelativeFieldPosition() {
    return displacement.clone();
  }

  public static VectorR getRelativeVelocity() {
    return velocity.clone();
  }

  public static VectorR getRelativeAcceleration() {
    return acceleration.clone();
  }

  public static VectorR getRelativeJerk() {
    return jerk.clone();
  }

  /*
   * positive (+) = left turn CCW
   * negative (-) = right turn CW
   */
  public static double getYawDegrees() {
    return -1 * gyro.getYaw() + yawOffsetDegrees;
  }

  // + LEFT
  public static double getRollDegrees() {
    return gyro.getRoll();
  }

  public static double getPitchDegrees() {
    return gyro.getPitch();
  }

  public static void resetGyro(double yawDegrees) {
    gyro.reset();
    gyro.calibrate();
    yawOffsetDegrees = yawDegrees;
  }
  
  public void resetDriveEncoders() {
    for (var mod : modules)
      mod.resetDriveEncoder();
  }

  @Override
  public void periodic() {
    modules.debugSmartDashboard();

    SmartDashboard.putNumber("pitch:", getPitchDegrees());
    SmartDashboard.putNumber("roll:", getRollDegrees());
    SmartDashboard.putNumber("gyro", getYawDegrees());
   // modules.debugSmartDashboard();


     SmartDashboard.putNumber("x field", displacement.getX());
     SmartDashboard.putNumber("y field", displacement.getY());

    // SmartDashboard.putNumber("distance [ft]",
    // getRelativeFieldPosition().getMagnitude());
    // SmartDashboard.putNumber("speed [ft/sec]",
    // getRelativeVelocity().getMagnitude());
    // SmartDashboard.putNumber("angle [degrees]",
    // Math.toDegrees(getRelativeFieldPosition().getAngle()));
    // SmartDashboard.putNumber("speed [ft/s]",
    // getRelativeVelocity().getMagnitude());
    // SmartDashboard.putNumber("accell [ft/s^2]",
    // getRelativeAccelleration().getMagnitude());
    // SmartDashboard.putNumber("jerk [ft/s^3]", getRelativeJerk().getMagnitude());
  }
}
