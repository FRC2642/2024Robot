// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.DataStreamJitterDetector;

public class LimelightSubsystem extends SubsystemBase {

  private final String networkTableName;

  /*
   * Limelight Detection pipeline, NOTE: pipeline order must follow enum order
   * Pipeline 0: CONE
   * Pipeline 1: CUBE
   * Pipeline 2: FIDUCIAL
   * Pipeline 3: TAPE
   */
  public enum DetectionType {
    CONE(0),
    CUBE(3), //1
    FIDUCIAL(2),
    RETROREFLECTIVE(1),//3
    NONE(-1);

    public final int pipeline;

    private DetectionType(int pipeline) {
      this.pipeline = pipeline;
    }
  }

  public enum DetectionError {
    NOT_STARTED,
    NO_DETECTIONS,
    TV_NULL,
    NO_PIPELINES,
    UNDETERMINED,
    NO_BOTPOSE,
    SUCCESS,
  }

  private DetectionType detectionType = DetectionType.FIDUCIAL;

  public void setDetectionType(DetectionType type) {
    if (detectionType != type) {
      detectionType = type;
      jitterDetectorX.reset();
      jitterDetectorY.reset();
    }
    // jitterDetectorY.reset();
  }

  public DetectionType getDetectionType() {
    return detectionType;
  }

  /*
   * Enum which states the most recent limelight detection error, determined in
   * the update method.
   */
  private DetectionError detectionError = DetectionError.NOT_STARTED;

  public DetectionError getDetectionError() {
    return detectionError;
  }

  private final DataStreamJitterDetector jitterDetectorX = new DataStreamJitterDetector();

  private final DataStreamJitterDetector jitterDetectorY = new DataStreamJitterDetector();

  public double confidence() {
    return detectionError == DetectionError.SUCCESS
        ? (((jitterDetectorX.getConfidence()) + jitterDetectorY.getConfidence()) / 2)
        : -1.0;
  }

  // private double x;
  // private double y;
  // private double area;
  private NetworkTable limelightTable;

  private void initialize() {
    limelightTable = NetworkTableInstance.getDefault().getTable(networkTableName);
    jitterDetectorX.reset();
    jitterDetectorY.reset();
  }

  private boolean isInitialized() {
    return limelightTable != null;
  }

  // private final DataStreamFilter filterX = new DataStreamFilter(10, 2d);
  // private final DataStreamFilter filterY = new DataStreamFilter(10, 2d);

  public double x;
  public double y;
  public double a;
  public boolean isDetection = false;

  public double botposeX;
  public double botposeY;
  public double botposeZ;
  public double botposeXRot;
  public double botposeYRot;
  public double botposeZRot;

  private void reset() {
    x = 0;
    y = 0;
    a = -1;
    isDetection = false;
    botposeX = 0;
    botposeXRot = 0;
    botposeY = 0;
    botposeYRot = 0;
    botposeZ = 0;
    botposeZRot = 0;
  }

  public DetectionError update() {
    reset();
    if (!isInitialized())
      return DetectionError.NOT_STARTED;

    NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");
    if (pipeline == null)
      return DetectionError.NO_PIPELINES;
    pipeline.setDouble(detectionType.pipeline);

    NetworkTableEntry tv = limelightTable.getEntry("tv");
    if (tv == null)
      return DetectionError.TV_NULL;
    isDetection = tv.getDouble(0.0) == 1.0;
    if (!isDetection)
      return DetectionError.NO_DETECTIONS;

    DetectionError updatederror = update2DMeasurements();
    if (updatederror != DetectionError.SUCCESS)
      return updatederror;

    if (detectionType == DetectionType.FIDUCIAL)
      updatederror = updateBotposeMeasurements();
    if (updatederror != DetectionError.SUCCESS)
      return updatederror;

    if (detectionType == DetectionType.FIDUCIAL) {
      jitterDetectorX.update(botposeX);
      jitterDetectorY.update(botposeY);
    } else {
      jitterDetectorX.update(x);
      jitterDetectorY.update(y);

    }

    return DetectionError.SUCCESS;
  }

  private DetectionError updateBotposeMeasurements() {
    NetworkTableEntry botpose = limelightTable.getEntry("botpose");
    if (botpose == null)
      return DetectionError.NO_BOTPOSE;
    double[] transform = botpose.getDoubleArray(new double[6]);

    botposeX = transform[0] * Constants.FOOT_PER_METER + 27.0416;
    botposeY = transform[1] * Constants.FOOT_PER_METER + 13.2916;
    botposeZ = transform[2] * Constants.FOOT_PER_METER + 0.0;
    botposeXRot = transform[3];
    botposeYRot = transform[4];
    botposeZRot = transform[5];
    return DetectionError.SUCCESS;
  }

  public DetectionError update2DMeasurements() {
    a = limelightTable.getEntry("ta").getDouble(0);
    x = limelightTable.getEntry("tx").getDouble(0);
    y = limelightTable.getEntry("ty").getDouble(0);
    return DetectionError.SUCCESS;
  }

  public LimelightSubsystem(String networkTableName) {
    this.networkTableName = networkTableName;
  }

  @Override
  public void periodic() {
    if (isInitialized()) {
      detectionError = update();
    } else {
      initialize();
      System.out.println("LIMELIGHT---------Initializing: [" + limelightTable + "] ----------- Error: " + detectionError.toString());
    }

    SmartDashboard.putNumber(networkTableName+ "X", x);
  }
}