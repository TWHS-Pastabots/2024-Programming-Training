package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IO.LED;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import frc.robot.subsystems.launcher.Launcher.LeBronTeam;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.vision.VisionTablesListener;

public class Robot extends TimedRobot {

  private Drivebase drivebase;
  private Climber climber;
  private Intake intake;
  private Launcher launcher;
  private LED litty;
  private VisionTablesListener visTables;

  private static XboxController driver;
  private static XboxController operator;

  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();
    litty = LED.getInstance();
    visTables = VisionTablesListener.getInstance();

    driver = new XboxController(0);
    operator = new XboxController(1);
    // drivebase.resetOdometry(new Pose2d(1, 1, new Rotation2d(0)));
  }

  @Override

  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    drivebase.periodic();

    visTables.printDetects();

    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

    SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());

    SmartDashboard.putString("Intake State", intake.getIntakeState().toString());
    SmartDashboard.putString("Launcher State", launcher.getLaunchState().toString());

    SmartDashboard.putBoolean("Launcher Breakbeam", launcher.getBreakBeam());
    SmartDashboard.putBoolean("Intake Breakbeam", intake.getBreakBeam());

    SmartDashboard.putBoolean("Brownout", launcher.hasBrownedOut());

    SmartDashboard.putNumber("Launcher Position", launcher.getLauncherPosition());
    SmartDashboard.putNumber("LeBron Position", launcher.getLeBronPosition());

  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    intake.updatePose();
  }

  @Override
  public void teleopInit() {
    litty.setBlue();
  }

  @Override
  public void teleopPeriodic() {
    // intake.updatePose();
    // launcher.updatePose();

    /* DRIVE CONTROLS */

    double ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    double xSpeed = drivebase.inputDeadband(driver.getLeftY());
    double rot = drivebase.inputDeadband(-driver.getRightX());

    if (driver.getAButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 180);
    } else if (driver.getBButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 270);
    } else if (driver.getYButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 0);
    } else if (driver.getXButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 90);
    } else if (driver.getLeftTriggerAxis() > 0) {
      drivebase.holdHeading(xSpeed, ySpeed);
    } else {
      drivebase.currHeading = -1;
      drivebase.drive(xSpeed, ySpeed, rot);
    }

    if (driver.getPOV() == 180) {
      launcher.setLauncherState(LauncherState.TEST);
    }

    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }

    /* INTAKE CONTROLS */

    if(operator.getRightBumper()){
      intake.setFlipperPower();
    } else if (operator.getLeftBumper()){
      intake.setReverseFlipperPower();
    } else {
      intake.setFlipperOff();
    }

    if(operator.getAButton()){
      intake.setRollerPower();
    } else if(operator.getBButton()){
      intake.setReverseRollerPower();
    } else {
      intake.setRollerOff();
    }

    if (operator.getLeftBumper()) {
      intake.setIntakeState(IntakeState.STOP);
      launcher.setLauncherState(LauncherState.HOVER);
      launcher.setLeBronTeam(LeBronTeam.CAVS);
      launcher.updatePose();
      launcher.moveLeBron();
      launcher.setLauncherOff();
      launcher.setFlickOff();
      litty.setBlue();
    }

    // *CLIMBER CONTROLS */

    if (driver.getRightBumper()) {
      climber.setClimbingPower();
    } else if (driver.getLeftBumper()) {
      climber.setReverseClimberPower();
    } else {
      climber.setClimberOff();
    }

    /* LAUNCHER CONTROLS */

    if (-operator.getRightY() > 0) {
    launcher.setPivotPower();
    } else if (-operator.getRightY() < 0) {
    launcher.setReversePivotPower();
    } else {
    launcher.setPivotOff();
    }

    if(operator.getRightTriggerAxis() > 0){
      launcher.setLauncherOn();
    } else if (operator.getLeftTriggerAxis() > 0){
      launcher.setReverseLauncherOn();
    } else {
      launcher.setLauncherOff();
    }

    if(operator.getXButton()){
      launcher.setFlickerOn();
    } else if(operator.getBButton()){
      launcher.setFlickerReverse();
    } else{
      launcher.setFlickOff();
    }

    if (operator.getPOV() == 0) {
      launcher.setLauncherState(LauncherState.SPEAKER);
    }
    if (operator.getPOV() == 90) {
      launcher.setLauncherState(LauncherState.AMP);
    }
    if (operator.getPOV() == 180) {
      launcher.setLauncherState(LauncherState.TOSS);
    }
    if (operator.getPOV() == 270) {
      launcher.setLauncherState(LauncherState.LONG);
    }

    if (operator.getXButton()) {
      intake.setReverseRollerPower();
      launcher.setFlickerReverse();
      launcher.setReverseLauncherOn();
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}