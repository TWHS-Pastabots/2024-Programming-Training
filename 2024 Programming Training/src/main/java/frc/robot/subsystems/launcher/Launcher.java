package frc.robot.subsystems.launcher;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IO.DigitalInputs;
import frc.robot.Ports;

public class Launcher {

    public enum LauncherState {
        AMP(-60.5, 1.0),
        ALTAMP(-55, 0.9),
        START(0, 0.0),
        LONG(-13.25, 1.0),
        HANDOFF(9, 0.5),
        HOVER(-3, 1.0),
        TOSS(-22, .80),
        SPEAKER(-56, 1.0),
        ALTSPEAKER(-23, 1.0),
        INTERLOPE(0.0, 1.0),
        TEST(-13.25, 1.0);

        public double position;
        public double launchSpeed;

        private LauncherState(double position, double launchSpeed) {
            this.position = position;
            this.launchSpeed = launchSpeed;
        }
    }

    public enum LeBronTeam {
        CAVS(-0.25),
        LAKERS(-20);

        public double position;

        private LeBronTeam(double position) {
            this.position = position;
        }
    }

    double anglePower = 0.2;

    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor;

    private CANSparkMax lebronMotor;

    private double increment = 1.0;

    private ArmFeedforward feedForward;
    private ArmFeedforward lebronFeedForward;

    private SparkMaxPIDController pivotController1;

    private SparkMaxPIDController lebronController;

    private static RelativeEncoder encoder;
    private static AbsoluteEncoder absEncoder;

    private static RelativeEncoder boxScore;

    private DigitalInputs breakBeam;

    private boolean[] connections = new boolean[8];

    private static LauncherState launchState = LauncherState.START;
    private static LeBronTeam leBronTeam = LeBronTeam.CAVS;

    public static Launcher instance;

    public Launcher() {
        shootMotor1 = new CANSparkMax(Ports.shootMotor1, MotorType.kBrushless);
        shootMotor1.restoreFactoryDefaults();

        shootMotor1.setSmartCurrentLimit(60);
        shootMotor1.setIdleMode(IdleMode.kBrake);
        shootMotor1.setInverted(false);
        shootMotor1.burnFlash();

        shootMotor2 = new CANSparkMax(Ports.shootMotor2, MotorType.kBrushless);
        shootMotor2.restoreFactoryDefaults();

        shootMotor2.setSmartCurrentLimit(60);
        shootMotor2.setIdleMode(IdleMode.kBrake);
        shootMotor2.setInverted(false);
        shootMotor2.burnFlash();

        flicker = new CANSparkMax(Ports.flicker, MotorType.kBrushless);
        flicker.restoreFactoryDefaults();

        flicker.setSmartCurrentLimit(20);
        flicker.setIdleMode(IdleMode.kBrake);
        flicker.setInverted(false);
        flicker.burnFlash();

        pivotMotor = new CANSparkMax(Ports.pivotMotor, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setSmartCurrentLimit(60);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(true);

        pivotMotor.setOpenLoopRampRate(1);

        lebronMotor = new CANSparkMax(Ports.lebron, MotorType.kBrushless);
        lebronMotor.restoreFactoryDefaults();

        lebronMotor.setSmartCurrentLimit(20);
        lebronMotor.setIdleMode(IdleMode.kBrake);

        feedForward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

        lebronFeedForward = new ArmFeedforward(0, 0, 0);

        encoder = pivotMotor.getEncoder();

        pivotController1 = pivotMotor.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(encoder);

        pivotController1.setOutputRange(-1, 1);

        lebronController = lebronMotor.getPIDController();

        boxScore = lebronMotor.getEncoder();
        boxScore.setPositionConversionFactor(1);

        lebronController.setFeedbackDevice(boxScore);

        lebronController.setOutputRange(-1, 1);

        lebronController.setP(LauncherConstants.lebronPCoefficient);
        lebronController.setI(LauncherConstants.lebronICoefficient);
        lebronController.setD(LauncherConstants.lebronDCoefficient);

        pivotMotor.burnFlash();
        lebronMotor.burnFlash();

        breakBeam = DigitalInputs.getInstance();

    }

    public void updatePose() {
    }

    public void setPivotPower() {
        pivotMotor.set(anglePower + feedForward.calculate(absEncoder.getPosition(), 0));
    }

    public void setReversePivotPower() {
        pivotMotor.set(anglePower + feedForward.calculate(absEncoder.getPosition(), 0));
    }

    public void eject() {
        shootMotor2.set(0);
        shootMotor1.set(launchState.launchSpeed);
    }

    public void setLeBronOn() {
        lebronMotor.set(-0.5);
    }

    public void setLeBronReverse() {
        lebronMotor.set(0.5);
    }

    public void setLeBronOff() {
        lebronMotor.set(0.0);
    }

    public void setPivotOff() {
        pivotMotor.set(0.0);
    }

    public double getTestPosition() {
        return LauncherState.TEST.position;
    }

    public double getLeBronPosition() {
        return boxScore.getPosition();
    }

    public void setLauncherOn() {
        if (launchState == LauncherState.AMP) {
            shootMotor1.set(launchState.launchSpeed * 0.1);
            shootMotor2.set(launchState.launchSpeed * 0.1);
        } else if (launchState == LauncherState.ALTAMP) {
            shootMotor1.set(-launchState.launchSpeed);
            shootMotor2.set(launchState.launchSpeed * 0.1);
        } else {
            shootMotor1.set(launchState.launchSpeed);
            shootMotor2.set(launchState.launchSpeed);
        }
    }

    public void setReverseLauncherOn() {
        shootMotor1.set(-launchState.launchSpeed);
        shootMotor2.set(-launchState.launchSpeed);
    }

    public void setLauncherOff() {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
    }

    public void setFlickerOn() {
        flicker.set(1.0);
    }

    public void setFlickerReverse() {
        flicker.set(-1.0);
    }

    public void setFlickerPartial() {
        flicker.set(0.85);
    }

    public void setFlickOff() {
        flicker.set(0);
    }

    public double getLauncherPosition() {
        return encoder.getPosition();
    }

    public boolean getBreakBeam() {
        return !breakBeam.getInputs()[Ports.launcherBreakBeam];
    }

    public LauncherState getLaunchState() {
        return launchState;
    }

    public double getPivotCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getLauncherPosition() - launchState.position) < tolerance;
    }

    public void setLauncherState(LauncherState state) {
        launchState = state;
    }

    public void setLeBronTeam(LeBronTeam team) {
        leBronTeam = team;
    }

    public void increaseIncrement() {
        increment += 0.25;
    }

    public void decreaseInrement() {
        increment -= 0.25;
    }

    public void increasePosition() {
        LauncherState.TEST.position = LauncherState.TEST.position - increment;
        if (launchState == LauncherState.SPEAKER) {
            LauncherState.SPEAKER.position = LauncherState.SPEAKER.position + increment;
        } else if (launchState == LauncherState.ALTSPEAKER) {
            LauncherState.ALTSPEAKER.position = LauncherState.ALTSPEAKER.position +
                    increment;
        }
    }

    public void decreasePosition() {
        LauncherState.TEST.position = LauncherState.TEST.position + increment;
    }

    public boolean[] launcherConnections() {

        if (shootMotor1.getBusVoltage() != 0) {
            connections[0] = true;
        } else {
            connections[0] = false;
        }

        if (shootMotor1.getOutputCurrent() != 0) {
            connections[1] = true;
        } else {
            connections[1] = false;
        }

        if (shootMotor2.getBusVoltage() != 0) {
            connections[2] = true;
        } else {
            connections[2] = false;
        }

        if (shootMotor2.getOutputCurrent() != 0) {
            connections[3] = true;
        } else {
            connections[3] = false;
        }

        if (pivotMotor.getBusVoltage() != 0) {
            connections[4] = true;
        } else {
            connections[4] = false;
        }

        if (pivotMotor.getOutputCurrent() != 0) {
            connections[5] = true;
        } else {
            connections[5] = false;
        }

        if (flicker.getBusVoltage() != 0) {
            connections[6] = true;
        } else {
            connections[6] = false;
        }

        if (flicker.getOutputCurrent() != 0) {
            connections[7] = true;
        } else {
            connections[7] = false;
        }

        return connections;
    }

    public boolean hasBrownedOut() {
        return pivotMotor.getFault(FaultID.kBrownout);
    }

    public void printConnections() {
        SmartDashboard.putBoolean("shootMotor1 Voltage", connections[0]);
        SmartDashboard.putBoolean("shootMotor1 Current", connections[1]);

        SmartDashboard.putBoolean("shootMotor2 Voltage", connections[2]);
        SmartDashboard.putBoolean("shootMotor2 Current", connections[3]);

        SmartDashboard.putBoolean("Pivot Voltage", connections[4]);
        SmartDashboard.putBoolean("Pivot Current", connections[5]);

        SmartDashboard.putBoolean("Flicker Voltage", connections[6]);
        SmartDashboard.putBoolean("Flicker Current", connections[7]);
    }

    public static Launcher getInstance() {
        if (instance == null)
            instance = new Launcher();
        return instance;
    }
}
