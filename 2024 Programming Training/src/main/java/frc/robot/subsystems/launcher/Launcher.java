package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.FaultID;

import frc.robot.subsystems.IO.DigitalInputs;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Ports;

public class Launcher {

    public enum LauncherState {
        // AMP(-48.5, 1.0),
        AMP(-60.5, 1.0),
        ALTAMP(-55, 0.9),
        START(0, 0.0),
        TRAP(-70.04991149902344, 0.8),
        LONG(-13.25, 1.0),
        HANDOFF(9, 0.5),
        HOVER(-3, 1.0),
        TOSS(-22, .80),
        AUTOMIDSHOT(-12, 1.0),
        AUTOLEFTSHOT(-13.5, 1.0),
        AUTORIGHTSHOT(-13.5, 1.0),
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

    private SparkMaxPIDController pivotController;
    private SparkMaxPIDController lebronController;

    private RelativeEncoder encoder;
    private RelativeEncoder boxScore;

    private ArmFeedforward feedForward;

    private DigitalInputs breakBeam;

    private boolean[] connections = new boolean[10];

    private static LauncherState launchState = LauncherState.LONG;
    private static LeBronTeam leBronTeam = LeBronTeam.CAVS;

    public static Launcher instance;


//*LAUNCHER DAY ONE START*/

    //Launcher constructor
    //Should intialize and configure: two shoot motors, one amp mechanism motor, one pivot motor
    //Things to consider: CAN ID, motor type, current limit, motor direction, motor idle mode
    //Launcher IDs: shoot motors(9, 13), pivot motor(10), amp motor(17), flicker(14)

    public Launcher() {

        //*Write here */

        //* */


        //* Ignore this */
        encoder = pivotMotor.getEncoder();

        pivotController = pivotMotor.getPIDController();

        pivotController.setP(LauncherConstants.pivotPCoefficient);
        pivotController.setI(LauncherConstants.pivotICoefficient);
        pivotController.setD(LauncherConstants.pivotDCoefficient);

        pivotController.setFeedbackDevice(encoder);

        pivotController.setOutputRange(-1, 1);

        feedForward = new ArmFeedforward(0.012, 0.017, 0.0, 0.0);

        lebronController = lebronMotor.getPIDController();

        boxScore = lebronMotor.getEncoder();
        boxScore.setPositionConversionFactor(1);

        lebronController.setFeedbackDevice(boxScore);

        lebronController.setOutputRange(-1, 1);

        lebronController.setP(LauncherConstants.lebronPCoefficient);
        lebronController.setI(LauncherConstants.lebronICoefficient);
        lebronController.setD(LauncherConstants.lebronDCoefficient);

        //* Ignore this */
    }

    //moves the launcher down
    public void setPivotPower() {}

    //Moves the launcher up
    public void setReversePivotPower() {}

    //Softly spits out ring
    public void eject() {}

    //Moves the amp mechanism down
    public void setLeBronOn() {}

    //Moves the amp mechanism up
    public void setLeBronReverse() {}

    //Turns off the amp mechanism
    public void setLeBronOff() {}

    //Turns off the launcher pivot
    public void setPivotOff() {}

    //Turns shoot motors on
    public void setLauncherOn() {}

    //Turns shoot motors on in reverse
    public void setReverseLauncherOn() {}

    //Turns shoot motors off
    public void setLauncherOff() {}

    //Turns flicker on
    public void setFlickerOn() {}

    //Turns flicker on in reverse
    public void setFlickerReverse() {}

    //Turns flicker off
    public void setFlickOff() {}


//* LAUNCHER DAY ONE END*/


    public double getLauncherPosition(){
        return encoder.getPosition();
    }

    public double getLeBronPosition(){
        return boxScore.getPosition();
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
        return false;
    }

    public void setLauncherState(LauncherState state) {
        launchState = state;
    }

    public void setLeBronTeam(LeBronTeam team) {
        leBronTeam = team;
    }
 
    public void updatePose(){
        pivotController.setReference(launchState.position, CANSparkMax.ControlType.kPosition, 0,
        feedForward.calculate(encoder.getPosition(), 0));

        lebronController.setReference(leBronTeam.position, CANSparkMax.ControlType.kPosition, 0);
}

    public void moveLeBron() {}

    public void setFlickerPartial(){
        flicker.set(0.5);
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

        if (flicker.getBusVoltage() != 0) {
            connections[7] = true;
        } else {
            connections[7] = false;
        }

        if(lebronMotor.getBusVoltage() != 0){
            connections[8] = true;
        } else {
            connections[8] = false;
        }

        if(lebronMotor.getOutputCurrent() != 0){
            connections[9] = true;
        } else {
            connections[9] = false;
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
