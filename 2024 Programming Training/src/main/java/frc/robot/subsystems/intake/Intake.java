package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.subsystems.IO.DigitalInputs;

public class Intake {

    public enum IntakeState {
        STOP(0.2),
        GROUND(-17),
        HANDOFF(-4),
        HOLD(-2.5);

        public double position;

        private IntakeState(double position) {
            this.position = position;
        }
    }

    private CANSparkMax roller;
    private CANSparkMax flipper;

    public IntakeState intakeState = IntakeState.STOP;
    public static Intake instance;

    private ArmFeedforward feedforward;
    private SparkMaxPIDController flipperController;

    private RelativeEncoder encoder;

    private DigitalInputs breakBeam;

    private boolean[] connections = new boolean[4];
    
//*INTAKE DAY ONE START*/

    //Intake Constructor
    //Should initialize and configure: one roller motor, one pivot motor
    //Things to consider: CAN ID, motor type, current limit, motor direction, motor idle mode
    //Intake IDs: shoot motors(12), pivot motor(16)

    public Intake() {
     
    }

    //Turns the rollers on
    public void setRollerPower() {}

    //Turns the rollers on in reverse
    public void setReverseRollerPower() {}

    //Turns the rollers off
    public void setRollerOff() {}

    //Flips the intake out
    public void setFlipperPower() {}

    //Flips the intake back up
    public void setReverseFlipperPower() {}

    //Turns of the intake pivot
    public void setFlipperOff() {}

//*INTAKE DAY ONE END*/

    public double getRollerCurrent() {
        return roller.getOutputCurrent();
    }

    public double getFlipperVoltage() {
        return flipper.getBusVoltage();
    }

    public double getFlipperCurrent() {
        return flipper.getOutputCurrent();
    }

    public double getFlipperPosition() {
        return encoder.getPosition();
    }

    public void updatePose() {
        flipperController.setReference(intakeState.position,
                CANSparkMax.ControlType.kPosition, 0,
                feedforward.calculate(encoder.getPosition(), 0));
    }


    public IntakeState getIntakeState() {
        return intakeState;
    }

    public boolean hasReachedPose(double tolerance) {
        return Math.abs(encoder.getPosition() - intakeState.position) < tolerance;
    }

    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public boolean getBreakBeam(){
        return !breakBeam.getInputs()[Ports.intakeBreakBeam];
    }

    public boolean[] intakeConnections() {
        if (roller.getBusVoltage() != 0) {
            connections[0] = true;
        } else {
            connections[0] = false;
        }

        if (roller.getOutputCurrent() != 0) {
            connections[1] = true;
        } else {
            connections[1] = false;
        }

        if (flipper.getBusVoltage() != 0) {
            connections[2] = true;
        } else {
            connections[2] = false;
        }

        if (flipper.getOutputCurrent() != 0) {
            connections[3] = true;
        } else {
            connections[3] = false;
        }

        return connections;
    }

    public void printConnections() {
        SmartDashboard.putBoolean("roller Voltage", connections[0]);
        SmartDashboard.putBoolean("roller Current", connections[1]);

        SmartDashboard.putBoolean("flipper Voltage", connections[2]);
        SmartDashboard.putBoolean("flipper Current", connections[3]);
    }

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }
}