package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLowLevel.ResetMode;
import com.revrobotics.SparkLowLevel.PersistMode;
import com.revrobotics.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final double kP = Constants.Elevator.kP;
    private final double kI = Constants.Elevator.kI;
    private final double kD = Constants.Elevator.kD;
    private final double kS = Constants.Elevator.kS;
    private final double cruiseVelocity = Constants.Elevator.cruiseVelocity;
    private final double acceleration = Constants.Elevator.acceleration;

    private final CANSparkMax masterMotor =
        new CANSparkMax(Constants.Elevator.masterID, MotorType.kBrushless);
    private final CANSparkMax slaveMotor =
        new CANSparkMax(Constants.Elevator.slaveID, MotorType.kBrushless);

    private SparkMaxConfig masterConfig;
    private SparkMaxConfig slaveConfig;
    private ClosedLoopConfig closedLoopConfig;
    private ClosedLoopController closedLoopController;

    private final DataLog log = DataLogManager.getLog();
    private final StringLogEntry elevatorLog =
        new StringLogEntry(log, "Elevator/Position");

    private double holdPosition = 0.0;

    public ElevatorSubsystem() {

        masterConfig = new SparkMaxConfig();
        slaveConfig = new SparkMaxConfig();
        closedLoopConfig = new ClosedLoopConfig();
        closedLoopController = masterMotor.getClosedLoopController();

        /* Motor Config */
        masterConfig.idleMode(com.revrobotics.SparkLowLevel.IdleMode.kBrake).voltageCompensation(12.0).smartCurrentLimit(45);

        slaveConfig.idleMode(com.revrobotics.SparkLowLevel.IdleMode.kBrake).follow(masterMotor).voltageCompensation(12.0).smartCurrentLimit(45);

        /* Closed Loop Config */
        closedLoopConfig.pid(kP, kI, kD);
        closedLoopConfig.maxMotionVelocity(cruiseVelocity);
        closedLoopConfig.maxMotionAcceleration(acceleration);
        closedLoopConfig.feedforward(kS);

        masterConfig.closedLoop.apply(closedLoopConfig);

        /* Apply configs */
        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void setPosition(double targetPosition) {
        holdPosition = targetPosition;
        closedLoopController.setReference(targetPosition, com.revrobotics.spark.ClosedLoopConfig.ControlType.kMAXMotionPosition);
    }

    public void moveManual(double volts) {
        masterMotor.setVoltage(volts);
    }

    public void stop() {
        masterMotor.stopMotor();
    }

    public void reset() {
        holdPosition = 0;
        masterMotor.getEncoder().setPosition(0);
    }

    public void holdElevatorPosition() {
        closedLoopController.setReference(holdPosition, com.revrobotics.spark.ClosedLoopConfig.ControlType.kMAXMotionPosition);
    }

    @Override
    public void periodic() {

        holdElevatorPosition();

        SmartDashboard.putNumber("Elevator Pos", masterMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Hold Target", holdPosition);
        SmartDashboard.putNumber("Elevator Velocity", masterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Elevator Motor Current", masterMotor.getOutputCurrent());

        elevatorLog.append(Double.toString(masterMotor.getEncoder().getPosition()));
    }
}
