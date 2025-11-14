package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final double RPSperVolt = Constants.Elevator.RPSperVolt;

    private final double kP = Constants.Elevator.kP;
    private final double kI = Constants.Elevator.kI;
    private final double kD = Constants.Elevator.kD;
    private final double kS = Constants.Elevator.kS;
    private final double kV = Constants.Elevator.kV;
    private final double kA = Constants.Elevator.kA;
    private final double kG = Constants.Elevator.kG;

    private final double cruiseVelocity = Constants.Elevator.cruiseVelocity;
    private final double acceleration = Constants.Elevator.acceleration;

    private final CANSparkMax masterMotor =
        new CANSparkMax(Constants.Elevator.masterID, MotorType.kBrushless);

    private final CANSparkMax slaveMotor =
        new CANSparkMax(Constants.Elevator.slaveID, MotorType.kBrushless);


    private final SparkPIDController pid = masterMotor.getPIDController();
    private final RelativeEncoder encoder = masterMotor.getEncoder();

    private final DataLog log = DataLogManager.getLog();
    private final StringLogEntry elevatorLog =
        new StringLogEntry(log, "Elevator/Position");

    private double holdPosition = 0.0;


    public ElevatorSubsystem() {

        masterMotor.restoreFactoryDefaults();
        slaveMotor.restoreFactoryDefaults();

        slaveMotor.follow(masterMotor, true);

        encoder.setPosition(0);

        masterMotor.setSmartCurrentLimit(50);
        slaveMotor.setSmartCurrentLimit(50);

        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setFF(kS);

        pid.setOutputRange(-1, 1);

        pid.setSmartMotionMaxVelocity(cruiseVelocity, 0);
        pid.setSmartMotionMaxAccel(acceleration, 0);
    }


    /*Public Methods*/

    public void setPosition(double targetPosition) {
        holdPosition = targetPosition;
        pid.setReference(targetPosition, ControlType.kSmartMotion);
    }

    public void moveManual(double volts) {
        masterMotor.setVoltage(volts);
        holdPosition = encoder.getPosition(); 
    }

    public void stop() {
        masterMotor.stopMotor();
    }

    public void reset() {
        encoder.setPosition(0);
        holdPosition = 0;
    }

    public void holdElevatorPosition() {
        pid.setReference(holdPosition, ControlType.kSmartMotion);
    }


    @Override
    public void periodic() {

        holdElevatorPosition();

        SmartDashboard.putNumber("Elevator Pos", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Hold Target", holdPosition);
        SmartDashboard.putNumber("Elevator Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Elevator Motor Current", masterMotor.getOutputCurrent());

        elevatorLog.append(Double.toString(encoder.getPosition()));
    }
}
