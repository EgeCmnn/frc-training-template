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

public class ClimbSubsystem extends SubsystemBase {

    private final CANSparkMax masterMotor = new CANSparkMax(Constants.Climb.masterID, MotorType.kBrushless);

    private final CANSparkMax slaveMotor = new CANSparkMax(Constants.Climb.slaveID, MotorType.kBrushless);
    
}
