package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class Commands {

    private final ElevatorSubsystem elevator;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ClimbSubsystem climb;

    public Commands(
        ElevatorSubsystem elevator,
        ShooterSubsystem shooter,
        IntakeSubsystem intake,
        ClimbSubsystem climb
    ) {
        this.elevator = elevator;
        this.shooter = shooter;
        this.intake = intake;
        this.climb = climb;
    }

    // -------------------------------------------------------
    //  L1 / L2 / L3 / L4 SCORE KOMUTLARI
    // -------------------------------------------------------

    public Command l1Score() {
        return new SequentialCommandGroup(
            // Elevator'ı L1 skor yüksekliğine götür
            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.L1_SCORE),
                elevator
            ),
            new WaitCommand(0.4),

            // L1 için shooter
            new InstantCommand(
                shooter::scoreL1,
                shooter
            ),
            new WaitCommand(0.3),

            // Shooter durdur
            new InstantCommand(
                shooter::stop,
                shooter
            ),

            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.HOLD),
                elevator //buradan cok emin degilim tam holdu anlamadim
            )
        );
    }


    public Command l2Score() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.L2),
                elevator
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::scoreL2L3,
                shooter
            ),
            new WaitCommand(0.3),

            new InstantCommand(
                shooter::stop,
                shooter
            ),

            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.HOLD),
                elevator
            )
        );
    }


    public Command l3Score() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.L3),
                elevator
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::scoreL2L3,
                shooter
            ),
            new WaitCommand(0.3),

            new InstantCommand(
                shooter::stop,
                shooter
            ),

            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.HOLD),
                elevator
            )
        );
    }


    public Command l4Score() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.L4_SCORE),
                elevator
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::scoreL4,
                shooter
            ),
            new WaitCommand(0.3),

            new InstantCommand(
                shooter::stop,
                shooter
            ),

            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.HOLD),
                elevator
            )
        );
    }

    /** 2. seviyedeki Algae'yi yerinden çıkarma ve tutma */
    public Command getL2Algae() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.L2_ALGAE),
                elevator
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::algae,
                shooter
            ),
            new WaitCommand(0.5),

            new InstantCommand(
                shooter::algaeHold,
                shooter
            )
        );
    }

    /** 3. seviyedeki Algae'yi yerinden çıkarma ve tutma */
    public Command getL3Algae() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.L3_ALGAE),
                elevator
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::algae,
                shooter
            ),
            new WaitCommand(0.5),

            new InstantCommand(
                shooter::algaeHold,
                shooter
            )
        );
    }


    public Command shootAlgaeToBarge() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.ALGAE_RELEASE),
                elevator
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::algaeBarge,
                shooter
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::stop,
                shooter
            )
        );
    }


    public Command shootAlgaeToProcessor() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevator.setPosition(Constants.Climb.Levels.ALGAE_RELEASE),
                elevator
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::algaeOut,
                shooter
            ),
            new WaitCommand(0.4),

            new InstantCommand(
                shooter::stop,
                shooter
            )
        );
    }

    // -------------------------------------------------------
    //  CORAL INTAKE COMMAND
    // -------------------------------------------------------


    public Command intakeCoral() {
        return new SequentialCommandGroup(

            new ParallelCommandGroup(
                new InstantCommand(
                    () -> elevator.setPosition(Constants.Climb.Levels.SAFE),
                    elevator
                ),
                new InstantCommand(
                    intake::intake,
                    intake
                )
            ),
            new WaitCommand(0.7),
            new InstantCommand(
                intake::stop,
                intake
            )
        );
    }

    // -------------------------------------------------------
    //  CLIMB COMMANDS
    // -------------------------------------------------------


    public Command openClimb() {
        return new RunCommand(
            climb::openFastClimb,
            climb
        );
    }


    public Command closeClimb() {
        return new RunCommand(
            climb::closeFastClimb,
            climb
        );
    }

    public Command holdClimb() {
        return new InstantCommand(
            climb::hold,
            climb
        );
    }


    public Command stopClimb() {
        return new InstantCommand(
            climb::stop,
            climb
        );
    }
}
