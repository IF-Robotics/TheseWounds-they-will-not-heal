package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmUp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem;

public class ParallelizingCommand extends SequentialCommandGroup {

    public ParallelizingCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, SpecMechSubsystem specMechSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        //specmech
                        new SequentialCommandGroup(
                                new InstantCommand(()->specMechSubsystem.closeClaw()),
                                new WaitCommand(150),
                                new InstantCommand(()->specMechSubsystem.setArm(specArmUp))
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(()->intakeSubsystem.setDiffy(15, 0)),
                                new InstantCommand(()->intakeSubsystem.clawExtraOpen()),
                                new InstantCommand(()->intakeSubsystem.openClaw()), //so that we save time when we intake later ig
                                secondaryArmSubsystem.setPitchYawSafe(0,0),
                                new WaitCommand(50),
                                new InstantCommand(()->armSubsystem.setArm(ArmSubsystem.armMinAngle))
                        )
                )
        );
    }
}
