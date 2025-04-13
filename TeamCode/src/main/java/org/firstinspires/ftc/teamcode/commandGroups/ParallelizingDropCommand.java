package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmWallIntake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem;

public class ParallelizingDropCommand extends SequentialCommandGroup {


    public ParallelizingDropCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, SpecMechSubsystem specMechSubsystem) {

        addCommands(
                new SequentialCommandGroup(
                    new InstantCommand(()->intakeSubsystem.setDiffy(0,0)), //maybe set roll to 90
                    //raise arm
                    new WaitForArmCommand(armSubsystem, 80, 45),
                    //move secondaryArm to the side, wait until above the side plates
                    new InstantCommand(()-> secondaryArmSubsystem.setDiffyYaw(100))
                )
                .alongWith(
                        new InstantCommand(()-> specMechSubsystem.openClaw())
                            .andThen(new InstantCommand(()-> specMechSubsystem.setArm(specArmWallIntake)))
                )
        );
    }

    public ParallelizingDropCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem) {

        addCommands(
                new InstantCommand(()->intakeSubsystem.setDiffy(0,0)), //maybe set roll to 90
                //raise arm
                new WaitForArmCommand(armSubsystem, 80, 45),
                //move secondaryArm to the side, wait until above the side plates
                new InstantCommand(()-> secondaryArmSubsystem.setDiffyYaw(90))
        );
    }


}
