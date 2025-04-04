package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideMiddleSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideRightSpikeFlip;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class FlipSpikes extends SequentialCommandGroup {

    public FlipSpikes(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        addCommands(
            new InstantCommand(()->driveSubsystem.enablePrecisePID(true)),
            new DriveToPointCommand(driveSubsystem, rightSideLeftSpikeFlip, 2, 5),
            new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem, 0),
            new WaitCommand(400),
            new ParallelCommandGroup(
                new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem).andThen(secondaryArmSubsystem.intakeSub()),
                new WaitCommand(450).andThen(new DriveToPointCommand(driveSubsystem, rightSideMiddleSpikeFlip, 2, 5))
            ),
            new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem, 0),
            new WaitCommand(400),
            new ParallelCommandGroup(
                new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem).andThen(secondaryArmSubsystem.intakeSub()),
                new WaitCommand(450).andThen(new DriveToPointCommand(driveSubsystem, rightSideRightSpikeFlip, 2, 5))
            ),
            new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem, 18),
            new WaitCommand(400),
            new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
            new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
        );

        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }
}
