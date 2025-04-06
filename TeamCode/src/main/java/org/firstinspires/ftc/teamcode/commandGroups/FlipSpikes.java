package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.secondaryPitchWallIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstWallPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideMiddleSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideRightSpikeFlip;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class FlipSpikes extends SequentialCommandGroup {

    public FlipSpikes(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        addCommands(
            new DriveToPointCommand(driveSubsystem, rightSideLeftSpikeFlip, 12, 5).withTimeout(1250),
            new InstantCommand(()->driveSubsystem.enablePrecisePID(true)), //so we accelerate faster
            new DriveToPointCommand(driveSubsystem, rightSideLeftSpikeFlip, 2, 5).withTimeout(500),

            new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem, 0, armReadySubIntakeX),
            new WaitCommand(500),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                        new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem, 0, armReadySubIntakeX+0.5),
                        new WaitCommand(500)
                ),
                new WaitCommand(450).andThen(new DriveToPointCommand(driveSubsystem, rightSideMiddleSpikeFlip, 2, 5).withTimeout(500))
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                        new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem, 25, armReadySubIntakeX-5.5),
                        new WaitCommand(500)
                ),
                //Wait 1000 so we dont exit boundary while we are still flipping the second sample
                new WaitCommand(1000).andThen(new DriveToPointCommand(driveSubsystem, rightSideRightSpikeFlip, 2, 5).withTimeout(500))
            ),
            new ParallelCommandGroup(
                new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem)
                        .andThen(new SecondaryArmCommand(secondaryArmSubsystem, secondaryPitchWallIntake))
                        .andThen(new InstantCommand(intakeSubsystem::clawExtraOpen)),
                new WaitCommand(1000)
                        .andThen(new InstantCommand(()->driveSubsystem.driveToPoint(firstWallPickUp)))
            ),
            new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
        );

        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }
}
