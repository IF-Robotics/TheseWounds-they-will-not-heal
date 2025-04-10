package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armBackX;
import static org.firstinspires.ftc.teamcode.other.Globals.armBackY;
import static org.firstinspires.ftc.teamcode.other.Globals.armHomeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHomeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchPlaceFrontHighRightChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class RetractAfterWallIntake extends SequentialCommandGroup {

    public RetractAfterWallIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem) {
        addCommands(
                //grab the specimen
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                new WaitCommand(200),
                //flip up the intake
//                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchPlaceFrontHighRightChamber, rollFrontHighChamber),
                new InstantCommand(()->armSubsystem.setSlide(12)),
                secondaryArmSubsystem.setPitchSafe(0),
                new WaitForArmCommand(armSubsystem, 46, 10).withTimeout(500),
                new ParallelCommandGroup(
                    new ArmCoordinatesCommand(armSubsystem, armBackX, armBackY), //wait for secondary arm yaw to clear nautilus
                    new WaitCommand(50).andThen(secondaryArmSubsystem.setPitchSafe(0)), //wait for sample to rotate
                    new InstantCommand(()->intakeSubsystem.setDiffy(20, 0))
                )

        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
