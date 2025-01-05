package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class DropCommand extends SequentialCommandGroup {


    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public DropCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new WaitForArmCommand(armSubsystem, 100, 10),
                new ArmCoordinatesCommand(armSubsystem, armBackX, armBackY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, rollWhenBasket)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}