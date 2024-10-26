package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class RetractFromBasket extends SequentialCommandGroup {

    public RetractFromBasket(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                //outtake
                new IntakeCommand(intakeSubsystem, outtakeBasketPower, pitchWhenBasket, 0),
                //wait
                new WaitCommand(500),
                //move intake out of the way
                new IntakeCommand(intakeSubsystem, 0, 0, rollWhenReadyIntake),
                //retract slides
                new ArmCoordinatesCommand(armSubsystem, armBackX, armBackY),
                //wait
                new WaitCommand(0),
                //move arm down
                new ArmCoordinatesCommand(armSubsystem, armHomeX, armHomeY)

        );
        addRequirements(armSubsystem, intakeSubsystem);
    }
}
