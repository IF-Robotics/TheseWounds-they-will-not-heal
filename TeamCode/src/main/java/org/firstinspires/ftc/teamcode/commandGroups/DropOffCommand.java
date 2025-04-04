package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armBackX;
import static org.firstinspires.ftc.teamcode.other.Globals.armBackY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchPlaceFrontHighRightChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenBasket;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class DropOffCommand extends SequentialCommandGroup {

    public DropOffCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem) {
        addCommands(
                new InstantCommand(()-> intakeSubsystem.clawExtraOpen()),
                new WaitCommand(100),
                new InstantCommand(()->armSubsystem.setSlide(ArmSubsystem.slideRetractMin))
//
//                new InstantCommand(() -> secondaryArmSubsystem.setDiffyYaw(0)),
//                new WaitCommand(200),
//
//                new SecondaryArmCommand(secondaryArmSubsystem, 135)

        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
//
//addCommands(
//            new WaitForSlideCommand(armSubsystem, 10, 5),
//            new InstantCommand(()->armSubsystem.setArm(90)),
//        new ConditionalCommand(
//                new InstantCommand(),
//                secondaryArmSubsystem.setPitchYawSafe(160, 90)
//                .andThen(new WaitCommand(300)),
//        ()-> secondaryArmSubsystem.getPitchAngle() > 155 && secondaryArmSubsystem.getYawAngle() > 75
//        ),
//        new InstantCommand(()->intakeSubsystem.clawExtraOpen()),
//        new WaitCommand(80),
//            new InstantCommand(()->intakeSubsystem.setDiffy(-90,0)),
//        new WaitCommand(80),
//            new InstantCommand(()->secondaryArmSubsystem.setDiffyYaw(0)),
//        new WaitCommand(500),
//            new InstantCommand(()->armSubsystem.setArm(45))
//        );
