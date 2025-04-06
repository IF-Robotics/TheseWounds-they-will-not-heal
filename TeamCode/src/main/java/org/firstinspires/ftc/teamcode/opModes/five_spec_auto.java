package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleSlow;
import org.firstinspires.ftc.teamcode.commandGroups.FlipSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.rightPreloadSpecScore;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

@Autonomous(name="5+0")
public class five_spec_auto extends Robot {

    @Override
    public void initialize(){
        super.initialize();
        slideRight.resetEncoder();
        slideLeft.resetEncoder();

        //schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber));
        manualArm = false;

//        new InstantCommand(() -> armSubsystem.setArm(90)).schedule(true);
        claw.setPosition(clawClose);
        intakeSubsystem.setDiffy(0,0);
        secondaryArmSubsystem.setDiffy(0, 0);

        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));

        schedule(new SequentialCommandGroup(
                new StartSpecAuto(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new rightPreloadSpecScore(driveSubsystem, intakeSubsystem, armSubsystem, secondaryArmSubsystem),
                new FlipSpikes(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem, firstWallPickUp),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new WaitCommand(300),
                new InstantCommand(()->intakeSubsystem.openClaw()),
                new InstantCommand(()->armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                new DriveToPointCommand(driveSubsystem, new Pose2d(50, -56, Rotation2d.fromDegrees(-180)), 1, 5)
        ));



    }


}
