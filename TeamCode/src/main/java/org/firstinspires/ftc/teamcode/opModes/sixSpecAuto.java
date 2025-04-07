package org.firstinspires.ftc.teamcode.opModes;
import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleFast;
import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleSlow;
import org.firstinspires.ftc.teamcode.commandGroups.FlipSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointDoubleSupplierCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

@Disabled
@Autonomous(name="6+0")

public class sixSpecAuto extends AutoBase {

    private double subX = 0;
    private double subY = 7.5;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

@Override
    public void initialize() {
        super.initialize();

        intakeSubsystem.setDiffy(0,0);
        secondaryArmSubsystem.setDiffy(0, 0);


        schedule(new SequentialCommandGroup(
                new StartSpecAuto(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new SecondaryArmCommand(secondaryArmSubsystem, secondaryPitchHighChamber, secondaryYawHighChamber),
                new WaitForArmCommand(armSubsystem, Math.toDegrees(Math.atan2(autoArmFrontHighChamberY, armFrontHighChamberX)), 5)
                        .andThen(new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY)),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, autoPitchFrontHighChamber, rollFrontHighChamber),

                new DriveToPointCommand(driveSubsystem, firstHighChamberRight,5, 5).withTimeout(1500),
                //open
                new WaitCommand(100),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                new WaitCommand(100),
                new InstantCommand(() -> armSubsystem.setArm(8)),

                new DriveToPointDoubleSupplierCommand(driveSubsystem, ()-> MathUtils.clamp(subX, -16, 16), ()->-32, new Rotation2d(), 5, 5).withTimeout(1500),
                new DriveToPointCommand(driveSubsystem, new Pose2d(1, -40, new Rotation2d(-0)), 5, 5),
                new InstantCommand(()->armSubsystem.setArmX(()->9+subY)),
                new InstantCommand(()->armSubsystem.setArmY(armSubIntakeY)),
                new WaitCommand(200),
                new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem, true).withTimeout(2000),
                new ParallelCommandGroup(
                    new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem).andThen(secondaryArmSubsystem.setPitchSafe(SecondaryArmSubsystem.hardStoppedHighPitch)),
                    new DriveToPointCommand(driveSubsystem, wallPickUp, 5, 5)
                ),
                new InstantCommand(()->intakeSubsystem.openClaw()),
                secondaryArmSubsystem.intakeSub(),
                new FlipSpikes(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem, firstWallPickUp),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new WaitCommand(300),
                new InstantCommand(()->intakeSubsystem.openClaw()),
                new InstantCommand(()->armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                new DriveToPointCommand(driveSubsystem, new Pose2d(50, -56, Rotation2d.fromDegrees(-180)), 1, 5)
        ));

        //gamepad input
        while(!isStarted() && !isStopRequested()){
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                subX -= 1;
                MathUtils.clamp(subX, -8, 6);
            }

            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                subX += 1;
                subX = MathUtils.clamp(subX, -8, 6);
            }

            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                subY += 1;
                subY = MathUtils.clamp(subY, 0, 15);
            }

            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                subY -= 1;
                subY = MathUtils.clamp(subY, 0, 15);
            }

            //change spec mode
            if(currentGamepad1.touchpad && !previousGamepad1.touchpad){
                teleopSpec = !teleopSpec;
            }

            telemetry.addData("subX (-8,6)", subX);
            telemetry.addData("subY(offesetFromBarrier)(0,15)", subY);

            //specMode
            if(teleopSpec){
                telemetry.addData("specMode", "\uD83D\uDCA7\uD83D\uDCA7true\uD83D\uDC7A\uD83D\uDC7A");
            }else{
                telemetry.addData("specMode", "\uD83D\uDC72\uD83D\uDC72false⭐⭐");
            }

            telemetry.update();
        }
    }
}
