package org.firstinspires.ftc.teamcode.opModes;
import static org.firstinspires.ftc.teamcode.opModes.TeleopOpMode.teleopSpec;
import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmUp;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmWallIntake;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specAutoStart;

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
import org.firstinspires.ftc.teamcode.commandGroups.DropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.DropOffCommand;
import org.firstinspires.ftc.teamcode.commandGroups.FlipSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingDropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointDoubleSupplierCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightToSample;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
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
        secondaryArmSubsystem.setDiffy(90, 0);

        specMechSubsystem.closeClaw();
        specMechSubsystem.setArm(specAutoStart);


        schedule(new SequentialCommandGroup(
                new StartSpecAuto(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new ParallelCommandGroup(
                        new InstantCommand(() -> specMechSubsystem.closeClaw()),
                        new InstantCommand(() -> specMechSubsystem.setArm(specArmUp)),
                        new InstantCommand(() -> armSubsystem.setArm(25)),
                        new InstantCommand(() -> armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                        new InstantCommand(() -> secondaryArmSubsystem.setDiffy(0, 0)),
                        new InstantCommand(() ->intakeSubsystem.setDiffy(0,0)),
                        new InstantCommand(() ->intakeSubsystem.openClaw())
                ),
                new InstantCommand(()->secondaryArmSubsystem.setDiffyPitch(SecondaryArmSubsystem.hardStoppedHighPitch)),
                new WaitCommand(50),
                new InstantCommand(()->secondaryArmSubsystem.setDiffyPitch(0)),
                new WaitCommand(100),
                new DriveToPointDoubleSupplierCommand(driveSubsystem, ()->subX, ()->firstHighChamberRight.getY(), firstHighChamberRight.getRotation(), 5, 5).withTimeout(1500)
                        .alongWith(new WaitCommand(300).andThen(new InstantCommand(() -> secondaryArmSubsystem.setDiffy(0, -30)))),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new WaitCommand(400),
                        new InstantCommand(() -> specMechSubsystem.openClaw()),
                        new WaitCommand(200),
                        new InstantCommand(() -> specMechSubsystem.setArm(specArmWallIntake))
                    ),
                    new SequentialCommandGroup(
                        new WaitCommand(200),
                        new LimelightToSample(driveSubsystem, armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem).withTimeout(2000),
                        new WaitCommand(5000).interruptOn(()->Math.abs(armSubsystem.getSlideError())<0.5&&driveSubsystem.getTranslationalError()<0.5),
                        new WaitCommand(100),
                        new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
                    )
                ),
                //open
                //arm to home pos


//                new DriveToPointDoubleSupplierCommand(driveSubsystem, ()-> MathUtils.clamp(subX, -16, 16), ()->-32, new Rotation2d(), 5, 5).withTimeout(1500),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(1, -40, new Rotation2d(-0)), 5, 5),
//                new InstantCommand(()->armSubsystem.setArmX(()->9+subY)),
//                new InstantCommand(()->armSubsystem.setArmY(armSubIntakeY)),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(500),
                            new DriveToPointCommand(driveSubsystem, rightSideLeftSpikeFlip, 12, 5).withTimeout(1250),
                            new InstantCommand(()->driveSubsystem.enablePrecisePID(true)), //so we accelerate faster
                            new DriveToPointCommand(driveSubsystem, rightSideLeftSpikeFlip, 2, 5).withTimeout(500)
                        ),
                        new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem)
                            .andThen(new WaitCommand(800))
                            .andThen(new ParallelizingDropCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem))
                ),
                new InstantCommand(()->intakeSubsystem.clawExtraOpen()),
                new WaitCommand(50),
                secondaryArmSubsystem.intakeSub(),
                new WaitCommand(50),
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
