package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.opModes.TeleopOpMode.teleopSpec;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.secondaryPitchWallIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstHighChamberRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstWallPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstWallPickUpSafe;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberSafeScore;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberSpecMech;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUpCheckpoint;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmStow;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmUp;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmWallIntake;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specAutoStart;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleFast;
import org.firstinspires.ftc.teamcode.commandGroups.FlipSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingCycles;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingDropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.PushSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.SixSpecSafeScore;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointDoubleSupplierCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightTeleopAimer;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem;

@Autonomous(name="6+0 SAFE \uD83E\uDDBA\uD83E\uDDBA\uD83E\uDDBA\uD83E\uDDBA\uD83E\uDDBA\uD83D\uDE2D\uD83D\uDE2D\uD83D\uDE4F\uD83D\uDE4F\uD83D\uDC02\uD83D\uDCA9")

public class SixSpecAutoSAFE extends AutoBase {

    private double subX1 = 0;
    private double subY1 = 7.5;


    int currentSubIndex = 1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void initialize() {
        super.initialize();

        intakeSubsystem.setDiffy(-20,0);
        secondaryArmSubsystem.setDiffy(90, 0);

        specMechSubsystem.closeClaw();
        specMechSubsystem.setArm(specAutoStart);

        limelightSubsystem.initializeCamera();


        schedule(new SequentialCommandGroup(
                new StartSpecAuto(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new ParallelCommandGroup(
                        new InstantCommand(() -> specMechSubsystem.closeClaw()),
                        new InstantCommand(() -> specMechSubsystem.setArm(specArmUp)),
                        new InstantCommand(() -> armSubsystem.setArm(25)),
                        new InstantCommand(() -> armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                        new InstantCommand(() -> secondaryArmSubsystem.setDiffy(-20, 0)),
                        new InstantCommand(() ->intakeSubsystem.setDiffy(0,0)),
                        new InstantCommand(() ->intakeSubsystem.openClaw())
                ),
                //servo nudge issue
                new InstantCommand(()->{
                    secondaryArmSubsystem.setDiffyPitch(SecondaryArmSubsystem.hardStoppedHighPitch);
                    specMechSubsystem.closeClaw();
                }),
                new WaitCommand(25),
                new InstantCommand(()->secondaryArmSubsystem.setDiffyPitch(0)),
                new WaitCommand(25),

                new DriveToPointDoubleSupplierCommand(driveSubsystem, ()->firstHighChamberRight.getX()+subX1, ()->firstHighChamberRight.getY(), firstHighChamberRight.getRotation(), 5, 5).withTimeout(1500)
                        .alongWith(new WaitCommand(300).andThen(new InstantCommand(() -> secondaryArmSubsystem.setDiffy(0, -30)))),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new InstantCommand(() -> specMechSubsystem.openClaw())
//                                new WaitCommand(1000),
//                                new InstantCommand(() -> specMechSubsystem.setArm(specArmWallIntake))
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),//keep it long for now
                                new LimelightTeleopAimer(armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem).withTimeout(1000),
                                new WaitCommand(1000).interruptOn(()->Math.abs(armSubsystem.getSlideError())<0.5),
                                new WaitCommand(100),
                                new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
                        )
                ),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(400),
                            new DriveToPointDoubleSupplierCommand(
                                    driveSubsystem,
                                    ()->driveSubsystem.getPos().getX(),
                                    ()->driveSubsystem.getPos().getY()-10,
                                    new Rotation2d(),
                                    5,
                                    5
                            ),
                            new DriveToPointCommand(driveSubsystem, new Pose2d(40.9, -52.2, Rotation2d.fromDegrees(-157)), 3, 5),
                            new InstantCommand(()->intakeSubsystem.setDiffy(180)),
                            new WaitCommand(200),
                            new InstantCommand(()->intakeSubsystem.clawExtraOpen()),
                            new PushSpikes(driveSubsystem)
                            ),
                        new SequentialCommandGroup(
                            new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem, true),
                            new WaitCommand(600),
                            new ArmCoordinatesCommand(armSubsystem, 8,10),
                            new WaitCommand(200),
//                            new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, -30, 0),
                            new InstantCommand(()->intakeSubsystem.setDiffy(-30, 0)),
                            new InstantCommand(() -> specMechSubsystem.setArm(specArmStow)),
//                            new ArmCoordinatesCommand(armSubsystem, 8,10),
//                            new InstantCommand(()->intakeSubsystem.openClaw()), //so that we save time when we intake later ig
                            secondaryArmSubsystem.setPitchYawSafe(0,0)
                        )
                ),
                new WaitCommand(500),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, -30, 180),
                new SixSpecSafeScore(armSubsystem, intakeSubsystem, secondaryArmSubsystem, driveSubsystem, firstWallPickUpSafe),
                new SixSpecSafeScore(armSubsystem, intakeSubsystem, secondaryArmSubsystem, driveSubsystem),
                new SixSpecSafeScore(armSubsystem, intakeSubsystem, secondaryArmSubsystem, driveSubsystem),
                new SixSpecSafeScore(armSubsystem, intakeSubsystem, secondaryArmSubsystem, driveSubsystem),
                new SixSpecSafeScore(armSubsystem, intakeSubsystem, secondaryArmSubsystem, driveSubsystem),
                new SixSpecSafeScore(armSubsystem, intakeSubsystem, secondaryArmSubsystem, driveSubsystem)

        ));
        while(!isStarted() && !isStopRequested()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentSubIndex == 1) {
                if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                    subX1 -= 1;
                    MathUtils.clamp(subX1, -8, 6);
                }

                if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                    subX1 += 1;
                    subX1 = MathUtils.clamp(subX1, -8, 6);
                }
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                currentSubIndex += 1;
                currentSubIndex = MathUtils.clamp(currentSubIndex, 1, 2);
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                currentSubIndex -= 1;
                currentSubIndex = MathUtils.clamp(currentSubIndex, 1, 2);
            }

            //change spec mode
            if (currentGamepad1.touchpad && !previousGamepad1.touchpad) {
                teleopSpec = !teleopSpec;
            }

            telemetry.addData("currentSubIndex", currentSubIndex);

            telemetry.addData("subX1 (-8,6)", subX1);



            //specMode
            if (teleopSpec) {
                telemetry.addData("specMode", "\uD83D\uDCA7\uD83D\uDCA7true\uD83D\uDC7A\uD83D\uDC7A");
            } else {
                telemetry.addData("specMode", "\uD83D\uDC72\uD83D\uDC72false⭐⭐");
            }

            telemetry.update();
        }

    }

}
