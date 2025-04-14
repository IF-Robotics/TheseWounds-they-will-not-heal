package org.firstinspires.ftc.teamcode.opModes;


import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmUp;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmWallIntake;

import android.util.Log;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleFast;
import org.firstinspires.ftc.teamcode.commandGroups.ClimbLevel3;
import org.firstinspires.ftc.teamcode.commandGroups.DropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.DropOffCommand;
import org.firstinspires.ftc.teamcode.commandGroups.FlipSample;
import org.firstinspires.ftc.teamcode.commandGroups.HighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeCloseCommand;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.LowBasketCommand;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingCommand;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingDropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterWallIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.commandGroups.ScoreHighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.HighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.ArmManualCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.JacobSlideValorantAimer;
import org.firstinspires.ftc.teamcode.commands.LimelightTeleopAimer;
import org.firstinspires.ftc.teamcode.commands.LimelightToSample;
import org.firstinspires.ftc.teamcode.commands.ResetSlides;
import org.firstinspires.ftc.teamcode.commandGroups.TeleopSpecScore;

import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveHeadingLocked;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;


@Disabled
@TeleOp(name="teleOpFunnyTest")
public class TeleopOpMode extends Robot {



    //buttons
    private Button cross1, back2, start2, dUp1, dDown1, dLeft1, dRight1, bRight1, bLeft1, triangle1, triangle2, square1, touchpad1, touchpad2, start1, square2, dUp2, bRight2, bLeft2, dRight2, dDown2, cross2, circle1, circle2, dLeft2, back1, stickButtonLeft2;
    private Trigger tLeft1, tRight1, tLeft2, tRight2;

    //teleop mode
    public static boolean teleopSpec = false;
    public static boolean parallelizing = false;


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void initialize(){
        super.initialize();

        //configureMoreCommands();
        configureButtons();
        manualArm = false;
        manualSlides = false;

//        new ArmCoordinatesCommand(armSubsystem, 12, 7).schedule(true);
        new InstantCommand(() -> secondaryArmSubsystem.setDiffyYaw(0)).schedule(true);

    }

    /*public void configureMoreCommands() {

    }*/

    public void configureButtons() {
        square1 = new GamepadButton(m_driver, GamepadKeys.Button.X);
        square2 = new GamepadButton(m_driverOp, GamepadKeys.Button.X);
        start2 = new GamepadButton(m_driverOp, GamepadKeys.Button.START);
        back2 = new GamepadButton(m_driverOp, GamepadKeys.Button.BACK);
        dUp1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_UP);
        dUp2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_UP);
        dDown1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_DOWN);
        dDown2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_DOWN);
        dLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_LEFT);
        dLeft2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_LEFT);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        bRight1 = new GamepadButton(m_driver, GamepadKeys.Button.RIGHT_BUMPER);
        bLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.LEFT_BUMPER);
        bRight2 = new GamepadButton(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        triangle1 = new GamepadButton(m_driver, GamepadKeys.Button.Y);
        triangle2 = new GamepadButton(m_driverOp, GamepadKeys.Button.Y);
        cross1 = new GamepadButton(m_driver, GamepadKeys.Button.A);
        cross2 = new GamepadButton(m_driverOp, GamepadKeys.Button.A);
        bLeft2 = new GamepadButton(m_driverOp, GamepadKeys.Button.LEFT_BUMPER);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        dRight2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_RIGHT);
        tLeft1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1);
        tRight1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1);
        tLeft2 = new Trigger(() -> m_driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1);
        tRight2 = new Trigger(() -> m_driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1);
        start1 = new GamepadButton(m_driver, GamepadKeys.Button.START);
        circle1 = new GamepadButton(m_driver, GamepadKeys.Button.B);
        circle2 = new GamepadButton(m_driverOp, GamepadKeys.Button.B);
        back1 = new GamepadButton(m_driver, GamepadKeys.Button.BACK);
        stickButtonLeft2 = new GamepadButton(m_driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON);




        //sub intake
        dUp1.whenPressed(new ConditionalCommand(
                new ConditionalCommand(
                        new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                        new SequentialCommandGroup(
                                new WaitForArmCommand(armSubsystem, 5, 5, true),
                                new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem)
                        ),
                        () -> (armSubsystem.getArmAngle() < 15 && armSubsystem.getCurrentY() < 20)
                ),
                new RetractFromBasket(driveSubsystem, armSubsystem, intakeSubsystem),
                () -> armSubsystem.getCurrentY() < 30

                ));
        dUp2.whenPressed(new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem));



        //dUp2.whenReleased(armInSubCommand);
        //rotate intake
        bLeft1.whenActive(new ConditionalCommand(
                new InstantCommand(() -> intakeSubsystem.rotateIntake()),
                new InstantCommand(() -> intakeSubsystem.rotateIntake(false)),
                ()->Math.abs(secondaryArmSubsystem.getYawAngle())<3
        ));
        bLeft2.whenActive(new ConditionalCommand(
                new InstantCommand(() -> intakeSubsystem.rotateIntake()),
                new InstantCommand(() -> intakeSubsystem.rotateIntake(false)),
                ()->Math.abs(secondaryArmSubsystem.getYawAngle())<3
        ));


        //intake close
        dRight1.whenPressed(new IntakeCloseCommand(armSubsystem, intakeSubsystem));
        dRight2.whenPressed(new IntakeCloseCommand(armSubsystem, intakeSubsystem));

        stickButtonLeft2.whenPressed(new InstantCommand(() -> {
            secondaryArmSubsystem.setDiffyYaw(0);
            intakeSubsystem.normalizeRollToSecondaryArm(0);
        }));

        //In-sub adjuster for secondary
        new Trigger(()->Math.abs(m_driverOp.getLeftX()) > .1).or(new Trigger(()->Math.abs(m_driverOp.getRightY()) > .1))
                .whenActive(
                    new InstantCommand(()->intakeSubsystem.setPitch(pitchWhenIntake))
                        .andThen(new JacobSlideValorantAimer(armSubsystem, intakeSubsystem, secondaryArmSubsystem, m_driverOp::getRightY, ()->m_driverOp.getLeftX()))
                );


//        new Trigger(()->Math.abs(m_driverOp.getRightY()) > .1)
//            .whenActive(new InstantCommand(()->intakeSubsystem.setPitch(pitchWhenIntake)).andThen(new ArmManualCommand(armSubsystem, m_driverOp::getLeftY, m_driverOp::getRightY)));
//        .whenActive(new JacobSlideValorantAimer(armSubsystem, m_driverOp::getRightY, ()->m_driverOp.getLeftX() * 90));

        //retract after intaking
        dDown1.whenPressed(new ConditionalCommand(
                new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                                        new HighBasketCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem)
                                ),
                                new SequentialCommandGroup(
                                        new RetractFromBasket(driveSubsystem, armSubsystem, intakeSubsystem),
                                        new WaitCommand(200),
                                        new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem)
                                ),
                                ()->armSubsystem.getCurrentY()<30
                        ),
                () -> teleopSpec == true
            )
        );
//        dDown1.whenPressed(new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem));
        //retract after intaking and basket (spec mode)
        dDown2.whenPressed(
                new ConditionalCommand(
                    //retract from intaking
                        //In-case we want to make parrelizing and non parrelizing idfferent. however, we don't at this time
                        new ConditionalCommand(
                                new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                                new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                                () -> parallelizing == false
                        ),
                    //retract from basket
                    new RetractFromBasket(driveSubsystem, armSubsystem, intakeSubsystem),
                    //if slides are up
                    () -> armSubsystem.getCurrentY() < 20
                )
        );

        //wall intake
//        tRight1.toggleWhenActive(new teleopSpecScore(driveSubsystem,armSubsystem,intakeSubsystem));
//        tLeft1.whenActive(new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem, false, ()->{return false;},m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX));
//        tLeft1.toggleWhenActive(new TeleDriveHeadingLocked(driveSubsystem, m_driver));
        tLeft1.whileActiveContinuous(new holdDTPosCommand(driveSubsystem)); //heading lock
//                .whenActive(new ConditionalCommand(
//                        new SequentialCommandGroup(
//                                new WaitForArmCommand(armSubsystem, ArmSubsystem.armMinAngle, 5),
//                                new WaitForSlideCommand(armSubsystem, 10, 2),
//                                new LimelightToSample(driveSubsystem, armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem)
//                        ),
//                        new LimelightToSample(driveSubsystem, armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem),
//                        ()-> armSubsystem.getArmAngle()>7 || armSubsystem.getSlideExtention()>11
//                )
//                );
//        //chambers
//        square2.whenPressed(new HighChamberCommand(armSubsystem, intakeSubsystem));
//        square2.whenReleased(new ScoreHighChamberCommand(armSubsystem, intakeSubsystem));
        //auto spec scoring
        circle1.toggleWhenPressed(new ConditionalCommand(
                new TeleopSpecScore(driveSubsystem,armSubsystem,intakeSubsystem, secondaryArmSubsystem),
                new ParallelCommandGroup(
                        new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                        secondaryArmSubsystem.setPitchSafe(secondaryPitchWallIntake),
                        new InstantCommand(()->armSubsystem.nautilusUp())
                ),
                () -> (armSubsystem.getTargetX() == armIntakeWallX && armSubsystem.getTargetY() == armIntakeWallY)
                )
        );

        tLeft2.whenActive(new ConditionalCommand(
                new ParallelCommandGroup(
                        new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                        new SecondaryArmCommand(secondaryArmSubsystem, secondaryPitchWallIntake, secondaryArmYaw)
                ),
                new RetractAfterWallIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                () -> {
                    armSubsystem.toggleWallState();
                    return armSubsystem.getWallState();
                }
        ));

        //dropping sample (into observation zone)
        square2.whenPressed(new ConditionalCommand(
                new DropCommand(armSubsystem, intakeSubsystem),
                new ParallelizingDropCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem, specMechSubsystem),
                () -> parallelizing == false
        ));

        square2.whenReleased(new ConditionalCommand(
                new DropOffCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new ParallelizingCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem, specMechSubsystem),
                () -> parallelizing == false
        ));
        tRight1.whenActive(
                new ConditionalCommand(
                        new DropCommand(armSubsystem, intakeSubsystem),
                        new SequentialCommandGroup(
                                new WaitForSlideCommand(armSubsystem, 8, 20),
                                new WaitForArmCommand(armSubsystem, 0, 5)
                        ),
                        () -> armSubsystem.getArmAngle() < 45
                )
        );


        //specMech
        square1.whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                //wallIntake
                                new InstantCommand(() -> specMechSubsystem.openClaw()),
                                new WaitCommand(200),
                                new InstantCommand(() -> specMechSubsystem.setArm(specArmWallIntake))
                        ),
                        new SequentialCommandGroup(
                                //ramSpec
                                new InstantCommand(() -> specMechSubsystem.closeClaw()),
                                new WaitCommand(200),
                                new InstantCommand(() -> specMechSubsystem.setArm(specArmUp))
                        ),
                        () -> {
                            specMechSubsystem.toggle();
                            return specMechSubsystem.active();
                        }
                )
        );

        circle2.whenPressed(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                //wallIntake
                                new InstantCommand(() -> specMechSubsystem.openClaw()),
                                new InstantCommand(() -> specMechSubsystem.setArm(specArmWallIntake))
                        ),
                        new SequentialCommandGroup(
                                //ramSpec
                                new InstantCommand(() -> specMechSubsystem.closeClaw()),
                                new WaitCommand(200),
                                new InstantCommand(() -> specMechSubsystem.setArm(specArmUp))
                        ),
                        () -> {
                            specMechSubsystem.toggle();
                            return specMechSubsystem.active();
                        }
                )
        );


        //baskets
        triangle2.whenPressed(
            new ConditionalCommand(
                new HighBasketCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new WaitForArmCommand(armSubsystem, ArmSubsystem.armMinAngle, 5),
                        new WaitForSlideCommand(armSubsystem, 10, 2),
                        new LimelightTeleopAimer(armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem)
                    ),
                    new LimelightTeleopAimer(armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem),
                    ()-> armSubsystem.getArmAngle()>7 || armSubsystem.getSlideExtention()>11
                ),
                ()->parallelizing==false
            )
        );

        triangle1.whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                    //move to high basket
                    new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),
                    new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket)
                        ),
                new SequentialCommandGroup(
                        new WaitForSlideCommand(armSubsystem, 8, 15),
                        //move arm back
                        new WaitForArmCommand(armSubsystem, 100, 45),

                        //move to high basket
                        new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket)
                ),
                () -> armSubsystem.getArmAngle() > 45
                )
        );

        //low basket
        cross2.whenPressed(new LowBasketCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem));
        cross1.whenPressed(new ConditionalCommand(
                        new SequentialCommandGroup(
                                //move to high basket
                                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armLowBasketY),
                                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket),
                                new SecondaryArmCommand(secondaryArmSubsystem, 30, 0)
                        ),
                        new SequentialCommandGroup(
                                new WaitForSlideCommand(armSubsystem, 10, 15),
                                //move arm back
                                new WaitForArmCommand(armSubsystem, 100, 45),

                                //move to high basket
                                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armLowBasketY),
                                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket),
                                new SecondaryArmCommand(secondaryArmSubsystem, 30, 0)
                        ),
                        () -> armSubsystem.getArmAngle() > 45
                )
        );

//        //climbing
//        bRight2.whenPressed(new ParallelCommandGroup(
//                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, rollFrontHighChamber), armPositionToClimb));
//        bRight2.whenReleased(new ClimbLevel3(armSubsystem, intakeSubsystem, gyro));

        //testing
        start2.whenPressed(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket));

        //reset pinpoint imu
        back1.whenPressed(new InstantCommand(() -> driveSubsystem.resetPinpointIMU()));
        //reset slides
        tRight2.whenActive(new ResetSlides(armSubsystem));

        //Default Commands
        driveSubsystem.setDefaultCommand(new TeleDriveCommand(driveSubsystem, m_driver, true, 10, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX));


    }

    @Override
    public void run(){
        super.run();


        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        //retract and move arm out of the way
        if(gamepad1.right_stick_button || gamepad2.back){
            schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, rollWhenBasket));
            schedule(new WaitForSlideCommand(armSubsystem, 8, 10));
            schedule(new InstantCommand(() -> armSubsystem.setArm(45)));
        }

        if(gamepad1.start){
            schedule(new ParallelCommandGroup(
                    new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitch, roll),
                    new SecondaryArmCommand(secondaryArmSubsystem, secondaryArmPitch, secondaryArmYaw)
            ));
        }


        //switch teleop mode
        if(currentGamepad1.touchpad && !previousGamepad1.touchpad){
            teleopSpec = !teleopSpec;
        }

        //switch parrallelizing
        if(currentGamepad2.touchpad && !previousGamepad2.touchpad){
            parallelizing = !parallelizing;
        }

        if(parallelizing){
            gamepad2.setLedColor(128,0,128, LED_DURATION_CONTINUOUS);
        } else{
            gamepad2.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
        }

//        //manual secondary arm  (couldnt figure out how to do it using conditional commands though didnt try very hard)
//        if(gamepad2.left_stick_button){
//            schedule(new InstantCommand(() -> secondaryArmSubsystem.setDiffyYaw(0)));
//        } else if(Math.abs(m_driverOp.getLeftX()) > .05){
//            schedule(new InstantCommand(() -> secondaryArmSubsystem.setDiffyYaw(m_driverOp.getLeftX() * 90 /*makes the range 180degrees*/)));
//        }

//        //manual slides
//        if(Math.abs(m_driverOp.getRightY()) > .1) {
//            schedule(new ArmManualCommand(armSubsystem, m_driverOp::getRightY));
//        }

        telemetry.addData("gamepad2 leftX", m_driverOp.getLeftX());

    }


}