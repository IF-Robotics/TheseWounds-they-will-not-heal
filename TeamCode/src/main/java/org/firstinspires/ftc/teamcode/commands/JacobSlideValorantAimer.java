package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

import java.util.function.DoubleSupplier;

public class JacobSlideValorantAimer extends CommandBase {
    private final double maxChangePerSecond = 60; //can only change 40 inches per second
    private final double timeToInvalidation = 0.02;

    double desiredX = 0;
    double desiredY = 0;
    private ArmSubsystem armSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private SecondaryArmSubsystem secondaryArmSubsystem;
    DoubleSupplier driverInput;
    DoubleSupplier yawInput;

    ElapsedTime timer = new ElapsedTime();

    public JacobSlideValorantAimer(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, DoubleSupplier driverInput, DoubleSupplier yawInput) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.secondaryArmSubsystem = secondaryArmSubsystem;
        this.driverInput = driverInput;
        this.yawInput = yawInput;
        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }

    @Override
    public void initialize() {

        desiredY = armSubsystem.getSlideX();
        desiredX = 0;
        timer.reset();
    }

    @Override
    public void execute() {
        double yVelocity = -driverInput.getAsDouble();

        if(timer.seconds()>timeToInvalidation&&Math.abs(yVelocity)>0.1) {
            desiredY += yVelocity * timeToInvalidation * maxChangePerSecond;
            timer.reset();
        }

        double desiredX = yawInput.getAsDouble();
        desiredX = MathUtils.clamp(desiredX, -1, 1);
        desiredX *= SecondaryArmSubsystem.secondaryArmLength;

        double yaw = Math.acos(desiredX/SecondaryArmSubsystem.secondaryArmLength); //0 through 180

        double slideCompensation = Math.sin(yaw) * SecondaryArmSubsystem.secondaryArmLength;

        yaw = Math.toDegrees(yaw); //convert to degrees
        yaw -=90; //make it -90 through 90
        yaw *= -1; //flip it for our input into the secondary arm subsystem

        secondaryArmSubsystem.setDiffyYaw(yaw);

        double desiredPosition = desiredY-slideCompensation+SecondaryArmSubsystem.secondaryArmLength;


        desiredPosition = MathUtils.clamp(desiredPosition, ArmSubsystem.slideRetractMin, 30-slideCompensation+SecondaryArmSubsystem.secondaryArmLength);


        armSubsystem.setArmX(desiredPosition);

        intakeSubsystem.normalizeRollToSecondaryArm(yaw);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmX(armSubsystem.getCurrentX());
    }

}
