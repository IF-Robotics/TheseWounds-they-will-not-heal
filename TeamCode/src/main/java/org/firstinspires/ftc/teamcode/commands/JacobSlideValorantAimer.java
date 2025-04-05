package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

import java.util.function.DoubleSupplier;

public class JacobSlideValorantAimer extends CommandBase {
    public static final double maxChangePerSecond = 25; //can only change 40 inches per second
    public static final double timeToInvalidation = 0.05;

    private double startPosition = ArmSubsystem.slideRetractMin;
    private double totalChangeToDriver = 0;
    private double startYaw = 0;
    private ArmSubsystem armSubsystem;
    DoubleSupplier driverInput;
    DoubleSupplier yawInput;

    ElapsedTime timer = new ElapsedTime();

    public JacobSlideValorantAimer(ArmSubsystem armSubsystem, DoubleSupplier driverInput, DoubleSupplier yawInput) {
        this.armSubsystem = armSubsystem;
        this.driverInput = driverInput;
        this.yawInput = yawInput;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        startPosition = armSubsystem.getSlideExtention();
        startYaw=yawInput.getAsDouble();

        timer.reset();
    }

    @Override
    public void execute() {
        double desiredPosition = startPosition;

        //actually initial - final but makes it cleaner for our slide calculations
        Log.i("jacobValorantAimerYaw", String.valueOf(yawInput.getAsDouble()));
        double changeInPosToYaw = Math.cos(Math.toRadians(startYaw)) - Math.cos(Math.toRadians(yawInput.getAsDouble()));
        changeInPosToYaw = MathUtils.clamp(changeInPosToYaw, -1, 1);
        changeInPosToYaw *= SecondaryArmSubsystem.secondaryArmLength;
        desiredPosition+=changeInPosToYaw;


        if(timer.seconds() > timeToInvalidation) {
            totalChangeToDriver += maxChangePerSecond * timeToInvalidation * -driverInput.getAsDouble();
            Log.i("jacobValorantAimerDriverInput", String.valueOf(driverInput.getAsDouble()));
            Log.i("jacobValorantAimerDriverTotal", String.valueOf(maxChangePerSecond * timeToInvalidation * -driverInput.getAsDouble()));
            Log.i("jacobValorantAimerDriverChange", String.valueOf(totalChangeToDriver));
            timer.reset();
        }

        desiredPosition+=totalChangeToDriver;

        desiredPosition = MathUtils.clamp(desiredPosition, ArmSubsystem.slideRetractMin, 30+changeInPosToYaw);

        armSubsystem.setSlide(desiredPosition);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setSlide(armSubsystem.getSlideExtention());
        armSubsystem.setArmX(armSubsystem.getCurrentX());
    }

}
