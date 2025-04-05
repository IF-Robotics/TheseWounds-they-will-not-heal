package org.firstinspires.ftc.teamcode.subSystems;


import static org.firstinspires.ftc.teamcode.other.Globals.clawClose;
import static org.firstinspires.ftc.teamcode.other.Globals.clawExtraOpen;
import static org.firstinspires.ftc.teamcode.other.Globals.clawHalfClose;
import static org.firstinspires.ftc.teamcode.other.Globals.clawOpen;
import static org.firstinspires.ftc.teamcode.other.Globals.teleopSpec;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

@Config
public class SpecMechSubsystem extends SubsystemBase {

    private Servo specClaw, specArm;

    private Telemetry telemetry;

    private double armAngle;

    public static int angleOffset = 0;


    //claw poses
    private double clawClose = .45;
    private double clawOpen = .18;

    //arm poses
    public static double specArmUp = 0, specArmWallIntake = .85, specArmStow = specArmWallIntake;

    //toggle
    private boolean armUp = false;

    public SpecMechSubsystem(Servo specClaw, Servo specArm, Telemetry telemetry) {
        this.specClaw = specClaw;
        this.specArm = specArm;
        this.telemetry = telemetry;
    }

    public void openClaw (){
        specClaw.setPosition(clawOpen);
    }

    public void closeClaw (){
        specClaw.setPosition(clawClose);
    }

    public void setArm(double angle){
        specArm.setPosition(angle);
    }

    // switch the toggle
    public void toggle() {
        armUp = !armUp;
    }

    // return the active state
    public boolean active() {
        return armUp;
    }

    @Override
    public void periodic() {

        telemetry.addData("specArmPos", specArm.getPosition());
        telemetry.addData("specClawPos", specClaw.getPosition());
    }

}
