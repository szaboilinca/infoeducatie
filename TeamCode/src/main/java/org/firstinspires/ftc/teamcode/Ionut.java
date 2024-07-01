package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

public class Ionut{
    Module Left;
    Module Right;
    public Ionut(LinearOpMode OpMode){

        Left = new Module(OpMode.hardwareMap, HardwareNames.LeftUp, HardwareNames.LeftDown);
        Right = new Module(OpMode.hardwareMap, HardwareNames.RightUp, HardwareNames.RightDown);
    }
    public void Drive(double x, double y, double head){
        List<ModuleState> wheelspeed=kinematics.InverseKinematics(x,y,head);

        Left.SetPOWER(wheelspeed.get(0));
        Right.SetPOWER(wheelspeed.get(1));
        Update();
    }
    public void Update(){
        Left.Update();
        Right.Update();
    }
    }

