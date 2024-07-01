package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Module {
    private DcMotorEx Up,Down;
    private PIDController Controller;
    private ModuleState CurrentState;
    private double TicksPerRev=464.5;//gear ratio calculat
    double p,d;
    public Module(HardwareMap hardware, String UpName, String DownName){
        Up=hardware.get(DcMotorEx.class,UpName);
        Down=hardware.get(DcMotorEx.class,DownName);
        Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Down.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Controller = new PIDController(p,0,d);

    }
    public void SetPOWER (ModuleState State){
        CurrentState=State;

    }
    public double normalise (double Ticks){
        if(Ticks>TicksPerRev)
            Ticks=Ticks-TicksPerRev;
        if(Ticks<TicksPerRev)
            Ticks=Ticks+TicksPerRev;
        return Ticks;
    }
    public void Update(){
        int MotorPositionUp=Up.getCurrentPosition();
        int MotorPositionDown=Down.getCurrentPosition();
        double differance=normalise(MotorPositionUp-MotorPositionDown);
        int AngleToTicks=(int) (CurrentState.angle*TicksPerRev/(2*Math.PI));
        double pid = Controller.calculate(differance, AngleToTicks);
        Up.setPower(pid+CurrentState.speed);
        Down.setPower(pid-CurrentState.speed);//ver. +-
    }
}
