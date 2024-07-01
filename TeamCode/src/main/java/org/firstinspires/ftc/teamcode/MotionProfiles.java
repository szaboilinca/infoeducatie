package org.firstinspires.ftc.teamcode;

public class MotionProfiles {
    double acc=0.5;
    double dec=0.5;
    double vmax=1.5;
    double d;
    double ta,tf,tc;
    double da,df,dc;
    double vmaxr,tar,tfr;
    double ttotal;
    public MotionProfiles(double d){//profil de miscare propriu zis
        this.d=d;
        ta=vmax/acc;
        tf=vmax/dec;
        da=ta*vmax/2;
        df=tf*vmax/2;
        dc=tc*vmax/2;
        if(da+df<d){
            tc=d/vmax-ta;
            dc=vmax*tc;
        }else{
            vmaxr=Math.sqrt(acc*d);
            tc=0;
            dc=0;
            ta=vmaxr/acc;
            tf=vmaxr/dec;
            da=ta*vmaxr/2;
            df=tf*vmaxr/2;
            vmax=vmaxr;
        }
        ttotal=ta+tc+tf;
    }
    public double GetDistance(double t){//localizare in fct de timp
        if(t<=ta){
            return acc*(t*t)/2;
        }
        if(t<=ta+tc){
            return da+vmax*(t-ta);
        }
        if(t<=ta+tc+tf){
            return da+dc+vmax*(t-ta-tc)-dec*(t-ta-tc)*(t-ta-tc)/2;
        }
        return 0;
    }
    public double GetSpeed(double t){
        if(t<=ta){
            return acc*t;
        }
        if(t<=ta+tc){
            return vmax;
        }
        if(t<=ta+tc+tf){
            return vmax-dec*(t-ta-tc);
        }
        return 0;
    }
}
