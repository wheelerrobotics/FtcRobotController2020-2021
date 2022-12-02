package org.firstinspires.ftc.teamcode.comp.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;

@TeleOp
@Disabled
public class PIDOdometry extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Odo o = new Odo();
        o.init(hardwareMap);

        waitForStart();


        while (opModeIsActive()){
            /*double xtar = 1000;
            double ytar = 1000;
            double thetatar = 1000;
            ElapsedTime et = new ElapsedTime();
            et.reset();
            Telemetry tele = FtcDashboard.getInstance().getTelemetry();
            double thresh = 0.1;
            //DISTANCE IN CENTIMETERS
            double rthresh = 0.1;
            double dx = o.getPose().x;
            double dy = o.getPose().y;
            double dtheta = o.getPose().theta;
            PID px = new PID(-0.025, 0 ,0, false); // -0.025, -0.00008, -0.2
            px.init(dx);
            px.setTarget(xtar);

            PID py = new PID(-0.030, 0 ,0, false);
            py.init(dy);
            py.setTarget(ytar);

            PID ptheta = new PID(0.005, 0, 0, false);
            ptheta.init(dtheta);
            ptheta.setTarget(thetatar);


            double dthresh = 0.05;


            ElapsedTime start = new ElapsedTime();
            start.reset();
            while (true){
                //motorDriveForwardEncoded(0.5, 100);
                if (dx < 800) tele.addData("dx", dx);
                if (dy < 800) tele.addData("dy", dy);
                if (dtheta < 800) tele.addData("dtheta", dtheta);
                //if (dri < 800) tele.addData("dri", dri);

                tele.addData("sl", pl.getDerivative());
                tele.addData("sb", pb.getDerivative());
                tele.addData("sr", pr.getDerivative());
                //            tele.addData("sri", pri.getDerivative());

                double[] out = {0, 0, 0, 0};
                //if(Math.random() > 0.6) camServo.setPosition(Math.random()); // really does nothing, but DONT DELETE
                double el = -pl.tick(dl);
                double er = pr.tick(dr); // error rotation
                double eb = pb.tick(db);
                //double eri = (ri == -1) ? pri.tick(dri) : 0;

                tele.addData("el", el);
                tele.addData("eb", eb);
                tele.addData("er", er);
                //tele.addData("eri", eri);

                for (int i = 0; i<out.length; i++) { // add the individual vectors
                    out[i] = el * this.left[i] + eb * this.back[i] + er * this.clock[i]; //+ eri * -this.left[i];
                }

                double abc = absmac(out); // get max value for scaling
                if (abc > 1){
                    for (int i = 0; i<out.length; i++){ // normalize based on greatest value
                        out[i] /= abs(abc);
                    }
                }

                for (int i = 0; i<out.length; i++){ // scale down
                    out[i] *= scale;
                    //if (out[i] < 0.05) out[i] = 0;
                }
                driveVector(out);

                // update vals
                dl = distanceLeft.getDistance(DistanceUnit.CM);
                db = distanceBack.getDistance(DistanceUnit.CM);
                //dri = distanceRight.getDistance(DistanceUnit.CM);
                dr = getAnglesDeg().firstAngle;

                tele.addData("o", out);
                tele.update();

                if(
                    //abs(eri) < thresh &&
                        abs(eb) < thresh &&
                                abs(el) < thresh &&
                                abs(er) < rthresh &&
                                abs(pb.getDerivative()) > dthresh &&
                                //abs(pri.getDerivative()) > dthresh &&
                                abs(pl.getDerivative()) > dthresh &&
                                abs(pr.getDerivative()) > dthresh
                ) break;

                if(start.milliseconds() > breakTime) break;
            }

            motorStop();*/
        }
    }
}
