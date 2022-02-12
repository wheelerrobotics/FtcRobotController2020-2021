package org.firstinspires.ftc.teamcode.comp.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

@Autonomous
public class FancyNavBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Meccanum mec = new Meccanum();

        mec.setOpModeOn(opModeIsActive());

        //meccanum.motorDriveRelativeFieldAngleEncoded(0, meccanum.NORMAL_SPEED, 775);
        // meccanum.turnDeg(90, 90, telemetry);
        /*
        JsonParser jp = new JsonParser();
        Object obj = jp.parse(new FileReader("./jsons/gameplanB.json"));
        JSONObject gameplan = (JSONObject) obj;
        for (Iterator<String> it = gameplan.keys(); it.hasNext(); ) {
            String k = it.next();
            String key = it.next();
            // this approach will be synchronous, but it could be async with a "completed" tag in the json move

            parseMove(gameplan.get(key));

        }*/

        //meccanum.motorDriveRelativeFieldAngleEncoded(0, meccanum.NORMAL_SPEED, 775);
        // meccanum.turnDeg(90, 90, telemetry);
        /*
        JsonParser jp = new JsonParser();
        Object obj = jp.parse(new FileReader("./jsons/gameplanB.json"));
        JSONObject gameplan = (JSONObject) obj;
        for (Iterator<String> it = gameplan.keys(); it.hasNext(); ) {
            String k = it.next();
            String key = it.next();
            // this approach will be synchronous, but it could be async with a "completed" tag in the json move

            parseMove(gameplan.get(key));

        }*/


    }

}
