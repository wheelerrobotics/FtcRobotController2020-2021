package org.firstinspires.ftc.teamcode.comp.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;


// THIS HAS ALL THE COMMENTED OUT STUFF THAT I WAS MAKING FOR A FOR NOW SCRAPPED JSON BASED SYSTEM

@Autonomous
@Disabled
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

        }
        public void parseMove(JSONObject move) throws JSONException {
        switch (move.get("type")){
            case "LINE":
                // do stuff
                if(move.get("relative").equals(1)){
                    //meccanum.motorDriveLineRobotAngle();
                }else{
                    //meccanum.motorDriveLineFieldAngle();
                }
            case "TURN":

            case "SPINNER":

            case "CLAW":

            case "ARM":

            default:
                throw new IllegalStateException("Unexpected value: " + move.get("type"));
        }

    }
        */


    }

}
