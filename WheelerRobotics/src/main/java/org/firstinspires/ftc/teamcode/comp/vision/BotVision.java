package org.firstinspires.ftc.teamcode.comp.vision;
/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class BotVision {
    OpenCvWebcam webcam;
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry tele = dash.getTelemetry();

    ColorIsolationPipeline pipeline = new ColorIsolationPipeline();

    public void setProc(ColorIsolationPipeline.processors p){
        pipeline.setProcessorSetting(p);
    }


    public void init(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.setParams(50F, 0.92F, 0.44F, 10F, 0.1F, 0.4F);
        pipeline.setProcessorSetting(ColorIsolationPipeline.processors.SIMPLE);
        webcam.setPipeline(pipeline);


        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // FtcDashboard.getInstance().startCameraStream(webcam, 20);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                tele.addLine("Opened!");
                tele.update();
            }

            @Override
            public void onError(int errorCode) {
                tele.addData("Crashed", "camera");
                tele.update();
            }
        });
    }
    public void setParams(float htar, float star, float ltar, float hthresh, float sthresh, float lthresh) {
        pipeline.setParams(htar, star, ltar, hthresh, sthresh, lthresh);
    }

    public int getConePosition(){
        int pos = 0;
        while (pos == 0) { // stall if camera not open
            pos = pipeline.getConePosition();
        }
        return pos;
    }


}


