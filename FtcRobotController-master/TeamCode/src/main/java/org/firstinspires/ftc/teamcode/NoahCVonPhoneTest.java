package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


import static org.firstinspires.ftc.teamcode.NoahCVTest.rectHeight;
import static org.firstinspires.ftc.teamcode.NoahCVTest.score;

@Autonomous(name="NoahCVTest", group="Autonomous")

public class NoahCVonPhoneTest extends LinearOpMode{

    private OpenCvCamera phoneCam;
    private NoahCVTest cvtest;
    public static double bestArea = 0;
    public static double currentArea = 0;
    public static double averageArea = 0;
    public static char cvCase;

    public void runOpMode(){


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        cvtest = new NoahCVTest();

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(cvtest);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (!opModeIsActive() && !isStopRequested()) {

            if(bestArea < 100){
                cvCase='A';
            } else if (bestArea > 101 && bestArea < 4500){
                cvCase='B';
            } else {
                cvCase='C';
            }

            telemetry.addData("currentArea", currentArea);
            telemetry.addData("averageArea", averageArea);
            telemetry.addData("rectHeight", rectHeight);
            telemetry.addData("score", score);
            telemetry.addData("area", bestArea);
            telemetry.addData("Case", cvCase);
            telemetry.update();

        }



    }
}
