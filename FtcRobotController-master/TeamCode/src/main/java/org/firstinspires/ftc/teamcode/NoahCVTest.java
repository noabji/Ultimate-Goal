package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.NoahCVonPhoneTest.bestArea;
import static org.firstinspires.ftc.teamcode.NoahCVonPhoneTest.averageArea;
import static org.firstinspires.ftc.teamcode.NoahCVonPhoneTest.currentArea;

public class NoahCVTest extends SkystoneDetector {

    private Mat rawImage = new Mat();
    private Mat yellowImage = new Mat();
    private Mat imageClone = new Mat();
    private Rect foundRect = new Rect();
    public static int rectHeight = 0;
    public static double score;

    public DogeCVColorFilter orangeFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100);

    public Mat process(Mat input){

        input.copyTo(rawImage);
        input.copyTo(yellowImage);

        List<MatOfPoint> contoursYellow = new ArrayList<>();

        orangeFilter.process(rawImage.clone(), yellowImage);

        Imgproc.findContours(yellowImage, contoursYellow, rawImage, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(yellowImage, contoursYellow, -1, new Scalar(0,30,30), 2);

        Rect rect = null;

        Rect bestRect = foundRect.clone();
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better

        for (int i = 0; i < contoursYellow.size(); i++) {
            Mat contour = contoursYellow.get(i);
            double contourArea = Imgproc.contourArea(contour);

            currentArea = contourArea;

            if (contourArea > bestArea){
                bestArea = contourArea;
            }

        }




        return yellowImage;

    }

}
