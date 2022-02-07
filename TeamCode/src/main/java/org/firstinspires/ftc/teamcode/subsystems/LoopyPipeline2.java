package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline; 
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class LoopyPipeline2 extends OpenCvPipeline {
    
    //Defines possible position outcomes
    public enum Position
    {
        LEFT,
        MIDDLE,
        RIGHT
    }
    
        //Defines dimensions for the boxes
        final int REGION_WIDTH = 2;
        final int REGION_HEIGHT = 2;
        
        public LinearOpMode l;
        
        //Defines starting points for the first box
        public int box1x;
        public int box1y;
        int startx;
        int starty;
        int maxx;
        int maxy;
        int sat_threshold = 30;
        int HueGoal = 110;
        int AllowedError = 20;

        //Defines a Mat specifically for the first box
        Mat box1_Mat;

        //Defines an int to take the hue of the first box
        int box1_average_hue;

        //And one for saturation
        int box1_average_sat;

        // int to hold distance from hue goal
        int box1_deviation;
        
        //Sets ints for lines
        int line1x;
        int line2x;
        
        public int leftHits;
        public int midHits;
        public int rightHits;

 

        // Sets up a variable to store our analysis
        public volatile Position position = Position.MIDDLE;  

          //Defines colors to draw box with
        final Scalar RED = new Scalar(225, 0, 0);
          
          public LoopyPipeline2(int x1, int x2, int minx, int miny, int Maxx, int Maxy, LinearOpMode input){
            line1x = x1;
            line2x = x2;
            startx = minx;
            starty = miny;
            maxx = Maxx;
            maxy = Maxy;
            l = input;
        }

        


  
    @Override
    public Mat processFrame(Mat inputMat) {
        leftHits = 0;
        midHits = 0;
        rightHits = 0;
        // Goods.clear();
        // GoodYs.clear();

        box1x = startx;
        box1y = starty;
        Mat RGBMat = inputMat;
        //Converts to HSV
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_RGB2HSV);
        while (box1y < maxy &! l.isStopRequested()) {

            while (box1x < maxx &! l.isStopRequested()){
                //Defines an anchor point for the first box using the dimensions
                Point BOX1_TOPLEFT_ANCHOR_POINT = new Point(box1x, box1y);
            
                //Defines the two points used to draw the first box
                Point box1_pointA = new Point(
                    BOX1_TOPLEFT_ANCHOR_POINT.x,
                    BOX1_TOPLEFT_ANCHOR_POINT.y);
                Point box1_pointB = new Point(
                    BOX1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    BOX1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
                //Sets the box1 Mat to the stuff in box1
                box1_Mat = inputMat.submat(new Rect(box1_pointA, box1_pointB));

                //Takes the average hue of box1
                box1_average_hue = (int) Core.mean(box1_Mat).val[0];

                //Does the same for saturation
                box1_average_sat = (int) Core.mean(box1_Mat).val[1];

                box1_deviation = Math.abs(box1_average_hue - HueGoal);
                
                // If hue is close enough and there is enough saturation, draw the box and count where it is
                if (box1_deviation < AllowedError && box1_average_sat > sat_threshold){
                    if (box1x < line1x) {
                        leftHits++;
                    } else if (box1x> line2x) {
                        rightHits++;
                    } else {
                        midHits++;
                    }
                    Imgproc.rectangle(
                        inputMat, // What to draw on
                        box1_pointA, // First point which defines the rectangle
                        box1_pointB, // Second point which defines the rectangle
                        RED, // The color the rectangle is drawn in
                        1); // Thickness of the rectangle lines
                }
                box1x++;
                }
            box1x = startx;
            box1y++;
        }

        

        //Sets position based on where the hits are
        if (leftHits > midHits && leftHits > rightHits) {
            position = Position.LEFT;
        } else if (midHits > rightHits) {
            position = Position.MIDDLE;
        } else {
            position = Position.RIGHT;
        }
            
            //Defines a bunch of points that we use to draw lines that will show where we are looking
            Point line0pointA = new Point(startx, starty);
            Point line0pointB = new Point(startx, maxy + REGION_HEIGHT);
            Point line1pointA = new Point(line1x +REGION_WIDTH/2, starty); //Splits the difference with region width - where the box falls more is where we detect
            Point line1pointB = new Point(line1x +REGION_WIDTH/2, maxy + REGION_HEIGHT);
            Point line2pointA = new Point(line2x +REGION_WIDTH/2, starty);
            Point line2pointB = new Point(line2x +REGION_WIDTH/2, maxy + REGION_HEIGHT);
            Point line3pointA = new Point(startx, starty);
            Point line3pointB = new Point(maxx + REGION_WIDTH, starty);
            Point line4pointA = new Point(maxx + REGION_WIDTH, starty);
            Point line4pointB = new Point(maxx + REGION_WIDTH, maxy + REGION_HEIGHT); //Shows right end of box
            Point line5pointA = new Point(startx, maxy + REGION_HEIGHT);
            Point line5pointB = new Point(maxx + REGION_WIDTH, maxy + REGION_HEIGHT); //Shows bottom end of box
            
            //Draws all the boundary lines
            Imgproc.line(
            inputMat, // What to draw on
            line0pointA, // First point which defines the rectangle
            line0pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
          
            Imgproc.line(
            inputMat, // What to draw on
            line1pointA, // First point which defines the rectangle
            line1pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.line(
            inputMat, // What to draw on
            line2pointA, // First point which defines the rectangle
            line2pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.line(
            inputMat, // What to draw on
            line3pointA, // First point which defines the rectangle
            line3pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.line(
            inputMat, // What to draw on
            line4pointA, // First point which defines the rectangle
            line4pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.line(
            inputMat, // What to draw on
            line5pointA, // First point which defines the rectangle
            line5pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
        l.telemetry.addData("Left hits", leftHits);
        l.telemetry.addData("Middle hits", midHits);
        l.telemetry.addData("Right hits", rightHits);
        l.telemetry.addData("Position", position);
        l.telemetry.update();
        //Shows the image
        return inputMat;
        
    }
    
}
