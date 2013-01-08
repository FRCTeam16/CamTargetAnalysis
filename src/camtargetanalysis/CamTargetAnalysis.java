//Aaron Sanders (atsanders@gmail.com) - Team 16

package camtargetanalysis;

import edu.wpi.first.smartdashboard.properties.DoubleProperty;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpijavacv.WPIPolygon;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.networktables.NetworkTableKeyNotDefined;
import java.util.Vector;

public class CamTargetAnalysis extends WPICameraExtension {

    private DoubleProperty polygonQuality;
    private IntegerProperty erodeAmount;
    private IntegerProperty dilateAmount;
    private IntegerProperty redMaxProperty;
    private IntegerProperty greenMinProperty;
    private IntegerProperty greenMaxProperty;
    private IntegerProperty blueMinProperty;
    private BooleanProperty showCrosshair;
    private BooleanProperty doTargetProcessing;
    private IntegerProperty crosshairVerticalInPx;
    private IntegerProperty crosshairHorizontalInPx;
    private BooleanProperty showBinaryImage;
    private IntegerProperty derivative;
    private IntegerProperty proportional;
    private IntegerProperty integral;
    private IntegerProperty crosshairBufferInPx;
    private BooleanProperty showBadPolygons;
    private BooleanProperty useCameraTable;
    private BooleanProperty ignoreCamData;

    public static final String NAME = "Cam Target Analysis";
    private WPIPolygon topTarget;
    private WPIPolygon middleLeftTarget;
    private WPIPolygon middleRightTarget;

    NetworkTable table = NetworkTable.getTable("camera");
    
    private int d;
    private int p;
    private int i;

    public CamTargetAnalysis() {
            polygonQuality = new DoubleProperty(this, "Polygon Quality", 30);
            erodeAmount = new IntegerProperty(this, "Erode Amount", 0);
            dilateAmount = new IntegerProperty(this, "Dilate Amount", 9);
            redMaxProperty = new IntegerProperty(this, "Red Maximum", 225);
            greenMinProperty = new IntegerProperty(this, "Green Minimum", 225);
            greenMaxProperty = new IntegerProperty(this, "Green Maximum", 255);
            blueMinProperty = new IntegerProperty(this, "Blue Minimum", 100);
            showBinaryImage = new BooleanProperty(this, "Show Binary Image", false);
            doTargetProcessing = new BooleanProperty(this, "Process Targeting", true);
            showCrosshair = new BooleanProperty(this, "Show Crosshair", true);
            crosshairVerticalInPx = new IntegerProperty(this, "Crosshair V (px)", 120);
            crosshairHorizontalInPx = new IntegerProperty(this, "Crosshair H (px)", 160);
            derivative = new IntegerProperty(this, "Derivative", 0);
            proportional = new IntegerProperty(this, "Proportional", 0);
            integral = new IntegerProperty(this, "Integral", 0);
            crosshairBufferInPx = new IntegerProperty(this, "Crosshair Buffer In Px", 3);
            showBadPolygons = new BooleanProperty(this, "Show Bad Polygons", false);
            useCameraTable = new BooleanProperty(this, "Use camera Network Table", false);
            ignoreCamData = new BooleanProperty(this, "Ignore Processing Data", false);

            topTarget = null;
            middleLeftTarget = null;
            middleRightTarget = null;
            
            p = -1;
            d = -1;
            i = -1;
    }

    @Override
    public WPIImage processImage(WPIColorImage rawImage) {

        if (!doTargetProcessing.getValue()) {
            return rawImage;
        }

        int redMax = redMaxProperty.getValue();
        int greenMin = greenMinProperty.getValue();
        int greenMax = greenMaxProperty.getValue();
        int blueMin = blueMinProperty.getValue();

        WPIBinaryImage blueChan = rawImage.getBlueChannel().getThreshold(blueMin);
        WPIBinaryImage redChan = rawImage.getRedChannel().getThresholdInverted(redMax);
        WPIBinaryImage greenChan1 = rawImage.getGreenChannel().getThreshold(greenMin);
        WPIBinaryImage greenChan2 = rawImage.getGreenChannel().getThresholdInverted(greenMax);

        blueChan.and(redChan);
        blueChan.and(greenChan1);
        blueChan.and(greenChan2);

        redChan.dispose();
        greenChan1.dispose();
        greenChan2.dispose();

        blueChan.erode(erodeAmount.getValue());
        blueChan.dilate(dilateAmount.getValue());

        if (showBinaryImage.getValue()) {
            return blueChan;
        }

        WPIContour[] contours = blueChan.findContours();

        blueChan.dispose();

        List<WPIPolygon> squares = new ArrayList<WPIPolygon>();
        List<WPIPolygon> badPolygons = new ArrayList<WPIPolygon>();

        // find initial polygons
        for (WPIContour c : contours) {
            WPIPolygon p = c.approxPolygon(polygonQuality.getValue());
            double ratio = ((double)c.getHeight()) / ((double)c.getWidth());
            if (p.isConvex() && p.getNumVertices() == 4 && ratio < 1.25 && ratio > 0.5) {
                squares.add(p);
            } else {
                badPolygons.add(p);
            }
        }

        // filter polygons with the same center (i.e. remove inner)
        List<WPIPolygon> polygonsFiltered = filterSimilar(squares);

        if (showBadPolygons.getValue()) {
            // draw the bad polys
            for (WPIPolygon p : badPolygons) {
                rawImage.drawPolygon(p, WPIColor.RED, 1);
            }
        }

        // draw the good polys
        //WPIPolygon topTarget = null;
        topTarget = null;
        for (WPIPolygon p : polygonsFiltered) {
            rawImage.drawPolygon(p, WPIColor.YELLOW, 1);

            // draw center points
            int cx = p.getX() + (p.getWidth() / 2);
            int cy = p.getY() + (p.getHeight() / 2);
            rawImage.drawPoint(new WPIPoint(cx, cy), WPIColor.YELLOW, 2);

            if (topTarget == null)
            {
                topTarget = p;
            }
            if (topTarget.getY() > p.getY())
            {
                topTarget = p;
            }
        }

        //WPIPolygon middleLeftTarget = null;
        //WPIPolygon middleRightTarget = null;
        middleLeftTarget = null;
        middleRightTarget = null;
        for (WPIPolygon p : polygonsFiltered) {
            if (p == topTarget) continue;

            int topTargetBottom_px = (topTarget.getY() + topTarget.getHeight());
            int pTargetTop_px = p.getY();
            
            if ((topTargetBottom_px >= pTargetTop_px))
            {
                if (topTarget.getX() < p.getX())
                {
                    middleLeftTarget = topTarget;
                    middleRightTarget = p;
                }
                else
                {
                    middleLeftTarget = p;
                    middleRightTarget = topTarget;
                }
            }
        }

        boolean targetFound = false;

        if (middleLeftTarget != null && middleRightTarget != null)
        {
            rawImage.drawPolygon(middleLeftTarget, WPIColor.YELLOW, 2);
            rawImage.drawPolygon(middleRightTarget, WPIColor.YELLOW, 2);
             
            //get value between middle targets
            int tarXFromCenter = ((middleRightTarget.getX() + middleRightTarget.getWidth()) + middleLeftTarget.getX()) / 2;
            
            rawImage.drawLine(new WPIPoint(tarXFromCenter,0), new WPIPoint(tarXFromCenter,rawImage.getHeight()), WPIColor.YELLOW, 1);
            //if (Robot.getTable().containsKey("IsShooting") && Robot.getTable().getBoolean("IsShooting"))
            //{

                if (useCameraTable.getValue()) {
                    synchronized(table){
                       table.beginTransaction();
                       table.putInt("TargetXFromCenter", crosshairHorizontalInPx.getValue()-tarXFromCenter);
                       table.putInt("TargetHeight", middleRightTarget.getHeight());
                       table.putInt("TargetWidth", middleRightTarget.getWidth());
                       table.putBoolean("Found", true);
                       table.putBoolean("IgnoreCamData", ignoreCamData.getValue());
                       table.endTransaction();
                    }
                }
                else {
                    Robot.getTable().putInt("TargetXFromCenter", crosshairHorizontalInPx.getValue()-tarXFromCenter);
                    Robot.getTable().putInt("TargetHeight", middleRightTarget.getHeight());
                    Robot.getTable().putInt("TargetWidth", middleRightTarget.getWidth());
                    Robot.getTable().putBoolean("Found", true);
                    Robot.getTable().putBoolean("IgnoreCamData", ignoreCamData.getValue());
                }
                //System.out.println("Target Found:true(middle);TargetXFromCenter:" + (crosshairHorizontalInPx.getValue()-tarXFromCenter) + ";TargetHeight:" + middleRightTarget.getHeight() + ";ARatio:" + (middleRightTarget.getHeight()/middleRightTarget.getWidth()) + ";");
            //}

            targetFound = true;

        }
        else if(topTarget != null)
        {
            //rawImage.drawLine(new WPIPoint(topTarget.getX(),topTarget.getY()+(topTarget.getHeight()/2)), new WPIPoint(topTarget.getX()+topTarget.getWidth(), (topTarget.getY()+topTarget.getHeight()/2)), WPIColor.YELLOW, 1);
            //rawImage.drawLine(new WPIPoint(topTarget.getX()+(topTarget.getWidth()/2),topTarget.getY()), new WPIPoint(topTarget.getX()+(topTarget.getWidth()/2),topTarget.getY()+topTarget.getHeight()), WPIColor.YELLOW, 1);
            
            rawImage.drawLine(new WPIPoint(0,topTarget.getY()+(topTarget.getHeight()/2)), new WPIPoint(rawImage.getWidth(), (topTarget.getY()+topTarget.getHeight()/2)), WPIColor.YELLOW, 1);
            rawImage.drawLine(new WPIPoint(topTarget.getX()+(topTarget.getWidth()/2),0), new WPIPoint(topTarget.getX()+(topTarget.getWidth()/2),rawImage.getHeight()), WPIColor.YELLOW, 1);
            
            //if (Robot.getTable().containsKey("IsShooting") && Robot.getTable().getBoolean("IsShooting"))
            //{
                if (useCameraTable.getValue()) {
                    synchronized(table){
                       table.beginTransaction();
                       table.putInt("TargetXFromCenter", crosshairHorizontalInPx.getValue() - (topTarget.getX() + (topTarget.getWidth() / 2)));
                       table.putInt("TargetHeight", topTarget.getHeight());
                       table.putInt("TargetWidth", topTarget.getWidth());
                       table.putBoolean("Found", true);
                       table.putBoolean("IgnoreCamData", ignoreCamData.getValue());
                       table.endTransaction();
                    }
                }
                else {
                    Robot.getTable().putInt("TargetXFromCenter", crosshairHorizontalInPx.getValue() - (topTarget.getX() + (topTarget.getWidth() / 2)));
                    Robot.getTable().putInt("TargetHeight", topTarget.getHeight());
                    Robot.getTable().putInt("TargetWidth", topTarget.getWidth());
                    Robot.getTable().putBoolean("Found", true);
                    Robot.getTable().putBoolean("IgnoreCamData", ignoreCamData.getValue());
                }
                //System.out.println("Target Found:true(top);TargetXFromCenter:" + (crosshairHorizontalInPx.getValue() - (topTarget.getX() + (topTarget.getWidth() / 2))) + ";TargetHeight:" + topTarget.getHeight() + ";ARatio:" + (topTarget.getHeight()/topTarget.getWidth()) + ";");
            //}

            targetFound = true;
        }

        if (!targetFound)
        {
            if (useCameraTable.getValue()) {
                table.putBoolean("Found", false);
            }
            else {
                Robot.getTable().putBoolean("Found", false);
            }
        }

        if (d != derivative.getValue())
        {
            if (useCameraTable.getValue()) {
                table.putInt("Derivative", derivative.getValue());
            }
            else {
                Robot.getTable().putInt("Derivative", derivative.getValue());
            }
            d = derivative.getValue();
        }

        if (i != integral.getValue())
        {
            if (useCameraTable.getValue()) {
                table.putInt("Integral", integral.getValue());
            }
            else {
                Robot.getTable().putInt("Integral", integral.getValue());
            }
            i = integral.getValue();
        }

        if (p != proportional.getValue())
        {
            if (useCameraTable.getValue()) {
                table.putInt("Proportional", proportional.getValue());
            }
            else {
                Robot.getTable().putInt("Proportional", proportional.getValue());
            }
            p = proportional.getValue();
        }

        return rawImage;
    }

    private List<WPIPolygon> filterSimilar(List<WPIPolygon> polys) {
        List<WPIPolygon> ret = new ArrayList<WPIPolygon>();
        ret.addAll(polys);

        List<WPIPolygon> removalQueue = new ArrayList<WPIPolygon>();

        for (WPIPolygon p : polys) {
            // find all polygons with similar center points
            List<WPIPolygon> similar = getSimilar(p, polys);

            // find the largest of the similar polygons
            WPIPolygon largest = getLargest(similar);

            // remove the largest poly
            similar.remove(largest);

            // queue the smaller polygons for removal
            removalQueue.addAll(similar);
        }

        // remove everything in the removal queue
        for (WPIPolygon p : removalQueue) {
            removeAll(p, ret);
        }

        return ret;
    }

    /**
     * Finds polygons with similar centers
     * @param poly the polygon to use for comparison
     * @param pool the pool of polygons to compare against
     * @return a list of similar polygons
     */
    private List<WPIPolygon> getSimilar(WPIPolygon poly, List<WPIPolygon> pool) {
        List<WPIPolygon> ret = new ArrayList<WPIPolygon>();

        int xCenter = poly.getX() + (poly.getWidth() / 2);
        int yCenter = poly.getY() + (poly.getHeight() / 2);

        for (WPIPolygon p : pool) {
            int pcx = p.getX() + (p.getWidth() / 2);
            int pcy = p.getY() + (p.getHeight() / 2);

            int dx = Math.abs(pcx - xCenter);
            int dy = Math.abs(pcy - yCenter);

            int distSquared = (dx * dx) + (dy * dy);

            if (distSquared < 20 * 20) {
                    ret.add(p);
            } // ignore this rect if it's too small
        }

        return ret;
    }

    public WPIPolygon getLargest(List<WPIPolygon> pool) {
        WPIPolygon ret = null;

        for (WPIPolygon p : pool) {
            if (ret == null || p.getArea() > ret.getArea()) {
                ret = p;
            }
        }

        return ret;
    }

    public <T> void removeAll(T obj, List<T> pool) {
        while (pool.contains(obj)) {
            pool.remove(obj);
        }
    }

    @Override
    protected void paintComponent(Graphics g) {

        if (getDrawnImage() != null) {
            BufferedImage image = getDrawnImage();
            Graphics2D g2d = image.createGraphics();
            g2d.setColor(Color.BLUE);
            boolean foundX = false;
            boolean foundY = false;

            if (showCrosshair.getValue())
            {
               if (middleLeftTarget != null && middleRightTarget != null) {
                   int tarXFromCenter = ((middleRightTarget.getX() + middleRightTarget.getWidth()) + middleLeftTarget.getX()) / 2;

                   if (tarXFromCenter >= (crosshairHorizontalInPx.getValue() - crosshairBufferInPx.getValue()) && tarXFromCenter <= (crosshairHorizontalInPx.getValue() + crosshairBufferInPx.getValue()))
                   {
                        foundY = true;
                   }
                   foundX = false;
               }
               else if(topTarget != null)
               {
                   int topTargetCenterX = (topTarget.getX()+(topTarget.getWidth()/2));

                   if (topTargetCenterX >= (crosshairHorizontalInPx.getValue() - crosshairBufferInPx.getValue()) && topTargetCenterX <= (crosshairHorizontalInPx.getValue() + crosshairBufferInPx.getValue()))
                   {
                        foundY = true;
                   }

                   int topTargetCenterY = (topTarget.getY()+(topTarget.getHeight()/2));

                   if (topTargetCenterY >= (crosshairVerticalInPx.getValue() - crosshairBufferInPx.getValue()) && topTargetCenterY <= (crosshairVerticalInPx.getValue() + crosshairBufferInPx.getValue()))
                   {
                        foundX = true;
                   }
               }

               if (foundX) {
                    g2d.setColor(Color.GREEN);
               }
               else
               {
                    g2d.setColor(Color.BLUE);
               }
               g2d.drawLine(0, crosshairVerticalInPx.getValue(), image.getWidth(), crosshairVerticalInPx.getValue());
               g2d.drawLine(0, crosshairVerticalInPx.getValue() - 1, image.getWidth(), crosshairVerticalInPx.getValue()-1);
               g2d.drawLine(0, crosshairVerticalInPx.getValue() + 1, image.getWidth(), crosshairVerticalInPx.getValue()+1);
               
               if (foundY) {
                    g2d.setColor(Color.GREEN);
               }
               else
               {
                    g2d.setColor(Color.BLUE);
               }
               g2d.drawLine(crosshairHorizontalInPx.getValue(), 0, crosshairHorizontalInPx.getValue(), image.getHeight());
               g2d.drawLine(crosshairHorizontalInPx.getValue() - 1, 0, crosshairHorizontalInPx.getValue() - 1, image.getHeight());
               g2d.drawLine(crosshairHorizontalInPx.getValue() - 1, 0, crosshairHorizontalInPx.getValue() - 1, image.getHeight());
            }

            g2d.dispose();
        }

        super.paintComponent(g);
    }
}