// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

/** Add your docs here. */
public class Path {
    private ArrayList<Point> points;

    public Path(ArrayList<Point> points) {
        
    }

    /**
     * 
     * @param num current segment of the path described
     * @return
     */
    public ArrayList<Point> getSegment(int num) {
        ArrayList<Point> segPoints= new ArrayList<>();
        if(num > points.size()) {
            return segPoints;
        }
        segPoints.add(points.get(num -1));
        segPoints.add(points.get(num));
        return segPoints;
    }

    /**
     * 
     * @param robotPoint Current Location of the robot
     * @param segNum Current segment of the line being travelled
     * @param radius Distance from robot that we select a point
     * @return Gives back the next point on the line to follow
     */
    public Point getNextPoint(Point robotPoint, int segNum, double radius) {
        ArrayList<Point> segPoints = getSegment(segNum);
        double dy = segPoints.get(1).getY() - segPoints.get(0).getY();
        dy = dy == 0 ? 0.001 : dy;
        double dx = segPoints.get(1).getX() - segPoints.get(0).getX();
        dx = dx == 0 ? 0.001 : dx;
        double slope = dy/dx;
        double yint = segPoints.get(1).getY() - (segPoints.get(1).getX() * slope);
        double pyInt = robotPoint.getY() + (robotPoint.getX() / slope);

        double intersectX = (pyInt - yint) / (slope + (1/slope));
        double intersectY = pyInt - (intersectX / slope);

        yint = segPoints.get(1).getY() - intersectY - ((segPoints.get(1).getX()-intersectX) * slope); 
        pyInt = robotPoint.getY() - intersectY + ((robotPoint.getX() - intersectX) / slope);

        double a = Math.pow(slope, 2) + 1;
        double b = 2 * slope * yint;
        double c = Math.pow(yint, 2) - Math.pow(radius, 2);

        double[] solutions = ExtraMath.solveQuadratic(a, b, c);

        Point lookAheadMax = new Point(145, 145, 0);
        Point lookAheadPoint = new Point(0, 0, 0);
        for(int i=0; i < solutions.length; i++) {
            if(segPoints.get(1).getX() > segPoints.get(0).getX()) {
                if(points.get(i).getX() > lookAheadPoint.getX()) {
                    lookAheadPoint.setX(points.get(i).getX());
                    lookAheadPoint.setY(points.get(i).getY());
                    lookAheadPoint.setAngleRad(points.get(i).getAngleRad());
                }
            } else {
                lookAheadPoint = lookAheadMax;
                if(points.get(i).getX() < lookAheadPoint.getX()) {
                    lookAheadPoint.setX(points.get(i).getX());
                    lookAheadPoint.setY(points.get(i).getY());
                    lookAheadPoint.setAngleRad(points.get(i).getAngleRad());
                }
            }
        }

        return new Point(segNum, segNum, segNum);
    }
}
