// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

/** Add your docs here. */
public class Path {
    private ArrayList<Point> points;
    double currentAngle = 0;
    double previousAngle = 0;
    double previousWanted = 0;

    public Path(ArrayList<Point> points) {
        this.points = points;   
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
        for(int i = 0; i <solutions.length; i++) {
            solutions[i] += intersectX;
        }

        yint = segPoints.get(1).getY() - (segPoints.get(1).getX() * slope);

        Point lookAheadMax = new Point(145, 145, 0);
        Point lookAheadPoint = new Point(0, 0, 0);
        for(int i=0; i < solutions.length; i++) {
            if(segPoints.get(1).getX() > segPoints.get(0).getX()) {
                if(solutions[i] > lookAheadPoint.getX()) {
                    lookAheadPoint.setX(solutions[i]);
                    lookAheadPoint.setY((solutions[i] * slope) + yint);
                    lookAheadPoint.setAngleRad(segPoints.get(1).getAngleRad());
                }
            } else {
                lookAheadPoint = lookAheadMax;
                if(solutions[i] < lookAheadPoint.getX()) {
                    lookAheadPoint.setX(solutions[i]);
                    lookAheadPoint.setY((solutions[i] * slope) + yint);
                    lookAheadPoint.setAngleRad(segPoints.get(1).getAngleRad());
                }
            }
        }

        if(segPoints.get(1).getX() > segPoints.get(0).getX()) {
            if(lookAheadPoint.getX() > segPoints.get(1).getX()) {
                lookAheadPoint = segPoints.get(1);
            }
        } else {
            if(lookAheadPoint.getX() < segPoints.get(1).getX()) {
                lookAheadPoint = segPoints.get(1);
            }
        }

        return new Point(lookAheadPoint.getX(), lookAheadPoint.getY(), lookAheadPoint.getAngleRad());
    }

    /**
     * 
     * @param robotPoint Current location of the robot
     * @param segNum Current segment of the line being travelled
     * @param radius Distance from the robot that we select a point
     * @return Returns the velocities in the x, y, and rotational format. Retrieve values in that order
     */
    public double[] getVelocities(Point robotPoint, int segNum, double radius, boolean shouldPID) {
       Point wanted = getNextPoint(robotPoint, segNum, radius);
        double dx = wanted.getX() - robotPoint.getX();
        double dy = wanted.getY() - robotPoint.getY();

        Point last = getSegment(segNum).get(1);
        double odx = last.getX() - robotPoint.getX();
        double ody = last.getY() - robotPoint.getY();

        double speed = 4.66;
        double mult = shouldPID ? speed : speed;
        double lastVeloc = Math.hypot(odx, ody) * mult; //3.5

        double currentAngle = previousWanted;
        double wantedAngle = ExtraMath.atanNew(dx, dy);
        double added = ExtraMath.angleError(currentAngle, wantedAngle);
        previousWanted = currentAngle - (added / 1);

        if(segNum < points.size() - 1) {
            lastVeloc = 4.66;
        }

        dx = Math.cos(previousWanted) * lastVeloc;
        dy = Math.sin(previousWanted) * lastVeloc;

        double magnitudeTest = Math.hypot(dx, dy);
        if(magnitudeTest < .1 && !shouldPID) {
            dx = dx * .1 / dx;
            dy = dy * .1 / dy;
        }

        // double angleVeloc = (wanted.getAngleRad() - robotPoint.getAngleRad()) * 7;
        double angleVeloc = -ExtraMath.angleError(robotPoint.getAngleRad(), wanted.getAngleRad()) * 3.33;

        double[] solutions = {dx,dy,angleVeloc};
        return solutions;
    }
}
