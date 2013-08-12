package us.ihmc.viewers;


import gov.nasa.arc.sirca.approachPatterns.DogLegApproachPattern;
import gov.nasa.arc.sirca.box.BoxConfiguration;
import gov.nasa.arc.sirca.maneuverabilityFunction.ApproachConstraints;
import gov.nasa.arc.sirca.maneuverabilityFunction.PathConstraints;
import gov.nasa.arc.sirca.path.ExtState;
import gov.nasa.arc.sirca.path.Path;
import gov.nasa.arc.sirca.rrt.ExtStateClosestPointFunction;
import gov.nasa.arc.sirca.rrt.RRTSearch;
import gov.nasa.arc.sirca.rrt.WaypointExtStateClosestPointFunction;
import gov.nasa.arc.sirca.util.FlightConstraintsUtility;
import org.apache.commons.collections.buffer.CircularFifoBuffer;
import processing.core.PApplet;
import processing.core.PConstants;
import us.ihmc.planning.configurationSpace.Topology;
import us.ihmc.planning.plan.rrt.RRT;
import us.ihmc.planning.plan.rrt.RRTListener;
import us.ihmc.planning.plan.rrt.RRTStateInterface;
import us.ihmc.planning.world.Obstacle;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;

import javax.vecmath.Point3d;
import java.awt.*;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.Vector;

/**
 * Created with IntelliJ IDEA.
 * User: teskridge
 * Date: 8/27/12
 * Time: 11:23 AM
 * To change this template use File | Settings | File Templates.
 */
public class RRTViewer extends RouteViewer3D implements RRTListener<ExtState>
{
    private static int numExpanded = 0;
    private Hashtable<ExtState, RoutePoint> stateToPoint = new Hashtable<ExtState, RoutePoint>();

    RRT rrt = null;


    public void setupBox()
    {
        super.setup();

        /*
        int maxDistance = 25;
        double[] maxBounds = new double[] {800,500,500,180,360};  // x,y,z,airspeed,heading
        double[] minBounds = new double[] {0,0,0,0,0};
        ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>();

        ArrayList<Point2d> pointList = new ArrayList<Point2d>();
        pointList.add(new Point2d(2.0, 0.0));
        pointList.add(new Point2d(2.0, 10.0));
        pointList.add(new Point2d(4.0, 10.0));
        pointList.add(new Point2d(4.0, 0.0));
        ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d(pointList);
        Obstacle ob1 = new Obstacle("obstacle_1", convexPolygon2d, 0.0, 3.0);


        pointList = new ArrayList<Point2d>();
        pointList.add(new Point2d(2.0, 0.0));
        pointList.add(new Point2d(2.0, 6.0));
        pointList.add(new Point2d(8.0, 6.0));
        pointList.add(new Point2d(8.0, 0.0));
        convexPolygon2d = new ConvexPolygon2d(pointList);
        Obstacle ob2 = new Obstacle("obstacle_2", convexPolygon2d, 3.0, 3.0);

        obstacles.add ( ob1 );
        obstacles.add ( ob2 );

        rrt = new RRT(null,null,maxDistance,maxBounds,minBounds,obstacles);
        rrt.addListener(this);
        */

        BoxConfiguration bc = new BoxConfiguration();
        RRTSearch rrtSearch = new RRTSearch(bc, false);

        double bestScore = Double.MAX_VALUE;
        Path bestPath = null;
        for (int i = 0; i < 40; i++ )
        {
            long startTime = System.currentTimeMillis();
            Vector<RRTListener<ExtState>> listeners = new Vector<RRTListener<ExtState>>();
//            listeners.add( rrtSearch );
            listeners.add( this );

            rrtSearch.initRRT(listeners);
            rrt = rrtSearch.rrt;
            rrtSearch.closestPointFunction = new WaypointExtStateClosestPointFunction(rrt, bc.getPath(), (PathConstraints)rrtSearch.constraints);
            rrt.setClosestPointFunction(rrtSearch.closestPointFunction);

            Topology<ExtState> gridToWorldMapping = new Topology<ExtState>()
            {

                public boolean isOpenSpace(ExtState extState)
                {
                    return true;
                }

                @Override
                public boolean isObstacle(ExtState extState)
                {
                    return false;
                }

                @Override
                public boolean isWithinBounds(ExtState extState)
                {
                    return true;
                }

                @Override
                public boolean pathExistsBetween(ExtState start, ExtState end)
                {
                    return rrt.graph.contains(start) && rrt.graph.contains(end);
                }
            };

            Path p = rrtSearch.generateRRT( );

            if (!FlightConstraintsUtility.isValidPath(p, bc))
            {
                FlightConstraintsUtility.DEBUG = true;
                FlightConstraintsUtility.isValidPath(p, bc);
                FlightConstraintsUtility.DEBUG = false;
            }
            else
            {
                double thisScore = p.getScore();
                System.out.println(i+":"+ thisScore+" " + (System.currentTimeMillis() - startTime) + " msec");
                if ( thisScore < bestScore )
                {
                    bestScore = thisScore;
                    bestPath = p;
                }
            }
        }

        System.out.println("Best score is : " + bestScore);
        System.out.println("Best Path is : ");
        if ( bestPath != null)
            System.out.println(bestPath.toString());



    }

    public void draw()
    {
        background(0);
        drawObstacles();
        drawRoutePoints();
        drawBoundingBox();
        drawTest();
        drawInfoLabel();
    }

    public void drawTest()
    {
        pushStyle();

        float alpha = 255;
        float alphaDifference = (200 / testPoints.maxSize());
        int i = 0;
        for (Iterator it = testPoints.iterator(); it.hasNext(); )
        {
            Vector<ExtState> pair = (Vector<ExtState>) it.next();
            fill(200,200,200,alpha-i*alphaDifference);
            stroke(255,255,255,alpha-i*alphaDifference);
            strokeWeight(4.0f);
            ExtState randomPoint = pair.elementAt(0);
            ExtState closestPoint = pair.elementAt(1);
            line(randomPoint.getXf(), randomPoint.getYf(), randomPoint.getZf(),
                    closestPoint.getXf(), closestPoint.getYf(), closestPoint.getZf());
            i++;
        }
        popStyle();
    }


    @Override
    public void drawInfoLabel()
    {
        textMode(PConstants.SCREEN);
        textFont(createFont("Arial Black", 12, true));
        stroke(255, 255, 255, 255);
        fill(255, 255, 255, 255);
        float[] camPos = _camera.getPosition();
        float[] camRot = _camera.getRotations();
        //text("x:" + camPos[0] + " y:" + camPos[1] + " z:" + camPos[2], 20f, 20f);
        //text("xr:" + camRot[0] + " yr:" + camRot[1] + " zr:" + camRot[2], 20f, 40f);
        if ( rrt == null )
            text((int)frameRate + "fps " + _points.size() + " nodes", 20f, 60f);
        else
            text((int)frameRate + "fps " + _points.size() + " nodes closest " + rrt.getClosestPointDistanceFromGoal(), 20f, 60f);
    }



    @Override
    public void plannerSetup()
    {

    }


    CircularFifoBuffer testPoints = new CircularFifoBuffer(5);

    @Override
    public void test( ExtState randomState, ExtState closestState)
    {
        Vector<ExtState> pair = new Vector(2);
        pair.add(randomState);
        pair.add(closestState);
        testPoints.add( pair );
    }

    @Override
    public void expanded(ExtState extState)
    {
        RRTStateInterface parent = rrt.getParent(extState);
        if ( parent != null)
        {

            RoutePoint parentPoint = stateToPoint.get(parent);
            if ( parentPoint == null )
            {
                System.out.println("Big error " + parent + " routepoint not found");
                return;
            }

            RoutePoint rp = new RoutePoint(numExpanded+"", extState,10,defaultSphereColor,parentPoint);
            stateToPoint.put(extState, rp );
            this._points.add( rp );
            numExpanded++;
        }
    }

    @Override
    public void start(ExtState startExtState)
    {

        stateToPoint.clear();
        this._points.clear();

        RoutePoint rp = stateToPoint.get(startExtState);

        if ( rp  == null )
        {
            rp = new RoutePoint("start", startExtState,50, Color.green.getRGB(), null);
            stateToPoint.put(startExtState,rp);
            this._points.add(rp);
        }
    }

    @Override
    public void goal(ExtState goalExtState)
    {
        RoutePoint rp = stateToPoint.get(goalExtState);
        if ( rp == null )
        {
            rp = new RoutePoint( "goal", goalExtState, 50, Color.red.getRGB(), null);
            stateToPoint.put(goalExtState, rp );
            this._points.add(rp);
            for ( RoutePoint p : this._points)
            {
                if ( p._name.equals("goal"))
                {
                    p._rgbColor = p._originalColor = defaultSphereColor;
                    p._name = "goal" + numExpanded;
                }
            }
        }
    }

    @Override
    public void obstacles(ArrayList<Obstacle> obstacles)
    {
        for ( Obstacle obs : obstacles )
        {
            this._obstacles.add( obs );
        }
    }


    int numObstacles= 0;
    public void obstacles(ArrayList<ConvexPolygon2d> obstacles, boolean flag)
    {
        for ( ConvexPolygon2d poly : obstacles)
        {
            _obstacles.add( new Obstacle("obst"+numObstacles,poly,0, 1500) );
            numObstacles++;
        }
    }

    public static void main(String[] args)
    {
        PApplet.main(new String[]{"--bgcolor=#000000", "us.ihmc.viewers.RRTViewer"});

    }

    private void runBox()
    {
        double maxAltitude = 1500.0;
        double minAltitude = 300.0;
        // unit conversion utilities
        double KTS_TO_FPS = 1.688;
        double G_TO_FPS = 32.17;

        // dynamic constraints
        double maxAcceleration = 0.0;
        double maxDeceleration = -0.1 * G_TO_FPS;

        double maxVelocity = 135.0 * KTS_TO_FPS;
        double minVelocity = 70.0 * KTS_TO_FPS;
        double diffVelocity = maxVelocity - minVelocity;

        double diffAltitude = maxAltitude - minAltitude;


        // waypoints
        ExtState[] states = new ExtState[] { new ExtState(3600.0, 3600.0, maxAltitude, maxVelocity, Math.toRadians(0.0), Math.toRadians(0)),
                                       new ExtState(3900.0, 3600.0, minAltitude + (diffAltitude * 0.8), minVelocity + (diffVelocity * 0.8), Math.toRadians(0.0), 0),
                                       new ExtState(4000.0, 3600.0, minAltitude + (diffAltitude * 0.8), minVelocity + (diffVelocity * 0.6), Math.toRadians(0.0), 0),
                                       new ExtState(7600.0, 3600.0, minAltitude + (diffAltitude * 0.8), minVelocity + (diffVelocity * 0.4), Math.toRadians(0.0), 0),
                                       new ExtState(8500.0, 2700.0, minAltitude + (diffAltitude * 0.6), minVelocity + (diffVelocity * 0.4), Math.toRadians(0.0), 0),
                                       new ExtState(8500.0, 900.0, minAltitude + (diffAltitude * 0.4), minVelocity + (diffVelocity * 0.4), Math.toRadians(0.0), 0),
                                       new ExtState(7600.0, 0.0, minAltitude + (diffAltitude * 0.2), minVelocity + (diffVelocity * 0.4), Math.toRadians(0.0), 0),
                                       new ExtState(3800.0, 0.0, minAltitude, minVelocity + (diffVelocity * 0.2), Math.toRadians(0.0), 0),
                                       new ExtState(0.0, 0.0, 0.0, 0.0, Math.toRadians(180.0), 0)  };

        ExtState newgoal = null;
        for (int i = 1; i < states.length; i++ )
        {
            rrt.start = states[i-1];
//            rrt.start = states[i-1];
//            start(states[i-1]);
//            if ( i != 1)
//            {
//                rrt.start.parent = rrt.goal;
//            }
            rrt.goal = states[i];
            goal(states[i]);
            newgoal = states[i];
            rrt.run();
        }
        selectPoint(stateToPoint.get(rrt.goal));
    }

    private void runTest()
    {
        ExtState start = new ExtState(100,150, 200, 90, Math.toRadians(0), Math.toRadians(0));
        ExtState goal  = randomPoint("goal", null)._Ext_state;

        rrt.start = start;
        rrt.goal = goal;
        goal(goal);
        rrt.run();
        selectPoint( stateToPoint.get( rrt.goal ));

    }

    @Override
    public void keyPressed()
    {
        super.keyPressed();    //Do all the other routeViewer3D stuff

        if ( key == 'R')
        {
            ExtState startExtState = randomPoint("start", null)._Ext_state;
            ExtState goalExtState = randomPoint("goal", null)._Ext_state;
            ExtState next = randomPoint("next", null)._Ext_state;

            start(startExtState);
            goal(goalExtState);
            expanded( next );

            _points.add( randomPoint("test", null));
        }
        if (key == 'r' )
        {
            final RRTViewer thisViewer = this;
            Thread t = new Thread() {
                public void run()
                {
                    final ApproachConstraints approach = new DogLegApproachPattern();
                    constraints = approach;
                    RRTSearch rrtSearch = new RRTSearch(approach, false);
                    thisViewer.obstacles( approach.getObstacles(), true);

                    double bestScore = Double.MAX_VALUE;
                    Path bestPath = null;
                    for (int i = 0; i < 40; i++ )
                    {
                        long startTime = System.currentTimeMillis();
                        Vector<RRTListener<ExtState>> listeners = new Vector<RRTListener<ExtState>>();
//                        listeners.add( rrtSearch );
                        listeners.add( thisViewer );

                        rrtSearch.initRRT(listeners);
                        rrt = rrtSearch.rrt;
                        rrtSearch.closestPointFunction = new ExtStateClosestPointFunction(rrt, approach.getBounds().getMinPoint(), approach.getBounds().getMaxPoint(), approach);
                        rrt.setClosestPointFunction(rrtSearch.closestPointFunction);

                        Topology<ExtState> gridToWorldMapping = new Topology<ExtState>()
                        {

                            public boolean isOpenSpace(ExtState extState)
                            {
                                return ! isObstacle(extState);
                            }

                            @Override
                            public boolean isObstacle(ExtState extState)
                            {
                                for ( ConvexPolygon2d obst : approach.getObstacles())
                                {
                                    if ( obst.isPointInside(extState.getX(), extState.getY()))
                                    {
                                        return true;
                                    }
                                }

                                return false;
                            }

                            @Override
                            public boolean isWithinBounds(ExtState extState)
                            {
                                Point3d min = approach.getBounds().getMinPoint();
                                Point3d max = approach.getBounds().getMaxPoint();

                                return (( min.getX() < extState.getX() && extState.getX() <= max.getX()) &&
                                        ( min.getY() < extState.getY() && extState.getY() <= max.getY()) &&
                                        ( min.getZ() < extState.getZ() && extState.getZ() <= max.getZ()) &&
                                        ( approach.getMinVelocity() <= extState.getVelocity() && extState.getVelocity() < approach.getMaxVelocity()));
                            }

                            @Override
                            public boolean pathExistsBetween(ExtState extState, ExtState extState2)
                            {
                                return rrt.graph.contains(extState) && rrt.graph.contains(extState2);
                            }
                        };



                        Path p = rrtSearch.generateRRT( );// gridToWorldMapping );

                        if (p != null )
                        {
                            if (!FlightConstraintsUtility.isValidPath(p, rrtSearch.constraints))
                            {
                                FlightConstraintsUtility.DEBUG = true;
                                FlightConstraintsUtility.isValidPath(p, rrtSearch.constraints);
                                FlightConstraintsUtility.DEBUG = false;
                            }
                            else
                            {
                                double thisScore = p.getScore();
                                System.out.println(i+":"+ thisScore+" " + (System.currentTimeMillis() - startTime) + " msec");
                                if ( thisScore < bestScore )
                                {
                                    bestScore = thisScore;
                                    bestPath = p;
                                }
                            }
                        }

                    }

                    System.out.println("Best score is : " + bestScore);
                    System.out.println("Best Path is : ");
                    if ( bestPath != null)
                        System.out.println(bestPath.toString());
                }
            };

            t.start();
        }
    }

}
