package us.ihmc.viewers;

import gov.nasa.arc.sirca.box.BoxConfiguration;
import gov.nasa.arc.sirca.path.ExtState;
import gov.nasa.arc.sirca.path.Path;
import gov.nasa.arc.sirca.rrt.RRTSearch;
import gov.nasa.arc.sirca.util.FlightConstraintsUtility;
import processing.core.PApplet;
import us.ihmc.planning.plan.astar.Topology;
import us.ihmc.planning.plan.rrt.RRT2;
import us.ihmc.planning.plan.rrt.RRTListener;
import us.ihmc.planning.plan.rrt.RRTStateInterface;
import gov.nasa.arc.sirca.rrt.WaypointExtStateClosestPointFunction;
import us.ihmc.planning.world.Obstacle;

import java.awt.*;
import java.util.ArrayList;
import java.util.Hashtable;
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

    RRT2 rrt = null;

    @Override
    public void setup()
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
            listeners.add( rrtSearch );
            listeners.add( this );

            rrtSearch.initRRT(listeners);
            rrt = rrtSearch.rrt;
            rrtSearch.closestPointFunction = new WaypointExtStateClosestPointFunction(rrt, bc.getPath(), rrtSearch.pathConstraints);
            rrt.setClosestPointFunction(rrtSearch.closestPointFunction);

            Topology<ExtState> gridToWorldMapping = new Topology<ExtState>()
            {
                @Override
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
            };

            rrt.setTopology(gridToWorldMapping);


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



    @Override
    public void plannerSetup()
    {

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

        RoutePoint rp = stateToPoint.get(startExtState);

        if ( rp  == null )
        {
            rp = new RoutePoint("start", startExtState,25, Color.green.getRGB(), null);
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
            rp = new RoutePoint( "goal", goalExtState, 25, Color.red.getRGB(), null);
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
        ExtState[] states = new ExtState[] { new ExtState(3600.0, 3600.0, maxAltitude, maxVelocity, Math.toRadians(0.0)),
                                       new ExtState(3900.0, 3600.0, minAltitude + (diffAltitude * 0.8), minVelocity + (diffVelocity * 0.8), Math.toRadians(0.0)),
                                       new ExtState(4000.0, 3600.0, minAltitude + (diffAltitude * 0.8), minVelocity + (diffVelocity * 0.6), Math.toRadians(0.0)),
                                       new ExtState(7600.0, 3600.0, minAltitude + (diffAltitude * 0.8), minVelocity + (diffVelocity * 0.4), Math.toRadians(0.0)),
                                       new ExtState(8500.0, 2700.0, minAltitude + (diffAltitude * 0.6), minVelocity + (diffVelocity * 0.4), Math.toRadians(0.0)),
                                       new ExtState(8500.0, 900.0, minAltitude + (diffAltitude * 0.4), minVelocity + (diffVelocity * 0.4), Math.toRadians(0.0)),
                                       new ExtState(7600.0, 0.0, minAltitude + (diffAltitude * 0.2), minVelocity + (diffVelocity * 0.4), Math.toRadians(0.0)),
                                       new ExtState(3800.0, 0.0, minAltitude, minVelocity + (diffVelocity * 0.2), Math.toRadians(0.0)),
                                       new ExtState(0.0, 0.0, 0.0, 0.0, Math.toRadians(180.0))  };

        ExtState newgoal = null;
        for (int i = 1; i < states.length; i++ )
        {
            rrt.updateStart( states[i-1], newgoal);
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
        ExtState start = new ExtState(100,150, 200, 90, Math.toRadians(0));
        ExtState goal  = randomPoint("goal", null)._Ext_state;

        rrt.updateStart( start, null );
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
            next.parent = startExtState;

            start(startExtState);
            goal(goalExtState);
            expanded( next );

            _points.add( randomPoint("test", null));
        }
        if (key == 'r' )
        {
            Thread t = new Thread() {
                public void run()
                {
//                    rrt.run();
//                    rrt.goal.printPath();
//                    selectPoint(stateToPoint.get(rrt.goal));
                    runTest();
//                    runBox();
                }
            };

            t.start();
        }
    }

}
