using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;

namespace FBE_CSharpUI
{
    class CollisionDetector
    {
        //old value is -1 e-1
        private const double CollisionThreshold = -1e-2;
        // Structure that stores the results of the PolygonCollision function
        public struct PolygonCollisionResult {
        // Are the polygons going to intersect forward in time?
        public bool WillIntersect;
        // Are the polygons currently intersecting?
        public bool Intersect;
        // The translation to apply to the first polygon to push the polygons apart.
        public Vector MinimumTranslationVector;
    }

    //Calculate the gravity center of a polygon

     public static Point getCenter(Polygon polygon)
     {
         double x = 0;
         double y = 0;
         int count = polygon.Points.Count;
         foreach (Point p in polygon.Points)
         {
             x += p.X;
             y += p.Y;
         }

         return new Point(x/count,y/count);
     }

    // Calculate the projection of a polygon on an axis
    // and returns it as a [min, max] interval
    public static void ProjectPolygon(Vector axis, Polygon polygon, 
                               ref float min, ref float max) {
        // To project a point on an axis use the dot product
        float dotProduct = (float) Vector.Multiply(axis, polygon.Points[0]-new Point(0,0));
        min = dotProduct;
        max = dotProduct;
        PointCollection points = polygon.Points;
        for (int i = 0; i < polygon.Points.Count; i++) {
            dotProduct = (float) Vector.Multiply(points[i] - new Point(0,0), axis);
            if (dotProduct < min) {
                min = dotProduct;
            } else {
                if (dotProduct> max) {
                    max = dotProduct;
                }
            }
        }
    }

    // Calculate the distance between [minA, maxA] and [minB, maxB]
    // The distance will be negative if the intervals overlap
    public static float IntervalDistance(float minA, float maxA, float minB, float maxB) {
        if (minA < minB) {
            return minB - maxA;
        } else {
            return minA - maxB;
        }
    }

    

    // Check if polygon A is going to collide with polygon B.
    // The last parameter is the *relative* velocity 
    // of the polygons (i.e. velocityA - velocityB)
    public static PolygonCollisionResult PolygonCollision(Polygon polygonA, 
                                  Polygon polygonB, Vector velocity) {
                                     

        PolygonCollisionResult result = new PolygonCollisionResult();
        // If either polygon has no vertices, they do not intersect
        if (polygonA.Points.Count == 0 || polygonB.Points.Count == 0) {
            result.Intersect = false;
            result.WillIntersect = false;
            return result;
        }
        /*
        change here to turn off/on collision detection
         * */
        result.Intersect = true;
        result.WillIntersect = true;     

        int edgeCountA = polygonA.Points.Count;
        int edgeCountB = polygonB.Points.Count;
        int totalCount = edgeCountA + edgeCountB;
        float minIntervalDistance = float.PositiveInfinity;
        Vector translationAxis = new Vector();
        Vector edge;

        // Loop through all the edges of both polygons
        for (int edgeIndex = 0; edgeIndex < edgeCountA + edgeCountB-1; edgeIndex++) {
            if (edgeIndex < edgeCountA) {
                edge = polygonA.Points[(edgeIndex+1)%edgeCountA]-polygonA.Points[edgeIndex];
            } else
            {
                int indexB = edgeIndex - edgeCountA;
                edge = polygonB.Points[(indexB+1)%edgeCountB] - polygonB.Points[indexB];
            }

            // ===== 1. Find if the polygons are currently intersecting =====

            // Find the axis perpendicular to the current edge
            Vector axis = new Vector(-edge.Y, edge.X);
            axis.Normalize();

            // Find the projection of the polygon on the current axis
            float minA = 0; float minB = 0; float maxA = 0; float maxB = 0;
            ProjectPolygon(axis, polygonA, ref minA, ref maxA);
            ProjectPolygon(axis, polygonB, ref minB, ref maxB);

            // Check if the polygon projections are currentlty intersecting
            if (IntervalDistance(minA, maxA, minB, maxB) >= CollisionThreshold)
                result.Intersect = false;

            // ===== 2. Now find if the polygons *will* intersect =====

            // Project the velocity on the current axis
            float velocityProjection = (float) Vector.Multiply(velocity,axis);

            // Get the projection of polygon A during the movement
            if (velocityProjection < 0) {
                minA += velocityProjection;
            } else {
                maxA += velocityProjection;
            }

            // Do the same test as above for the new projection
            float intervalDistance = IntervalDistance(minA, maxA, minB, maxB);
            if (intervalDistance >= CollisionThreshold) result.WillIntersect = false;

            // If the polygons are not intersecting and won't intersect, exit the loop
            if (!result.Intersect && !result.WillIntersect) break;

            // Check if the current interval distance is the minimum one. If so store
            // the interval distance and the current distance.
            // This will be used to calculate the minimum translation vector
            intervalDistance = Math.Abs(intervalDistance);
            if (intervalDistance < minIntervalDistance) {
                minIntervalDistance = intervalDistance;
                translationAxis = axis;
                Vector d = getCenter(polygonA) - getCenter(polygonB);
                //Vector d = polygonA.Center - polygonB.Center;
                if (Vector.Multiply(d,translationAxis) < 0)
                    translationAxis = -translationAxis;
            }
        }

        // The minimum translation vector
        // can be used to push the polygons appart.
        if (result.WillIntersect)
            result.MinimumTranslationVector = 
                   translationAxis * minIntervalDistance;
    
        return result;
    }
        }
}
