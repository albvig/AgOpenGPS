using System;
using System.Collections.Generic;

using System.Drawing;
using System.Diagnostics;

namespace AgOpenGPS
{
    public class Triangle : CPolygon
    {
        public Triangle(vec2 p0, vec2 p1, vec2 p2)
        {
            polygonPts = new vec2[] { p0, p1, p2 };
        }
    }

    public class CPolygon
    {
        public CPolygon() { }

        //Create a copy of the bndEarPts from aligned boundary
        public CPolygon(vec2[] _points)
        {
            polygonPts = _points;
        }

        public CPolygon(vec3[] _points)
        {
            polygonPts = new vec2[_points.Length];
            for (int i = 0; i < _points.Length; i++)
            {
                polygonPts[i] = new vec2(_points[i].easting, _points[i].northing);
            }
        }

        public vec2[] polygonPts;

        // Triangulate the polygon. http://www-cgrl.cs.mcgill.ca/~godfried/teaching/cg-projects/97/Ian/cutting_ears.html
        public List<Triangle> Triangulate()
        {
            // Copy the _points into a scratch array.
            vec2[] pts = new vec2[polygonPts.Length];
            Array.Copy(polygonPts, pts, polygonPts.Length);

            // Make a scratch polygon.
            CPolygon pgon = new CPolygon(pts);

            // Orient the polygon clockwise.
            pgon.OrientPolygonClockwise();

            // Make room for the triangles.
            List<Triangle> triangles = new List<Triangle>();

            // While the the polygon has more than three _points, remove an ear.
            while (pgon.polygonPts.Length > 3)
            {
                // Remove an ear from the polygon.
                pgon.RemoveEar(triangles);
            }

            // Copy the last three _points into their own triangle.
            triangles.Add(new Triangle(pgon.polygonPts[0], pgon.polygonPts[1], pgon.polygonPts[2]));

            return triangles;
        }
        
        //Find the indexes of three _points that form an "ear."
        private void FindEar(ref int A, ref int B, ref int C)
        {
            int num_points = polygonPts.Length;

            for (A = 0; A < num_points; A++)
            {
                B = (A + 1) % num_points;
                C = (B + 1) % num_points;

                if (FormsEar(polygonPts, A, B, C)) return;
            }

            // We should never get here because there should
            // always be at least two ears.
            Debug.Assert(false);
        }

        // Return true if the three _points form an ear.
        private static bool FormsEar(vec2[] points, int A, int B, int C)
        {
            // See if the angle ABC is concave.
            if (GetAngle(
                points[A].easting, points[A].northing,
                points[B].easting, points[B].northing,
                points[C].easting, points[C].northing) > 0)
            {
                // This is a concave corner so the triangle
                // cannot be an ear.
                return false;
            }

            // Make the triangle A, B, C.
            Triangle triangle = new Triangle(
                points[A], points[B], points[C]);

            // Check the other _points to see 
            // if they lie in triangle A, B, C.
            for (int i = 0; i < points.Length; i++)
            {
                if ((i != A) && (i != B) && (i != C))
                {
                    if (triangle.PointInPolygon(points[i].easting, points[i].northing))
                    {
                        // This point is in the triangle 
                        // do this is not an ear.
                        return false;
                    }
                }
            }

            // This is an ear.
            return true;
        }

        // Remove an ear from the polygon and add it to the triangles array.
        private void RemoveEar(List<Triangle> triangles)
        {
            // Find an ear.
            int A = 0, B = 0, C = 0;
            FindEar(ref A, ref B, ref C);

            // Create a new triangle for the ear.
            triangles.Add(new Triangle(polygonPts[A], polygonPts[B], polygonPts[C]));

            // Remove the ear from the polygon.
            RemovePointFromArray(B);
        }

        // Remove point target from the array.
        private void RemovePointFromArray(int target)
        {
            vec2[] pts = new vec2[polygonPts.Length - 1];
            Array.Copy(polygonPts, 0, pts, 0, target);
            Array.Copy(polygonPts, target + 1, pts, target, polygonPts.Length - target - 1);
            polygonPts = pts;
        }

        // Return the angle ABC.Return a value between PI and -PI.
        public static double GetAngle(double Ax, double Ay, double Bx, double By, double Cx, double Cy)
        {
            // Get the dot product.
            double dot_product = DotProduct(Ax, Ay, Bx, By, Cx, Cy);

            // Get the cross product.
            double cross_product = CrossProductLength(Ax, Ay, Bx, By, Cx, Cy);

            // Calculate the angle.
            return (double)Math.Atan2(cross_product, dot_product);
        }

        //Self explanatory
        public bool PointInPolygon(double X, double Y)
        {
            // Get the angle between the point and the
            // first and last vertices.
            int max_point = polygonPts.Length - 1;
            double total_angle = GetAngle(
                polygonPts[max_point].easting, polygonPts[max_point].northing,
                X, Y,
                polygonPts[0].easting, polygonPts[0].northing);

            // Add the angles from the point
            // to each other pair of vertices.
            for (int i = 0; i < max_point; i++)
            {
                total_angle += GetAngle(
                    polygonPts[i].easting, polygonPts[i].northing,
                    X, Y,
                    polygonPts[i + 1].easting, polygonPts[i + 1].northing);
            }

            // The total angle should be 2 * PI or -2 * PI if
            // the point is in the polygon and close to zero
            // if the point is outside the polygon.
            return (Math.Abs(total_angle) > 1);
        }

        // Return true if the polygon is oriented clockwise.
        public bool PolygonIsOrientedClockwise()
        {
            return (SignedPolygonArea() < 0);
        }

        // If the polygon is oriented counterclockwise, reverse the order of its _points.
        private void OrientPolygonClockwise()
        {
            if (!PolygonIsOrientedClockwise())
                Array.Reverse(polygonPts);
        }

        // Return the cross product AB x BC.
        public static double CrossProductLength(double Ax, double Ay,
            double Bx, double By, double Cx, double Cy)
        {
            // Get the vectors' coordinates.
            double BAx = Ax - Bx;
            double BAy = Ay - By;
            double BCx = Cx - Bx;
            double BCy = Cy - By;

            // Calculate the Z coordinate of the cross product.
            return (BAx * BCy - BAy * BCx);
        }

        // dot product AB · BC. Note that AB · BC = |AB| * |BC| * Cos(theta).
        private static double DotProduct(double Ax, double Ay,
            double Bx, double By, double Cx, double Cy)
        {
            // Get the vectors' coordinates.
            double BAx = Ax - Bx;
            double BAy = Ay - By;
            double BCx = Cx - Bx;
            double BCy = Cy - By;

            // Calculate the dot product.
            return (BAx * BCx + BAy * BCy);
        }

        // Area Routines ----------------------------------------
        public double PolygonArea()
        {
            // Return the absolute value of the signed area.
            // The signed area is negative if the polygon is
            // oriented clockwise.
            return Math.Abs(SignedPolygonArea());
        }

        // The value will be negative if the polygon is oriented clockwise.
        private double SignedPolygonArea()
        {
            // Add the first point to the end.
            int num_points = polygonPts.Length;
            vec2[] pts = new vec2[num_points + 1];
            polygonPts.CopyTo(pts, 0);
            pts[num_points] = polygonPts[0];

            // Get the areas.
            double area = 0;
            for (int i = 0; i < num_points; i++)
            {
                area +=
                    (pts[i + 1].easting - pts[i].easting) *
                    (pts[i + 1].northing + pts[i].northing) / 2;
            }

            // Return the result.
            return area;
        }

        // Return true if the polygon is convex.
        public bool PolygonIsConvex()
        {
            // For each set of three adjacent _points A, B, C,
            // find the dot product AB · BC. If the sign of
            // all the dot products is the same, the angles
            // are all positive or negative (depending on the
            // order in which we visit them) so the polygon
            // is convex.
            bool got_negative = false;
            bool got_positive = false;
            int num_points = polygonPts.Length;
            int B, C;
            for (int A = 0; A < num_points; A++)
            {
                B = (A + 1) % num_points;
                C = (B + 1) % num_points;

                double cross_product =
                    CrossProductLength(
                        polygonPts[A].easting, polygonPts[A].northing,
                        polygonPts[B].easting, polygonPts[B].northing,
                        polygonPts[C].easting, polygonPts[C].northing);
                if (cross_product < 0)
                {
                    got_negative = true;
                }
                else if (cross_product > 0)
                {
                    got_positive = true;
                }
                if (got_negative && got_positive) return false;
            }

            // If we got this far, the polygon is convex.
            return true;
        }

        public List<Triangle> TriangulateWithInnerPolygon(CPolygon innerPolygon)
        {
            // Ensure the outer polygon is oriented clockwise.
            this.OrientPolygonClockwise();

            // Ensure the inner polygon is oriented counter-clockwise.
            if (innerPolygon.PolygonIsOrientedClockwise())
            {
                Array.Reverse(innerPolygon.polygonPts);
            }

            // Find a bridge between the outer and inner polygons.
            int outerIndex = FindBridgePoint(this.polygonPts, innerPolygon.polygonPts);
            int innerIndex = FindBridgePoint(innerPolygon.polygonPts, this.polygonPts);

            // Stitch the polygons together to form a single polygon.
            vec2[] stitchedPolygon = StitchPolygons(this.polygonPts, innerPolygon.polygonPts, outerIndex, innerIndex);

            // Create a new polygon from the stitched vertices.
            CPolygon stitched = new CPolygon(stitchedPolygon);

            // Triangulate the stitched polygon.
            List<Triangle> triL = stitched.Triangulate();
            triL.Add(new Triangle(this.polygonPts[outerIndex], this.polygonPts[outerIndex + 1], innerPolygon.polygonPts[innerIndex]));
            triL.Add(new Triangle(innerPolygon.polygonPts[innerIndex], this.polygonPts[outerIndex + 1], innerPolygon.polygonPts[innerIndex - 1]));

            return triL;
        }

        private int FindBridgePoint(vec2[] poly1, vec2[] poly2)
        {
            // Example: Find the closest point for simplicity.
            int bestIndex = 0;
            int best = 0;
            double minDistance = double.MaxValue;

            for (int i = 0; i < poly1.Length; i++)
            {
                for (int j = 0; j < poly2.Length; j++)
                {
                    double distance = DistanceSquared(poly1[i], poly2[j]);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        bestIndex = i;
                        best = j;
                    }
                }
            }

            return bestIndex;
        }

        private vec2[] StitchPolygons(vec2[] outer, vec2[] inner, int outerIndex, int innerIndex)
        {
            List<vec2> stitched = new List<vec2>();

            // Add outer vertices up to the bridge point.
            for (int i = 0; i <= outerIndex; i++)
            {
                stitched.Add(outer[i]);
            }

            // Add inner vertices, starting at the bridge point.
            for (int i = innerIndex; i < inner.Length; i++)
            {
                stitched.Add(inner[i]);
            }
            for (int i = 0; i < innerIndex; i++)
            {
                stitched.Add(inner[i]);
            }

            // Complete the outer loop.
            for (int i = outerIndex + 1; i < outer.Length; i++)
            {
                stitched.Add(outer[i]);
            }

            return stitched.ToArray();
        }

        private double DistanceSquared(vec2 p1, vec2 p2)
        {
            double dx = p1.easting - p2.easting;
            double dy = p1.northing - p2.northing;
            return dx * dx + dy * dy;
        }
    }
}
