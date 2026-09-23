using System;

namespace Gssoft.Gscad.Geometry
{
    /// <summary>
    /// 浩辰CAD 几何类型的轻量替身，仅提供 TruckKinematics / GeometryUtil 实际用到的成员。
    /// 目的：让插件主工程的真实源码可以脱离 CAD 环境编译进控制台程序做数值校验。
    /// </summary>
    public struct Point2d
    {
        public double X;
        public double Y;

        public Point2d(double x, double y) { X = x; Y = y; }

        public static Point2d Origin => new Point2d(0.0, 0.0);

        public static Point2d operator +(Point2d a, Vector2d v) => new Point2d(a.X + v.X, a.Y + v.Y);
        public static Point2d operator -(Point2d a, Vector2d v) => new Point2d(a.X - v.X, a.Y - v.Y);
        public static Vector2d operator -(Point2d a, Point2d b) => new Vector2d(a.X - b.X, a.Y - b.Y);

        public override string ToString() => string.Format("({0:F3}, {1:F3})", X, Y);
    }

    public struct Point3d
    {
        public double X;
        public double Y;
        public double Z;

        public Point3d(double x, double y, double z) { X = x; Y = y; Z = z; }
        public static Point3d Origin => new Point3d(0.0, 0.0, 0.0);
    }

    public struct Vector2d
    {
        public double X;
        public double Y;

        public Vector2d(double x, double y) { X = x; Y = y; }

        public double Length => Math.Sqrt(X * X + Y * Y);
        public double LengthSq => X * X + Y * Y;

        public Vector2d GetNormal()
        {
            double len = Length;
            if (len < 1e-12) throw new InvalidOperationException("Cannot normalize zero vector.");
            return new Vector2d(X / len, Y / len);
        }

        public static Vector2d XAxis => new Vector2d(1.0, 0.0);
        public static Vector2d YAxis => new Vector2d(0.0, 1.0);

        public static Vector2d operator +(Vector2d a, Vector2d b) => new Vector2d(a.X + b.X, a.Y + b.Y);
        public static Vector2d operator -(Vector2d a, Vector2d b) => new Vector2d(a.X - b.X, a.Y - b.Y);
        public static Vector2d operator -(Vector2d a) => new Vector2d(-a.X, -a.Y);
        public static Vector2d operator *(Vector2d a, double s) => new Vector2d(a.X * s, a.Y * s);
        public static Vector2d operator *(double s, Vector2d a) => new Vector2d(a.X * s, a.Y * s);
        public static double operator *(Vector2d a, Vector2d b) => a.X * b.X + a.Y * b.Y;

        public double DotProduct(Vector2d b) => X * b.X + Y * b.Y;
        public double CrossProduct(Vector2d b) => X * b.Y - Y * b.X;

        public override string ToString() => string.Format("[{0:F3}, {1:F3}]", X, Y);
    }
}
