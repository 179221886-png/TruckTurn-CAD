using System;

namespace TruckTurn
{
    /// <summary>二维跨运车车体位姿。坐标单位为米，HeadingRad 逆时针为正。</summary>
    public struct CarrierPose
    {
        public double X;
        public double Y;
        public double HeadingRad;

        public CarrierPose(double x, double y, double headingRad)
        {
            X = x;
            Y = y;
            HeadingRad = CarrierMath.NormalizeAngleRad(headingRad);
        }
    }

    /// <summary>车体坐标系速度旋量：Vx 向前，Vy 向左，YawRate 逆时针为正。</summary>
    public struct CarrierTwist
    {
        public double Vx;
        public double Vy;
        public double YawRate;

        public CarrierTwist(double vx, double vy, double yawRate)
        {
            Vx = vx;
            Vy = vy;
            YawRate = yawRate;
        }
    }

    public struct CarrierWheelState
    {
        public CarrierWheelId Id;
        public double LocalX;
        public double LocalY;
        public double WorldX;
        public double WorldY;
        public double SteerAngleRad;
        public double RollingSpeed;

        public double SteerAngleDeg { get { return SteerAngleRad * 180.0 / Math.PI; } }
    }

    public sealed class CarrierFrame
    {
        public CarrierPose Pose;
        public CarrierTwist Twist;
        public CarrierSteeringMode Mode;
        public CarrierWheelState[] Wheels;

        public CarrierFrame(CarrierPose pose, CarrierTwist twist, CarrierSteeringMode mode,
            CarrierWheelState[] wheels)
        {
            Pose = pose;
            Twist = twist;
            Mode = mode;
            Wheels = wheels ?? new CarrierWheelState[0];
        }
    }

    public struct CarrierTurnRadii
    {
        public bool IsStraight;
        public double ReferencePoint;
        public double InnerWheel;
        public double OuterWheel;
        public double InnerBody;
        public double OuterBody;
    }

    public static class CarrierMath
    {
        public const double Epsilon = 1e-10;

        public static double NormalizeAngleRad(double angle)
        {
            if (double.IsNaN(angle) || double.IsInfinity(angle)) return angle;
            while (angle <= -Math.PI) angle += 2.0 * Math.PI;
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            return angle;
        }

        public static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        public static bool Finite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
