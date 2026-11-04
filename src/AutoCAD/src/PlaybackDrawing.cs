using System;
using System.Collections.Generic;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using GI = Autodesk.AutoCAD.GraphicsInterface;

namespace TruckTurn
{
    /// <summary>
    /// One moving vehicle, represented by a fixed, reusable set of native transient polylines.
    /// Managed Drawable cannot manufacture its own native object on every supported CAD host.
    /// No entity here is appended to a database, and no envelope is recomputed.
    /// </summary>
    internal sealed class PlaybackDrawable : IDisposable
    {
        internal sealed class Stroke
        {
            public Point3d[] Points;
            public bool Closed;
            public short Color;
        }

        readonly List<Polyline> entities = new List<Polyline>();
        readonly List<bool> registered = new List<bool>();
        bool disposed;
        public bool HasRegisteredEntities { get { return registered.Exists(value => value); } }

        public void SetPose(PlaybackRoute route, PlaybackPose pose, Matrix3d transform)
        {
            var next = new List<Stroke>();
            if (route.Kind == PlaybackVehicleKind.Carrier)
                Carrier(next, route.CarrierParameters, pose, transform);
            else
                Truck(next, route.TruckParameters, pose, transform,
                    route.Kind == PlaybackVehicleKind.Fire ? (short)1 : (short)3);
            if (disposed) throw new ObjectDisposedException("PlaybackDrawable");
            if (entities.Count == 0)
            {
                foreach (Stroke line in next)
                {
                    var polyline = new Polyline(line.Points.Length);
                    entities.Add(polyline);
                    registered.Add(false);
                    for (int i = 0; i < line.Points.Length; i++)
                        polyline.AddVertexAt(i, new Point2d(line.Points[i].X, line.Points[i].Y), 0, 0, 0);
                }
            }
            if (entities.Count != next.Count)
                throw new InvalidOperationException("回放车辆轮廓发生变化，已停止以保护临时图形。");
            for (int n = 0; n < next.Count; n++)
            {
                Stroke line = next[n];
                Polyline entity = entities[n];
                if (entity.NumberOfVertices != line.Points.Length)
                    throw new InvalidOperationException("回放车辆轮廓顶点数不一致。");
                for (int i = 0; i < line.Points.Length; i++)
                    entity.SetPointAt(i, new Point2d(line.Points[i].X, line.Points[i].Y));
                entity.Elevation = line.Points[0].Z;
                entity.Closed = line.Closed;
                entity.ColorIndex = line.Color;
            }
        }

        public void Update(GI.TransientManager manager, IntegerCollection viewports)
        {
            if (disposed) throw new ObjectDisposedException("PlaybackDrawable");
            for (int i = 0; i < entities.Count; i++)
            {
                if (registered[i]) manager.UpdateTransient(entities[i], viewports);
                else
                {
                    // A native call can throw after partially registering its drawable.
                    // In that uncertain case cleanup must attempt Erase before Dispose.
                    registered[i] = true;
                    registered[i] = manager.AddTransient(entities[i],
                        GI.TransientDrawingMode.DirectShortTerm, 128, viewports);
                    if (!registered[i]) throw new InvalidOperationException("CAD 未能创建临时回放图形。");
                }
            }
        }

        public void Erase(GI.TransientManager manager, IntegerCollection viewports)
        {
            System.Exception first = null;
            for (int i = 0; i < entities.Count; i++)
            {
                if (!registered[i]) continue;
                try
                {
                    if (manager.EraseTransient(entities[i], viewports)) registered[i] = false;
                    else if (first == null)
                        first = new InvalidOperationException("CAD 尚未确认移除全部回放临时图形。");
                }
                catch (System.Exception ex) { if (first == null) first = ex; }
            }
            if (first != null) throw first;
        }

        public void Dispose()
        {
            if (disposed) return;
            if (HasRegisteredEntities)
                throw new InvalidOperationException("仍注册的临时图形不能释放原生对象。");
            disposed = true;
            foreach (Polyline entity in entities)
                try { entity.Dispose(); } catch { }
            entities.Clear(); registered.Clear();
        }

        static void Add(List<Stroke> output, IList<Point2d> points, bool closed,
            short color, Matrix3d transform)
        {
            if (points == null || points.Count < 2) return;
            var world = new Point3d[points.Count];
            for (int i = 0; i < points.Count; i++)
                world[i] = new Point3d(points[i].X, points[i].Y, 0).TransformBy(transform);
            output.Add(new Stroke { Points = world, Closed = closed, Color = color });
        }

        static void Truck(List<Stroke> output, VehicleParams p, PlaybackPose pose,
            Matrix3d transform, short bodyColor)
        {
            var u1 = new Vector2d(Math.Cos(pose.HeadingRad), Math.Sin(pose.HeadingRad));
            var u2 = new Vector2d(Math.Cos(pose.TrailerHeadingRad), Math.Sin(pose.TrailerHeadingRad));
            var origin = new Point2d(pose.X, pose.Y);
            Point2d kingpin = p.Articulated ? origin + p.TractorRearToKingpin * u1
                : origin + p.TractorWheelbase * u1;
            var f = new Frame { O1 = origin, U1 = u1, U2 = u2, K = kingpin,
                O2 = p.Articulated ? kingpin - p.TrailerKingpinToRearAxle * u2 : kingpin };
            if (p.Articulated)
            {
                Add(output, TruckKinematics.TractorRect(f, p), true, bodyColor, transform);
                Add(output, TruckKinematics.ChassisRect(f, p), true, bodyColor, transform);
                Add(output, TruckKinematics.TrailerRect(f, p), true, bodyColor, transform);
            }
            else
                foreach (var outline in TruckKinematics.RigidBodyOutlines(f, p))
                    Add(output, outline, false, bodyColor, transform);

            // Use the stored steering angle, not a tangent inferred from forward displacement.
            // This keeps straight frames straight, and keeps reverse travel facing the right way.
            Point2d front = origin + p.TractorWheelbase * u1;
            foreach (var tire in TruckKinematics.TireRectsWithSteer(f, p,
                0.5 * p.UnitScale, 0.25 * p.UnitScale, pose.FrontSteerRad))
                Add(output, tire, true, 1, transform);
            Add(output, new[] { front, front + Math.Max(p.TractorFrontOverhang,
                0.5 * p.UnitScale) * u1 }, false, 4, transform);
        }


        static void Carrier(List<Stroke> output, CarrierParams p, PlaybackPose pose,
            Matrix3d transform)
        {
            var cp = new CarrierPose(pose.X / p.UnitScale, pose.Y / p.UnitScale, pose.HeadingRad);
            Add(output, CarrierCadGeometry.BodyPolygon(p, cp), true, 3, transform);
            Add(output, CarrierCadGeometry.ContainerPolygon(p, cp), true, 30, transform);
            var wheels = new CarrierWheelState[4];
            CarrierWheelLocation[] locations = p.ResolvedWheelLocations();
            double c = Math.Cos(cp.HeadingRad), s = Math.Sin(cp.HeadingRad);
            for (int i = 0; i < 4; i++)
            {
                double x = locations[i].X;
                double y = locations[i].Y;
                wheels[i] = new CarrierWheelState { Id = (CarrierWheelId)i, LocalX = x, LocalY = y,
                    WorldX = cp.X + c * x - s * y, WorldY = cp.Y + s * x + c * y,
                    SteerAngleRad = pose.WheelAnglesRad[i] };
            }
            var frame = new CarrierFrame(cp, new CarrierTwist(), (CarrierSteeringMode)pose.SteeringMode, wheels);
            for (int i = 0; i < 4; i++)
                Add(output, CarrierCadGeometry.WheelPolygon(p, frame, i), true, 1, transform);
            Point2d center = CarrierCadGeometry.ToCad(cp.X, cp.Y, p);
            Point2d nose = CarrierCadGeometry.ToCad(cp.X + c * p.OverallLength * 0.65,
                cp.Y + s * p.OverallLength * 0.65, p);
            Add(output, new[] { center, nose }, false, 4, transform);
        }
    }
}
