using System;
using System.Collections.Generic;
using System.Linq;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    static partial class VerifyProgram
    {
        static int RunDimensionConsistencyRegression()
        {
            int count = 0;
            Action<bool, string> check = (ok, name) => { count++; if (!ok) throw new Exception(name); };
            Action<double, double, string> near = (x, y, name) => check(Math.Abs(x - y) < 1e-8, name);
            try
            {
                foreach (double unit in new[] { 1.0, 100.0, 1000.0 })
                {
                    var p = VehicleParams.Defaults();
                    p.TractorFrontOverhang = 1.5; p.TractorWheelbase = 3.6;
                    p.ChassisRearOverhang = .9; p.TractorRearToKingpin = .6;
                    p.TrailerKingpinToFront = .39; p.TrailerKingpinToRearAxle = 11.05;
                    p.TrailerRearOverhang = 1.56; p.MaxArticulationAngleDeg = 90;
                    p.ExplicitCabGeometry = true; p.BodyWidthIncludesMirrors = true;
                    p.KingpinToCabRear = p.SafeCabGap() + .01;
                    p = p.InUnitScale(unit);
                    string error;
                    check(p.ValidateGeometry(out error), "canonical dimensions: " + error);
                    near(p.TractorLengthMeters(), 6, "tractor real length");
                    near(p.TrailerLengthMeters(), 13, "trailer real length");
                    foreach (double angle in new[] { 0.0, .63, 2.2 })
                    {
                        var u = new Vector2d(Math.Cos(angle), Math.Sin(angle));
                        var origin = new Point2d(100 * unit, -70 * unit);
                        var f = new Frame { O1 = origin, U1 = u, U2 = u,
                            K = origin + p.TractorRearToKingpin * u,
                            O2 = origin + (p.TractorRearToKingpin - p.TrailerKingpinToRearAxle) * u };
                        var body = TruckKinematics.Corners(f, p).Concat(TruckKinematics.ChassisRect(f, p));
                        var x = body.Select(q => ((q.X - origin.X) * u.X + (q.Y - origin.Y) * u.Y) / unit).ToArray();
                        near(x.Max() - x.Min(), p.LengthMeters(), "true combination extent, not summed lengths");
                        for (int axles = 1; axles <= 3; axles++)
                        {
                            p.TrailerAxles = axles;
                            var tires = TruckKinematics.TireRectsWithSteer(f, p, .5 * unit, .25 * unit, .2);
                            check(tires.Count == 4 + 2 * axles, "articulated visible axles");
                            var tracks = TruckKinematics.BuildWheelCenterTracks(new List<Frame> { f, f }, p);
                            check(tracks.Length == tires.Count + 1, "articulated tracks match visible wheels");
                            CheckWheelCenters(tires, tracks, unit, near);
                        }
                    }
                    p.TrailerAxles = 3;
                    double original = p.KingpinToCabRear;
                    p.KingpinToCabRear = .1 * unit;
                    check(!p.ValidateGeometry(out error), "conflicting explicit gap rejected");
                    near((p.CabRearX() - p.TractorRearToKingpin) / unit, .1, "new geometry never silently moves cabin");
                    p.ExplicitCabGeometry = false;
                    check(p.CabRearX() - p.TractorRearToKingpin > p.KingpinToCabRear, "legacy automatic geometry remains readable");
                    var legacy = PlaybackRouteFactory.FromTruck(p, new List<Frame> { Straight(p) });
                    var decoded = PlaybackRouteCodec.Decode(PlaybackRouteCodec.Encode(legacy));
                    near(decoded.TruckParameters.CabRearX() / unit, p.CabRearX() / unit, "legacy replay geometry unchanged");
                    p.ExplicitCabGeometry = true; p.KingpinToCabRear = original;
                    var saved = VehicleParams.FromCsv(p.ToCsv());
                    check(saved.ExplicitCabGeometry && saved.BodyWidthIncludesMirrors, "new CSV geometry flags");
                    check(BitConverter.DoubleToInt64Bits(saved.KingpinToCabRear) == BitConverter.DoubleToInt64Bits(p.KingpinToCabRear), "CSV full double precision");

                    var shortTrailer = p.Copy();
                    shortTrailer.TrailerKingpinToRearAxle = 2.8 * unit;
                    shortTrailer.TrailerKingpinToFront = 0;
                    shortTrailer.TrailerRearOverhang = .3 * unit;
                    shortTrailer.TrailerAxles = 3;
                    check(!shortTrailer.ValidateGeometry(out error), "front trailer tire extent rejected, not just axle center");

                    p.Articulated = false; p.TractorWheelbase = 10 * unit;
                    p.TractorFrontOverhang = 1.5 * unit; p.RigidRearOverhang = 3 * unit;
                    p.TractorWidth = 3 * unit; p.WheelTrack = 2 * unit;
                    p.SteerAngleDeg = 35;
                    for (int axles = 2; axles <= 6; axles++)
                    {
                        p.RigidAxles = axles;
                        check(p.ValidateGeometry(out error), "rigid axle group valid: " + error);
                        var f = Straight(p);
                        var tires = TruckKinematics.TireRects(f, p, .5 * unit, .25 * unit, 1);
                        check(tires.Count == axles * 2, "rigid wheel count");
                        var tracks = TruckKinematics.BuildWheelCenterTracks(new List<Frame> { f, f }, p);
                        check(tracks.Length == tires.Count + 1, "rigid tracks match visible wheels");
                        CheckWheelCenters(tires, tracks, unit, near);
                    }
                    check(TruckKinematics.BuildWheelCenterTracks(new List<Frame> { Straight(p) }, p, false).Length == 1,
                        "front-only preview avoids unused wheel paths");
                    p.WheelTrack = p.TractorWidth;
                    check(!p.ValidateGeometry(out error), "protruding wheel width rejected");
                    p.WheelTrack = 2 * unit; p.TractorFrontOverhang = 0;
                    check(!p.ValidateGeometry(out error), "protruding wheel length rejected");
                }
                Console.WriteLine("PASS dimension core: " + count + " assertions; dimensions, CSV, legacy replay, full multi-axle wheels/tracks.");
                return 0;
            }
            catch (Exception ex) { Console.WriteLine("FAIL dimension core: " + ex.Message); return 1; }
        }

        static Frame Straight(VehicleParams p)
        {
            return new Frame { O1 = new Point2d(0, 0), U1 = new Vector2d(1, 0), U2 = new Vector2d(1, 0),
                K = new Point2d(p.TractorRearToKingpin, 0),
                O2 = new Point2d(p.TractorRearToKingpin - p.TrailerKingpinToRearAxle, 0) };
        }
        static void CheckWheelCenters(List<Point2d[]> tires, List<Point2d>[] tracks, double unit,
            Action<double, double, string> near)
        {
            for (int i = 0; i < tires.Count; i++)
            {
                near(tires[i].Average(q => q.X) / unit, tracks[i + 1][0].X / unit, "wheel-track X shared");
                near(tires[i].Average(q => q.Y) / unit, tracks[i + 1][0].Y / unit, "wheel-track Y shared");
            }
        }
    }
}
