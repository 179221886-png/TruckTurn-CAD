using System;
using System.IO;
using System.Xml.Serialization;
using Gssoft.Gscad.Geometry;
using TruckTurn;

namespace TruckTurn
{
internal static class FireVehicleVerify
{
    private static int _assertions;
    private static void Assert(bool ok, string message)
    { _assertions++; if (!ok) throw new Exception(message); }
    private static void Near(double x, double y, string message)
    { Assert(Math.Abs(x - y) <= Math.Max(1, Math.Abs(y)) * 1e-10, message); }

    public static void Run()
    {
        _assertions = 0;
        var profiles = FireVehicleCatalog.CreateProfiles();
        Assert(profiles.Count == 5, "catalog count");
        var same = FireVehicleCatalog.CreateProfiles();
        Assert(!object.ReferenceEquals(profiles[0], same[0]), "isolated profile objects");
        string error;
        foreach (var spec in profiles)
        {
            if (!spec.Supported) { Assert(!string.IsNullOrEmpty(spec.SourceUrl), "unsupported structure has source"); continue; }
            Assert(FireVehicleCatalog.ValidateGeometry(spec.WidthM, spec.HeightM, spec.WheelbaseM, spec.FrontOverhangM,
                spec.RearOverhangM, spec.TrackM, spec.SteerDeg, 1, out error), spec.Name + " invalid: " + error);
            Near(spec.LengthM, spec.FrontOverhangM + spec.WheelbaseM + spec.RearOverhangM, "length equals true axle geometry");
            foreach (double scale in new[] { 1.0, 100.0, 1000.0, 3.280839895, 1000000.0 })
            {
                var previous = VehicleParams.Defaults(); previous.ShowAllNodeVehicles = true; previous.ShowGhost = false;
                string before = previous.ToCsv();
                var p = FireVehicleCatalog.BuildVehicle(spec.WidthM, spec.HeightM, spec.WheelbaseM, spec.FrontOverhangM,
                    spec.RearOverhangM, spec.TrackM, spec.SteerDeg, scale, previous);
                Assert(!p.Articulated && p.RigidAxles == 2, "two axle front steer");
                Near(p.LengthMeters(), spec.LengthM, "length scale"); Near(p.WidthMeters(), spec.WidthM, "width scale");
                Near(p.TractorWheelbase / scale, spec.WheelbaseM, "true wheelbase"); Near(p.RigidRearOverhang / scale, spec.RearOverhangM, "true rear overhang");
                Near(p.SteerAngleDeg, spec.SteerDeg, "angle must not scale"); Near(p.WheelTrack / scale, spec.TrackM, "track scale");
                Assert(p.ShowAllNodeVehicles && !p.ShowGhost, "display inherited independently");
                Assert(p.BodyWidthIncludesMirrors, "fire width includes fixed protrusions; no extra mirrors outside envelope");
                Frame frame = new Frame { O1 = new Point2d(0, 0), U1 = new Vector2d(1, 0), U2 = new Vector2d(1, 0), O2 = new Point2d(0, 0), K = new Point2d(0, 0) };
                foreach (var line in TruckKinematics.RigidBodyOutlines(frame, p))
                    foreach (var vertex in line)
                        Assert(Math.Abs(vertex.Y) <= p.TractorWidth * 0.5 + 1e-8, "every body vertex stays within input envelope width");
                // Playback uses XmlSerializer on its nested public VehicleParams snapshot.
                var serializer = new XmlSerializer(typeof(VehicleParams));
                VehicleParams xmlCopy;
                using (var writer = new StringWriter())
                {
                    serializer.Serialize(writer, p);
                    using (var reader = new StringReader(writer.ToString())) xmlCopy = (VehicleParams)serializer.Deserialize(reader);
                }
                Assert(xmlCopy.BodyWidthIncludesMirrors, "XML playback snapshot preserves fire-width semantics");
                Near(xmlCopy.TractorWheelbase, p.TractorWheelbase, "XML snapshot wheelbase");
                foreach (var line in TruckKinematics.RigidBodyOutlines(frame, xmlCopy))
                    foreach (var vertex in line)
                        Assert(Math.Abs(vertex.Y) <= xmlCopy.TractorWidth * 0.5 + 1e-8, "XML playback outline stays in envelope width");
                Assert(previous.ToCsv() == before, "input parameters unchanged");
                var saved = VehicleParams.FromCsv(p.ToCsv()); Near(saved.LengthMeters(), p.LengthMeters(), "CSV roundtrip");
            }
        }
        double[] bad = { double.NaN, double.PositiveInfinity, double.NegativeInfinity, -1 };
        foreach (double value in bad)
        {
            Assert(!FireVehicleCatalog.ValidateGeometry(value, 3, 4.5, 1.4, 2.6, 1.95, 38, 1, out error), "invalid width");
            Assert(!FireVehicleCatalog.ValidateGeometry(2.5, 3, value, 1.4, 2.6, 1.95, 38, 1, out error), "invalid wheelbase");
            Assert(!FireVehicleCatalog.ValidateGeometry(2.5, 3, 4.5, 1.4, 2.6, 1.95, 38, value, out error), "invalid scale");
        }
        Assert(!FireVehicleCatalog.ValidateGeometry(2.5, 3, 4.5, 1.4, 2.6, 2.6, 38, 1, out error), "track wider than body");
        Assert(!FireVehicleCatalog.ValidateGeometry(2.5, 3, 4.5, 1.4, 2.6, 1.95, 0, 1, out error), "zero steering");
        Assert(!FireVehicleCatalog.ValidateGeometry(2.5, 3, 4.5, 1.4, 2.6, 1.95, 90, 1, out error), "singular steering");
        Assert(!FireVehicleCatalog.ValidateGeometry(2.5, 3, 4.5, 1.4, 2.6, 1.95, 38, 0, out error), "zero scale");
        bool rejected = false;
        try { FireVehicleCatalog.BuildVehicle(2.5, 3, double.NaN, 1, 2, 1.9, 38, 1, null); }
        catch (ArgumentException) { rejected = true; }
        Assert(rejected, "factory must fail closed");
        Assert(!VehicleParams.Defaults().BodyWidthIncludesMirrors, "legacy truck semantics default unchanged");
        Console.WriteLine("PASS Fire catalog / source-unit geometry / defensive values: " + _assertions + " assertions. UI compiled only; no GUI launched.");
    }
}


}

