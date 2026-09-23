using System;
using System.Globalization;
using System.IO;
using Gssoft.Gscad.Geometry;
using TruckTurn;

namespace TruckTurn
{
    /// <summary>_dump_rigidbody.cs — 输出 v4.9.17 刚性车精致轮廓 CSV（三种车长），供 python 画核对图。</summary>
    partial class VerifyProgram
    {
        public static void _dumpRigidBody()
        {
            string csv = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "_rigidbody_dump.csv");
            var inv = CultureInfo.InvariantCulture;
            using (var w = new StreamWriter(csv, false, System.Text.Encoding.UTF8))
            {
                w.WriteLine("VEHICLE,SEG,X,Y");
                foreach (var (name, total, width) in new[] { ("微卡4.5m", 4.5, 1.70), ("中卡9.6m", 9.6, 2.50), ("重卡12m", 12.0, 2.55) })
                {
                    var p = VehicleParams.Defaults();
                    p.Articulated = false;
                    p.UnitScale = 1.0;
                    p.TractorFrontOverhang = 0.25 * total;
                    p.TractorWheelbase = 0.55 * total;
                    p.RigidRearOverhang = 0.20 * total;
                    p.TractorWidth = width;
                    p.WheelTrack = Math.Max(0.9, width * 0.75);

                    var f = new Frame
                    {
                        O1 = new Point2d(0, 0),
                        U1 = new Vector2d(1, 0),
                        K = new Point2d(p.TractorWheelbase, 0),
                        U2 = new Vector2d(1, 0),
                        O2 = new Point2d(p.TractorWheelbase, 0)
                    };
                    var segs = TruckKinematics.RigidBodyOutlines(f, p);
                    for (int i = 0; i < segs.Count; i++)
                        foreach (var pt in segs[i])
                            w.WriteLine(string.Format(inv, "{0},{1},{2:F3},{3:F3}", name, i, pt.X, pt.Y));
                    // 轮胎（每条独立段号，避免绘图时连成跨车斜线）
                    var tires = TruckKinematics.TireRects(f, p, 0.5, 0.25, 0);
                    for (int t = 0; t < tires.Count; t++)
                    {
                        var rect = tires[t];
                        for (int i = 0; i < 4; i++)
                            w.WriteLine(string.Format(inv, "{0},TIRE{1},{2:F3},{3:F3}", name, t, rect[i].X, rect[i].Y));
                        w.WriteLine(string.Format(inv, "{0},TIRE{1},{2:F3},{3:F3}", name, t, rect[0].X, rect[0].Y));
                    }
                }
            }
            Console.WriteLine("CSV: " + csv);
        }
    }
}
