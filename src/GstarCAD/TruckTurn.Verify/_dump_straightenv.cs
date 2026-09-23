using System;
using System.Collections.Generic;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    /// <summary>
    /// _dump_straightenv.cs — 复现「笔直前进时黄色包络有时不出现」（刚性车高发）。
    /// 逐场景：不同航向角 × 不同直行距离，刚性/铰接，跑 EnvelopeTracks +
    /// SimplifyEnvelopeForSpline，打印 帧数/回退/RAW顶点/稀化顶点。
    /// </summary>
    internal static partial class VerifyProgram
    {
        static void _dumpStraightEnv()
        {
            double[] headings = { 0, 15, 30, 45, 73, 90, 123.4, 180, 270.7 };
            double[] dists = { 5, 10, 20, 40, 80 };
            foreach (bool art in new[] { false, true })
            {
                var p = VehicleParams.Defaults();
                p.Articulated = art;
                Console.WriteLine("\n=== " + (art ? "铰接" : "刚性") + " ===");
                Console.WriteLine("heading dist frames fallback raw simp");
                foreach (double h in headings)
                    foreach (double d in dists)
                    {
                        var u = new Vector2d(Math.Cos(h * Math.PI / 180), Math.Sin(h * Math.PI / 180));
                        var frames = TruckKinematics.SimulateFromFrontAxle(p, new Point2d(0, 0), u, 1,
                            0.0, d, TruckKinematics.FineStepDeg);
                        var env = TruckKinematics.EnvelopeTracks(frames, p);
                        int simpN = -1;
                        if (env != null && env.Count >= 3)
                        {
                            // 与 Commands 出图同参数：tol=0.002，iters=60
                            var (simp, _) = GeometryUtil.SimplifyEnvelopeForSpline(env, 0.002, 60);
                            simpN = simp.Count;
                        }
                        Console.WriteLine("{0,7:F1} {1,5:F0} {2,6} {3,8} {4,5} {5,4}",
                            h, d, frames.Count, TruckKinematics.EnvelopeFallbackUsed,
                            env == null ? -1 : env.Count, simpN);
                    }
            }
        }
    }
}
