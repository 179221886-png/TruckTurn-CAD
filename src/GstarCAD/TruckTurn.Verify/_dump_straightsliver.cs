using System;
using System.Collections.Generic;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    /// <summary>
    /// _dump_straightsliver.cs — 复现「近似直行时包络塌缩成细缝」。
    /// 走与 Jig 确认段完全相同的管线：TrySolveAutoSide → SimulateFromStateFront
    /// → EnvelopeTracks → SimplifyEnvelopeForSpline。刚性车，目标正前方 20m、
    /// 横向偏移 0 ~ 0.5m 扫描。输出 RAW/稀化顶点数与稀化环面积、最小宽度。
    /// </summary>
    internal static partial class VerifyProgram
    {
        static void _dumpStraightSliver()
        {
            var p = VehicleParams.Defaults();
            p.Articulated = false;
            var startFrame = MakeStartFrame(p, new Point2d(0, 0), new Vector2d(1, 0));
            var front0 = TruckKinematics.FrontAxle(startFrame, p);
            double phi2 = Math.Atan2(startFrame.U2.Y, startFrame.U2.X);

            Console.WriteLine("latOff  steer   turnDeg    straight        R   frames  raw  simp  area(m2)  minWid");
            double[] offs = { 0.0, 0.001, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0 };
            foreach (double lat in offs)
            {
                var target = new Point2d(front0.X + 20.0, front0.Y + lat);
                bool ok = TruckKinematics.TrySolveAutoSide(p, front0, startFrame.U1, 1, target,
                    out int steer, out double turnDeg, out double straight, out double rOverride);
                List<Frame> frames;
                double R;
                if (ok)
                {
                    R = rOverride;
                    frames = TruckKinematics.SimulateFromStateFront(p, front0, startFrame.U1, phi2,
                        steer, turnDeg, straight, 1, TruckKinematics.FineStepDeg, rOverride);
                }
                else
                {
                    R = TruckKinematics.TurnRadius(p);
                    turnDeg = TruckKinematics.TurnTowardTargetAtMinRadius(p, front0, startFrame.U1, steer, target, 1);
                    straight = 0.0;
                    frames = TruckKinematics.SimulateFromStateFront(p, front0, startFrame.U1, phi2,
                        steer, turnDeg, straight, 1, TruckKinematics.FineStepDeg, null);
                }
                var env = TruckKinematics.EnvelopeTracks(frames, p);
                int simpN = -1; double area = -1, minWid = -1;
                if (env != null && env.Count >= 3)
                {
                    var (simp, _) = GeometryUtil.SimplifyEnvelopeForSpline(env, 0.002, 60);
                    simpN = simp.Count;
                    area = Math.Abs(SignedArea(simp));
                    minWid = MinLoopWidth(simp);
                }
                Console.WriteLine("{0,6:F3} {1,6} {2,8:F4} {3,9:F3} {4,9:F1} {5,6} {6,5} {7,5} {8,8:F2} {9,7:F3}",
                    lat, steer, turnDeg, straight, R, frames.Count,
                    env == null ? -1 : env.Count, simpN, area, minWid);
            }
        }

        static double SignedArea(List<Point2d> loop)
        {
            double s = 0;
            for (int i = 0; i < loop.Count; i++)
            {
                var a = loop[i]; var b = loop[(i + 1) % loop.Count];
                s += a.X * b.Y - b.X * a.Y;
            }
            return s * 0.5;
        }

        /// <summary>逐顶点测到「非相邻边」的最短距离的最小值（近似环最小宽度）</summary>
        static double MinLoopWidth(List<Point2d> loop)
        {
            double min = double.MaxValue;
            int n = loop.Count;
            for (int i = 0; i < n; i++)
            {
                double d2 = MinDistToNonAdjacent(loop, i);
                if (d2 < min) min = d2;
            }
            return min;
        }

        static double MinDistToNonAdjacent(List<Point2d> loop, int idx)
        {
            int n = loop.Count;
            var p = loop[idx];
            double best = double.MaxValue;
            for (int i = 0; i < n; i++)
            {
                if (i == idx || i == (idx + 1) % n || i == (idx - 1 + n) % n) continue;
                var (d, _) = GeometryUtil.DistancePointToSegment(p, loop[i], loop[(i + 1) % n]);
                if (d < best) best = d;
            }
            return best;
        }
    }
}
