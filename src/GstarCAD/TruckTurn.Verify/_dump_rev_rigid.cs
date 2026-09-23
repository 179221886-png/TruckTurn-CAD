// v4.9.15+ 调试：复现「刚性单车倒退，预览正确、左键确认后多转一圈」
// 预览与确认用同一组 (side, turnDeg, straight, radius)，仅步长不同
// （PreviewStep=1.0° vs OutputStepDeg=0.25°）。若两者扫过角度/终点不一致，
// 说明 SimulateRigidReverseFromState 对步长敏感；若一致，则问题在 Jig 字段时序。
using System;
using System.Collections.Generic;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    partial class VerifyProgram
    {
        public static void _dumpRevRigid()
        {
            Console.WriteLine("\n=== 刚性倒退：预览 vs 确认一致性（同参数不同步长） ===");
            var p = VehicleParams.Defaults();
            p.Articulated = false;

            // 截图姿态：车头朝左上（~135°），光标在车尾右后方
            double phi = 135.0 * Math.PI / 180.0;
            var u = new Vector2d(Math.Cos(phi), Math.Sin(phi));
            var start = new Frame { O1 = new Point2d(0, 0), U1 = u, K = new Point2d(0, 0) + p.TractorWheelbase * u,
                                    U2 = u, O2 = new Point2d(0, 0) + p.TractorWheelbase * u };

            var targets = new (string label, Point2d t)[]
            {
                ("正后方 10m",       new Point2d(0, 0) - 10 * u),
                ("右后 8m/偏右 3m",  new Point2d(0, 0) - 8 * u + 3 * new Vector2d(u.Y, -u.X)),
                ("右后 6m/偏右 6m",  new Point2d(0, 0) - 6 * u + 6 * new Vector2d(u.Y, -u.X)),
                ("左后 8m/偏左 4m",  new Point2d(0, 0) - 8 * u - 4 * new Vector2d(u.Y, -u.X)),
                ("贴近右后 3m/3m",   new Point2d(0, 0) - 3 * u + 3 * new Vector2d(u.Y, -u.X)),
                // v4.9.16：光标越过「车尾垂线」到前方 —— 修复前回退分支给 180° 半圆怪物，
                // 修复后应判不可行（预览为空、段不生成）。
                ("右侧稍前 a=+0.5,d=-4",  new Point2d(0, 0) + 0.5 * u + 4 * new Vector2d(u.Y, -u.X)),
                ("右侧稍前 a=+0.1,d=-4",  new Point2d(0, 0) + 0.1 * u + 4 * new Vector2d(u.Y, -u.X)),
                ("正前方 5m",             new Point2d(0, 0) + 5 * u),
                ("右前方 a=+6,d=-6",      new Point2d(0, 0) + 6 * u + 6 * new Vector2d(u.Y, -u.X)),
                ("右侧线后 a=-0.1,d=-4",  new Point2d(0, 0) - 0.1 * u + 4 * new Vector2d(u.Y, -u.X)),
            };

            Console.WriteLine("  {0,-16} {1,6} {2,8} {3,8} {4,8} | {5,9} {6,9} | {7,9} {8,9} | {9,7}",
                "目标", "side", "turnDeg", "straight", "R2", "预览末x", "预览末y", "确认末x", "确认末y", "扫角差°");
            foreach (var c in targets)
            {
                int side; double turnDeg, rR, straight;
                bool ok = TruckKinematics.TrySolveReverseArc(p, start, c.t,
                                                             out side, out turnDeg, out rR, out straight);
                string tag = ok ? "弧" : "退";
                if (!ok)
                {
                    ok = TruckKinematics.ReverseTurnTowardAtMinRadius(p, start, c.t,
                                                                      out side, out turnDeg, out rR);
                    straight = 0.0;
                }
                if (!ok) { Console.WriteLine("  {0,-16} 不可行", c.label); continue; }

                var prev = TruckKinematics.SimulateReverseFromState(p, start, side, turnDeg, straight, rR, 1.0);
                var fine = TruckKinematics.SimulateReverseFromState(p, start, side, turnDeg, straight, rR, 0.25);
                var lp = prev[prev.Count - 1]; var lf = fine[fine.Count - 1];
                double swp = SweptDeg(fine); double swp2 = SweptDeg(prev);
                Console.WriteLine("  {0,-16} {1,6} {2,8:F2} {3,8:F2} {4,8:F2} | {5,9:F3} {6,9:F3} | {7,9:F3} {8,9:F3} | {9,7:F2} ({10})",
                    c.label, side, turnDeg, straight, rR, lp.O1.X, lp.O1.Y, lf.O1.X, lf.O1.Y, swp - swp2, tag);
                if (turnDeg > 180.0 + 1e-6)
                    Console.WriteLine("    ★★ 求解器输出转角 >180°！");
            }
        }

        /// <summary>帧序列首尾航向差（度，0~360）。</summary>
        static double SweptDeg(List<Frame> frames)
        {
            if (frames == null || frames.Count < 2) return 0.0;
            double a0 = Math.Atan2(frames[0].U1.Y, frames[0].U1.X);
            double a1 = Math.Atan2(frames[frames.Count - 1].U1.Y, frames[frames.Count - 1].U1.X);
            double d = (a1 - a0) * 180.0 / Math.PI;
            while (d < 0) d += 360.0;
            while (d >= 360.0) d -= 360.0;
            return d;
        }
    }
}
