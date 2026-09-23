using System;
using System.Collections.Generic;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    /// <summary>
    /// _dump_revfull.cs — 复现「倒退确认段多绕一整圈」。
    /// 铰接车先前进转 90°（模拟用户截图里的绿色当前姿态），
    /// 然后对车尾周围 360° 全方向、多距离的目标点走 Jig 同款管线
    /// （TrySolveReverseArc → 失败 ReverseTurnTowardAtMinRadius → SimulateReverseFromState），
    /// 比较「求解转角 turnDeg」与「模拟帧里 φ2 的实际总扫掠角」，
    /// 凡是 实际扫掠 > turnDeg+1° 或 > 185° 的目标点全部打印出来。
    /// </summary>
    internal static partial class VerifyProgram
    {
        static void _dumpRevFull()
        {
            var p = VehicleParams.Defaults();
            p.Articulated = true;

            // 当前姿态：前进左转 90° 后的末帧（带稳态铰接角）
            var f0 = MakeStartFrame(p, new Point2d(0, 0), new Vector2d(1, 0));
            double phi20 = Math.Atan2(f0.U2.Y, f0.U2.X);
            var turn = TruckKinematics.SimulateFromStateFront(p, TruckKinematics.FrontAxle(f0, p),
                f0.U1, phi20, 1, 90.0, 0.0, 1, TruckKinematics.FineStepDeg, null);
            var cur = turn[turn.Count - 1];
            Console.WriteLine("当前姿态：O2=({0:F2},{1:F2}) 挂车航向={2:F1}° 铰接角={3:F1}°",
                cur.O2.X, cur.O2.Y,
                Math.Atan2(cur.U2.Y, cur.U2.X) * 180 / Math.PI,
                NormalizeAngleDeg(Math.Atan2(cur.U2.Y, cur.U2.X) - Math.Atan2(cur.U1.Y, cur.U1.X)));

            int bad = 0, tested = 0;
            Console.WriteLine("\n异常目标点（实际扫掠 与 求解转角 不符 或 >185°）：");
            Console.WriteLine("目标(dist,ang°)  分支   side  turnDeg    R2      实际扫掠°");
            for (int angI = 0; angI < 72; angI++)
            {
                double angDeg = angI * 5.0;
                double ang = angDeg * Math.PI / 180.0;
                foreach (double dist in new[] { 3, 6, 10, 15, 22, 30, 45, 60 })
                {
                    var target = new Point2d(cur.O2.X + dist * Math.Cos(ang),
                                             cur.O2.Y + dist * Math.Sin(ang));
                    // Jig 同款管线
                    int side; double turnDeg, rR, straight;
                    string branch = "精确";
                    bool ok = TruckKinematics.TrySolveReverseArc(p, cur, target,
                        out side, out turnDeg, out rR, out straight);
                    if (!ok)
                    {
                        branch = "回退";
                        ok = TruckKinematics.ReverseTurnTowardAtMinRadius(p, cur, target,
                            out side, out turnDeg, out rR);
                        straight = 0.0;
                    }
                    if (!ok) continue;
                    tested++;

                    var frames = TruckKinematics.SimulateReverseFromState(p, cur,
                        side, turnDeg, straight, rR, TruckKinematics.FineStepDeg);
                    // 实际总扫掠 = 逐帧 φ2 变化累加（含直行段为 0）
                    double sweep = 0;
                    for (int i = 1; i < frames.Count; i++)
                    {
                        double a1 = Math.Atan2(frames[i - 1].U2.Y, frames[i - 1].U2.X);
                        double a2 = Math.Atan2(frames[i].U2.Y, frames[i].U2.X);
                        sweep += NormalizeAngleDeg(a2 - a1);
                    }
                    double sweepAbs = Math.Abs(sweep);
                    if (sweepAbs > turnDeg + 1.0 || sweepAbs > 185.0)
                    {
                        bad++;
                        Console.WriteLine("({0,4:F0},{1,6:F1}°) {2} {3,4} {4,8:F2} {5,8:F2} {6,9:F2}",
                            dist, angDeg, branch, side, turnDeg, rR, sweep);
                    }
                }
            }
            Console.WriteLine("\n共测试 {0} 个可行目标点，异常 {1} 个", tested, bad);
        }
    }
}
