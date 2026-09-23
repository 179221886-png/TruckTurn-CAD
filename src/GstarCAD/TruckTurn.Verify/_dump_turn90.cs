// 临时补丁：跑 TRUCKTURN90 实际路径，打出阶段1→阶段2 拖车位移
using System;
using System.Collections.Generic;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    partial class VerifyProgram
    {
        public static void _dumpTurn90()
        {
            Console.WriteLine("\n=== TRUCKTURN90 实际路径（SimulateTurn90AndStraighten） ===");
            var p = VehicleParams.Defaults();
            p.Articulated = true;
            var startFront = new Point2d(0, 0);
            var heading = new Vector2d(0, 1);
            int sign = +1;
            var frames = TruckKinematics.SimulateTurn90AndStraighten(p, startFront, heading, sign, TruckKinematics.FineStepDeg, 1);
            Console.WriteLine($"  总帧数: {frames.Count}");
            for (int k = 0; k < frames.Count; k += Math.Max(1, frames.Count/25))
            {
                var f = frames[k];
                var front = TruckKinematics.FrontAxle(f, p);
                double phi2 = Math.Atan2(f.U2.Y, f.U2.X);
                double phi1 = Math.Atan2(f.U1.Y, f.U1.X);
                double th = phi2 - phi1;
                while (th > Math.PI) th -= 2*Math.PI;
                while (th < -Math.PI) th += 2*Math.PI;
                Console.WriteLine($"  帧 {k,4}: 前轴=({front.X,7:F2},{front.Y,7:F2})  O2=({f.O2.X,7:F2},{f.O2.Y,7:F2})  th={th*180/Math.PI,6:F2}°");
            }
            int turnEnd = frames.Count - 1;
            for (int k = 1; k < frames.Count - 1; k++)
            {
                double a1 = Math.Atan2(frames[k].U1.Y, frames[k].U1.X);
                double a2 = Math.Atan2(frames[k+1].U1.Y, frames[k+1].U1.X);
                double da = a2 - a1; while (da > Math.PI) da -= 2*Math.PI; while (da < -Math.PI) da += 2*Math.PI;
                if (Math.Abs(da) < 1e-6) { turnEnd = k; break; }
            }
            Console.WriteLine($"  阶段切换帧: {turnEnd} / {frames.Count}");
            if (turnEnd > 0 && turnEnd < frames.Count - 1)
            {
                var a = frames[turnEnd - 1];
                var b = frames[turnEnd + 1];
                var aF = TruckKinematics.FrontAxle(a, p);
                var bF = TruckKinematics.FrontAxle(b, p);
                double tha = Math.Atan2(a.U2.Y,a.U2.X) - Math.Atan2(a.U1.Y,a.U1.X);
                double thb = Math.Atan2(b.U2.Y,b.U2.X) - Math.Atan2(b.U1.Y,b.U1.X);
                Console.WriteLine($"  阶段1末（帧{turnEnd-1}）: 前轴=({aF.X:F2},{aF.Y:F2})  O2=({a.O2.X:F2},{a.O2.Y:F2})  th={tha*180/Math.PI:F2}°");
                Console.WriteLine($"  阶段2首（帧{turnEnd+1}）: 前轴=({bF.X:F2},{bF.Y:F2})  O2=({b.O2.X:F2},{b.O2.Y:F2})  th={thb*180/Math.PI:F2}°");
                Console.WriteLine($"  阶段1末→阶段2首：前轴 Δ=({bF.X-aF.X:F2},{bF.Y-aF.Y:F2})  O2 Δ=({b.O2.X-a.O2.X:F2},{b.O2.Y-a.O2.Y:F2})");
            }
        }
    }
}
