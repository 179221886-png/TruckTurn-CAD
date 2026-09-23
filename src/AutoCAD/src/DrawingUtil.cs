using System;
using System.Collections.Generic;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.Colors;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;

namespace TruckTurn
{
    /// <summary>图形绘制辅助：图层、多段线、矩形、Spline</summary>
    public static class DrawingUtil
    {
        /// <summary>确保图层存在并返回记录（不存在则创建，指定颜色索引 0-255）</summary>
        public static LayerTableRecord EnsureLayer(Database db, Transaction tr, string name, short colorIndex)
        {
            var lt = (LayerTable)tr.GetObject(db.LayerTableId, OpenMode.ForRead);
            if (lt.Has(name))
                return (LayerTableRecord)tr.GetObject(lt[name], OpenMode.ForRead);

            lt.UpgradeOpen();
            var ltr = new LayerTableRecord
            {
                Name = name,
                Color = Color.FromColorIndex(ColorMethod.ByAci, colorIndex)
            };
            var id = lt.Add(ltr);
            tr.AddNewlyCreatedDBObject(ltr, true);
            return (LayerTableRecord)tr.GetObject(id, OpenMode.ForRead);
        }

        /// <summary>添加一条(可选闭合)多段线</summary>
        public static ObjectId AddPolyline(Database db, Transaction tr, List<Point2d> pts, bool closed, string layer)
        {
            return AddPolyline(db, tr, pts, null, closed, layer);
        }

        /// <summary>添加一条带 bulge 的(可选闭合)多段线。bulges 为 null 或长度不足时补 0。</summary>
        public static ObjectId AddPolyline(Database db, Transaction tr, List<Point2d> pts, List<double> bulges, bool closed, string layer)
        {
            var pl = new Polyline();
            for (int i = 0; i < pts.Count; i++)
            {
                double b = (bulges != null && i < bulges.Count) ? bulges[i] : 0.0;
                pl.AddVertexAt(i, pts[i], b, 0, 0);
            }
            pl.Closed = closed;
            pl.Layer = layer;
            var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
            btr.AppendEntity(pl);
            tr.AddNewlyCreatedDBObject(pl, true);
            return pl.ObjectId;
        }

        /// <summary>
        /// v4.8 方案三：用浩辰原生 Spline（fit-spline）出图，替代带 bulge 的多段线。
        /// 三次（degree=3）样条在顶点处 C2 连续，把离散 footprint 并集留下的密集台阶
        /// 从「尖角」变成「平滑曲线」，根治包络锯齿。
        ///
        /// ★ 关键前提：fit-spline 是**插值**样条，会精确穿过每一个传入顶点。
        ///   如果直接把原始包络顶点（含 1~3 cm 台阶）丢进来，台阶会被逐点穿过，
        ///   锯齿只会从「尖角」变成「波浪」，不会消失。
        ///   所以调用方必须先用 GeometryUtil.SimplifyEnvelopeForSpline 稀化（并外推补偿）。
        ///
        /// ★ periodic=true 时是闭合样条，首尾自动接续，拟合点里**不要**重复首点。
        /// </summary>
        /// ★ 浩辰的 Spline 构造器对拟合点数、参数化方式有内部约束（periodic 样条尤其），
        ///   一旦触发异常不能让整条命令崩掉，所以这里兜底退回多段线：
        ///   锯齿还在，但图能出来、包络几何没错。是否降级由 out 参数告诉调用方去提示用户。
        /// </summary>
        public static ObjectId AddSpline(Database db, Transaction tr,
                                         List<Point2d> pts, bool closed, string layer,
                                         out bool fellBack)
        {
            fellBack = false;
            if (pts == null || pts.Count < (closed ? 3 : 2))
                return ObjectId.Null;

            try
            {
                var fit = new Point3dCollection();
                for (int i = 0; i < pts.Count; i++)
                    fit.Add(new Point3d(pts[i].X, pts[i].Y, 0));

                // Spline(fitPoints, periodic, knotParameterization, degree, fitTolerance)
                //
                // v4.9.2 调参：fitTolerance 从 1e-6 放宽到 2e-3 (2mm)。
                //   1e-6 等于强制插值（穿过每个顶点），会把 simp 上残留的几毫米起伏
                //   忠实复现成样条波浪（典型波长 3~6 顶点、Laplacian 杀不掉的低频残留）。
                //   2e-3 让样条有 2mm 的拟合余地，可以吸收这些微噪，肉眼看到的就是真正平滑的曲线。
                //   安全侧由 OffsetClosedLoop 的外推补偿保证：simp 已经比原始 envelope 外扩 1.5~3cm，
                //   样条再向内 2mm 仍远在原始包络之外，corners_outside 不变量不受影响。
                var spline = new Spline(fit, closed, KnotParameterizationEnum.Uniform, 3, 2e-3);
                spline.Layer = layer;

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                btr.AppendEntity(spline);
                tr.AddNewlyCreatedDBObject(spline, true);
                return spline.ObjectId;
            }
            catch (System.Exception)
            {
                fellBack = true;
                return AddPolyline(db, tr, pts, closed, layer);
            }
        }

        /// <summary>
        /// 添加闭合矩形(4 个角点，按顺序)。
        /// </summary>
        public static ObjectId AddRectangle(Database db, Transaction tr, Point2d[] corners, string layer)
        {
            var pl = new Polyline();
            for (int i = 0; i < 4; i++)
                pl.AddVertexAt(i, corners[i], 0, 0, 0);
            pl.Closed = true;
            pl.Layer = layer;
            var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
            btr.AppendEntity(pl);
            tr.AddNewlyCreatedDBObject(pl, true);
            return pl.ObjectId;
        }

        /// <summary>添加一条普通线段</summary>
        public static ObjectId AddLine(Database db, Transaction tr, Point2d a, Point2d b, string layer)
        {
            var line = new Line(new Point3d(a.X, a.Y, 0), new Point3d(b.X, b.Y, 0));
            line.Layer = layer;
            var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
            btr.AppendEntity(line);
            tr.AddNewlyCreatedDBObject(line, true);
            return line.ObjectId;
        }

        /// <summary>添加多行文字标注</summary>
        public static ObjectId AddMText(Database db, Transaction tr, Point2d pos, string content,
                                        double height, string layer, double attachment = 1)
        {
            var mt = new MText();
            mt.Location = new Point3d(pos.X, pos.Y, 0);
            mt.Contents = content;
            mt.TextHeight = height;
            mt.Layer = layer;
            var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
            btr.AppendEntity(mt);
            tr.AddNewlyCreatedDBObject(mt, true);
            return mt.ObjectId;
        }
    }
}
