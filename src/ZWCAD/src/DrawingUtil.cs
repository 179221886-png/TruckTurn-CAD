using System;
using System.Collections.Generic;
using ZwSoft.ZwCAD.ApplicationServices;
using ZwSoft.ZwCAD.Colors;
using ZwSoft.ZwCAD.DatabaseServices;
using ZwSoft.ZwCAD.Geometry;

namespace TruckTurn
{
    /// <summary>图形绘制辅助：图层、圆弧多段线、矩形</summary>
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

        // 当前包络输出采用圆弧 bulge 多段线。移植版不保留未调用且中望旧版不支持的
        // periodic fit-spline 辅助入口，避免用不等价构造器悄悄改变闭合边界。

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
