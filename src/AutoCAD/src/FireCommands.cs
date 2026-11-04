using System;
using System.Collections.Generic;
using System.Windows.Forms;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Runtime;
using CadApplication = Autodesk.AutoCAD.ApplicationServices.Application;

namespace TruckTurn
{
    public partial class TruckTurnCommands
    {
        private const string FireParamsKey = "TruckTurnFireParamsV1";

        [CommandMethod("FIREDRIVE")]
        public void FireDrive()
        {
            Document doc = CadApplication.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            PlaybackCommands.StopAll();
            string stage = "消防车选型";
            VehicleParams previousTruck = _params;
            var previousHeading = _lastHeading;
            try
            {
                double storedHeight;
                var storedFire = ReadFireParams(doc.Database, out storedHeight);
                using (var form = new FireTruckSizeForm(storedFire, DetectDrawingUnitScale(doc.Database), storedHeight))
                {
                    if (CadApplication.ShowModalDialog(form) != DialogResult.OK || form.Result == null) return;
                    SaveFireParams(doc, form.Result, form.ResultHeightM);
                    doc.Editor.WriteMessage("\n消防车：{0}。仅适用两轴前桥转向；参考模板不等同厂家认证。\n{1}",
                        form.SelectedProfileName, form.SourceSummary);
                    TruckDriveBody(doc, doc.Editor, doc.Database, ref stage, form.Result,
                        PlaybackVehicleKind.Fire, form.SelectedProfileName, form.SourceSummary);
                }
            }
            catch (System.Exception ex)
            {
                doc.Editor.WriteMessage("\n[FIREDRIVE 异常 @ {0}] {1}: {2}", stage, ex.GetType().Name, ex.Message);
            }
            finally
            {
                // 消防选型独立存储，不污染货车/半挂车上次选型。
                _params = previousTruck;
                _lastHeading = previousHeading;
            }
        }

        [CommandMethod("FIRE")]
        public void FireDriveAlias() { FireDrive(); }

        private static VehicleParams ReadFireParams(Database db, out double height)
        {
            height = 3.30;
            try
            {
                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var nod = (DBDictionary)tr.GetObject(db.NamedObjectsDictionaryId, OpenMode.ForRead);
                    if (!nod.Contains(FireParamsKey)) return null;
                    var record = tr.GetObject(nod.GetAt(FireParamsKey), OpenMode.ForRead) as Xrecord;
                    if (record == null) return null;
                    using (var data = record.Data)
                    {
                        if (data == null) return null;
                        var values = data.AsArray();
                        if ((values.Length != 1 && values.Length != 2) || !(values[0].Value is string)) return null;
                        if (values.Length == 2 && values[1].TypeCode == (int)DxfCode.Real)
                        {
                            double candidate = Convert.ToDouble(values[1].Value);
                            if (FireVehicleCatalog.IsFinite(candidate) && candidate >= 0.5 && candidate <= 6.0) height = candidate;
                        }
                        var p = VehicleParams.FromCsv((string)values[0].Value);
                        if (p == null || p.Articulated || !FireVehicleCatalog.IsFinite(p.UnitScale) || p.UnitScale <= 0) return null;
                        // 新校验可能指出旧图的轮胎外伸冲突：保留原数值供用户修正，不能静默丢回默认。
                        foreach (double value in new[] { p.TractorWidth, p.TractorWheelbase,
                            p.TractorFrontOverhang, p.RigidRearOverhang, p.WheelTrack, p.SteerAngleDeg })
                            if (!FireVehicleCatalog.IsFinite(value)) return null;
                        return p;
                    }
                }
            }
            catch (System.Exception) { return null; } // 无效/旧记录只重开默认模板，不影响出图。
        }

        private static void SaveFireParams(Document doc, VehicleParams p, double height)
        {
            try
            {
                using (var tr = doc.Database.TransactionManager.StartTransaction())
                {
                    var nod = (DBDictionary)tr.GetObject(doc.Database.NamedObjectsDictionaryId, OpenMode.ForWrite);
                    Xrecord record;
                    if (nod.Contains(FireParamsKey))
                        record = (Xrecord)tr.GetObject(nod.GetAt(FireParamsKey), OpenMode.ForWrite);
                    else
                    {
                        record = new Xrecord();
                        nod.SetAt(FireParamsKey, record);
                        tr.AddNewlyCreatedDBObject(record, true);
                    }
                    using (var data = new ResultBuffer(new TypedValue((int)DxfCode.Text, p.ToCsv()),
                        new TypedValue((int)DxfCode.Real, height))) record.Data = data;
                    tr.Commit();
                }
            }
            catch (System.Exception ex) { doc.Editor.WriteMessage("\n消防选型未能保存：{0}；本次仍可绘制。", ex.Message); }
        }

        private static PlaybackRoute TryCreateTruckPlayback(Editor ed, IList<Frame> frames,
            PlaybackVehicleKind kind, string name, string source)
        {
            try
            {
                var route = PlaybackRouteFactory.FromTruck(_params, frames, kind, name);
                route.SourceSummary = source ?? "";
                return route;
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n本次保留静态轨迹，动画快照未生成：{0}", ex.Message);
                return null;
            }
        }
    }
}
