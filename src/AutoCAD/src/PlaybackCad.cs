using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using CadApplication = Autodesk.AutoCAD.ApplicationServices.Application;
using GI = Autodesk.AutoCAD.GraphicsInterface;

[assembly: CommandClass(typeof(TruckTurn.PlaybackCommands))]

namespace TruckTurn
{
    /// <summary>Replay data belongs to each result block, not a global parameter record.</summary>
    public static class PlaybackCadStore
    {
        const string Key = "TRUCKTURN_PLAYBACK_V1";
        const string Header = "TruckTurn.Playback/1";
        // Xrecord BinaryChunk follows the DXF 310 limit, including on .NET Framework hosts.
        const int ChunkSize = 127;
        const int MaximumPayload = PlaybackRouteCodec.MaximumEncodedBytes;

        public static void Attach(Database db, Transaction tr, BlockReference br, PlaybackRoute route)
        {
            if (db == null || tr == null || br == null || route == null)
                throw new ArgumentNullException("回放数据及结果块不能为空。");
            byte[] payload = PlaybackRouteCodec.Encode(route);
            if (payload.Length > MaximumPayload) throw new InvalidDataException("回放数据超过存储限制。");
            double[] original = br.BlockTransform.ToArray();
            var values = new List<TypedValue>(18 + payload.Length / ChunkSize);
            values.Add(new TypedValue((int)DxfCode.Text, Header));
            for (int i = 0; i < 16; i++) values.Add(new TypedValue((int)DxfCode.Real, original[i]));
            values.Add(new TypedValue((int)DxfCode.Int32, payload.Length));
            for (int start = 0; start < payload.Length; start += ChunkSize)
            {
                var part = new byte[Math.Min(ChunkSize, payload.Length - start)];
                Buffer.BlockCopy(payload, start, part, 0, part.Length);
                values.Add(new TypedValue((int)DxfCode.BinaryChunk, part));
            }

            if (!br.IsWriteEnabled) br.UpgradeOpen();
            if (br.ExtensionDictionary.IsNull) br.CreateExtensionDictionary();
            var dictionary = (DBDictionary)tr.GetObject(br.ExtensionDictionary, OpenMode.ForWrite);
            Xrecord record;
            if (dictionary.Contains(Key))
                record = (Xrecord)tr.GetObject(dictionary.GetAt(Key), OpenMode.ForWrite);
            else
            {
                record = new Xrecord();
                dictionary.SetAt(Key, record);
                tr.AddNewlyCreatedDBObject(record, true);
            }
            using (var data = new ResultBuffer(values.ToArray())) record.Data = data;
        }

        public static bool TryRead(Database db, Transaction tr, BlockReference br,
            out PlaybackRoute route, out Matrix3d transform, out string reason)
        {
            route = null;
            transform = Matrix3d.Identity;
            reason = null;
            if (br == null || br.ExtensionDictionary.IsNull)
            {
                reason = "此块没有轨迹回放数据。旧版静态轨迹无法可靠还原动画，请重新生成。";
                return false;
            }
            try
            {
                var dictionary = (DBDictionary)tr.GetObject(br.ExtensionDictionary, OpenMode.ForRead);
                if (!dictionary.Contains(Key))
                {
                    reason = "此块没有轨迹回放数据。请选本版本新生成的完整轨迹块。";
                    return false;
                }
                var record = (Xrecord)tr.GetObject(dictionary.GetAt(Key), OpenMode.ForRead);
                TypedValue[] values;
                using (var buffer = record.Data)
                {
                    if (buffer == null) throw new InvalidDataException("回放记录为空。");
                    values = buffer.AsArray();
                }
                if (values.Length < 19 || values[0].TypeCode != (int)DxfCode.Text ||
                    !Header.Equals(values[0].Value as string, StringComparison.Ordinal))
                    throw new InvalidDataException("不支持的回放记录版本。");
                var baseline = new double[16];
                for (int i = 0; i < baseline.Length; i++)
                {
                    if (values[i + 1].TypeCode != (int)DxfCode.Real)
                        throw new InvalidDataException("回放坐标变换记录不完整。");
                    baseline[i] = Convert.ToDouble(values[i + 1].Value);
                    if (!Finite(baseline[i])) throw new InvalidDataException("回放坐标变换无效。");
                }
                if (values[17].TypeCode != (int)DxfCode.Int32)
                    throw new InvalidDataException("回放数据长度记录无效。");
                int count = Convert.ToInt32(values[17].Value);
                if (count <= 0 || count > MaximumPayload)
                    throw new InvalidDataException("回放数据大小超出允许范围。");
                var payload = new byte[count];
                int offset = 0;
                for (int i = 18; i < values.Length; i++)
                {
                    var part = values[i].Value as byte[];
                    if (values[i].TypeCode != (int)DxfCode.BinaryChunk || part == null ||
                        part.Length == 0 || part.Length > ChunkSize || part.Length > count - offset)
                        throw new InvalidDataException("回放数据分块已损坏。");
                    Buffer.BlockCopy(part, 0, payload, offset, part.Length);
                    offset += part.Length;
                }
                if (offset != count) throw new InvalidDataException("回放数据缺失，不能播放。");
                Matrix3d delta = br.BlockTransform * new Matrix3d(baseline).Inverse();
                if (!AcceptTransform(delta, out reason)) return false;
                route = PlaybackRouteCodec.Decode(payload);
                transform = delta;
                return true;
            }
            catch (System.Exception ex)
            {
                reason = "无法读取轨迹回放：" + ex.Message;
                return false;
            }
        }

        /// <summary>Only planar, orientation-preserving, uniform scales have a sound playback meaning.</summary>
        internal static bool AcceptTransform(Matrix3d matrix, out string reason)
        {
            reason = "轨迹块包含镜像、非等比缩放或三维倾斜，暂不支持准确回放。请还原变换后重试。";
            Point3d origin = Point3d.Origin.TransformBy(matrix);
            Vector3d x = new Point3d(1, 0, 0).TransformBy(matrix) - origin;
            Vector3d y = new Point3d(0, 1, 0).TransformBy(matrix) - origin;
            Vector3d z = new Point3d(0, 0, 1).TransformBy(matrix) - origin;
            double scale = x.Length;
            if (!Finite(origin.X) || !Finite(origin.Y) || !Finite(origin.Z) || !Finite(scale) ||
                !Finite(x.X) || !Finite(x.Y) || !Finite(x.Z) ||
                !Finite(y.X) || !Finite(y.Y) || !Finite(y.Z) ||
                !Finite(z.X) || !Finite(z.Y) || !Finite(z.Z) ||
                scale < 1e-9 || scale > 1e9) return false;
            double tolerance = 1e-7 * scale;
            if (Math.Abs(y.Length - scale) > tolerance || Math.Abs(z.Length - scale) > tolerance ||
                Math.Abs(x.Z) > tolerance || Math.Abs(y.Z) > tolerance ||
                Math.Abs(z.X) > tolerance || Math.Abs(z.Y) > tolerance || z.Z <= 0 ||
                Math.Abs(x.DotProduct(y)) > tolerance * scale ||
                x.X * y.Y - x.Y * y.X <= 0) return false;
            reason = null;
            return true;
        }

        static bool Finite(double value) { return !double.IsNaN(value) && !double.IsInfinity(value); }
    }

    public sealed class PlaybackCommands
    {
        [CommandMethod("TTPLAY")]
        public void Play()
        {
            PlaybackSession.CloseActive();
            Document doc = CadApplication.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            var options = new PromptEntityOptions("\n选择本版本生成的完整轨迹块以回放：");
            options.SetRejectMessage("\n请选择轨迹块，而不是单根线或普通图元。");
            options.AddAllowedClass(typeof(BlockReference), true);
            PromptEntityResult selected = doc.Editor.GetEntity(options);
            if (selected.Status == PromptStatus.OK) Open(doc, selected.ObjectId);
        }

        [CommandMethod("TTSTOP")]
        public void Stop() { PlaybackSession.CloseActive(); }

        public static void StopAll() { PlaybackSession.CloseActive(); }

        public static void OfferPlayback(Document doc, ObjectId resultBlockId)
        {
            if (doc == null || resultBlockId.IsNull) return;
            var options = new PromptKeywordOptions("\n轨迹已保存，是否打开动画回放？ [播放(P)/不播放(N)] <N>：");
            options.Keywords.Add("Play", "P", "播放(P)");
            options.Keywords.Add("No", "N", "不播放(N)");
            options.Keywords.Default = "No";
            options.AllowNone = true;
            PromptResult result = doc.Editor.GetKeywords(options);
            if (result.Status == PromptStatus.OK && result.StringResult == "Play")
                Open(doc, resultBlockId);
            else
                doc.Editor.WriteMessage("\n以后可输入 TTPLAY，选择该轨迹块再次播放。\n");
        }

        static void Open(Document doc, ObjectId id)
        {
            try
            {
                PlaybackRoute route;
                Matrix3d transform;
                string reason;
                using (var tr = doc.Database.TransactionManager.StartTransaction())
                {
                    var block = tr.GetObject(id, OpenMode.ForRead) as BlockReference;
                    if (!PlaybackCadStore.TryRead(doc.Database, tr, block, out route, out transform, out reason))
                    {
                        doc.Editor.WriteMessage("\n" + reason + "\n");
                        return;
                    }
                    tr.Commit();
                }
                PlaybackSession.Start(doc, route, transform);
            }
            catch (System.Exception ex)
            { doc.Editor.WriteMessage("\n无法打开轨迹回放：" + ex.Message + "\n"); }
        }
    }

    /// <summary>
    /// All callbacks run on the host's WinForms message thread. There are no workers,
    /// per-frame database transactions, envelope computations or Editor.Regen calls.
    /// </summary>
    internal sealed class PlaybackSession : IDisposable
    {
        static PlaybackSession active;
        sealed class DeferredGraphics
        {
            public Document Document;
            public PlaybackDrawable Drawable;
            public GI.TransientManager Manager;
            public IntegerCollection Viewports;
        }
        // Only an exceptional host erase failure reaches this list. Keep a strong reference:
        // disposing/finalizing a native entity that CAD may still hold would be unsafe.
        // There are no retry timers or permanent event hooks; TTSTOP / the next replay retries.
        static readonly List<DeferredGraphics> deferredGraphics = new List<DeferredGraphics>();
        readonly Document document;
        readonly PlaybackRoute route;
        readonly Matrix3d transform;
        readonly PlaybackPlayerForm form;
        readonly System.Windows.Forms.Timer timer;
        readonly Stopwatch clock = new Stopwatch();
        readonly PlaybackDrawable drawable = new PlaybackDrawable();
        readonly IntegerCollection viewports = new IntegerCollection();
        readonly GI.TransientManager manager;
        bool disposed;
        bool running;
        double position;
        double speed = 1;

        PlaybackSession(Document doc, PlaybackRoute replay, Matrix3d delta)
        {
            document = doc;
            route = replay;
            transform = delta;
            manager = GI.TransientManager.CurrentTransientManager;
            form = new PlaybackPlayerForm(route.DisplayName, route.DurationSeconds);
            timer = new System.Windows.Forms.Timer { Interval = 33 };
            timer.Tick += Tick;
            form.PlayRequested += Play;
            form.PauseRequested += Pause;
            form.StopRequested += Stop;
            form.SeekRequested += Seek;
            form.SpeedChanged += ChangeSpeed;
            form.FormClosed += PlayerClosed;
            document.CommandWillStart += CommandStarting;
            CadApplication.DocumentManager.DocumentToBeDeactivated += DocumentLeaving;
            CadApplication.DocumentManager.DocumentToBeDestroyed += DocumentLeaving;
        }

        public static void Start(Document doc, PlaybackRoute route, Matrix3d transform)
        {
            CloseActive();
            PlaybackRouteCodec.Validate(route);
            PlaybackSession next = null;
            try
            {
                next = new PlaybackSession(doc, route, transform);
                active = next;
                CadApplication.ShowModelessDialog(next.form);
            }
            catch
            {
                if (next != null) next.Dispose();
                throw;
            }
        }

        public static void CloseActive()
        {
            if (active != null) active.Dispose();
            RetryDeferredGraphics();
        }

        static void RetryDeferredGraphics()
        {
            Document current;
            try { current = CadApplication.DocumentManager.MdiActiveDocument; }
            catch { return; }
            if (current == null) return;
            for (int i = deferredGraphics.Count - 1; i >= 0; i--)
            {
                DeferredGraphics item = deferredGraphics[i];
                if (item.Document != current) continue;
                try
                {
                    using (current.LockDocument()) item.Drawable.Erase(item.Manager, item.Viewports);
                    item.Drawable.Dispose();
                    item.Viewports.Clear();
                    deferredGraphics.RemoveAt(i);
                }
                catch { /* Host still owns it: retain safely, without a polling loop. */ }
            }
        }

        void CommandStarting(object sender, CommandEventArgs e) { Dispose(); }
        void PlayerClosed(object sender, System.Windows.Forms.FormClosedEventArgs e) { Dispose(); }
        void DocumentLeaving(object sender, DocumentCollectionEventArgs e)
        { if (e.Document == document) Dispose(); }

        bool Ready()
        {
            if (disposed) return false;
            if (CadApplication.DocumentManager.MdiActiveDocument == document) return true;
            Dispose();
            return false;
        }

        void Play()
        {
            if (!Ready()) return;
            if (position >= route.DurationSeconds) position = 0;
            if (!Draw()) return;
            if (route.DurationSeconds <= 0)
            {
                running = false;
                timer.Stop(); clock.Reset();
                form.SetPosition(0, "此路线仅包含一个姿态；停止可移除临时车辆。");
                return;
            }
            running = true;
            clock.Restart();
            timer.Start();
            form.SetPosition(position, "播放中 · 示意速度，非实际行驶计时");
        }

        void Pause()
        {
            if (!Ready()) return;
            AdvanceClock();
            running = false;
            timer.Stop(); clock.Reset();
            if (!Draw()) return;
            form.SetPosition(position, "已暂停；可拖动进度。运行其他 CAD 命令将关闭回放。");
        }

        void Stop()
        {
            if (disposed) return;
            running = false; timer.Stop(); clock.Reset(); position = 0;
            try { Erase(); }
            catch (System.Exception ex)
            {
                try { document.Editor.WriteMessage("\n清理回放临时图形失败：" + ex.Message + "\n"); } catch { }
                Dispose();
                return;
            }
            form.SetPosition(0, "已停止，临时车辆已移除；可再次播放。");
        }

        void Seek(double seconds)
        {
            if (!Ready()) return;
            position = Math.Max(0, Math.Min(route.DurationSeconds, seconds));
            clock.Restart();
            if (!Draw()) return;
            form.SetPosition(position, running ? "播放中 · 示意速度，非实际行驶计时" : "定位预览；点击播放继续。");
        }

        void ChangeSpeed(double multiplier)
        {
            if (!Ready()) return;
            AdvanceClock();
            speed = multiplier;
            clock.Restart();
        }

        void AdvanceClock()
        {
            if (!running) return;
            position += clock.Elapsed.TotalSeconds * speed;
            clock.Restart();
            if (position > route.DurationSeconds) position = route.DurationSeconds;
        }

        void Tick(object sender, EventArgs e)
        {
            if (!Ready() || !running) return;
            AdvanceClock();
            if (position >= route.DurationSeconds)
            {
                if (form.Loop && route.DurationSeconds > 0) position = 0;
                else { running = false; timer.Stop(); clock.Reset(); }
            }
            if (Draw()) form.SetPosition(position, running ? null : "播放完成；可重播、拖动进度或停止清理。");
        }

        bool Draw()
        {
            if (!Ready()) return false;
            try
            {
                // A short modeless document lock avoids host re-entrancy. No DB write occurs.
                using (document.LockDocument())
                {
                    drawable.SetPose(route, PlaybackTimeline.Sample(route, position), transform);
                    drawable.Update(manager, viewports);
                }
                return true;
            }
            catch (System.Exception ex)
            {
                try { document.Editor.WriteMessage("\n回放已停止：" + ex.Message + "\n"); } catch { }
                Dispose();
                return false;
            }
        }

        void Erase()
        {
            drawable.Erase(manager, viewports);
        }

        public void Dispose()
        {
            if (disposed) return;
            disposed = true;
            running = false;
            timer.Stop(); timer.Dispose(); clock.Stop();
            form.FormClosed -= PlayerClosed;
            try { document.CommandWillStart -= CommandStarting; } catch { }
            try { CadApplication.DocumentManager.DocumentToBeDeactivated -= DocumentLeaving; } catch { }
            try { CadApplication.DocumentManager.DocumentToBeDestroyed -= DocumentLeaving; } catch { }
            try { Erase(); } catch { }
            if (drawable.HasRegisteredEntities)
            {
                deferredGraphics.Add(new DeferredGraphics { Document = document, Drawable = drawable,
                    Manager = manager, Viewports = viewports });
                try { document.Editor.WriteMessage("\nCAD 尚未移除全部临时图形，已安全延迟释放；可输入 TTSTOP 重试清理。\n"); } catch { }
            }
            else
            {
                try { drawable.Dispose(); } catch { }
                viewports.Clear();
            }
            try { if (!form.IsDisposed) form.Close(); } catch { }
            if (active == this) active = null;
        }
    }
}
