using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using System.Xml;
using System.Xml.Serialization;

namespace TruckTurn
{
    /// <summary>
    /// 仅定型数据的回放存储格式。禁止 BinaryFormatter / 类型名反序列化。
    /// GZip 内为受限二进制帧和定型 XML 参数；读入时限制压缩/解压大小、计数和全部几何值。
    /// </summary>
    public static class PlaybackRouteCodec
    {
        public const int MaximumPoseCount = 200000;
        public const int MaximumEncodedBytes = 32 * 1024 * 1024;
        public const int MaximumDecodedBytes = 32 * 1024 * 1024;
        const int MaximumParameterBytes = 128 * 1024;
        const int MaximumNameChars = 256;
        const int MaximumSourceChars = 4096;
        const uint Magic = 0x31505454; // TTP1
        const uint EnvelopeMagic = 0x43505454; // TTPC: size/checksum envelope around GZip
        static readonly Encoding StrictUtf8 = new UTF8Encoding(false, true);
        static readonly XmlSerializer TruckSerializer = new XmlSerializer(typeof(VehicleParams));
        static readonly XmlSerializer CarrierSerializer = new XmlSerializer(typeof(CarrierParams));

        public static byte[] Encode(PlaybackRoute route)
        {
            Validate(route);
            using (var raw = new MemoryStream())
            {
                using (var writer = new BinaryWriter(raw, StrictUtf8, true))
                {
                    writer.Write(Magic);
                    writer.Write(route.Version);
                    writer.Write((int)route.Kind);
                    writer.Write(route.UnitScale);
                    WriteString(writer, route.DisplayName ?? "", MaximumNameChars);
                    WriteString(writer, route.SourceSummary ?? "", MaximumSourceChars);
                    byte[] parameters = route.Kind == PlaybackVehicleKind.Carrier
                        ? SerializeParameters(route.CarrierParameters, CarrierSerializer)
                        : SerializeParameters(route.TruckParameters, TruckSerializer);
                    writer.Write(parameters.Length);
                    writer.Write(parameters);
                    writer.Write(route.Poses.Count);
                    foreach (PlaybackPose p in route.Poses)
                    {
                        writer.Write(p.TimeSeconds); writer.Write(p.X); writer.Write(p.Y);
                        writer.Write(p.HeadingRad); writer.Write(p.TrailerHeadingRad); writer.Write(p.FrontSteerRad);
                        writer.Write(p.SteeringMode); writer.Write(p.TravelDirection);
                        int wheels = p.WheelAnglesRad == null ? 0 : p.WheelAnglesRad.Length;
                        writer.Write(wheels);
                        for (int i = 0; i < wheels; i++) writer.Write(p.WheelAnglesRad[i]);
                    }
                }
                if (raw.Length > MaximumDecodedBytes) throw Bad("回放数据超过存储上限，请分段录制。");
                raw.Position = 0;
                using (var encoded = new MemoryStream())
                {
                    using (var gzip = new GZipStream(encoded, CompressionMode.Compress, true)) raw.CopyTo(gzip);
                    byte[] compressed = encoded.ToArray();
                    if (compressed.Length + 40 > MaximumEncodedBytes) throw Bad("回放数据压缩后超过存储上限。");
                    using (var result = new MemoryStream())
                    using (var writer = new BinaryWriter(result, StrictUtf8, true))
                    using (var sha = SHA256.Create())
                    {
                        writer.Write(EnvelopeMagic); writer.Write(compressed.Length);
                        writer.Write(sha.ComputeHash(compressed)); writer.Write(compressed);
                        writer.Flush();
                        return result.ToArray();
                    }
                }
            }
        }

        public static PlaybackRoute Decode(byte[] bytes)
        {
            if (bytes == null || bytes.Length == 0 || bytes.Length > MaximumEncodedBytes)
                throw Bad("回放数据为空或大小超出上限。");
            try
            {
                byte[] compressed;
                using (var envelope = new MemoryStream(bytes, false))
                using (var reader = new BinaryReader(envelope, StrictUtf8, true))
                using (var sha = SHA256.Create())
                {
                    if (reader.ReadUInt32() != EnvelopeMagic) throw Bad("不是 TruckTurn 压缩回放数据。");
                    int length = reader.ReadInt32();
                    if (length < 1 || length != bytes.Length - 40) throw Bad("回放压缩长度不完整。");
                    byte[] expected = ReadExact(reader, 32);
                    compressed = ReadExact(reader, length);
                    byte[] actual = sha.ComputeHash(compressed);
                    int mismatch = 0;
                    for (int i = 0; i < actual.Length; i++) mismatch |= actual[i] ^ expected[i];
                    if (mismatch != 0) throw Bad("回放数据校验失败，数据可能损坏。");
                }
                using (var input = new MemoryStream(compressed, false))
                using (var gzip = new GZipStream(input, CompressionMode.Decompress))
                using (var raw = new MemoryStream())
                {
                    var buffer = new byte[8192];
                    int read;
                    while ((read = gzip.Read(buffer, 0, buffer.Length)) > 0)
                    {
                        if (raw.Length + read > MaximumDecodedBytes) throw Bad("回放解压数据超出上限。");
                        raw.Write(buffer, 0, read);
                    }
                    raw.Position = 0;
                    using (var reader = new BinaryReader(raw, StrictUtf8, true))
                    {
                        if (reader.ReadUInt32() != Magic) throw Bad("不是 TruckTurn 回放数据。");
                        int version = reader.ReadInt32();
                        if (version != 1) throw Bad("不支持此回放格式版本，请使用对应版本插件。");
                        var route = new PlaybackRoute
                        {
                            Version = version, Kind = (PlaybackVehicleKind)reader.ReadInt32(),
                            UnitScale = reader.ReadDouble(), DisplayName = ReadString(reader, MaximumNameChars),
                            SourceSummary = ReadString(reader, MaximumSourceChars)
                        };
                        if (!Enum.IsDefined(typeof(PlaybackVehicleKind), route.Kind)) throw Bad("回放车型类别无效。");
                        int parameterLength = reader.ReadInt32();
                        if (parameterLength < 1 || parameterLength > MaximumParameterBytes) throw Bad("回放参数长度无效。");
                        byte[] parameters = ReadExact(reader, parameterLength);
                        if (route.Kind == PlaybackVehicleKind.Carrier)
                            route.CarrierParameters = (CarrierParams)DeserializeParameters(parameters, CarrierSerializer);
                        else route.TruckParameters = (VehicleParams)DeserializeParameters(parameters, TruckSerializer);
                        int count = reader.ReadInt32();
                        if (count < 1 || count > MaximumPoseCount) throw Bad("回放帧数量超出上限或为空。");
                        // 每帧至少 60 字节，拒绝伪造计数在剩余字节不足时预分配大列表。
                        if ((long)count * 60 > raw.Length - raw.Position) throw Bad("回放帧数据不完整。");
                        route.Poses = new List<PlaybackPose>(count);
                        for (int i = 0; i < count; i++)
                        {
                            var p = new PlaybackPose
                            {
                                TimeSeconds = reader.ReadDouble(), X = reader.ReadDouble(), Y = reader.ReadDouble(),
                                HeadingRad = reader.ReadDouble(), TrailerHeadingRad = reader.ReadDouble(), FrontSteerRad = reader.ReadDouble(),
                                SteeringMode = reader.ReadInt32(), TravelDirection = reader.ReadInt32()
                            };
                            int wheels = reader.ReadInt32();
                            int expectedWheels = route.Kind == PlaybackVehicleKind.Carrier ? 4 : 0;
                            if (wheels != expectedWheels) throw Bad("回放车轮数量无效。");
                            p.WheelAnglesRad = new double[wheels];
                            for (int w = 0; w < wheels; w++) p.WheelAnglesRad[w] = reader.ReadDouble();
                            route.Poses.Add(p);
                        }
                        if (raw.Position != raw.Length) throw Bad("回放数据包含未知尾随内容。");
                        Validate(route);
                        return route;
                    }
                }
            }
            catch (InvalidDataException) { throw; }
            catch (Exception ex) when (ex is EndOfStreamException || ex is IOException || ex is InvalidOperationException ||
                                       ex is XmlException || ex is DecoderFallbackException || ex is ArgumentException)
            {
                throw new InvalidDataException("回放数据损坏或不受支持，未执行播放。", ex);
            }
        }

        internal static VehicleParams CloneTruckParameters(VehicleParams p)
        {
            return (VehicleParams)DeserializeParameters(SerializeParameters(p, TruckSerializer), TruckSerializer);
        }
        internal static CarrierParams CloneCarrierParameters(CarrierParams p)
        {
            return (CarrierParams)DeserializeParameters(SerializeParameters(p, CarrierSerializer), CarrierSerializer);
        }

        static byte[] SerializeParameters(object p, XmlSerializer serializer)
        {
            if (p == null) throw Bad("回放缺少车辆参数。");
            using (var output = new MemoryStream())
            {
                var settings = new XmlWriterSettings { Encoding = StrictUtf8, Indent = false, CloseOutput = false };
                using (var writer = XmlWriter.Create(output, settings)) serializer.Serialize(writer, p);
                if (output.Length > MaximumParameterBytes) throw Bad("车辆参数快照超出上限。");
                return output.ToArray();
            }
        }

        static object DeserializeParameters(byte[] data, XmlSerializer serializer)
        {
            var settings = new XmlReaderSettings
            {
                DtdProcessing = DtdProcessing.Prohibit, XmlResolver = null,
                MaxCharactersInDocument = MaximumParameterBytes, MaxCharactersFromEntities = 0,
                IgnoreWhitespace = true, CloseInput = false
            };
            using (var input = new MemoryStream(data, false))
            using (var reader = XmlReader.Create(input, settings))
            {
                object result = serializer.Deserialize(reader);
                // 强制读到 EOF，以便 XML 对象后的第二根节点/非法尾部也不能被忽略。
                while (reader.Read()) { }
                return result;
            }
        }

        public static void Validate(PlaybackRoute route)
        {
            if (route == null) throw Bad("回放路线为空。");
            if (route.Version != 1) throw Bad("不支持此回放格式版本。");
            if (!Enum.IsDefined(typeof(PlaybackVehicleKind), route.Kind)) throw Bad("回放车型类别无效。");
            CheckText(route.DisplayName, MaximumNameChars);
            CheckText(route.SourceSummary, MaximumSourceChars);
            if (!InRange(route.UnitScale, 1e-6, 1e9)) throw Bad("回放图纸单位无效。");
            if (route.Kind == PlaybackVehicleKind.Carrier)
            {
                CarrierParams p = route.CarrierParameters;
                if (p == null || route.TruckParameters != null) throw Bad("搬运车参数快照无效。");
                ValidateCarrier(p);
                if (p.UnitScale != route.UnitScale) throw Bad("回放单位与参数快照不一致。");
            }
            else
            {
                VehicleParams p = route.TruckParameters;
                if (p == null || route.CarrierParameters != null) throw Bad("货车参数快照无效。");
                ValidateTruck(p);
                if (p.UnitScale != route.UnitScale) throw Bad("回放单位与参数快照不一致。");
                if (route.Kind == PlaybackVehicleKind.Fire && p.Articulated) throw Bad("消防车不支持铰接参数。");
            }
            if (route.Poses == null || route.Poses.Count < 1 || route.Poses.Count > MaximumPoseCount)
                throw Bad("回放帧数无效或超出上限。");
            double lastTime = -1;
            foreach (PlaybackPose p in route.Poses)
            {
                if (p == null || !InRange(p.TimeSeconds, 0, 86400 * 30.0) || p.TimeSeconds <= lastTime)
                    throw Bad("回放时间必须有限且严格递增。");
                if (lastTime == -1 && p.TimeSeconds != 0) throw Bad("回放首帧必须从零秒开始。");
                lastTime = p.TimeSeconds;
                if (!InRange(p.X, -1e12, 1e12) || !InRange(p.Y, -1e12, 1e12) ||
                    !InRange(p.HeadingRad, -1e9, 1e9) || !InRange(p.TrailerHeadingRad, -1e9, 1e9) ||
                    !InRange(p.FrontSteerRad, -Math.PI, Math.PI)) throw Bad("回放姿态包含无效坐标或角度。");
                if (p.TravelDirection != 1 && p.TravelDirection != -1) throw Bad("回放行驶方向无效。");
                int wheelCount = p.WheelAnglesRad == null ? 0 : p.WheelAnglesRad.Length;
                if (wheelCount != (route.Kind == PlaybackVehicleKind.Carrier ? 4 : 0)) throw Bad("回放车轮数量无效。");
                if (route.Kind == PlaybackVehicleKind.Carrier &&
                    !Enum.IsDefined(typeof(CarrierSteeringMode), p.SteeringMode)) throw Bad("回放转向模式无效。");
                if (route.Kind != PlaybackVehicleKind.Carrier && p.SteeringMode != 0) throw Bad("货车回放转向模式无效。");
                if (p.WheelAnglesRad != null)
                    foreach (double angle in p.WheelAnglesRad)
                        if (!InRange(angle, -Math.PI, Math.PI)) throw Bad("回放车轮转角无效。");
            }
        }

        static void ValidateTruck(VehicleParams p)
        {
            ValidatePublicScalars(p);
            double s = p.UnitScale;
            if (!InRange(s, 1e-6, 1e9) || !InRange(p.TractorWheelbase / s, 0.01, 10000) ||
                !InRange(p.TractorWidth / s, 0.01, 10000) || !InRange(p.WheelTrack / s, 0.001, 10000) ||
                !InRange(p.TractorFrontOverhang / s, 0, 10000) || !InRange(p.RigidRearOverhang / s, 0, 10000) ||
                !InRange(p.SteerAngleDeg, 0, 89.99) || !InRange(p.MaxArticulationAngleDeg, 0, 180))
                throw Bad("货车参数超出回放安全范围。");
            if ((p.Articulated && (p.TrailerAxles < 1 || p.TrailerAxles > 3)) ||
                (!p.Articulated && (p.RigidAxles < 2 || p.RigidAxles > 6)))
                throw Bad("货车轴数参数无效。");
            if (p.Articulated && (!InRange(p.TrailerKingpinToRearAxle / s, 0.01, 10000) ||
                !InRange(p.TrailerWidth / s, 0.01, 10000) || !InRange(p.TrailerRearOverhang / s, 0, 10000) ||
                !InRange(p.TrailerKingpinToFront / s, 0, 10000))) throw Bad("挂车参数超出回放安全范围。");
        }

        static void ValidateCarrier(CarrierParams p)
        {
            ValidatePublicScalars(p);
            if (!Enum.IsDefined(typeof(CarrierArchitecture), p.Architecture) ||
                !Enum.IsDefined(typeof(CarrierDataStatus), p.DataStatus) ||
                !Enum.IsDefined(typeof(CarrierContainerPreset), p.ContainerPreset) ||
                (p.SteeringCapabilities & ~(CarrierSteeringCapabilities)127) != 0)
                throw Bad("搬运车参数枚举值无效。");
            // 旧块回放只验证安全读取所需的数值/类型，不用新版本外廓业务规则拒绝历史快照。
            if (p.Validate(false).Count != 0) throw Bad("搬运车参数未通过有效性检查。");
            if (!InRange(p.OverallLength, 0.01, 10000) || !InRange(p.OverallWidth, 0.01, 10000) ||
                !InRange(p.Wheelbase, 0.01, 10000) || !InRange(p.TrackWidth, 0.01, 10000) ||
                !InRange(p.WheelLength, 0.001, 10000) || !InRange(p.WheelWidth, 0.001, 10000) ||
                !InRange(p.SafetyMargin, 0, 10000) || !InRange(p.ContainerOffsetX, -10000, 10000) ||
                !InRange(p.ContainerOffsetY, -10000, 10000) || !InRange(p.Twin20Gap, 0, 10000))
                throw Bad("搬运车几何参数超出回放安全范围。");
            CarrierContainerDimensions box = p.ContainerDimensions();
            if (!InRange(box.Length, 0, 10000) || !InRange(box.Width, 0, 10000) || !InRange(box.Height, 0, 10000))
                throw Bad("集装箱尺寸超出回放安全范围。");
            CheckText(p.PresetName, MaximumNameChars); CheckText(p.Manufacturer, MaximumNameChars); CheckText(p.Model, MaximumNameChars);
            CarrierWheelLocation[] raw = { p.FrontLeft, p.FrontRight, p.RearLeft, p.RearRight };
            for (int i = 0; i < 4; i++)
            {
                CarrierWheelLocation w = raw[i];
                // NaN 是默认矩形轮位的明确 sentinel；Infinity 和其他数值溢出不允许。
                if ((int)w.Id != i || (!double.IsNaN(w.X) && !InRange(w.X, -10000, 10000)) ||
                    (!double.IsNaN(w.Y) && !InRange(w.Y, -10000, 10000)) ||
                    !InRange(w.MaxSteerAngleDeg, 0.001, 180)) throw Bad("搬运车轮位参数无效。");
            }
        }

        static void ValidatePublicScalars(object p)
        {
            foreach (FieldInfo field in p.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance))
            {
                if (field.FieldType == typeof(double) && !InRange((double)field.GetValue(p), -1e12, 1e12))
                    throw Bad("车辆参数包含非有限或过大数值：" + field.Name);
                if (field.FieldType == typeof(string)) CheckText((string)field.GetValue(p), MaximumSourceChars);
            }
        }

        static void CheckText(string text, int maximum)
        {
            if (text == null) return;
            if (text.Length > maximum) throw Bad("回放文字长度超出上限。");
            for (int i = 0; i < text.Length; i++)
                if (char.IsControl(text[i]) && text[i] != '\n' && text[i] != '\r' && text[i] != '\t')
                    throw Bad("回放文字包含无效控制字符。");
            // 拒绝无配对代理项，避免编码/跨运行时字符串行为不一致。
            try { StrictUtf8.GetByteCount(text); }
            catch (EncoderFallbackException ex) { throw new InvalidDataException("回放文字编码无效。", ex); }
        }
        static bool InRange(double x, double min, double max)
        {
            return PlaybackTimeline.Finite(x) && x >= min && x <= max;
        }
        static InvalidDataException Bad(string message) { return new InvalidDataException(message); }
        static void WriteString(BinaryWriter writer, string value, int maxChars)
        {
            CheckText(value, maxChars);
            byte[] bytes = StrictUtf8.GetBytes(value);
            writer.Write(bytes.Length); writer.Write(bytes);
        }
        static string ReadString(BinaryReader reader, int maxChars)
        {
            int length = reader.ReadInt32();
            if (length < 0 || length > maxChars * 4) throw Bad("回放文字长度无效。");
            string result = StrictUtf8.GetString(ReadExact(reader, length));
            CheckText(result, maxChars);
            return result;
        }
        static byte[] ReadExact(BinaryReader reader, int length)
        {
            byte[] result = reader.ReadBytes(length);
            if (result.Length != length) throw Bad("回放数据不完整。");
            return result;
        }
    }
}
