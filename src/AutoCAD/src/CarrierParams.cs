using System;
using System.Collections.Generic;

namespace TruckTurn
{
    /// <summary>跨运车运动学架构。V1 只计算四轮车型，多轴类型用于持久化和后续扩展。</summary>
    public enum CarrierArchitecture
    {
        FourWheelLinked = 0,
        FourWheelIndependent = 1,
        MultiAxle = 2
    }

    public enum CarrierDataStatus
    {
        Generic = 0,
        UserDefined = 1,
        ManufacturerVerified = 2
    }

    [Flags]
    public enum CarrierSteeringCapabilities
    {
        None = 0,
        Longitudinal = 1 << 0,
        FrontOnly = 1 << 1,
        RearOnly = 1 << 2,
        CounterPhaseFourWheel = 1 << 3,
        Crab = 1 << 4,
        Lateral = 1 << 5,
        Pivot = 1 << 6
    }

    public enum CarrierSteeringMode
    {
        Longitudinal = 0,
        FrontOnly = 1,
        RearOnly = 2,
        CounterPhaseFourWheel = 3,
        Crab = 4,
        Lateral = 5,
        Pivot = 6
    }

    public enum CarrierTurningRadiusReference
    {
        BodyReferencePoint = 0,
        InnerWheel = 1,
        OuterWheel = 2,
        OuterBody = 3
    }

    public enum CarrierContainerPreset
    {
        None = 0,
        Iso20Gp = 1,
        Iso40Gp = 2,
        Iso40Hc = 3,
        Iso45Hc = 4,
        Custom = 5,
        Twin20 = 6
    }

    public enum CarrierWheelId
    {
        FrontLeft = 0,
        FrontRight = 1,
        RearLeft = 2,
        RearRight = 3
    }

    /// <summary>车轮中心在车体坐标系中的位置。X 向前，Y 向左，单位为米。</summary>
    public struct CarrierWheelLocation
    {
        public CarrierWheelId Id;
        public double X;
        public double Y;
        public double MaxSteerAngleDeg;

        public CarrierWheelLocation(CarrierWheelId id, double x, double y, double maxSteerAngleDeg)
        {
            Id = id;
            X = x;
            Y = y;
            MaxSteerAngleDeg = maxSteerAngleDeg;
        }
    }

    public struct CarrierContainerDimensions
    {
        public double Length;
        public double Width;
        public double Height;

        public CarrierContainerDimensions(double length, double width, double height)
        {
            Length = length;
            Width = width;
            Height = height;
        }
    }

    /// <summary>
    /// 集装箱跨运车参数。所有几何字段均使用米，角度字段名称明确带 Deg。
    /// 本类不引用 CAD 类型，可直接进入无 CAD 依赖的验证工程。
    /// </summary>
    public class CarrierParams
    {
        public const int CurrentSchemaVersion = 3;

        public int SchemaVersion = CurrentSchemaVersion;
        public string PresetName = "通用四轮独立转向跨运车";
        public string Manufacturer = "";
        public string Model = "";
        public CarrierDataStatus DataStatus = CarrierDataStatus.Generic;
        public CarrierArchitecture Architecture = CarrierArchitecture.FourWheelIndependent;
        public CarrierSteeringCapabilities SteeringCapabilities =
            CarrierSteeringCapabilities.Longitudinal |
            CarrierSteeringCapabilities.CounterPhaseFourWheel |
            CarrierSteeringCapabilities.Crab |
            CarrierSteeringCapabilities.Lateral;

        // 车体外廓与轮位，单位 m。默认值仅用于通用算法演示，不代表厂家车型。
        public double OverallLength = 8.0;
        public double OverallWidth = 6.0;
        public double Wheelbase = 6.0;
        public double TrackWidth = 4.0;
        public double WheelLength = 1.2;
        public double WheelWidth = 0.45;

        // 每个轮位可独立覆盖。位置为 NaN 时由 Wheelbase/TrackWidth 生成矩形轮位。
        public CarrierWheelLocation FrontLeft =
            new CarrierWheelLocation(CarrierWheelId.FrontLeft, double.NaN, double.NaN, 90.0);
        public CarrierWheelLocation FrontRight =
            new CarrierWheelLocation(CarrierWheelId.FrontRight, double.NaN, double.NaN, 90.0);
        public CarrierWheelLocation RearLeft =
            new CarrierWheelLocation(CarrierWheelId.RearLeft, double.NaN, double.NaN, 90.0);
        public CarrierWheelLocation RearRight =
            new CarrierWheelLocation(CarrierWheelId.RearRight, double.NaN, double.NaN, 90.0);

        public double MaxSteerRateDegPerSecond = 15.0;
        public double MinimumReferenceRadius = 9.0;
        public CarrierTurningRadiusReference TurningRadiusReference =
            CarrierTurningRadiusReference.BodyReferencePoint;

        public double SafetyMargin = 0.30;
        public double UnitScale = 1.0;

        // CAD 输出开关；独立于旧货车 VehicleParams，避免修改既有 CSV 语义。
        public bool ShowReferencePath = false;
        public bool ShowEquipmentEnvelope = true;
        public bool ShowContainerEnvelope = false;
        public bool ShowSafetyEnvelope = true;
        public bool ShowWheelTracks = false;
        public bool ShowBody = true;
        /// <summary>
        /// 连续驾驶时是否保留每个确认节点的完整车辆姿态。
        /// false=仅首尾，true=全部节点；不影响包络、轨迹和动态预览。
        /// </summary>
        public bool ShowAllNodeVehicles = false;

        public CarrierContainerPreset ContainerPreset = CarrierContainerPreset.Iso40Gp;
        public double CustomContainerLength = 12.192;
        public double CustomContainerWidth = 2.438;
        public double CustomContainerHeight = 2.591;
        public double ContainerOffsetX = 0.0;
        public double ContainerOffsetY = 0.0;
        public double ContainerSlewAngleDeg = 0.0;
        public double Twin20Gap = 0.076;

        public static CarrierParams GenericFourWheelIndependent()
        {
            return new CarrierParams();
        }

        public static CarrierParams GenericFourWheelLinked()
        {
            var p = new CarrierParams();
            p.PresetName = "通用四轮联动转向跨运车";
            p.Architecture = CarrierArchitecture.FourWheelLinked;
            p.SteeringCapabilities =
                CarrierSteeringCapabilities.Longitudinal |
                CarrierSteeringCapabilities.FrontOnly |
                CarrierSteeringCapabilities.RearOnly |
                CarrierSteeringCapabilities.CounterPhaseFourWheel;
            p.FrontLeft.MaxSteerAngleDeg = 45.0;
            p.FrontRight.MaxSteerAngleDeg = 45.0;
            p.RearLeft.MaxSteerAngleDeg = 45.0;
            p.RearRight.MaxSteerAngleDeg = 45.0;
            return p;
        }

        public CarrierWheelLocation[] ResolvedWheelLocations()
        {
            double hx = Wheelbase * 0.5;
            double hy = TrackWidth * 0.5;
            return new[]
            {
                Resolve(FrontLeft, +hx, +hy),
                Resolve(FrontRight, +hx, -hy),
                Resolve(RearLeft, -hx, +hy),
                Resolve(RearRight, -hx, -hy)
            };
        }

        public bool Supports(CarrierSteeringMode mode)
        {
            CarrierSteeringCapabilities flag;
            switch (mode)
            {
                case CarrierSteeringMode.Longitudinal: flag = CarrierSteeringCapabilities.Longitudinal; break;
                case CarrierSteeringMode.FrontOnly: flag = CarrierSteeringCapabilities.FrontOnly; break;
                case CarrierSteeringMode.RearOnly: flag = CarrierSteeringCapabilities.RearOnly; break;
                case CarrierSteeringMode.CounterPhaseFourWheel: flag = CarrierSteeringCapabilities.CounterPhaseFourWheel; break;
                case CarrierSteeringMode.Crab: flag = CarrierSteeringCapabilities.Crab; break;
                case CarrierSteeringMode.Lateral: flag = CarrierSteeringCapabilities.Lateral; break;
                case CarrierSteeringMode.Pivot: flag = CarrierSteeringCapabilities.Pivot; break;
                default: return false;
            }
            return (SteeringCapabilities & flag) != 0;
        }

        public CarrierContainerDimensions ContainerDimensions()
        {
            switch (ContainerPreset)
            {
                case CarrierContainerPreset.None: return new CarrierContainerDimensions(0.0, 0.0, 0.0);
                case CarrierContainerPreset.Iso20Gp: return new CarrierContainerDimensions(6.058, 2.438, 2.591);
                case CarrierContainerPreset.Iso40Gp: return new CarrierContainerDimensions(12.192, 2.438, 2.591);
                case CarrierContainerPreset.Iso40Hc: return new CarrierContainerDimensions(12.192, 2.438, 2.896);
                case CarrierContainerPreset.Iso45Hc: return new CarrierContainerDimensions(13.716, 2.438, 2.896);
                case CarrierContainerPreset.Twin20:
                    return new CarrierContainerDimensions(2.0 * 6.058 + Twin20Gap, 2.438, 2.591);
                case CarrierContainerPreset.Custom:
                    return new CarrierContainerDimensions(
                        CustomContainerLength, CustomContainerWidth, CustomContainerHeight);
                default:
                    throw new InvalidOperationException("未知集装箱预设。 ");
            }
        }

        public List<string> Validate()
        {
            var errors = new List<string>();
            if (SchemaVersion != CurrentSchemaVersion) errors.Add("不支持的 CarrierParams schemaVersion。 ");
            if (!Positive(OverallLength)) errors.Add("整机长度必须大于 0。 ");
            if (!Positive(OverallWidth)) errors.Add("整机宽度必须大于 0。 ");
            if (!Positive(Wheelbase)) errors.Add("轴距必须大于 0。 ");
            if (!Positive(TrackWidth)) errors.Add("轮距必须大于 0。 ");
            if (!Positive(WheelLength) || !Positive(WheelWidth)) errors.Add("轮胎长宽必须大于 0。 ");
            if (!Positive(MaxSteerRateDegPerSecond)) errors.Add("转向角速度必须大于 0。 ");
            if (!Positive(MinimumReferenceRadius)) errors.Add("最小参考点半径必须大于 0。 ");
            if (!Enum.IsDefined(typeof(CarrierTurningRadiusReference), TurningRadiusReference))
                errors.Add("最小转弯半径的测量基准无效。 ");
            if (SafetyMargin < 0.0 || !Finite(SafetyMargin)) errors.Add("安全裕量不能为负数。 ");
            if (!Positive(UnitScale)) errors.Add("UnitScale 必须大于 0。 ");
            if (Architecture == CarrierArchitecture.MultiAxle)
                errors.Add("V1 运动学内核暂不支持多轴跨运车。 ");
            if (!Supports(CarrierSteeringMode.Longitudinal))
                errors.Add("车型至少必须支持纵向行驶。 ");

            var wheels = ResolvedWheelLocations();
            for (int i = 0; i < wheels.Length; i++)
            {
                CarrierWheelLocation w = wheels[i];
                if (!Finite(w.X) || !Finite(w.Y)) errors.Add(w.Id + " 轮位坐标无效。 ");
                if (!(w.MaxSteerAngleDeg > 0.0 && w.MaxSteerAngleDeg <= 180.0) ||
                    !Finite(w.MaxSteerAngleDeg))
                    errors.Add(w.Id + " 最大转角必须在 (0, 180] 度。 ");
            }

            CarrierContainerDimensions c = ContainerDimensions();
            if (ContainerPreset != CarrierContainerPreset.None &&
                (!Positive(c.Length) || !Positive(c.Width) || !Positive(c.Height)))
                errors.Add("集装箱长宽高必须大于 0。 ");
            if (!Finite(ContainerOffsetX) || !Finite(ContainerOffsetY) || !Finite(ContainerSlewAngleDeg))
                errors.Add("集装箱偏置或旋转角无效。 ");
            return errors;
        }

        static CarrierWheelLocation Resolve(CarrierWheelLocation input, double defaultX, double defaultY)
        {
            input.X = Finite(input.X) ? input.X : defaultX;
            input.Y = Finite(input.Y) ? input.Y : defaultY;
            return input;
        }

        static bool Positive(double value) { return value > 0.0 && Finite(value); }
        static bool Finite(double value) { return !double.IsNaN(value) && !double.IsInfinity(value); }
    }
}
