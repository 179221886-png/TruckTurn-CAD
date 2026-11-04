using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Security.Cryptography;
using System.Text;
using TruckTurn;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
internal static class PlaybackVerify
{
    static int assertions;
    static void Check(bool ok, string name) { assertions++; if (!ok) throw new Exception(name); }
    static void Near(double a, double b, string name) { Check(Math.Abs(a - b) < 1e-9, name + " " + a + " vs " + b); }
    static void Reject(Action action, string name)
    {
        try { action(); } catch (Exception ex) when (ex is InvalidDataException || ex is ArgumentException) { assertions++; return; }
        throw new Exception("must reject: " + name);
    }
    static Frame F(double x, double y, double h, double trailer)
    {
        return new Frame { O1 = new Point2d(x,y), U1 = new Vector2d(Math.Cos(h),Math.Sin(h)), U2 = new Vector2d(Math.Cos(trailer),Math.Sin(trailer)) };
    }
    public static void Run()
    {
        assertions = 0;
        var p = VehicleParams.Defaults();
        var frames = new List<Frame> { F(0,0,0,0), F(1.4,0,0,0), F(2.8,0,0,0) };
        var r = PlaybackRouteFactory.FromTruck(p, frames);
        r.SourceSummary = "公开来源 / 前后悬待核验 / https://example.invalid/fire";
        Near(r.DurationSeconds, 2.0, "distance time");
        p.TractorWidth = 100;
        Near(r.TruckParameters.TractorWidth, 2.5, "truck snapshot");
        Near(PlaybackTimeline.Sample(r, 0.5).X, .7, "sample distance");
        Near(PlaybackTimeline.Sample(r,-10).X, 0, "lower clamp");
        Near(PlaybackTimeline.Sample(r,100).X,2.8, "upper clamp");
        var d = PlaybackRouteCodec.Decode(PlaybackRouteCodec.Encode(r));
        Check(d.SourceSummary == r.SourceSummary, "source metadata roundtrip");
        Check(d.Poses.Count == frames.Count, "no frame drop");
        Check(!object.ReferenceEquals(d.TruckParameters,r.TruckParameters),"decode param isolation");
        var mid = PlaybackTimeline.Sample(d,.5); mid.X=123;
        Near(d.Poses[0].X,0,"sample isolated");
        var mm = VehicleParams.Defaults(); mm.UnitScale=1000; mm.TractorWheelbase*=1000; mm.TractorWidth*=1000; mm.TractorFrontOverhang*=1000;
        mm.WheelTrack*=1000; mm.TrailerKingpinToRearAxle*=1000; mm.TrailerWidth*=1000;
        var rm = PlaybackRouteFactory.FromTruck(mm,new List<Frame>{F(0,0,0,0),F(1400,0,0,0),F(2800,0,0,0)});
        Near(rm.DurationSeconds,r.DurationSeconds,"mm/m duration same");

        var seam = PlaybackRouteFactory.FromTruck(VehicleParams.Defaults(),new List<Frame>{F(0,0,179*Math.PI/180,-179*Math.PI/180),F(1,0,-179*Math.PI/180,179*Math.PI/180)});
        var s=PlaybackTimeline.Sample(seam,seam.DurationSeconds*.5);
        Near(Math.Abs(s.HeadingRad),Math.PI,"tractor short seam"); Near(Math.Abs(s.TrailerHeadingRad),Math.PI,"trailer short seam");
        var reverse=PlaybackRouteFactory.FromTruck(VehicleParams.Defaults(),new List<Frame>{F(0,0,0,0),F(-1,0,-.1,0),F(-2,0,-.2,0)});
        Check(reverse.Poses[0].TravelDirection == -1,"reverse direction"); Check(reverse.Poses[0].FrontSteerRad > 0,"reverse steering correct");
        var straight=PlaybackRouteFactory.FromTruck(VehicleParams.Defaults(),new List<Frame>{F(0,0,0,0),F(1,.1,.2,0),F(2,.3,.2,0),F(3,.5,.2,0)});
        Near(straight.Poses[straight.Poses.Count-1].FrontSteerRad,0,"final straight wheels reset");
        double theta=.1, radius=10;
        var circular=PlaybackRouteFactory.FromTruck(VehicleParams.Defaults(),new List<Frame>{F(0,0,0,0),F(radius*Math.Sin(theta),radius*(1-Math.Cos(theta)),theta,0)});
        Near(circular.Poses[0].FrontSteerRad,Math.Atan(.6),"exact chord curvature");
        var fp=new VehicleParams{Articulated=false,TractorWheelbase=3.8,TractorWidth=2.5,TractorFrontOverhang=1.2,RigidRearOverhang=2,RigidAxles=2,WheelTrack=2,SteerAngleDeg=35,UnitScale=1};
        var fire=PlaybackRouteFactory.FromTruck(fp,frames,PlaybackVehicleKind.Fire,"参考消防车型");fire.SourceSummary="参数来源与估算字段说明";
        Check(PlaybackRouteCodec.Decode(PlaybackRouteCodec.Encode(fire)).Kind==PlaybackVehicleKind.Fire,"fire route / unused trailer axles");

        var cp = CarrierParams.GenericFourWheelIndependent();
        cp.UnitScale=1000;
        var cfs = new List<CarrierFrame>();
        for (int i=0;i<3;i++) cfs.Add(CarrierKinematics.Solve(cp,new CarrierPose(-i, -i*.5, .2),CarrierMotionCommand.Crab(-1,28)));
        var cr=PlaybackRouteFactory.FromCarrier(cp,cfs);
        Near(cr.Poses[1].X,-1000,"carrier CAD coordinate");
        Check(cr.Poses[0].TravelDirection == -1,"reverse crab");
        cp.OverallWidth=123; Near(cr.CarrierParameters.OverallWidth,6,"carrier snapshot");
        var cd=PlaybackRouteCodec.Decode(PlaybackRouteCodec.Encode(cr));
        for(int i=0;i<cr.Poses.Count;i++) for(int w=0;w<4;w++) Near(cd.Poses[i].WheelAnglesRad[w],cr.Poses[i].WheelAnglesRad[w],"wheel roundtrip");
        Check(double.IsNaN(cd.CarrierParameters.FrontLeft.X),"wheel NaN sentinel preserved");

        var parkStart=CarrierKinematics.Solve(cr.CarrierParameters,new CarrierPose(0,0,0),CarrierMotionCommand.Longitudinal(0));
        var parkEnd=CarrierKinematics.Solve(cr.CarrierParameters,new CarrierPose(0,0,0),CarrierMotionCommand.Crab(1,30));
        parkEnd.Twist=new CarrierTwist(0,0,0);
        var park=PlaybackRouteFactory.FromCarrier(cr.CarrierParameters,new List<CarrierFrame>{parkStart,parkEnd});
        Near(park.DurationSeconds,2,"parking wheel time");
        Near(PlaybackTimeline.Sample(park,1).WheelAnglesRad[0],15*Math.PI/180,"parking steer interpolates");
        var one=PlaybackRouteFactory.FromTruck(VehicleParams.Defaults(),new List<Frame>{F(3,4,0,0)});
        Near(one.DurationSeconds,0,"one frame duration"); Near(PlaybackTimeline.Sample(one,100).Y,4,"one frame sample");
        PlaybackRouteCodec.Decode(PlaybackRouteCodec.Encode(one));

        byte[] bytes=PlaybackRouteCodec.Encode(r);
        Reject(()=>PlaybackRouteCodec.Decode(null),"null"); Reject(()=>PlaybackRouteCodec.Decode(new byte[50]),"garbage");
        for(int n=1;n<40;n++){var truncated=new byte[bytes.Length-n];Array.Copy(bytes,truncated,truncated.Length); Reject(()=>PlaybackRouteCodec.Decode(truncated),"truncated");}
        var random=new Random(77);
        for(int i=0;i<300;i++){byte[] bad=(byte[])bytes.Clone();int at=random.Next(bad.Length);bad[at]^=(byte)(1<<random.Next(8));Reject(()=>PlaybackRouteCodec.Decode(bad),"bit corruption");}
        r.Version=2;Reject(()=>PlaybackRouteCodec.Encode(r),"version");r.Version=1;
        r.Poses[1].TimeSeconds=0;Reject(()=>PlaybackRouteCodec.Encode(r),"non monotonic");r.Poses[1].TimeSeconds=1;
        r.Poses[1].X=double.NaN;Reject(()=>PlaybackRouteCodec.Encode(r),"NaN");r.Poses[1].X=1.4;
        r.Poses[0].WheelAnglesRad=new double[4];Reject(()=>PlaybackRouteCodec.Encode(r),"wheel count");r.Poses[0].WheelAnglesRad=new double[0];
        r.DisplayName="\0bad";Reject(()=>PlaybackRouteCodec.Encode(r),"control char");r.DisplayName="bad\ud800";Reject(()=>PlaybackRouteCodec.Encode(r),"surrogate");r.DisplayName="good";
        r.SourceSummary=new string('x',4097);Reject(()=>PlaybackRouteCodec.Encode(r),"source limit");r.SourceSummary="";
        cr.CarrierParameters.FrontLeft.X=double.PositiveInfinity;Reject(()=>PlaybackRouteCodec.Encode(cr),"wheel Infinity");cr.CarrierParameters.FrontLeft.X=double.NaN;
        cr.CarrierParameters.SafetyMargin=double.NaN;Reject(()=>PlaybackRouteCodec.Encode(cr),"parameter NaN");cr.CarrierParameters.SafetyMargin=.3;
        Reject(()=>PlaybackRouteFactory.FromTruck(VehicleParams.Defaults(),new List<Frame>()),"empty frames");
        r.Poses=new List<PlaybackPose>(new PlaybackPose[PlaybackRouteCodec.MaximumPoseCount+1]);Reject(()=>PlaybackRouteCodec.Encode(r),"frame bound");
        // Resealed adversarial payloads prove validation does not rely on checksum alone.
        byte[] raw=Unpack(bytes);raw[4]=2;Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"raw unknown version");
        raw=Unpack(bytes); raw[8]=99;Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"raw unknown vehicle");
        raw=Unpack(bytes); int poseOffset=PoseOffset(raw);Array.Copy(BitConverter.GetBytes(double.NaN),0,raw,poseOffset+8,8);
        Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"raw NaN coordinate");
        raw=Unpack(bytes);poseOffset=PoseOffset(raw);Array.Copy(BitConverter.GetBytes(0),0,raw,poseOffset-4,4);
        Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"raw empty count");
        raw=Unpack(bytes);poseOffset=PoseOffset(raw);Array.Copy(BitConverter.GetBytes(int.MaxValue),0,raw,poseOffset-4,4);
        Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"raw oversized count");
        raw=Unpack(bytes);poseOffset=PoseOffset(raw);Array.Copy(BitConverter.GetBytes(4),0,raw,poseOffset+56,4);
        Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"raw wrong wheel count");
        raw=Unpack(bytes);raw=ReplaceXml(raw,"<!DOCTYPE VehicleParams [<!ENTITY ex SYSTEM 'file:///C:/Windows/win.ini'>]><VehicleParams><TractorWidth>&ex;</TractorWidth></VehicleParams>");
        Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"XML DTD/XXE");
        raw=ReplaceXml(Unpack(bytes),"<VehicleParams xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xsi:type='System.String'><TractorWheelbase>2</TractorWheelbase></VehicleParams>");
        Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"XML arbitrary type");
        raw=Unpack(bytes); byte[] rawTail=new byte[raw.Length+1];Array.Copy(raw,rawTail,raw.Length);Reject(()=>PlaybackRouteCodec.Decode(Pack(rawTail)),"raw trailing bytes");
        raw=new byte[PlaybackRouteCodec.MaximumDecodedBytes+1];Reject(()=>PlaybackRouteCodec.Decode(Pack(raw)),"zip bomb limit");
        Console.WriteLine("PASS PlaybackData: "+assertions+" assertions; snapshots, exact frames, units, time, seam/reverse/crab/parking/single-frame, codec bounds+corruption.");
    }
    static byte[] Unpack(byte[] b){using(var input=new MemoryStream(b,40,b.Length-40))using(var gz=new GZipStream(input,CompressionMode.Decompress))using(var output=new MemoryStream()){gz.CopyTo(output);return output.ToArray();}}
    static int XmlOffset(byte[] b){using(var input=new MemoryStream(b))using(var r=new BinaryReader(input)){input.Position=20;int n=r.ReadInt32();input.Position+=n;n=r.ReadInt32();input.Position+=n;return (int)input.Position;}}
    static int PoseOffset(byte[] b){int offset=XmlOffset(b);return offset+4+BitConverter.ToInt32(b,offset)+4;}
    static byte[] ReplaceXml(byte[] b,string xml)
    {
        int offset=XmlOffset(b), oldLength=BitConverter.ToInt32(b,offset);byte[] x=Encoding.UTF8.GetBytes(xml);
        using(var output=new MemoryStream())using(var w=new BinaryWriter(output)){w.Write(b,0,offset);w.Write(x.Length);w.Write(x);w.Write(b,offset+4+oldLength,b.Length-offset-4-oldLength);return output.ToArray();}
    }
    static byte[] Pack(byte[] b)
    {
        byte[] zipped;using(var output=new MemoryStream()){using(var gz=new GZipStream(output,CompressionMode.Compress,true))gz.Write(b,0,b.Length);zipped=output.ToArray();}
        using(var output=new MemoryStream())using(var w=new BinaryWriter(output))using(var sha=SHA256.Create()){w.Write((uint)0x43505454);w.Write(zipped.Length);w.Write(sha.ComputeHash(zipped));w.Write(zipped);return output.ToArray();}
    }
}


}

