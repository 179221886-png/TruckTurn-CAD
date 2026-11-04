using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    static partial class VerifyProgram
    {
        /// <summary>Blocking regressions for B01 / B02 / B03. No CAD process is required.</summary>
        static int RunP1FixRegression()
        {
            // The tests use current released source, not a second implementation of its solver.
            // Geometric expectations below are independent physical constraints.
            static VehicleParams Scaled(double scale, bool articulated = true)
            {
                var p = VehicleParams.Defaults();
                p.Articulated = articulated;
                p.UnitScale = scale;
                p.TractorWheelbase *= scale;
                p.TractorFrontOverhang *= scale;
                p.TractorRearToKingpin *= scale;
                p.TractorWidth *= scale;
                p.TrailerKingpinToRearAxle *= scale;
                p.TrailerRearOverhang *= scale;
                p.TrailerWidth *= scale;
                p.WheelTrack *= scale;
                p.KingpinToCabRear *= scale;
                p.ChassisWidth *= scale;
                p.ChassisRearOverhang *= scale;
                p.CabClearance *= scale;
                p.TrailerKingpinToFront *= scale;
                p.RigidRearOverhang = 1.5 * scale;
                return p;
            }
            static double Distance(Point2d a, Point2d b) => (a - b).Length;
            static double Angle(Vector2d a, Vector2d b) => Math.Abs(Math.Atan2(a.X*b.Y-a.Y*b.X, a.X*b.X+a.Y*b.Y));
            static Point2d Normalized(Point2d point, double scale) => new(point.X / scale, point.Y / scale);
            static Vector2d Direction(double radians) => new(Math.Cos(radians), Math.Sin(radians));
            static double Area(List<Point2d> polygon)
            {
                if(polygon.Count<3)return 0;
                double twiceArea=0; var origin=polygon[0];
                for(int i=0;i<polygon.Count;i++)
                {
                    var a=polygon[i]-origin; var b=polygon[(i+1)%polygon.Count]-origin;
                    twiceArea+=a.X*b.Y-a.Y*b.X;
                }
                return Math.Abs(twiceArea*.5);
            }
            static double OutsideDistance(Point2d point,List<Point2d> polygon)
            {
                if(GeometryUtil.PointInClosedPolygon(point,polygon))return 0;
                return GeometryUtil.DistancePointToPolyline(point,polygon).dist;
            }
            static double SampledBoundaryDistance(List<Point2d> from,List<Point2d> to)
            {
                double maximum=0;
                for(int i=0;i<from.Count;i++)
                for(int sample=0;sample<5;sample++)
                {
                    var a=from[i]; var b=from[(i+1)%from.Count];
                    var q=a+(sample/5.0)*(b-a);
                    maximum=Math.Max(maximum,GeometryUtil.DistancePointToPolyline(q,to).dist);
                }
                return maximum;
            }
            static Frame MakeFrame(VehicleParams p, double tractorHeading, double trailerHeading)
            {
                var u1 = Direction(tractorHeading);
                var u2 = Direction(trailerHeading);
                var o1 = new Point2d(13*p.UnitScale, -7*p.UnitScale);
                var k = o1 + p.TractorRearToKingpin*u1;
                return new Frame { O1=o1, U1=u1, K=k, U2=u2, O2=k-p.TrailerKingpinToRearAxle*u2 };
            }
            
            int assertions=0, failures=0;
            void Check(bool condition, string detail)
            {
                assertions++;
                if (condition) return;
                failures++;
                Console.WriteLine("FAIL " + detail);
            }
            void ComparePoses(List<Frame> baseline, List<Frame> scaled, double scale, string label)
            {
                Check(Math.Abs(baseline.Count-scaled.Count)<=1,
                    $"{label}: equivalent physical path must not inflate frames ({baseline.Count} vs {scaled.Count})");
                if (baseline.Count==0 || scaled.Count==0) return;
                // Compare the terminal pose even when an old implementation has wildly different samples.
                var a=baseline[^1]; var b=scaled[^1];
                double rearError=Distance(a.O1, Normalized(b.O1,scale));
                double trailerError=Distance(a.O2,Normalized(b.O2,scale));
                Check(rearError<=1e-6, $"{label}: terminal tractor position error={rearError:G8} m");
                Check(trailerError<=2e-5, $"{label}: terminal trailer position error={trailerError:G8} m");
                Check(Angle(a.U1,b.U1)<=1e-7, $"{label}: terminal tractor heading differs");
                Check(Angle(a.U2,b.U2)<=2e-6, $"{label}: terminal trailer heading differs");
                if (baseline.Count != scaled.Count) return;
                double maxRear=0,maxTrailer=0,maxHeading=0;
                for(int i=0;i<baseline.Count;i++)
                {
                    a=baseline[i]; b=scaled[i];
                    maxRear=Math.Max(maxRear,Distance(a.O1,Normalized(b.O1,scale)));
                    maxTrailer=Math.Max(maxTrailer,Distance(a.O2,Normalized(b.O2,scale)));
                    maxHeading=Math.Max(maxHeading,Angle(a.U2,b.U2));
                }
                Check(maxRear<=1e-6 && maxTrailer<=2e-5 && maxHeading<=2e-6,
                    $"{label}: full-pose errors rear={maxRear:G8} trailer={maxTrailer:G8} heading={maxHeading:G8}");
            }
            
            bool redOnly=false;
            // A tie budget larger than tol must seed the first candidate and keep a real bend.
            var tiePoints = new List<Point2d>
            {
                new Point2d(0, 0), new Point2d(0.5, 0.2), new Point2d(1, 0)
            };
            var tieResult = GeometryUtil.SimplifyPolyline(tiePoints, 0.01, distanceTieTolerance: 1.0);
            Check(tieResult.Count == 3,
                "RDP: tie budget > tol must not discard the only point deviating by 0.2 m");
            Console.WriteLine("B01: trailer physical total length / rear-axle tail invariant");
            foreach(double scale in new[]{1.0,100.0,1000.0})
            foreach(double front in new[]{0.0,0.39,1.5})
            foreach(double tractorHeading in new[]{-2.6,0.0,1.1})
            foreach(double articulation in new[]{-0.42,0.0,0.42})
            {
                var p=Scaled(scale);
                p.TrailerKingpinToFront=front*scale;
                p.TrailerRearOverhang=1.56*scale;
                p.TrailerKingpinToRearAxle=(13-front-1.56)*scale;
                var f=MakeFrame(p,tractorHeading,tractorHeading+articulation);
                var rectangle=TruckKinematics.TrailerRect(f,p);
                var actualFront=new Point2d((rectangle[0].X+rectangle[1].X)*.5,(rectangle[0].Y+rectangle[1].Y)*.5);
                var actualTail=new Point2d((rectangle[2].X+rectangle[3].X)*.5,(rectangle[2].Y+rectangle[3].Y)*.5);
                var physicalTail=f.O2-p.TrailerRearOverhang*f.U2;
                string label=$"B01 scale={scale} front={front} heading={tractorHeading} articulation={articulation}";
                double measured=Distance(actualFront,actualTail)/scale;
                Check(Math.Abs(measured-13)<=1e-9,$"{label}: expected 13 m, measured={measured:G8} m");
                Check(Math.Abs(p.TrailerLengthMeters()-measured)<=1e-9,$"{label}: displayed length and footprint differ");
                Check(Distance(actualTail,physicalTail)/scale<=1e-9,$"{label}: physical tail error={Distance(actualTail,physicalTail)/scale:G8} m");
            }
            
            Console.WriteLine("B03: same 90deg + 10m path and pure 10m preview in m / cm / mm");
            foreach(bool articulated in new[]{false,true})
            foreach(int direction in new[]{1,-1})
            foreach(int side in new[]{1,-1})
            {
                var p=Scaled(1,articulated);
                var baseline=TruckKinematics.SimulateFromStateFront(p,Point2d.Origin,Direction(.4),.4,side,90,10,direction,TruckKinematics.FineStepDeg);
                foreach(double scale in new[]{100.0,1000.0})
                {
                    var q=Scaled(scale,articulated);
                    var watch=Stopwatch.StartNew();
                    var frames=TruckKinematics.SimulateFromStateFront(q,Point2d.Origin,Direction(.4),.4,side,90,10*scale,direction,TruckKinematics.FineStepDeg);
                    watch.Stop();
                    string label=$"B03 art={articulated} dir={direction} side={side} scale={scale}";
                    Console.WriteLine($"{label} frames_m={baseline.Count} frames_scaled={frames.Count} ms={watch.Elapsed.TotalMilliseconds:F2}");
                    ComparePoses(baseline,frames,scale,label);
                }
            }
            foreach(bool articulated in new[]{false,true})
            foreach(int direction in new[]{1,-1})
            {
                var p=Scaled(1,articulated);
                var baseline=TruckKinematics.SimulateFromStateFront(p,Point2d.Origin,Vector2d.XAxis,0,1,0,10,direction,4,toothBudget:TruckKinematics.PreviewToothBudgetM);
                foreach(double scale in new[]{100.0,1000.0})
                {
                    var q=Scaled(scale,articulated);
                    var frames=TruckKinematics.SimulateFromStateFront(q,Point2d.Origin,Vector2d.XAxis,0,1,0,10*scale,direction,4,toothBudget:TruckKinematics.PreviewToothBudgetM);
                    var label=$"B03 preview art={articulated} dir={direction} scale={scale}";
                    Console.WriteLine($"{label} frames_m={baseline.Count} frames_scaled={frames.Count}");
                    ComparePoses(baseline,frames,scale,label);
                }
            }
            
            Console.WriteLine("B03: actual reverse-control entry, non-collinear start, 90deg + 10m");
            foreach(bool articulated in new[]{false,true})
            foreach(int side in new[]{1,-1})
            foreach(bool shortTrailer in articulated?new[]{false,true}:new[]{false})
            {
                var p=Scaled(1,articulated);
                if(shortTrailer) p.TrailerKingpinToRearAxle=1.2;
                var start=MakeFrame(p,-.4,articulated?-.1:-.4);
                var baseline=TruckKinematics.SimulateReverseFromState(p,start,side,90,10,30,.25);
                foreach(double scale in new[]{100.0,1000.0})
                {
                    var q=Scaled(scale,articulated);
                    if(shortTrailer) q.TrailerKingpinToRearAxle=1.2*scale;
                    var scaledStart=MakeFrame(q,-.4,articulated?-.1:-.4);
                    var frames=TruckKinematics.SimulateReverseFromState(q,scaledStart,side,90,10*scale,30*scale,.25);
                    var label=$"B03 reverse-control art={articulated} side={side} shortTrailer={shortTrailer} scale={scale}";
                    Console.WriteLine($"{label} frames_m={baseline.Count} frames_scaled={frames.Count}");
                    ComparePoses(baseline,frames,scale,label);
                }
            }
            
            Console.WriteLine("B03: compatibility entry points remain unit-equivalent");
            foreach(bool articulated in new[]{false,true})
            foreach(int entry in new[]{0,1,2})
            {
                List<Frame> Run(VehicleParams p)
                {
                    var start=new Point2d(2*p.UnitScale,-4*p.UnitScale);
                    return entry switch
                    {
                        0=>TruckKinematics.Simulate(p,start,Direction(.7),1,90,10*p.UnitScale,.25),
                        1=>TruckKinematics.SimulateFromState(p,start,Direction(.7),.9,1,90,10*p.UnitScale,1,.25),
                        _=>TruckKinematics.SimulateFromFrontAxle(p,start,Direction(.7),1,90,10*p.UnitScale,.25)
                    };
                }
                var baseline=Run(Scaled(1,articulated));
                foreach(double scale in new[]{100.0,1000.0})
                {
                    var frames=Run(Scaled(scale,articulated));
                    var label=$"B03 compatibility art={articulated} entry={entry} scale={scale}";
                    Console.WriteLine($"{label} frames_m={baseline.Count} frames_scaled={frames.Count}");
                    ComparePoses(baseline,frames,scale,label);
                }
            }
            
            if(!redOnly)
            {
                Console.WriteLine("B03: production final/confirmed-preview envelope smoothing scale equivalence");
                foreach(bool articulated in new[]{false,true})
                foreach(int side in new[]{1,-1})
                foreach(bool preview in new[]{false,true})
                {
                    List<Point2d> baseline=null;
                    double baselineArea=0;
                    foreach(double scale in new[]{1.0,100.0,1000.0})
                    {
                        var p=Scaled(scale,articulated);
                        p.TrailerKingpinToFront=.39*scale;
                        var allFrames=TruckKinematics.SimulateFromStateFront(p,Point2d.Origin,Vector2d.XAxis,0,side,90,10*scale,1,.25);
                        var frames=preview?TruckKinematics.PreviewFrames(allFrames,320):allFrames;
                        var watch=Stopwatch.StartNew();
                        var raw=TruckKinematics.EnvelopeTracks(frames,p);
                        string label=$"B03 envelope art={articulated} side={side} preview={preview} scale={scale}";
                        Check(raw.Count>=3 && !TruckKinematics.EnvelopeFallbackUsed,label+": raw envelope failed / convex-hull fallback");
                        if(raw.Count<3)continue;
                        var simplified=GeometryUtil.SimplifyEnvelopeForSpline(raw,.002,smoothIters:60,unitScale:scale);
                        var normalized=simplified.loop.Select(q=>Normalized(q,scale)).ToList();
                        var area=Area(normalized);
                        var normalizedRaw=raw.Select(q=>Normalized(q,scale)).ToList();
                        double maxRawOutside=normalizedRaw.Max(q=>OutsideDistance(q,normalized));
                        double maxFootprintOutside=0;
                        foreach(var frame in frames)
                        {
                            foreach(var q in TruckKinematics.TractorRect(frame,p))maxFootprintOutside=Math.Max(maxFootprintOutside,OutsideDistance(Normalized(q,scale),normalized));
                            if(p.Articulated)
                            {
                                foreach(var q in TruckKinematics.TrailerRect(frame,p))maxFootprintOutside=Math.Max(maxFootprintOutside,OutsideDistance(Normalized(q,scale),normalized));
                                var chassis=TruckKinematics.ChassisRect(frame,p);
                                if(chassis!=null)foreach(var q in chassis)maxFootprintOutside=Math.Max(maxFootprintOutside,OutsideDistance(Normalized(q,scale),normalized));
                            }
                        }
                        Check(maxRawOutside<=1e-5,label+$": raw envelope outside smoothed contour by {maxRawOutside:G8} m");
                        Check(maxFootprintOutside<=1e-5,label+$": sampled physical footprint outside contour by {maxFootprintOutside:G8} m");
                        Check(simplified.outset/scale<=.1,label+$": excessive physical outset {simplified.outset/scale:G8} m");
                        double boundaryDelta=0,relativeAreaDelta=0;
                        if(baseline==null){baseline=normalized;baselineArea=area;}
                        else
                        {
                            boundaryDelta=Math.Max(SampledBoundaryDistance(baseline,normalized),SampledBoundaryDistance(normalized,baseline));
                            relativeAreaDelta=Math.Abs(area-baselineArea)/baselineArea;
                            Check(boundaryDelta<=.003,label+$": normalized boundary deviation {boundaryDelta:G8} m exceeds 3 mm");
                            Check(relativeAreaDelta<=1e-4,label+$": normalized area relative difference {relativeAreaDelta:G8}");
                            Check(Math.Abs(normalized.Count-baseline.Count)<=Math.Max(2,baseline.Count/10),label+$": unexpected vertex inflation {baseline.Count} to {normalized.Count}");
                        }
                        Console.WriteLine($"{label} frames={frames.Count} raw={raw.Count} simplified={normalized.Count} area_m2={area:F9} outset_m={simplified.outset/scale:F9} boundary_delta_m={boundaryDelta:G8} relative_area_delta={relativeAreaDelta:G8} raw_outside_m={maxRawOutside:G8} body_outside_m={maxFootprintOutside:G8} ms={watch.Elapsed.TotalMilliseconds:F2}");
                    }
                }
            }
            
            if(!redOnly)
            {
                Console.WriteLine("B02: shortcut must start from the complete articulated pose");
                foreach(double scale in new[]{1.0,100.0,1000.0})
                foreach(double previousTurn in new[]{-45.0,45.0})
                foreach(int direction in new[]{1,-1})
                foreach(int side in new[]{1,-1})
                {
                    var p=Scaled(scale);
                    var before=TruckKinematics.SimulateFromFrontAxle(p,new Point2d(2*scale,-3*scale),Direction(-.6),previousTurn>0?1:-1,Math.Abs(previousTurn),0,.25);
                    var start=before[^1];
                    var copy=new Frame{O1=start.O1,U1=start.U1,K=start.K,U2=start.U2,O2=start.O2};
                    var frames=TruckKinematics.SimulateTurn90AndStraightenFromState(p,start,side,.25,direction);
                    string label=$"B02 scale={scale} previous={previousTurn} dir={direction} side={side}";
                    Check(frames!=null && frames.Count>1,label+": no usable frames");
                    if(frames==null || frames.Count<2) continue;
                    var first=frames[0]; var next=frames[1];
                    Check(Distance(start.O1,first.O1)/scale<=1e-9,label+": initial tractor jumped");
                    Check(Distance(start.O2,first.O2)/scale<=1e-9,label+": initial trailer rear axle jumped");
                    Check(Distance(start.K,first.K)/scale<=1e-9,label+": initial kingpin jumped");
                    Check(Angle(start.U1,first.U1)<=1e-9 && Angle(start.U2,first.U2)<=1e-9,label+": initial heading reset");
                    double movement=Distance(start.O2,next.O2)/scale;
                    Check(movement<.6,label+$": second frame has unphysical discontinuity {movement:G8} m");
                    Check(Distance(copy.O2,start.O2)==0 && Angle(copy.U2,start.U2)==0,label+": input frame was mutated");
                    foreach(var f in frames)
                    {
                        Check(Distance(f.K,f.O1+p.TractorRearToKingpin*f.U1)/scale<=1e-8,label+": invalid tractor/kingpin linkage");
                        Check(Distance(f.O2,f.K-p.TrailerKingpinToRearAxle*f.U2)/scale<=1e-8,label+": invalid trailer/kingpin linkage");
                        Check(double.IsFinite(f.O2.X)&&double.IsFinite(f.O2.Y),label+": nonfinite state");
                    }
                    Console.WriteLine($"{label} frames={frames.Count} first_to_next_trailer_m={movement:F6}");
                }
            }
            Console.WriteLine($"SUMMARY assertions={assertions} failures={failures}");
            return failures;
        }
    }
}
