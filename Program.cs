using System;
using System.Collections.Generic;
using System.IO;

namespace PathFinderModel
{
    class Program
    {
        static string history = @"D:\Soft\PathFinderSolution\Computation_modules\vessels_position_history.txt";
        static string info = @"D:\Soft\PathFinderSolution\Computation_modules\vessels_info.txt";

        //static MovingSubject msShip;
        //static List<MovingSubject> msSubjects;
        //static List<double> dAngles;
        //static double dMinimumDistance;
        //static List<ManeuverType> mtManeuverTypes;

        static void TestPowell()
        {
            var result = MotionSimulation.GetOptimumSolution(MotionSimulation.TestMethod, new List<double> { 12.0, 7.0 }, null);
            //Console.WriteLine(result[0] + " " + result[1]);
        }

        static void Main(string[] args)
        {
            System.Diagnostics.Debug.AutoFlush = true;
            //TestPowell();
            //return;
            Vector vec1 = new Vector { x = 1, y = 1 };
            Vector vec2 = new Vector { x = 1, y = 2 };

            double angle = Vector.GetAngle(vec1, vec2) / 3.14 * 180;

            while (true)
            {
                try
                {
                    MovingSubject ship = null;
                    MovingSubject s1 = null;
                    MovingSubject s2 = null;
                    MovingSubject s3 = null;
                    MovingSubject s4 = null;
                    MovingSubject s5 = null;
                    MovingSubject s6 = null;
                    MovingSubject s7 = null;
                    using (StreamReader reader = new StreamReader(info))
                    {
                        string vessels_info = reader.ReadToEnd();
                        string[] info = vessels_info.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

                        if (info.Length == 8)
                        {
                            ship = new MovingSubject
                            {
                                coords = new Vector { x = Convert.ToDouble(info[0]), y = Convert.ToDouble(info[1]) },
                                v = new Vector { x = Convert.ToDouble(info[2]), y = Convert.ToDouble(info[3]) }
                            };

                            s1 = new MovingSubject
                            {
                                coords = new Vector { x = Convert.ToDouble(info[4]), y = Convert.ToDouble(info[5]) },
                                v = new Vector { x = Convert.ToDouble(info[6]), y = Convert.ToDouble(info[7]) }
                            };

                            s2 = new MovingSubject
                            {
                                coords = new Vector { x = s1.coords.x + 100, y = s1.coords.y + 100 },
                                v = new Vector { x = s1.v.x, y = s1.v.y }
                            };

                            s3 = new MovingSubject
                            {
                                coords = new Vector { x = s1.coords.x + 200, y = s1.coords.y + 200 },
                                v = new Vector { x = s1.v.x, y = s1.v.y }
                            };

                            s4 = new MovingSubject
                            {
                                coords = new Vector { x = s1.coords.x, y = s1.coords.y + 50 },
                                v = new Vector { x = s1.v.x * 1.2, y = s1.v.y * 1.5 }
                            };

                            s5 = new MovingSubject
                            {
                                coords = new Vector { x = s1.coords.x - 200, y = s1.coords.y + 200 },
                                v = new Vector { x = -s1.v.x, y = s1.v.y }
                            };

                            List<MovingSubject> subjects = new List<MovingSubject> { s1, s2, s3, s4 };//, s5 };
                            List<double> angles = new List<double> { 70.0 / 180.0 * Math.PI, 60.0 / 180.0 * Math.PI, 60.0 / 180.0 * Math.PI, 150.0 / 180.0 * Math.PI, 70.0 / 180.0 * Math.PI };//, 70.0 / 180.0 * Math.PI };
                            List<ManeuverType> maneuverTypes = new List<ManeuverType> { ManeuverType.On_the_right, ManeuverType.On_the_left, ManeuverType.On_the_left, ManeuverType.On_the_right, ManeuverType.On_the_right };//, ManeuverType.On_the_left };

                            //msShip = ship;
                            //msSubjects = subjects;
                            //dAngles = angles;
                            //mtManeuverTypes = maneuverTypes;

                            double minimumDistance = 30;
                            //dMinimumDistance = minimumDistance;
                            Vector target = new Vector { x = 300, y = 1000 };
                            double optimalV = 150.0;
                            double maxTime = 1000.0;


                            IsManeuverEnded isManeuverEndedDelegate = IsManeuverEndedMethod;

                            MotionSimulation.ship = ship;
                            MotionSimulation.subjects = subjects;
                            MotionSimulation.minimumDistance = minimumDistance;
                            MotionSimulation.target = target;
                            MotionSimulation.isManeuverEnded = isManeuverEndedDelegate;
                            MotionSimulation.optimalV = optimalV;
                            MotionSimulation.maxTime = maxTime;

                            List<double> startAngles = new List<double>();
                            for (int i = 0; i < angles.Count; ++i)
                                startAngles.Add(angles[i]);


                            List<double> result = MotionSimulation.Calculate(MotionSimulation.RoutePathCost, angles, maneuverTypes, 0.01, 500.0);

                            //List<double> result = MotionSimulation.GetOptimumSolution(MotionSimulation.RoutePathCost, angles, maneuverTypes);

                            for (int i = 0; i < result.Count; ++i)
                            {
                                Console.WriteLine("Final angle: " + result[i]);
                            }
                            angles = result;


                            MotionSimulation.DoManeuver(ship, subjects, angles, maneuverTypes, minimumDistance, target, isManeuverEndedDelegate,
                                optimalV, maxTime);
                            
                            /*

                            Tuple<SubjectsPriority, SubjectPosition[]> situation = MotionSimulation.GetSituation(s1, s2);
                            //Console.WriteLine(situation.Item1 + " " + situation.Item2[0] + " " + situation.Item2[1]);

                            ManeuverType maneuverType = ManeuverType.On_the_left;
                            double minimumDistance = 30;
                            double startAngle = 70.0 / 180.0 * Math.PI;
                            double endAngle = 60.0 / 180.0 * Math.PI;

                            Tuple<Vector, double> maneuver = MotionSimulation.GetLineManeuver(s2, s1, startAngle, minimumDistance, maneuverType);

                            s2.v = maneuver.Item1;

                            List<MovingSubject> subjects = new List<MovingSubject> { s1, s2 };

                            ClearPositionFile();
                            int n = 1000;
                            for (int i = 0; i < n; ++i)
                            {
                                MotionSimulation.DoSimulationStep(subjects, maneuver.Item2 / (n + 0.0));
                                SavePositions(subjects);
                            }

                            List<Tuple<Vector, double>> shipSteps = MotionSimulation.GetRouteManeuver(s2, s1, endAngle, minimumDistance, maneuverType);
                            for(int i = 0; i < shipSteps.Count; ++i)
                            {
                                s2.coords.x = shipSteps[i].Item1.x;
                                s2.coords.y = shipSteps[i].Item1.y;
                                s1.coords.x += s1.v.x * shipSteps[i].Item2;
                                s1.coords.y += s1.v.y * shipSteps[i].Item2;

                                SavePositions(subjects);
                            }
                            */
                        }
                    }
                }
                catch(Exception ex)
                {
                }

                break;
            }
        }

        static void SavePositions(List<MovingSubject> subjects)
        {
            using(StreamWriter writer = new StreamWriter(history, true))
            {
                foreach(MovingSubject subject in subjects)
                {
                    writer.Write(subject.coords.x + " " + subject.coords.y + " ");
                }
                writer.WriteLine();
            }
        }

        static void ClearPositionFile()
        {
            using (StreamWriter writer = new StreamWriter(history))
            {
                writer.Write("");
            }
        }

        static bool IsManeuverEndedMethod(MovingSubject ship, List<MovingSubject> msSubjects, List<double> dAngles, List<ManeuverType> mtManeuverTypes, int subjectIndex, double dMinimumDistance)
        {
            MovingSubject subject = msSubjects[subjectIndex];

            Vector position = new Vector { x = ship.coords.x - subject.coords.x, y = ship.coords.y - subject.coords.y };
            double cosAlpha = position.x / position.module;
            double sinAlpha = position.y / position.module;
            double currentAngle = Vector.GetAngle(cosAlpha, sinAlpha);

            //
            using (StreamWriter writer = new StreamWriter("IsManeuverEndedLogs.txt", true))
            {
                writer.WriteLine(dAngles[subjectIndex + 1] + " " + currentAngle);
            }
            return Math.Abs(dAngles[subjectIndex + 1] % Math.PI - currentAngle) < 0.15;
            if (subjectIndex < msSubjects.Count - 1)
            {
                Tuple<Vector, double> res = MotionSimulation.GetLineManeuver(ship, msSubjects[subjectIndex + 1], dAngles[subjectIndex + 1], dMinimumDistance, mtManeuverTypes[subjectIndex + 1]);
                if (MotionSimulation.IsNotCollision(new MovingSubject
                    {
                        coords = new Vector { x = ship.coords.x, y = ship.coords.y },
                        v = new Vector { x = res.Item1.x, y = res.Item1.y }
                    },
                    subject, 10.0, dMinimumDistance))
                {
                    return true;
                }
            }

            return Math.Abs(dAngles[subjectIndex + 1] - currentAngle) < 0.05;
        }
    }
}
