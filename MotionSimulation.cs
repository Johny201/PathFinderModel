using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFinderModel
{
    public enum SubjectsPriority
    {
        subject1,
        subject2,
        equals
    }

    //AS - Another Subject, S - Subject
    public enum SubjectPosition
    {
        In_front,
        AS_Behind_S,
        S_Behind_AS,
        AS_on_left_side_of_S,
        S_on_left_side_of_AS,
        AS_on_right_side_of_S,
        S_on_right_side_of_AS,
        Towards,
        Default
    }

    public enum ManeuverType
    {
        On_the_left,
        On_the_right,
        On_the_right_cut,
        Towards
    }

    delegate bool IsManeuverEnded(MovingSubject ship, List<MovingSubject> msSubjects, List<double> dAngles, List<ManeuverType> maneuverTypes, int subjectIndex, double dMinimumDistance);
    class MotionSimulation
    {
        #region Temporary
        public static MovingSubject ship;
        public static List<MovingSubject> subjects;
        public static double minimumDistance;
        public static Vector target;
        public static IsManeuverEnded isManeuverEnded;
        public static double optimalV;
        public static double maxTime;
        #endregion Temporary
        #region Info about subject and ship interaction
        public static Tuple<SubjectsPriority, SubjectPosition[]> GetSituation(MovingSubject s1, MovingSubject s2)
        {
            Vector subjectsLine = new Vector { x = s2.coords.x - s1.coords.x, y = s2.coords.y - s1.coords.y };
            double s1Angle = Vector.GetAngle(subjectsLine, s1.v);
            double s2Angle = Vector.GetAngle(new Vector { x = -subjectsLine.x, y = -subjectsLine.y }, s2.v);
            double s1AngleAbs = Math.Abs(s1Angle);
            double s2AngleAbs = Math.Abs(s2Angle);

            List<SubjectPosition> positions = new List<SubjectPosition>();
            SubjectPosition s1Pos = SubjectPosition.Default;
            SubjectPosition s2Pos = SubjectPosition.Default;
            SubjectsPriority priority = SubjectsPriority.equals;

            if(s1AngleAbs > Constants.BOT_SIDE_ANGLE)
            {
                s1Pos = SubjectPosition.AS_Behind_S;
            }
            if(s2AngleAbs > Constants.BOT_SIDE_ANGLE)
            {
                s2Pos = SubjectPosition.AS_Behind_S;
            }
            if(s1AngleAbs > Constants.TOWARDS_ANGLE && s1AngleAbs <= Constants.BOT_SIDE_ANGLE && s1Angle < 0)
            {
                s1Pos = SubjectPosition.AS_on_left_side_of_S;
            }
            if (s2AngleAbs > Constants.TOWARDS_ANGLE && s2AngleAbs <= Constants.BOT_SIDE_ANGLE && s2Angle < 0)
            {
                s2Pos = SubjectPosition.AS_on_left_side_of_S;
            }
            if(s1AngleAbs <= Constants.TOWARDS_ANGLE)
            {
                s1Pos = SubjectPosition.In_front;
            }
            if(s2AngleAbs <= Constants.TOWARDS_ANGLE)
            {
                s2Pos = SubjectPosition.In_front;
            }
            if (s1AngleAbs <= Constants.TOWARDS_ANGLE && s2AngleAbs <= Constants.TOWARDS_ANGLE)
            {
                s1Pos = SubjectPosition.Towards;
                s2Pos = SubjectPosition.Towards;
            }
            if(s1AngleAbs > Constants.TOWARDS_ANGLE && s1AngleAbs <= Constants.BOT_SIDE_ANGLE && s1Angle > 0)
            {
                s1Pos = SubjectPosition.AS_on_right_side_of_S;
            }
            if (s2AngleAbs > Constants.TOWARDS_ANGLE && s2AngleAbs <= Constants.BOT_SIDE_ANGLE && s2Angle > 0)
            {
                s2Pos = SubjectPosition.AS_on_right_side_of_S;
            }

            if(s1Pos != s2Pos)
            {
                if (s1Pos == SubjectPosition.AS_Behind_S)
                    priority = SubjectsPriority.subject1;
                if (s2Pos == SubjectPosition.AS_Behind_S)
                    priority = SubjectsPriority.subject2;
                if (s1Pos == SubjectPosition.AS_on_left_side_of_S)
                    priority = SubjectsPriority.subject1;
                if (s2Pos == SubjectPosition.AS_on_left_side_of_S)
                    priority = SubjectsPriority.subject2;
                if (s1Pos == SubjectPosition.AS_on_right_side_of_S)
                    priority = SubjectsPriority.subject2;
                if (s2Pos == SubjectPosition.AS_on_right_side_of_S)
                    priority = SubjectsPriority.subject1;
            }

            Tuple<SubjectsPriority, SubjectPosition[]> result = new Tuple<SubjectsPriority, SubjectPosition[]>(priority, new SubjectPosition[] { s1Pos, s2Pos });
            return result;
        }

        //Возвращает относительное положение и скорости. Первое судно - в начале координат, скорость относительно неподвижного второго.
        public static Tuple<Vector, Vector> GetRelativePositionAndVelocity(MovingSubject s1, MovingSubject s2)
        {
            Vector v = new Vector
            {
                x = s1.v.x - s2.v.x,
                y = s1.v.y - s2.v.y
            };

            return new Tuple<Vector, Vector>(
                new Vector { x = s2.coords.x - s1.coords.x, y = s2.coords.y - s1.coords.y },
                new Vector { x = v.x, y = v.y }
            );
        }

        //Производится поворот системы координат на угол вектор относительной скорости судов, что переводит вектор скорости в вектор, у которого значение компоненты y == 0,
        //а компонента x равна модулю вектора скорости. Положение судов тоже переносится. Благодаря этому расстояние между суднами по оси y в новой системе координат будет
        //известно и неизменно, а по оси x будет изменяться в соответствии с модулем вектора скорости. Данный метод возвращает вектор расстояния между суднами и вектор
        //скорости их сближения, компонента y которого равна нулю
        public static Tuple<Vector, Vector> GetRelativePositionAndVelocityInRotatedCoordinates(MovingSubject s1, MovingSubject s2)
        {
            Vector v = new Vector
            {
                x = s1.v.x - s2.v.x,
                y = s1.v.y - s2.v.y
            };

            if(v.module != 0.0)
            {
                double cosAlpha = v.x / v.module;
                double sinAlpha = v.y / v.module;

                Vector s1Coords = new Vector
                {
                    x = s1.coords.x,
                    y = s1.coords.y
                };
                Vector s2Coords = new Vector
                {
                    x = s2.coords.x,
                    y = s2.coords.y
                };

                v = v.RotateVector(cosAlpha, sinAlpha);
                s1Coords = s1Coords.RotateVector(cosAlpha, sinAlpha);
                s2Coords = s2Coords.RotateVector(cosAlpha, sinAlpha);
            }

            return new Tuple<Vector, Vector>(
                new Vector { x = s2.coords.x - s1.coords.x, y = s2.coords.y - s1.coords.y },
                new Vector { x = v.x, y = v.y }
            );
        }

        //Возвращает минимальную дистанцию между двумя суднами в период указанного времени.
        public static double GetMinimumDistance(MovingSubject s1, MovingSubject s2, double t)
        {
            Tuple<Vector, Vector> distanceAndVelocity = GetRelativePositionAndVelocityInRotatedCoordinates(s1, s2);

            Vector distance = distanceAndVelocity.Item1;
            Vector v = distanceAndVelocity.Item2;

            if(v.x != 0.0 && ((v.x > 0.0 && distance.x > 0.0) || (v.x < 0.0 && distance.x < 0.0)))
            {
                double convergenceTime = distance.x / v.x;
                if(convergenceTime > t)
                {
                    double xMin = distance.x - v.x * t;
                    double yMin = distance.y;
                    return Math.Sqrt(xMin * xMin + yMin * yMin);
                }
                return distance.y;
            }

            return distance.module;
        }

        public double GetMinimumDistance(MovingSubject s1, MovingSubject s2)
        {
            Tuple<Vector, Vector> distanceAndVelocity = GetRelativePositionAndVelocityInRotatedCoordinates(s1, s2);

            if (distanceAndVelocity.Item1.x != 0.0 && 
                ((distanceAndVelocity.Item1.x > 0.0 && distanceAndVelocity.Item2.x > 0.0) || (distanceAndVelocity.Item1.x < 0.0 && distanceAndVelocity.Item2.x < 0.0)))
            {
                return distanceAndVelocity.Item1.y;
            }
            return distanceAndVelocity.Item1.module;
        }

        public static bool IsNotCollision(MovingSubject ship, MovingSubject subject, double time, double allowedMinimumDistance)
        {
            return GetMinimumDistance(ship, subject, time) >= allowedMinimumDistance;
        }
        /*
        static double GetMinimumDistance(MovingSubject ship, MovingSubject subject, double time)
        {
            Vector v = new Vector { x = ship.v.x - subject.v.x, y = ship.v.y - subject.v.y };

            double cosAlpha = v.x / v.module;
            double sinAlpha = v.y / v.module;

            Vector position = new Vector { x = subject.coords.x - ship.coords.x, y = subject.coords.y - ship.coords.y };

            Vector rotatedPosition = position.RotateVector(cosAlpha, sinAlpha);
            Vector rotatedV = v.RotateVector(cosAlpha, sinAlpha);

            double minimumDistanceTime = rotatedPosition.x / rotatedV.x;
            if(minimumDistanceTime > time)
            {
                Vector rotatedMinimumDistancePosition = new Vector { x = rotatedPosition.x - rotatedV.x * time, y = rotatedPosition.y };
                return rotatedMinimumDistancePosition.module;
            }

            return Math.Abs(rotatedPosition.y);
        }
        */

        //Основано на get_k версии Python
        public static double[] GetRelativeVelocitySlope(MovingSubject ship, MovingSubject subject, double startAngle, double minimumDistance)
        {
            //Решение квадратного уравнения. Решение - 2 угла наклона относительной скорости с двух сторон относительно субъекта
            double a = Math.Pow(minimumDistance, 2) - Math.Pow((ship.coords.x - subject.coords.x), 2);
            double b = 2.0 * (ship.coords.x - subject.coords.x) * (ship.coords.y - subject.coords.y);
            double c = Math.Pow(minimumDistance, 2) - Math.Pow((ship.coords.y - subject.coords.y), 2);

            double descr = Math.Pow(b, 2) - 4.0 * a * c;
            double sqrtDescr = Math.Sqrt(descr);

            double[] slope = new double[]
            {
                (-b + sqrtDescr) / (2.0 * a),
                (-b - sqrtDescr) / (2.0 * a)
            };

            return SortFromLeftToRightSlopes(slope);

            //D**2 * (k**2 + 1) = (k(xsh - x0) - (y0 - ysh))**2
            //= k**2 (xsh - x0)**2 - 2 k (xsh - x0)(y0 - ysh) + (y0 - ysh)**2
            //(D**2 - (xsh - x0)**2) k**2 + 2 (xsh - x0)(y0 - ysh) + D**2 - (y0 - ysh)**2 = 0
        }

        private static double[] SortFromLeftToRightSlopes(double[] slope)
        {
            if (slope[0] * slope[1] < 0)
            {
                if (slope[1] <= 0)
                    slope = new double[] { slope[1], slope[0] };
            }
            else if (slope[0] < slope[1])
            {
                slope = new double[] { slope[1], slope[0] };
            }

            return slope;
        }

        public static void DoSimulationStep(List<MovingSubject> subjects, double t)
        {
            foreach (MovingSubject subject in subjects)
            {
                subject.coords.x += subject.v.x * t;
                subject.coords.y += subject.v.y * t;
            }
        }

        public static void CreateOvalTrackOfSubjects(List<MovingSubject> subjects, double t)
        {

        }
        #endregion Info about subject and ship interaction

        #region One subject maneuvers
        public static Tuple<MovingSubject, List<MovingSubject>, List<Tuple<Vector, double>>> GetRouteManeuver(MovingSubject ship, List<MovingSubject> subjects, List<double> angles,
            List<ManeuverType> maneuverTypes, int subjectIndex, double minimumDistance, IsManeuverEnded isManeuverEnded, bool isClockwise)
        {
            double endAngle = angles[subjectIndex + 1];
            ManeuverType maneuverType = maneuverTypes[subjectIndex];
            MovingSubject subject = subjects[subjectIndex];
            List<Tuple<Vector, double>> steps = new List<Tuple<Vector, double>>();

            MovingSubject shipProcessing = new MovingSubject
            {
                coords = new Vector { x = ship.coords.x, y = ship.coords.y },
                v = new Vector { x = ship.v.x, y = ship.v.y }
            };

            List<MovingSubject> subjectsProcessing = new List<MovingSubject>();
            for(int i = 0; i < subjects.Count; ++i)
            {
                subjectsProcessing.Add(new MovingSubject
                {
                    coords = new Vector { x = subjects[i].coords.x, y = subjects[i].coords.y },
                    v = new Vector { x = subjects[i].v.x, y = subjects[i].v.y }
                });
            }

            MovingSubject subjectProcessing = subjectsProcessing[subjectIndex];

            double dt = 0.01;
            Vector position = new Vector { x = shipProcessing.coords.x - subjectProcessing.coords.x, y = shipProcessing.coords.y - subjectProcessing.coords.y };
            double cosAlpha = position.x / position.module;
            double sinAlpha = position.y / position.module;
            double currentAngle = Vector.GetAngle(cosAlpha, sinAlpha);
            double additionalVModule = subjectProcessing.v.module;
            Vector v;
            int nSteps = 100000;
            shipProcessing.v.x *= 1.5;
            shipProcessing.v.y *= 1.5;
            shipProcessing.v.x = 1.5 * subjectProcessing.v.x;
            shipProcessing.v.y = 1.5 * subjectProcessing.v.y;
            while (!isManeuverEnded(shipProcessing, subjectsProcessing, angles, maneuverTypes, subjectIndex, minimumDistance) && nSteps-- > 0)
            {
                position = new Vector { x = shipProcessing.coords.x - subjectProcessing.coords.x, y = shipProcessing.coords.y - subjectProcessing.coords.y };
                cosAlpha = position.x / position.module;
                sinAlpha = position.y / position.module;
                currentAngle = Vector.GetAngle(cosAlpha, sinAlpha);

                //(-mV sin + subjVX ) ** 2 + (mV cos + subjVY) ** 2 = vsh ** 2
                //mV ** 2 sin ** 2 - 2 mV sin subjVX + mV **2 cos ** 2 + 2 mv cos subjVY + subjVX ** 2 + subjVY ** 2 = vsh ** 2
                //mV ** 2 + subjV ** 2 - 2 mv sin subjVX + 2 mv cos subjVY = vsh ** 2
                //mV ** 2 + 2 mV (cos subjVY - sin subjVX) + subjV ** 2 - vsh ** 2 = 0

                double a = 1.0;
                double b = 2.0 * (subject.v.y * cosAlpha - subject.v.x * sinAlpha);
                double c = Math.Pow(subjectProcessing.v.module, 2) - Math.Pow(shipProcessing.v.module, 2);
                double D = Math.Pow(b, 2) - 4 * a * c;
                double sqrtD = Math.Sqrt(D);
                double[] sol = new double[]
                {
                    (-b + sqrtD) / (2.0 * a),
                    (-b - sqrtD) / (2.0 * a)
                };

                additionalVModule = sol[0] > 0 ? sol[0] : sol[1];

                //if(maneuverType == ManeuverType.On_the_right)
                {
                    if (isClockwise)
                    {
                        v = new Vector
                        {
                            x = additionalVModule * sinAlpha + subjectProcessing.v.x,
                            y = -additionalVModule * cosAlpha + subjectProcessing.v.y
                        };
                    }
                    else
                    {
                        v = new Vector
                        {
                            x = -additionalVModule * sinAlpha + subjectProcessing.v.x,
                            y = additionalVModule * cosAlpha + subjectProcessing.v.y
                        };
                    }
                }
                //else
                //{
                //    v = new Vector
                //    {
                //        x = additionalVModule * sinAlpha + subjectProcessing.v.x,
                //        y = -additionalVModule * cosAlpha + subjectProcessing.v.y
                //    };
                //}

                shipProcessing.coords.x += v.x * dt;
                shipProcessing.coords.y += v.y * dt;

                DoSimulationStep(subjectsProcessing, dt);

                //subjectProcessing.coords.x = subjectProcessing.coords.x + subjectProcessing.v.x * dt;
                //subjectProcessing.coords.y = subjectProcessing.coords.y + subjectProcessing.v.y * dt;

                steps.Add(new Tuple<Vector, double>(new Vector { x = v.x, y = v.y }, dt));
            }

            return new Tuple<MovingSubject, List<MovingSubject>, List<Tuple<Vector, double>>>(shipProcessing, subjectsProcessing, steps);
        }

        public static Tuple<Vector, double> GetLineManeuver(MovingSubject ship, MovingSubject subject, double startAngle, double minimumDistance, ManeuverType maneuverType)
        {
            Vector position = new Vector { x = subject.coords.x - ship.coords.x, y = subject.coords.y - ship.coords.y };

            if (position.module == 0.0)
                return new Tuple<Vector, double>(new Vector { x = 0.0, y = 0.0 }, 0.0);

            double cosAlpha = position.x / position.module;
            double sinAlpha = position.y / position.module;
            double rotateAngle = Vector.GetAngle(cosAlpha, sinAlpha);

            MovingSubject shipRotated = new MovingSubject
            {
                coords = ship.coords.RotateVector(cosAlpha, sinAlpha),
                v = ship.v.RotateVector(cosAlpha, sinAlpha)
            };
            MovingSubject subjectRotated = new MovingSubject
            {
                coords = subject.coords.RotateVector(cosAlpha, sinAlpha),
                v = subject.v.RotateVector(cosAlpha, sinAlpha)
            };

            double startAngleRotated = startAngle + rotateAngle;

            Tuple<Vector, double> resultRotated = GetLineManeuverRotated(shipRotated, subjectRotated, startAngleRotated, minimumDistance, maneuverType);

            double rotateBackCos = Math.Cos(-rotateAngle);
            double rotateBackSin = Math.Sin(-rotateAngle);
            Vector v = resultRotated.Item1.RotateVector(rotateBackCos, rotateBackSin);

            Tuple<Vector, double> result = new Tuple<Vector, double>(v, resultRotated.Item2);
            
            return result;
        }

        public static Tuple<Vector, double> GetLineManeuverRotated(MovingSubject ship, MovingSubject subject, double startAngle, double minimumDistance, ManeuverType maneuverType)
        {
            Vector position = new Vector { x = subject.coords.x - ship.coords.x, y = subject.coords.y - ship.coords.y };

            if (position.module == 0.0)
                return new Tuple<Vector, double>(new Vector { x = 0.0, y = 0.0 }, 0.0);

            double[] slopes = GetRelativeVelocitySlope(ship, subject, startAngle, minimumDistance);
            double slope = slopes[0];

            if (maneuverType == ManeuverType.On_the_right)
                slope = slopes[1];


            double startAngleSlope = Math.Tan(startAngle);

            Vector v = new Vector();
            v.x = (slope * subject.v.x - subject.v.y) / (slope - startAngleSlope);
            v.y = v.x * startAngleSlope;

            double a = -slope;
            double b = 1.0;
            double c = slope * ship.coords.x - ship.coords.y;
            double yship2 = (a * (-b * subject.coords.x + a * subject.coords.y) - b * c) / (Math.Pow(a, 2) + Math.Pow(b, 2));
            //double xship2 = (b * (b * subject.coords.x - a * subject.coords.y) - a * c) / (Math.Pow(a, 2) + Math.Pow(b, 2));
            double t = (yship2 - ship.coords.y) / (v.y - subject.v.y);

            //double d = Math.Sqrt(Math.Pow(yship2 - subject.coords.y, 2) + Math.Pow(xship2 - subject.coords.x, 2));

            return new Tuple<Vector, double>(v, t);
        }
        #endregion One subject maneuvers

        #region Several subjects maneuvers
        public static void DoManeuver(MovingSubject ship, List<MovingSubject> subjects, List<double> angles, List<ManeuverType> maneuverTypes, double minimumDistance, Vector target, IsManeuverEnded isManeuverEnded,
            double optimalV, double maxTime)
        {
            MovingSubject currentShip = new MovingSubject { coords = new Vector { x = ship.coords.x, y = ship.coords.y }, v = new Vector { x = ship.v.x, y = ship.v.y } };
            List<MovingSubject> currentSubjects = new List<MovingSubject>();
            for (int i = 0; i < subjects.Count; ++i)
            {
                currentSubjects.Add(new MovingSubject
                {
                    coords = new Vector { x = subjects[i].coords.x, y = subjects[i].coords.y },
                    v = new Vector { x = subjects[i].v.x, y = subjects[i].v.y }
                });
            }
            List<Tuple<double, double>> velocitiesAndTimes = GetVelocitiesAndTimes(currentShip, currentSubjects, angles, maneuverTypes, minimumDistance, target, isManeuverEnded, optimalV, maxTime);

            ClearPositionFile();

            List<MovingSubject> allSubjectsList = new List<MovingSubject> { ship };
            foreach (MovingSubject ms in subjects)
                allSubjectsList.Add(ms);

            double spentTime = 0.0;
            for(int i = 0; i < subjects.Count; ++i)
            {
                Tuple<Vector, double> maneuver = MotionSimulation.GetLineManeuver(ship, subjects[i], angles[i], minimumDistance, maneuverTypes[i]);

                double tan1 = Math.Tan(angles[i]);
                double tan2 = maneuver.Item1.y / maneuver.Item1.x;
                ship.v = maneuver.Item1;
                int n = 1000;
                for (int j = 0; j < n; ++j)
                {
                    if(maneuver.Item2 < 0.0)
                    {
                        int a = 6;
                    }
                    MotionSimulation.DoSimulationStep(allSubjectsList, maneuver.Item2 / (n + 0.0));
                    
                    SavePositions(allSubjectsList);
                }
                spentTime += maneuver.Item2;
                
                
                List<Tuple<Vector, double>> shipStepsClockwise = MotionSimulation.GetRouteManeuver(ship, subjects, angles, maneuverTypes, i, minimumDistance, isManeuverEnded, true).Item3;
                List<Tuple<Vector, double>> shipStepsCounterClockwise = MotionSimulation.GetRouteManeuver(ship, subjects, angles, maneuverTypes, i, minimumDistance, isManeuverEnded, false).Item3;
                List<Tuple<Vector, double>> shipSteps = shipStepsClockwise.Count > shipStepsCounterClockwise.Count ? shipStepsCounterClockwise : shipStepsClockwise;
                int step = 5;
                for (int j = 0; j < shipSteps.Count; ++j)
                {
                    ship.v.x = shipSteps[j].Item1.x;
                    ship.v.y = shipSteps[j].Item1.y;

                    MotionSimulation.DoSimulationStep(allSubjectsList, shipSteps[j].Item2);
                    if(j % step == 0)
                    {
                        SavePositions(allSubjectsList);
                    }
                }

                spentTime += shipSteps.Sum(st => st.Item2);
            }

            //Console.WriteLine();
            //Console.WriteLine();

            Vector relativePosition = new Vector { x = target.x - ship.coords.x, y = target.y - ship.coords.y };
            double cosAlpha = relativePosition.x / relativePosition.module;
            double sinAlpha = relativePosition.y / relativePosition.module;

            double lastLineV = optimalV;
            double remainingTime = maxTime - spentTime;

            if (remainingTime > 0)
            {
                double minRequiredV = relativePosition.module / remainingTime;
                lastLineV = optimalV > minRequiredV ? optimalV : minRequiredV;
            }

            ship.v = new Vector { x = lastLineV * cosAlpha, y = lastLineV * sinAlpha };
            double t = relativePosition.module / lastLineV;

            int n2 = 1000;
            for (int j = 0; j < n2; ++j)
            {
                MotionSimulation.DoSimulationStep(allSubjectsList, t / (n2 + 0.0));

                SavePositions(allSubjectsList);
            }

            //Console.WriteLine("spentTime = " + spentTime);
        }

        public static List<Tuple<double, double>> GetVelocitiesAndTimes(MovingSubject ship, List<MovingSubject> subjects, List<double> angles, List<ManeuverType> maneuverTypes, double minimumDistance, Vector target, IsManeuverEnded isManeuverEnded,
            double optimalV, double maxTime)
        {
            List<Tuple<double, double>> result = new List<Tuple<double, double>>();
            ClearPositionFile();

            List<MovingSubject> allSubjectsList = new List<MovingSubject> { ship };
            foreach (MovingSubject ms in subjects)
            {
                allSubjectsList.Add(ms);
            }

            double spentTime = 0.0;
            for (int i = 0; i < subjects.Count; ++i)
            {
                Tuple<Vector, double> maneuver = MotionSimulation.GetLineManeuver(ship, subjects[i], angles[i], minimumDistance, maneuverTypes[i]);
                
                double tan1 = Math.Tan(angles[i]);
                double tan2 = maneuver.Item1.y / maneuver.Item1.x;
                ship.v = maneuver.Item1;

                int n = 1000;
                for (int j = 0; j < n; ++j)
                {
                    if (maneuver.Item2 < 0.0)
                    {
                        int a = 6;
                    }
                    MotionSimulation.DoSimulationStep(allSubjectsList, maneuver.Item2 / (n + 0.0));
                }
                result.Add(new Tuple<double, double>(ship.v.module, maneuver.Item2));
                
                spentTime += maneuver.Item2;
                
                var routeClockwise = MotionSimulation.GetRouteManeuver(ship, subjects, angles, maneuverTypes, i, minimumDistance, isManeuverEnded, true);
                var routeCounterClockwise = MotionSimulation.GetRouteManeuver(ship, subjects, angles, maneuverTypes, i, minimumDistance, isManeuverEnded, false);
                
                List<Tuple<Vector, double>> shipStepsClockwise = routeClockwise.Item3;
                List<Tuple<Vector, double>> shipStepsCounterClockwise = routeCounterClockwise.Item3;
                List<Tuple<Vector, double>> shipSteps = shipStepsClockwise.Count > shipStepsCounterClockwise.Count ? shipStepsCounterClockwise : shipStepsClockwise;

                var route = shipStepsClockwise.Count > shipStepsCounterClockwise.Count ? routeCounterClockwise : routeClockwise;

                ship = route.Item1;
                subjects = route.Item2;
                allSubjectsList = new List<MovingSubject> { ship };
                foreach (MovingSubject ms in subjects)
                {
                    allSubjectsList.Add(ms);
                }

                //for (int j = 0; j < shipSteps.Count; ++j)
                //{
                //    ship.v.x = shipSteps[j].Item1.x;
                //    ship.v.y = shipSteps[j].Item1.y;

                //    MotionSimulation.DoSimulationStep(allSubjectsList, shipSteps[j].Item2);
                //}

                double routeTime = shipSteps.Sum(st => st.Item2);
                spentTime += routeTime;

                result.Add(new Tuple<double, double>(maneuver.Item1.module, routeTime));
            }



            Vector relativePosition = new Vector { x = target.x - ship.coords.x, y = target.y - ship.coords.y };
            double cosAlpha = relativePosition.x / relativePosition.module;
            double sinAlpha = relativePosition.y / relativePosition.module;

            double lastLineV = optimalV;
            double remainingTime = maxTime - spentTime;

            if (remainingTime > 0)
            {
                double minRequiredV = relativePosition.module / remainingTime;
                lastLineV = optimalV > minRequiredV ? optimalV : minRequiredV;
            }

            ship.v = new Vector { x = lastLineV * cosAlpha, y = lastLineV * sinAlpha };
            double t = relativePosition.module / lastLineV;

            MotionSimulation.DoSimulationStep(allSubjectsList, t);
            result.Add(new Tuple<double, double>(ship.v.module, t));

            return result;
        }
        #endregion Several subjects maneuvers

        #region Auxilary block
        static string history = @"D:\Soft\PathFinderSolution\Computation_modules\vessels_position_history.txt";
        static string info = @"D:\Soft\PathFinderSolution\Computation_modules\vessels_info.txt";

        static void SavePositions(List<MovingSubject> subjects)
        {
            using (StreamWriter writer = new StreamWriter(history, true))
            {
                foreach (MovingSubject subject in subjects)
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

        public static List<MovingSubject> GetNextStepShips()
        {
            List<MovingSubject> nextSubjects = new List<MovingSubject>();

            return nextSubjects;
        }

        public static double GetCost(List<double> velocities, List<double> times)
        {
            double allTime = times.Sum(t => t);

            double result = 0.0;
            double vOptimal = 11.3;
            double minCost = 25.0;
            for(int i = 0; i < velocities.Count; ++i)
            {
                if (allTime > maxTime || times[i] is double.NaN || times[i] < 0.0)
                    result += 100000000;
                else
                    result += (Math.Pow(velocities[i] - vOptimal, 2) + minCost) * times[i];
            }

            return result;
        }

        public static double FindMinimum()
        {
            return 0.0;
        }
        #endregion Auxilary block

        #region Optimization methods
        //Powell method
        public delegate double PathCost(List<double> angles, List<ManeuverType> maneuverTypes);

        public static double TestMethod(List<double> angles, List<ManeuverType> maneuverTypes)
        {
            double x = angles[0];
            double y = angles[1];

            return 5 + 3 * Math.Pow(x, 4) + Math.Pow(y, 8) + Math.Sin(y) * Math.Sin(y) * Math.Pow(x, 6) + Math.Sin(x) * Math.Sin(x) * Math.Pow(y, 2);
            return Math.Pow(angles[0], 2) + Math.Pow(angles[1], 2);
        }

        public static double CostOfRoute(MovingSubject ship, List<MovingSubject> subjects, List<double> angles, List<ManeuverType> maneuverTypes, double minimumDistance,
            Vector target, IsManeuverEnded isManeuverEnded, double optimalV, double maxTime)
        {
            MovingSubject currentShip = new MovingSubject { coords = new Vector { x = ship.coords.x, y = ship.coords.y }, v = new Vector { x = ship.v.x, y = ship.v.y } };
            List<MovingSubject> currentSubjects = new List<MovingSubject>();
            for (int i = 0; i < subjects.Count; ++i)
            {
                currentSubjects.Add(new MovingSubject
                {
                    coords = new Vector { x = subjects[i].coords.x, y = subjects[i].coords.y },
                    v = new Vector { x = subjects[i].v.x, y = subjects[i].v.y }
                });
            }
            List<Tuple<double, double>> velocitiesAndTimes = GetVelocitiesAndTimes(currentShip, currentSubjects, angles, maneuverTypes, minimumDistance, target, isManeuverEnded, optimalV, maxTime);
            
            double cost = GetCost(velocitiesAndTimes.Select(el => el.Item1).ToList(), velocitiesAndTimes.Select(el => el.Item2).ToList());

            return cost;
        }

        public static double RoutePathCost(List<double> angles, List<ManeuverType> maneuverTypes)
        {
            return CostOfRoute(ship, subjects, angles, maneuverTypes, minimumDistance, target, isManeuverEnded, optimalV, maxTime);
        }

        //public List<double> GetOptimumSolution1(PathCost pathCost, List<double> startAngles, double h = 0.1, double tol = 0.000001)
        //{
        //    List<double> d = new List<double>();
        //    for (int i = 0; i < startAngles.Count; ++i)
        //        d.Add(0.0);
        //    List<double> u = new List<double>();
        //    for(int j = 0; j < 30; ++j)
        //    {
        //        List<double> xOld = new List<double>(startAngles);
        //        double fOld = pathCost(xOld, null);

        //        for(int i = 0; i < startAngles.Count; ++i)
        //        {
        //            double v = u[i];

        //        }
        //    }
        //}

        public static List<double> GetOptimumSolution(PathCost pathCost, List<double> startAngles, List<ManeuverType> maneuverTypes, double h = Math.PI, double tol = 0.000001)
        {
            int n = startAngles.Count;
            List<double> current = new List<double>();
            for (int i = 0; i < startAngles.Count; ++i)
                current.Add(startAngles[i]);

            List<double> hs = new List<double>();
            for (int i = 0; i < startAngles.Count; ++i)
                hs.Add(h);

            
            double curH = GetOptimumStep(pathCost, current, maneuverTypes, h, n - 1, tol);
            current[n - 1] += curH;

            

            do
            {
                for (int i = 0; i < startAngles.Count; ++i)
                    startAngles[i] = current[i];

                for (int i = 0; i < n; ++i)
                {
                    List<double> oldCurrent = new List<double>();
                    for (int ix = 0; ix < current.Count; ++ix)
                        oldCurrent.Add(current[ix]);
                    hs[i] = GetOptimumStep(pathCost, current, maneuverTypes, hs[i], i, tol);
                    current = oldCurrent;
                    
                    //hs[i] = curH;
                    if(hs[i] != 0.0)
                    {
                        current[i] += hs[i];
                    }
                    
                    //Console.WriteLine("hs[i]");
                    //Console.WriteLine(hs[i]);
                }

                StringBuilder sb = new StringBuilder("start point: ");
                for (int i = 0; i < startAngles.Count; ++i)
                {
                    sb.Append(startAngles[i]).Append(" ");
                }
                Console.WriteLine(sb.ToString());

                sb = new StringBuilder("current point: ");
                for (int i = 0; i < current.Count; ++i)
                {
                    sb.Append(current[i]).Append(" ");
                }
                Console.WriteLine(sb.ToString());
                
                Console.WriteLine();
            } while (GetModule(current, startAngles) > tol);
            if(costAndAnglesDictionary.Count > 0)
            {
                double minKey = costAndAnglesDictionary.Min(e => e.Key);
                current = costAndAnglesDictionary.Where(el => el.Key == minKey).Select(el => el.Value).ToArray()[0];
            }
            return current;
        }

        static double GetOptimumStep(PathCost pathCost, List<double> angles, List<ManeuverType> maneuverTypes, double h, int index, double tol)
        {
            int n = angles.Count;
            List<double> start = new List<double>();
            for (int i = 0; i < angles.Count; ++i)
                start.Add(angles[i]);
            start[index] -= h;

            List<double> end = new List<double>();
            for (int i = 0; i < angles.Count; ++i)
                end.Add(angles[i]);
            end[index] += h;

            List<double> currentMean = new List<double>();
            for (int i = 0; i < angles.Count; ++i)
                currentMean.Add(angles[i]);
            //currentMean[index] += h / 2.0;

            double fStart = pathCost(start, maneuverTypes);
            double fEnd = pathCost(end, maneuverTypes);
            double fMean = pathCost(currentMean, maneuverTypes);
            while(Math.Abs(fEnd - fStart) > tol)
            {
                List<double> savingAngles = new List<double>();
                for(int i = 0; i < currentMean.Count; ++i)
                {
                    savingAngles.Add(currentMean[i]);
                }
                if(!costAndAnglesDictionary.ContainsKey(fMean))
                    costAndAnglesDictionary.Add(fMean, savingAngles);
                if(fEnd > fStart)
                {
                    currentMean[index] -= h / 2.0;
                    fEnd = fMean;
                }
                else
                {
                    currentMean[index] += h / 2.0;
                    fStart = fMean;
                }
                fMean = pathCost(currentMean, maneuverTypes);

                h /= 2.0;

                Console.WriteLine("Cost: " + fMean);
            }

            return currentMean[index] - angles[index];
        }

        static Dictionary<double, List<double>> costAndAnglesDictionary = new Dictionary<double, List<double>>();

        static double GetModule(List<double> angles1, List<double> angles2)
        {
            double result = 0.0;
            for(int i = 0; i < angles1.Count; ++i)
            {
                result += Math.Pow(angles1[i] - angles2[i], 2);
            }

            return Math.Sqrt(result);
        }

        //Gradient descent method
        private static double GetDerivative(PathCost function, List<double> point, List<ManeuverType> maneuverTypes, double delta)
        {
            List<double> incrementPoint = new List<double>();
            for (int i = 0; i < point.Count; ++i)
                incrementPoint.Add(point[i] + delta);
            return (function(incrementPoint, maneuverTypes) - function(point, maneuverTypes)) / (point.Count * delta);
        }

        public double GetDerivative(Func<double, double> function, double point, double delta)
        {
            return (function(point + delta) - function(point - delta)) /
               (2 * delta);
        }

        public static List<double> Calculate(PathCost function, List<double> startPoint, List<ManeuverType> maneuverTypes, double delta, double eps)
        {
            StringBuilder sb = new StringBuilder("start point: ");
            for (int i = 0; i < startPoint.Count; ++i)
            {
                sb.Append(startPoint[i]).Append(" ");
            }
            Console.WriteLine(sb.ToString());

            double alpha = 0.000001;
            double alphaDecreaseRate = 0.9;
            List<double> currentPoint = startPoint;
            while (true)
            {
                List<double> currentPointIncremented = new List<double>(currentPoint.ToArray());

                double currentValue = function(currentPoint, maneuverTypes);
                var newPoint = new List<double>();
                for (int i = 0; i < currentPoint.Count; i++)
                {
                    currentPointIncremented[i] += delta;
                    double deriviative = (function(currentPointIncremented, maneuverTypes) - function(currentPoint, maneuverTypes)) / (2 * delta);
                    newPoint.Add(currentPoint[i] - alpha * (1.0 / Convert.ToDouble(startPoint.Count)) * deriviative);
                    currentPointIncremented[i] -= delta;

                    Console.WriteLine(newPoint[i]);
                }
                double newValue = function(newPoint, maneuverTypes);

                sb = new StringBuilder("current point: ");
                for(int i = 0; i < currentPoint.Count; ++i)
                {
                    sb.Append(currentPoint[i]).Append(" ");
                }
                Console.WriteLine(sb.ToString());

                Console.WriteLine("alpha: " + alpha);
                Console.WriteLine("current value: " + currentValue + " new value: " + newValue);
                Console.WriteLine();
                Console.WriteLine();
                

                //Console.WriteLine(newValue);

                if (newValue > currentValue)
                    alpha *= alphaDecreaseRate;
                else
                {
                    if (currentValue - newValue <= eps)
                    {
                        //Console.WriteLine("Final value = " + newValue);
                        return newPoint;
                    }
                    else
                        currentPoint = newPoint;
                }
            }
        }
        
        private static List<double> CopyPointWithReplace(List<double> point, double replace, int replaceIndex)
        {
            var result = new List<double>();
            for (var i = 0; i < point.Count; i++)
                if (i == replaceIndex)
                    result.Add(replace);
                else
                    result.Add(point[i]);

            return result;
        }
        #endregion Optimization methods
    }
}
