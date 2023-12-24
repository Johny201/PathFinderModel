using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFinderModel
{
    class Vector
    {
        #region Fields
        private double _x;
        private double _y;
        private double _module;
        public double x
        {
            get
            {
                return _x;
            }
            set
            {
                _x = value;
                _calculateModule();
            }
        }
        public double y
        {
            get
            {
                return _y;
            }
            set
            {
                _y = value;
                _calculateModule();
            }
        }
        public double module
        {
            get
            {
                return _module;
            }
        }
        #endregion Fields
        #region Methods
        private void _calculateModule() { _module = Math.Sqrt(_x * _x + _y * _y); }

        public static double GetAngle(Vector v1, Vector v2)
        {
            double angle = Math.Acos((v1.x * v2.x + v1.y * v2.y) / (v1.module * v2.module));
            if (v1.x * v2.y - v1.y * v2.x < 0)
                angle = -angle;
            return angle;
        }

        public static double GetAngle(double cosAlpha, double sinAlpha)
        {
            double angle = Math.Acos(cosAlpha);
            if (sinAlpha < 0)
                angle = -angle;
            return angle;
        }

        public Vector RotateVector(double cosAlpha, double sinAlpha)
        {
            Vector result = new Vector
            {
                x = this.x * cosAlpha - this.y * sinAlpha,
                y = this.x * sinAlpha + this.y * cosAlpha
            };
            return result;
        }
        #endregion Methods
    }
}
