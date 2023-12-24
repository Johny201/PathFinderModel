using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFinderModel
{
    class MovingSubject
    {
        #region Fields
        public Vector coords { get; set; }
        public Vector v { get; set; }
        public double circulationRadius { get; set; }
        #endregion Fields

        #region Methods
        public void Move(double t)
        {
            coords.x += v.x * t;
            coords.y += v.y * t;
        }

        public void Move(double t, Vector a)
        {
            coords.x += v.x * t + a.x * a.x * t / 2.0;
            coords.y += v.y * t + a.y * a.y * t / 2.0;
        }

        //public void Rotate(double angle)
        //{

        //}
        #endregion Methods
    }
}
