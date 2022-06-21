using Extreme.Mathematics;
using Extreme.Mathematics.Optimization;

namespace AlgorytmSeidla.Models
{
    public class Limitation
    {
        public List<double> LeftSide { get; set; }

        public ConstraintType ConstraintType { get; set; }

        public double RigthSide { get; set; }
    }
}
