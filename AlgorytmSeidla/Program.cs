using Extreme.Mathematics;
using Extreme.Mathematics.Optimization;

class Program
{
    static void Main(string[] args)
    {
        LinearProgram linearProgram = new LinearProgram();

        // Next, we add two variables: we specify the name, the cost,
        // and optionally the lower and upper bound.
        linearProgram.AddVariable("X", 5);
        linearProgram.AddVariable("Y", 3);

        // Next, we add constraints. Constraints also have a name.
        // We also specify the coefficients of the variables,
        // the lower bound and the upper bound.
        linearProgram.AddLinearConstraint("C1", Vector.Create(1.0, 2.0), ConstraintType.LessThanOrEqual, 6.0);
        linearProgram.AddLinearConstraint("C2", Vector.Create(5.0, 2.0), ConstraintType.LessThanOrEqual, 10.0);
        // If a constraint is a simple equality or inequality constraint,
        // you can supply a LinearProgramConstraintType value and the
        // right-hand side of the constraint.

        // We can now solve the linear program:
        linearProgram.ExtremumType = ExtremumType.Maximum;
        var x = linearProgram.Solve();
        Console.WriteLine("Primal: \n{0:F1}", x);
        Console.WriteLine("Optimal value:   {0:F1}", linearProgram.OptimalValue);
    }
}