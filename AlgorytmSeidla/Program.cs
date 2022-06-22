// w opraciu o http://adam.chlipala.net/berkeley/classes/scribe270.pdf?fbclid=IwAR0HNFD2N_nFQ-xZJ8pUtG2giuT4_bUbEdGTri-A56U-hEWcG--kw4acnv0

using AlgorytmSeidla.Models;
using Extreme.Mathematics;
using Extreme.Mathematics.LinearAlgebra;
using Extreme.Mathematics.Optimization;

class Program
{
    private static readonly Random random = new Random();

    static void Main(string[] args)
    {
        List<Limitation> limitations = new List<Limitation>();
        List<double> goalFunction = new List<double>();
        List<List<double?>> range = new List<List<double?>>();
        InitializeProgram(out limitations, out goalFunction, out range);
        var isMaxFunction = false;
        ReverseVectorIfIsMinimum(goalFunction, isMaxFunction);
        var result = Seidel(limitations, goalFunction);
        for (int i = 0; i < result.Count; i++)
        {
            Console.WriteLine($"x_{i}: {Math.Round(result[i], 2)}");
        }
    }

    private static DenseVector<double> Seidel(List<Limitation> limitations, List<double> goalFunction)
    {
        var limitationsCount = limitations.Count;
        var goalFunctionCount = goalFunction.Count;

        //Jeśli tylko jedna zmienna to potraktuj jako równości i rozwiąż
        if (goalFunctionCount == 1)
        {
            var result = CountWithOneVariable(limitations, goalFunction);
            return result;
        }
        //Jeśli tyle samo zmiennych ile ograniczeń rozwiąż korzystając z Gaussa
        else if (limitationsCount == goalFunctionCount)
        {
            var result = CountWithLimitationsCountEqualGoalFunctionCount(limitations, goalFunction);
            return result;
        }
        else
        {
            List<Limitation> newLimitations = new List<Limitation>();
            Limitation h = new Limitation();
            //Wybieramy losowe ograniczenie
            var randomValue = random.Next() % limitationsCount;
            for (int i = 0; i < limitationsCount; i++)
            {
                if (i == randomValue)
                {
                    h = limitations[i];
                }
                else
                {
                    newLimitations.Add(limitations[i]);
                }
            }
            //Wyliczamy x* z nowych ograniczeń (bez ograniczenia "zabranego" do h)
            var result = Seidel(newLimitations, goalFunction);

            if(result == null)
            {
                return null;
            }
            //Jeśli x* spełnia ograniczenie h zwracamy końcowy wynik
            else if(Satisfies(h, result))
            {
                return result;
            }
            //Jeśli nie podstawiamy równość dla wcześniej wybranego ograniczenia i zaczynamy seidla od nowa
            else
            {
                limitations[randomValue].ConstraintType = ConstraintType.Equal;
                return Seidel(limitations, goalFunction);
            }

        }
    }

    private static bool Satisfies(Limitation h, DenseVector<double> result)
    {
        //Liczymy iloczyn macierzy
        double check = 0;
        for(int i = 0; i < h.LeftSide.Count; i++)
        {
            check += h.LeftSide[i] * result[i];
        }

        //Sprawdzamy spełnienie orgraniczenia h w zależności od znaku warunku
        if(h.ConstraintType == ConstraintType.Equal)
        {
            if(check == h.RigthSide)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (h.ConstraintType == ConstraintType.LessThanOrEqual)
        {
            if (check <= h.RigthSide)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (h.ConstraintType == ConstraintType.GreaterThanOrEqual)
        {
            if (check >= h.RigthSide)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        //dodatkowy else, bez którego nie zadziała metoda (wszystkie ścieżki muszą zwracać wartość) - prawdopodobnie nieosiągalny
        else
        {
            return false;
        }
    }

    private static void ReverseVectorIfIsMinimum(List<double> goalFunction, bool isMaxFunction)
    {
        if (!isMaxFunction)
        {
            for (int i = 0; i < goalFunction.Count; i++)
            {
                goalFunction[i] = goalFunction[i] * -1;
            }
        }
    }

    private static DenseVector<double> CountWithLimitationsCountEqualGoalFunctionCount(List<Limitation> limitations, List<double> goalFunction)
    {
        var limitationsCount = limitations.Count;
        var goalFunctionCount = goalFunction.Count;

        var matrixLeftValues = new double[limitationsCount * goalFunctionCount];
        var matrixRightValues = new double[limitations.Count];

        int i = 0;
        int j = 0;
        foreach (var limitation in limitations)
        {
            matrixRightValues[j] = limitation.RigthSide;
            j++;
            foreach (var value in limitation.LeftSide)
            {
                matrixLeftValues[i] = value;
                i++;
            }
        }
        var matrixLeft = Matrix.Create(limitationsCount, goalFunctionCount, matrixLeftValues, MatrixElementOrder.RowMajor);
        var matrixRigth = Matrix.Create(j, 1, matrixRightValues, MatrixElementOrder.RowMajor);

        var det = matrixLeft.GetDeterminant();

        if (det == 0)
        {
            var result = LinearProgram(limitations, goalFunction);
            return result;
        }
        else
        {
            var inverseMatrix = matrixLeft.GetInverse();
            var result = (inverseMatrix * matrixRigth).GetColumn(0).AsDenseVector();
            return result;
        }
    }

    private static DenseVector<double> CountWithOneVariable(List<Limitation> limitations, List<double> goalFunction)
    {
        var result = LinearProgram(limitations, goalFunction);
        return result;
    }

    private static DenseVector<double> LinearProgram(List<Limitation> limitations, List<double> goalFunction)
    {
        var goalFunctionCount = goalFunction.Count;

        LinearProgram linearProgram = new LinearProgram();

        for (int i = 0; i < goalFunctionCount; i++)
        {
            linearProgram.AddVariable($"X_{i}", goalFunction[i], 0, double.MaxValue);
        }

        for (int i = 0; i < limitations.Count; i++)
        {
            var limitation = limitations[i];

            linearProgram.AddLinearConstraint($"C_{i}", limitation.LeftSide, limitation.ConstraintType, limitation.RigthSide);
        }

        linearProgram.ExtremumType = ExtremumType.Maximum;
        var x = linearProgram.Solve();
        return x;
    }

    private static void InitializeProgram(out List<Limitation> limitations, out List<double> goalFunction, out List<List<double?>> range)
    {
        limitations = new List<Limitation>();
        limitations.Add(new Limitation()
        {
            LeftSide = new List<double> { 6, 2 },
            ConstraintType = ConstraintType.LessThanOrEqual,
            RigthSide = 24,
        });
        limitations.Add(new Limitation()
        {
            LeftSide = new List<double> { 1, 3 },
            ConstraintType = ConstraintType.LessThanOrEqual,
            RigthSide = 12,
        });
        limitations.Add(new Limitation()
        {
            LeftSide = new List<double> { 2, 1 },
            ConstraintType = ConstraintType.LessThanOrEqual,
            RigthSide = 4,
        });
        limitations.Add(new Limitation()
        {
            LeftSide = new List<double> { -2, 1 },
            ConstraintType = ConstraintType.LessThanOrEqual,
            RigthSide = 0,
        });
        //limitations.Add(new Limitation()
        //{
        //    LeftSide = new List<double> { 2, 2 },
        //    ConstraintType = ConstraintType.LessThanOrEqual,
        //    RigthSide = 3,
        //});
        goalFunction = new List<double>() { 1, 1 };



        //limitations = new List<Limitation>();
        //limitations.Add(new Limitation()
        //{
        //    LeftSide = new List<double> { -3, 1 },
        //    ConstraintType = ConstraintType.LessThanOrEqual,
        //    RigthSide = 4,
        //});
        //limitations.Add(new Limitation()
        //{
        //    LeftSide = new List<double> { 2, 1 },
        //    ConstraintType = ConstraintType.LessThanOrEqual,
        //    RigthSide = 4,
        //});
        //limitations.Add(new Limitation()
        //{
        //    LeftSide = new List<double> { 6, 2 },
        //    ConstraintType = ConstraintType.LessThanOrEqual,
        //    RigthSide = 24,
        //});
        //limitations.Add(new Limitation()
        //{
        //    LeftSide = new List<double> { -2, -2 },
        //    ConstraintType = ConstraintType.LessThanOrEqual,
        //    RigthSide = -3,
        //});
        //limitations.Add(new Limitation()
        //{
        //    LeftSide = new List<double> { -2, 1 },
        //    ConstraintType = ConstraintType.LessThanOrEqual,
        //    RigthSide = 0,
        //});
        //goalFunction = new List<double>() { -1, 1 };





        range = new List<List<double?>>()
        {
            new List<double?>()
            {
                0, null
            },
            new List<double?>()
            {
                0, null
            }
        };
    }
}