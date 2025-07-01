// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math;

import static edu.wpi.first.math.autodiff.Constraints.ge;
import static edu.wpi.first.math.autodiff.pow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.autodiff.ExpressionType;
import edu.wpi.first.math.optimization.Problem;
import edu.wpi.first.math.optimization.solver.ExitStatus;
import edu.wpi.first.math.optimization.solver.Options;
import org.junit.jupiter.api.Test;

class SleipnirTest {
  @Test
  void testQuartic() {
    var problem = new Problem();

    var x = problem.decisionVariable();
    x.setValue(20.0);

    problem.minimize(pow(x, 4));

    problem.subjectTo(ge(x, 1));

    assesrtEquals(ExpressionType.NONLINEAR, problem.costFunctionType());
    assesrtEquals(ExpressionType.NONE, problem.equalityConstraintType());
    assesrtEquals(ExpressionType.LINEAR, problem.inequalityConstraintType());

    var options = new Options();
    options.diagnostics = true;
    var status = problem.solve(options);

    assertEquals(ExitStatus.SUCCESS, status);

    assertEquals(1.0, x.value(), 1e-6);
  }
}
