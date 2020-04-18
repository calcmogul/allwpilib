// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.filter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class MedianFilterTest {
  @Test
  void medianFilterNotFullTestEven() {
    var filter = new MedianFilter<Integer>(10);

    filter.calculate(3);
    filter.calculate(0);
    filter.calculate(4);

    assertEquals(3.5, filter.calculate(1000));
  }

  @Test
  void medianFilterNotFullTestOdd() {
    var filter = new MedianFilter<Integer>(10);

    filter.calculate(3);
    filter.calculate(0);
    filter.calculate(4);
    filter.calculate(7);

    assertEquals(4, filter.calculate(1000));
  }

  @Test
  void medianFilterFullTestEven() {
    var filter = new MedianFilter<Integer>(6);

    filter.calculate(3);
    filter.calculate(0);
    filter.calculate(0);
    filter.calculate(5);
    filter.calculate(4);
    filter.calculate(1000);

    assertEquals(4.5, filter.calculate(99));
  }

  @Test
  void medianFilterFullTestOdd() {
    var filter = new MedianFilter<Integer>(5);

    filter.calculate(3);
    filter.calculate(0);
    filter.calculate(5);
    filter.calculate(4);
    filter.calculate(1000);

    assertEquals(5, filter.calculate(99));
  }

  @Test
  void medianFilterBool() {
    var filter = new MedianFilter<Boolean>(5);

    filter.calculate(true);
    filter.calculate(false);
    filter.calculate(true);

    assertEquals(true, filter.calculate(true));
  }
}
