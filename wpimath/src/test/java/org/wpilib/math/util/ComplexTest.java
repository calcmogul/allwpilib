// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.util;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ComplexTest {
  private static final double kEpsilon = 1e-12;

  private static void assertComplexEquals(double real, double imag, Complex actual) {
    assertAll(
        () -> assertEquals(real, actual.getReal(), kEpsilon, "real part of " + actual),
        () -> assertEquals(imag, actual.getImag(), kEpsilon, "imaginary part of " + actual));
  }

  @Test
  void testAccessors() {
    var z = new Complex(3.0, -4.0);
    assertEquals(3.0, z.getReal());
    assertEquals(-4.0, z.getImag());

    assertComplexEquals(0.0, 0.0, Complex.ZERO);
    assertComplexEquals(1.0, 0.0, Complex.ONE);
  }

  @Test
  void testPolar() {
    assertComplexEquals(1.0, Math.sqrt(3.0), Complex.polar(2.0, Math.PI / 3.0));
    assertComplexEquals(0.0, -3.0, Complex.polar(3.0, -Math.PI / 2.0));

    var z = Complex.polar(2.5, 0.7);
    assertEquals(2.5, z.abs(), kEpsilon);
    assertEquals(0.7, z.log().getImag(), kEpsilon);
  }

  @Test
  void testIsZero() {
    assertTrue(Complex.ZERO.isZero());
    assertTrue(new Complex(-0.0, -0.0).isZero());
    assertFalse(Complex.ONE.isZero());
    assertFalse(new Complex(0.0, 1e-300).isZero());
  }

  @Test
  void testNorms() {
    var z = new Complex(3.0, -4.0);
    assertEquals(5.0, z.abs());
    assertEquals(25.0, z.abs2());
    assertEquals(7.0, z.norm1());

    // abs() shouldn't overflow for large components
    assertEquals(5e300, new Complex(3e300, 4e300).abs(), 1e288);
  }

  @Test
  void testConjAndNegate() {
    var z = new Complex(3.0, -4.0);
    assertComplexEquals(3.0, 4.0, z.conj());
    assertComplexEquals(-3.0, 4.0, z.negate());
  }

  @Test
  void testArithmetic() {
    var z = new Complex(3.0, -4.0);
    var w = new Complex(-1.5, 2.0);

    assertComplexEquals(1.5, -2.0, z.plus(w));
    assertComplexEquals(5.0, -4.0, z.plus(2.0));
    assertComplexEquals(4.5, -6.0, z.minus(w));
    assertComplexEquals(1.0, -4.0, z.minus(2.0));
    assertComplexEquals(3.5, 12.0, z.times(w));
    assertComplexEquals(6.0, -8.0, z.times(2.0));
    assertComplexEquals(-2.0, 0.0, z.div(w));
    assertComplexEquals(1.5, -2.0, z.div(2.0));

    // i² = −1
    var i = new Complex(0.0, 1.0);
    assertComplexEquals(-1.0, 0.0, i.times(i));

    // z / z = 1
    assertComplexEquals(1.0, 0.0, w.div(w));
  }

  @Test
  void testSqrt() {
    assertComplexEquals(2.0, -1.0, new Complex(3.0, -4.0).sqrt());
    assertComplexEquals(0.0, 0.0, Complex.ZERO.sqrt());
    assertComplexEquals(3.0, 0.0, new Complex(9.0, 0.0).sqrt());

    // Branch cut along the negative real axis follows the sign of the imaginary part
    assertComplexEquals(0.0, 2.0, new Complex(-4.0, 0.0).sqrt());
    assertComplexEquals(0.0, -2.0, new Complex(-4.0, -0.0).sqrt());

    // Real part shouldn't suffer from catastrophic cancellation near the negative real axis
    var root = new Complex(-4.0, 1e-20).sqrt();
    assertEquals(2.5e-21, root.getReal(), 1e-35);
    assertEquals(2.0, root.getImag(), kEpsilon);

    // Principal root has a nonnegative real part
    var w = new Complex(-1.5, -2.0);
    var wRoot = w.sqrt();
    assertTrue(wRoot.getReal() >= 0.0);
    assertComplexEquals(w.getReal(), w.getImag(), wRoot.times(wRoot));
  }

  @Test
  void testLog() {
    assertComplexEquals(1.6094379124341003, -0.9272952180016122, new Complex(3.0, -4.0).log());
    assertComplexEquals(0.0, 0.0, Complex.ONE.log());
    assertComplexEquals(0.0, Math.PI, new Complex(-1.0, 0.0).log());
    assertComplexEquals(0.0, Math.PI / 2.0, new Complex(0.0, 1.0).log());
  }

  @Test
  void testLog1p() {
    // Agrees with log(1 + z) when |z| isn't small
    var z = new Complex(3.0, -4.0);
    var expected = z.plus(1.0).log();
    assertComplexEquals(expected.getReal(), expected.getImag(), z.log1p());

    // When 1 + z rounds to 1, log(1 + z) ≈ z
    var tiny = new Complex(1e-20, -3e-20).log1p();
    assertEquals(1e-20, tiny.getReal(), 1e-35);
    assertEquals(-3e-20, tiny.getImag(), 1e-35);

    // Branch cut along the negative real axis
    assertComplexEquals(Math.log(0.5), Math.PI, new Complex(-1.5, 0.0).log1p());

    // Accurate for small |z| where computing log(1 + z) directly loses precision. Expected value
    // from mpmath with 40 digits of precision.
    var small = new Complex(1e-10, 2e-10).log1p();
    assertEquals(1.000000000150000036395530659762484103141e-10, small.getReal(), 1e-25);
    assertEquals(1.999999999800000072857727949761937564736e-10, small.getImag(), 1e-25);
  }

  @Test
  void testExp() {
    assertComplexEquals(-0.09285491028402633, 0.2028916804701695, new Complex(-1.5, 2.0).exp());
    assertComplexEquals(1.0, 0.0, Complex.ZERO.exp());

    // Euler's identity
    assertComplexEquals(-1.0, 0.0, new Complex(0.0, Math.PI).exp());

    // exp() and log() are inverses
    var w = new Complex(-1.5, 2.0);
    assertComplexEquals(w.getReal(), w.getImag(), w.log().exp());
  }

  @Test
  void testSinh() {
    assertComplexEquals(0.8860929093625314, 2.139040009980677, new Complex(-1.5, 2.0).sinh());
    assertComplexEquals(Math.sinh(2.0), 0.0, new Complex(2.0, 0.0).sinh());
    assertComplexEquals(0.0, Math.sin(2.0), new Complex(0.0, 2.0).sinh());
  }

  @Test
  void testAsin() {
    assertComplexEquals(-0.6065115181997547, 1.6224941488715938, new Complex(-1.5, 2.0).asin());
    assertComplexEquals(Math.PI / 6.0, 0.0, new Complex(0.5, 0.0).asin());
    assertComplexEquals(0.0, 0.0, Complex.ZERO.asin());
    assertComplexEquals(Math.PI / 2.0, 0.0, Complex.ONE.asin());
  }

  @Test
  void testAtanh() {
    assertComplexEquals(-0.22008968066202295, 1.2452579660726568, new Complex(-1.5, 2.0).atanh());
    assertComplexEquals(0.5493061443340549, 0.0, new Complex(0.5, 0.0).atanh());
    assertComplexEquals(0.0, 0.0, Complex.ZERO.atanh());
  }

  @Test
  void testPow() {
    var z = new Complex(3.0, -4.0);
    var w = new Complex(-1.5, 2.0);

    assertComplexEquals(2.0, -1.0, z.pow(0.5));
    assertComplexEquals(7.24784450716211, -6.717514421272205, w.pow(2.5));
    assertComplexEquals(-0.29341412520201454, -0.07899965259416156, w.pow(-1.3));

    // Integer powers agree with repeated multiplication
    var w3 = w.times(w).times(w);
    assertComplexEquals(w3.getReal(), w3.getImag(), w.pow(3.0));

    assertComplexEquals(1.0, 0.0, w.pow(0.0));

    // Zero base
    assertComplexEquals(0.0, 0.0, Complex.ZERO.pow(0.5));
    assertComplexEquals(1.0, 0.0, Complex.ZERO.pow(0.0));
    assertEquals(Double.POSITIVE_INFINITY, Complex.ZERO.pow(-1.0).getReal());
  }

  @Test
  void testEquals() {
    var z = new Complex(3.0, -4.0);

    assertEquals(z, new Complex(3.0, -4.0));
    assertEquals(z.hashCode(), new Complex(3.0, -4.0).hashCode());
    assertNotEquals(z, new Complex(3.0, 4.0));
    assertNotEquals(z, new Complex(3.0 + 1e-15, -4.0));
    assertNotEquals(z, "Complex(3.0, -4.0)");

    // −0.0 and 0.0 compare equal and hash equally
    var negativeZero = new Complex(-0.0, -0.0);
    assertEquals(Complex.ZERO, negativeZero);
    assertEquals(Complex.ZERO.hashCode(), negativeZero.hashCode());
  }

  @Test
  void testToString() {
    assertEquals("Complex(3.0, -4.0)", new Complex(3.0, -4.0).toString());
  }
}
