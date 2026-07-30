// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.util;

import java.util.Objects;

/** Immutable complex number. */
public final class Complex {
  /** The complex number 0 + 0i. */
  public static final Complex ZERO = new Complex(0.0, 0.0);

  /** The complex number 1 + 0i. */
  public static final Complex ONE = new Complex(1.0, 0.0);

  private final double m_real;
  private final double m_imag;

  /**
   * Constructs a complex number.
   *
   * @param real The real part.
   * @param imag The imaginary part.
   */
  public Complex(double real, double imag) {
    m_real = real;
    m_imag = imag;
  }

  /**
   * Constructs a complex number from polar form r(cos(θ) + i sin(θ)).
   *
   * @param r The magnitude.
   * @param theta The angle in radians.
   * @return The complex number.
   */
  public static Complex polar(double r, double theta) {
    return new Complex(r * Math.cos(theta), r * Math.sin(theta));
  }

  /**
   * Returns the real part.
   *
   * @return The real part.
   */
  public double getReal() {
    return m_real;
  }

  /**
   * Returns the imaginary part.
   *
   * @return The imaginary part.
   */
  public double getImag() {
    return m_imag;
  }

  /**
   * Returns true if both the real and imaginary parts are zero.
   *
   * @return True if both the real and imaginary parts are zero.
   */
  public boolean isZero() {
    return m_real == 0.0 && m_imag == 0.0;
  }

  /**
   * Returns |z|.
   *
   * @return |z|.
   */
  public double abs() {
    return Math.hypot(m_real, m_imag);
  }

  /**
   * Returns |z|².
   *
   * @return |z|².
   */
  public double abs2() {
    return m_real * m_real + m_imag * m_imag;
  }

  /**
   * Returns the 1-norm |Re(z)| + |Im(z)|.
   *
   * @return |Re(z)| + |Im(z)|.
   */
  public double norm1() {
    return Math.abs(m_real) + Math.abs(m_imag);
  }

  /**
   * Returns the complex conjugate.
   *
   * @return The complex conjugate.
   */
  public Complex conj() {
    return new Complex(m_real, -m_imag);
  }

  /**
   * Returns −z.
   *
   * @return −z.
   */
  public Complex negate() {
    return new Complex(-m_real, -m_imag);
  }

  /**
   * Returns z + other.
   *
   * @param other The addend.
   * @return z + other.
   */
  public Complex plus(Complex other) {
    return new Complex(m_real + other.m_real, m_imag + other.m_imag);
  }

  /**
   * Returns z + s.
   *
   * @param s The real addend.
   * @return z + s.
   */
  public Complex plus(double s) {
    return new Complex(m_real + s, m_imag);
  }

  /**
   * Returns z − other.
   *
   * @param other The subtrahend.
   * @return z − other.
   */
  public Complex minus(Complex other) {
    return new Complex(m_real - other.m_real, m_imag - other.m_imag);
  }

  /**
   * Returns z − s.
   *
   * @param s The real subtrahend.
   * @return z − s.
   */
  public Complex minus(double s) {
    return new Complex(m_real - s, m_imag);
  }

  /**
   * Returns z · other.
   *
   * @param other The multiplier.
   * @return z · other.
   */
  public Complex times(Complex other) {
    return new Complex(
        m_real * other.m_real - m_imag * other.m_imag,
        m_real * other.m_imag + m_imag * other.m_real);
  }

  /**
   * Returns z · s.
   *
   * @param s The real multiplier.
   * @return z · s.
   */
  public Complex times(double s) {
    return new Complex(m_real * s, m_imag * s);
  }

  /**
   * Returns z / other.
   *
   * @param other The divisor.
   * @return z / other.
   */
  public Complex div(Complex other) {
    double d = other.abs2();
    return new Complex(
        (m_real * other.m_real + m_imag * other.m_imag) / d,
        (m_imag * other.m_real - m_real * other.m_imag) / d);
  }

  /**
   * Returns z / s.
   *
   * @param s The real divisor.
   * @return z / s.
   */
  public Complex div(double s) {
    return new Complex(m_real / s, m_imag / s);
  }

  /**
   * Returns the principal square root, which has a nonnegative real part.
   *
   * @return The principal square root.
   */
  public Complex sqrt() {
    if (isZero()) {
      return ZERO;
    }

    // Avoid catastrophic cancellation by computing the larger of the real and imaginary parts
    // directly and deriving the other from Im(z) = 2⋅Re(√z)⋅Im(√z)
    double r = abs();
    if (m_real >= 0.0) {
      double t = Math.sqrt(0.5 * (r + m_real));
      return new Complex(t, m_imag / (2.0 * t));
    } else {
      double t = Math.sqrt(0.5 * (r - m_real));
      return new Complex(Math.abs(m_imag) / (2.0 * t), Math.copySign(t, m_imag));
    }
  }

  /**
   * Returns the principal natural logarithm, which has an imaginary part in (−π, π].
   *
   * @return The principal natural logarithm.
   */
  public Complex log() {
    return new Complex(Math.log(abs()), Math.atan2(m_imag, m_real));
  }

  /**
   * Returns log(1 + z), which is accurate even when |z| is small.
   *
   * @return log(1 + z).
   */
  public Complex log1p() {
    if (norm1() >= 0.5) {
      return plus(1.0).log();
    }

    // log(1 + z) = log|1 + z| + i arg(1 + z)
    //
    // where
    //
    //   log|1 + z| = ½ log((1 + x)² + y²)
    //              = ½ log1p(2x + x² + y²)
    //
    // Computing 2x + x² + y² directly avoids the rounding error in forming 1 + x.
    return new Complex(
        0.5 * Math.log1p(m_real * (2.0 + m_real) + m_imag * m_imag),
        Math.atan2(m_imag, 1.0 + m_real));
  }

  /**
   * Returns eᶻ.
   *
   * @return eᶻ.
   */
  public Complex exp() {
    double r = Math.exp(m_real);
    return new Complex(r * Math.cos(m_imag), r * Math.sin(m_imag));
  }

  /**
   * Returns the hyperbolic sine.
   *
   * @return The hyperbolic sine.
   */
  public Complex sinh() {
    return new Complex(Math.sinh(m_real) * Math.cos(m_imag), Math.cosh(m_real) * Math.sin(m_imag));
  }

  /**
   * Returns the principal inverse sine −i log(iz + √(1 − z²)).
   *
   * @return The principal inverse sine.
   */
  public Complex asin() {
    var l = new Complex(-m_imag, m_real).plus(ONE.minus(times(this)).sqrt()).log();
    return new Complex(l.m_imag, -l.m_real);
  }

  /**
   * Returns the principal inverse hyperbolic tangent ½ log((1 + z)/(1 − z)).
   *
   * @return The principal inverse hyperbolic tangent.
   */
  public Complex atanh() {
    var num = new Complex(1.0 + m_real, m_imag);
    var den = new Complex(1.0 - m_real, -m_imag);
    return num.div(den).log().times(0.5);
  }

  /**
   * Returns the principal power zᵖ = exp(p log(z)).
   *
   * @param p The real exponent.
   * @return zᵖ.
   */
  public Complex pow(double p) {
    if (isZero()) {
      return new Complex(Math.pow(0.0, p), 0.0);
    }
    return log().times(p).exp();
  }

  @Override
  public String toString() {
    return String.format("Complex(%s, %s)", m_real, m_imag);
  }

  /**
   * Checks equality between this Complex and another object. Both parts are compared exactly.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    return obj instanceof Complex other && m_real == other.m_real && m_imag == other.m_imag;
  }

  @Override
  public int hashCode() {
    // Adding 0.0 maps −0.0 to 0.0 so values that compare equal hash equally
    return Objects.hash(m_real + 0.0, m_imag + 0.0);
  }
}
