package edu.wpi.first.wpilibj.math;

import org.ejml.simple.SimpleMatrix;

import java.util.Objects;

public class MatBuilder<R extends Num, C extends Num> {
    private final Nat<R> rows;
    private final Nat<C> cols;

    public final Matrix<R, C> fill(double... data) {
        if (Objects.requireNonNull(data).length != this.rows.getNum() * this.cols.getNum()) {
            throw new IllegalArgumentException("Invalid matrix data provided. Wanted " + this.rows.getNum() + " x " + this.cols.getNum() + " matrix, but got " + data.length + " elements");
        } else {
            return new Matrix<>(new SimpleMatrix(this.rows.getNum(), this.cols.getNum(), true, data));
        }
    }

    public MatBuilder(Nat<R> rows, Nat<C> cols) {
        this.rows = Objects.requireNonNull(rows);
        this.cols = Objects.requireNonNull(cols);
    }
}
