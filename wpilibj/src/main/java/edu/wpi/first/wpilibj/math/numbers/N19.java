package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N19 extends Num implements Nat<N19> {
    private N19() {
    }

    @Override
    public int getNum() {
        return 19;
    }

    public static final N19 N19 = new N19();
}