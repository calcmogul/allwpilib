package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N5 extends Num implements Nat<N5> {
    private N5() {
    }

    @Override
    public int getNum() {
        return 5;
    }

    public static final N5 N5 = new N5();
}