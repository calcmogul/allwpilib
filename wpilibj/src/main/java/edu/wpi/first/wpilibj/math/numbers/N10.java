package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N10 extends Num implements Nat<N10> {
    private N10() {
    }

    @Override
    public int getNum() {
        return 10;
    }

    public static final N10 N10 = new N10();
}