package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N14 extends Num implements Nat<N14> {
    private N14() {
    }

    @Override
    public int getNum() {
        return 14;
    }

    public static final N14 N14 = new N14();
}