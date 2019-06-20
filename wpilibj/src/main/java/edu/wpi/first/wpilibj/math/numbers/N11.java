package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N11 extends Num implements Nat<N11> {
    private N11() {
    }

    @Override
    public int getNum() {
        return 11;
    }

    public static final N11 N11 = new N11();
}