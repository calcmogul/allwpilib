package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N17 extends Num implements Nat<N17> {
    private N17() {
    }

    @Override
    public int getNum() {
        return 17;
    }

    public static final N17 N17 = new N17();
}