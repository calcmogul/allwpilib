package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N4 extends Num implements Nat<N4> {
    private N4() {
    }

    @Override
    public int getNum() {
        return 4;
    }

    public static final N4 N4 = new N4();
}