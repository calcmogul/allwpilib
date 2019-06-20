package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N9 extends Num implements Nat<N9> {
    private N9() {
    }

    @Override
    public int getNum() {
        return 9;
    }

    public static final N9 N9 = new N9();
}