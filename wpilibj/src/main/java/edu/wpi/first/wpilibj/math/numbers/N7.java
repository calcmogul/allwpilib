package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N7 extends Num implements Nat<N7> {
    private N7() {
    }

    @Override
    public int getNum() {
        return 7;
    }

    public static final N7 N7 = new N7();
}