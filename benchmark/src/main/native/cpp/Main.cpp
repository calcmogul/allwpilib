// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <benchmark/benchmark.h>

#include "CartPoleBenchmark.hpp"
#include "DiscretizationBenchmark.hpp"
#include "TravelingSalesmanBenchmark.hpp"

BENCHMARK(BM_CartPole);
BENCHMARK(BM_DiscretizeAB_Dense<1, 1>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Structured<1, 1>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Dense<2, 1>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Structured<2, 1>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Dense<2, 2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Structured<2, 2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Dense<3, 2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Structured<3, 2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Dense<5, 2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Structured<5, 2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Dense<7, 2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Structured<7, 2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Dense<10, 4>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAB_Structured<10, 4>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Dense<1>)->Arg(10)->Arg(200)->Arg(1500)->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Structured<1>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Dense<2>)->Arg(10)->Arg(200)->Arg(1500)->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Structured<2>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Dense<3>)->Arg(10)->Arg(200)->Arg(1500)->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Structured<3>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Dense<5>)->Arg(10)->Arg(200)->Arg(1500)->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Structured<5>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Dense<7>)->Arg(10)->Arg(200)->Arg(1500)->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Structured<7>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Dense<10>)->Arg(10)->Arg(200)->Arg(1500)->Arg(50000);
BENCHMARK(BM_DiscretizeAQ_Structured<10>)
    ->Arg(10)
    ->Arg(200)
    ->Arg(1500)
    ->Arg(50000);
BENCHMARK(BM_TravelingSalesman_Transform);
BENCHMARK(BM_TravelingSalesman_Twist);

BENCHMARK_MAIN();
