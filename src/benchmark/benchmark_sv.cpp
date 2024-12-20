#include <benchmark/benchmark.h>

static void vector1(benchmark::State& state) {
  std::vector<int> index(100);
  for (auto _ : state) {
    std::vector<double> x(index.size());
    for (int i = 0; i < index.size(); i++) {
      x[i] = index[i];
    }
  }
}

BENCHMARK(vector1)->Iterations(100)->MinWarmUpTime(1);

static void vector2(benchmark::State& state) {
  std::vector<int> index(100);
  for (auto _ : state) {
    std::vector<double> x;
    x.reserve(index.size());
    for (auto i : index) {
      x.emplace_back(i);
    }
  }
}

BENCHMARK(vector2)->Iterations(100)->MinWarmUpTime(1);

BENCHMARK_MAIN();