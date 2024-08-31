// (Slow) implementation of Muller's 1956 "Walk on Spheres" algorithm
// Corresponds to the naïve estimator given in Equation 5 of
// Sawhney & Crane, "Monte Carlo Geometry Processing" (2020).
// NOTE: this code makes a few shortcuts for the sake of code brevity; may
// be more suitable for tutorials than for production code/evaluation.
// To compile: c++ -std=c++17 -O3 -pedantic -Wall main.cpp -o wos
#include <algorithm>
#include <array>
#include <complex>
#include <functional>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
using namespace std;

// use std::complex to implement 2D vectors
using Vec2D = complex<float>;
float dot(Vec2D u, Vec2D v) { return real(conj(u) * v); }
float length(Vec2D u) { return sqrt(norm(u)); }

// a segment is just a pair of points
using Segment = array<Vec2D, 2>;

// returns the point on segment s closest to x
Vec2D closestPoint(Vec2D x, Segment s) {
  Vec2D u = s[1] - s[0];
  float t = clamp(dot(x - s[0], u) / dot(u, u), 0.f, 1.f);
  return (1 - t) * s[0] + t * s[1];
}

// returns a random value in the range [rMin,rMax]
float random(float rMin, float rMax) {
  const float rRandMax = 1. / (float)RAND_MAX;
  float u = rRandMax * (float)rand();
  return u * (rMax - rMin) + rMin;
}

// solves a Laplace equation Δu = 0 at x0, where the boundary is given
// by a collection of segments, and the boundary conditions are given
// by a function g that can be evaluated at any point in space
float solve(Vec2D x0, vector<Segment> segments, function<float(Vec2D)> g) {
  const float eps = 0.01;   // stopping tolerance
  const int nWalks = 128;   // number of Monte Carlo samples
  const int maxSteps = 16;  // maximum walk length

  float sum = 0.;
  for (int i = 0; i < nWalks; i++) {
    Vec2D x = x0;
    float R;
    int steps = 0;
    do {
      R = numeric_limits<float>::max();
      for (auto s : segments) {
        Vec2D p = closestPoint(x, s);
        R = min(R, length(x - p));
      }
      float theta = random(0., 2. * M_PI);
      x = x + Vec2D(R * cos(theta), R * sin(theta));
      steps++;
    } while (R > eps && steps < maxSteps);

    sum += g(x);
  }
  return sum / nWalks;  // Monte Carlo estimate
}

float checker(Vec2D x) {
  const float s = 6.;
  return fmod(floor(s * real(x)) + floor(s * imag(x)), 2.);
}

vector<Segment> scene = {{{Vec2D(0.5, 0.1), Vec2D(0.9, 0.5)}},
                         {{Vec2D(0.5, 0.9), Vec2D(0.1, 0.5)}},
                         {{Vec2D(0.1, 0.5), Vec2D(0.5, 0.1)}},
                         {{Vec2D(0.5, 0.33333333), Vec2D(0.5, 0.6666666)}},
                         {{Vec2D(0.33333333, 0.5), Vec2D(0.6666666, 0.5)}}};

int main(int argc, char** argv) {
  srand(time(NULL));
  ofstream out("out.csv");

  int s = 128;  // image size
  for (int j = 0; j < s; j++) {
    cerr << "row " << j << " of " << s << endl;
    for (int i = 0; i < s; i++) {
      Vec2D x0((float)i / (float)s, (float)j / (float)s);
      float u = solve(x0, scene, checker);
      out << u;
      if (i < s - 1) out << ",";
    }
    out << endl;
  }
  return 0;
}