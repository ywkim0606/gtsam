/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file    testSparseDiscreteDistribution.cpp
 * @brief   unit tests for SparseDiscreteDistribution
 * @author  Yoonwoo Kim
 * @date    May 2023
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/sparse/SparseDiscreteDistribution.h>
#include <gtsam/discrete/Signature.h>

using namespace gtsam;

static const DiscreteKey X(0, 2);

/* ************************************************************************* */
TEST(SparseDiscreteDistribution, constructors) {
  TableFactor f(X, "0.4 0.6");
  SparseDiscreteDistribution expected(f);

  SparseDiscreteDistribution actual(X % "2/3");
  EXPECT_LONGS_EQUAL(1, actual.nrFrontals());
  EXPECT_LONGS_EQUAL(0, actual.nrParents());
  EXPECT(assert_equal(expected, actual, 1e-9));

  const std::vector<double> pmf{0.4, 0.6};
  SparseDiscreteDistribution actual2(X, pmf);
  EXPECT_LONGS_EQUAL(1, actual2.nrFrontals());
  EXPECT_LONGS_EQUAL(0, actual2.nrParents());
  EXPECT(assert_equal(expected, actual2, 1e-9));
}

/* ************************************************************************* */
TEST(SparseDiscreteDistribution, Multiply) {
  DiscreteKey A(0, 2), B(1, 2);
  SparseDiscreteConditional conditional(A | B = "1/2 2/1");
  SparseDiscreteDistribution prior(B, "1/2");
  SparseDiscreteConditional actual = prior * conditional;  // P(A|B) * P(B)

  EXPECT_LONGS_EQUAL(2, actual.nrFrontals());  // = P(A,B)
  TableFactor factor(A & B, "1 4 2 2");
  SparseDiscreteConditional expected(2, factor);
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(SparseDiscreteDistribution, operator) {
  SparseDiscreteDistribution prior(X % "2/3");
  EXPECT_DOUBLES_EQUAL(prior(0), 0.4, 1e-9);
  EXPECT_DOUBLES_EQUAL(prior(1), 0.6, 1e-9);
}

/* ************************************************************************* */
TEST(SparseDiscreteDistribution, pmf) {
  SparseDiscreteDistribution prior(X % "2/3");
  std::vector<double> expected{0.4, 0.6};
  EXPECT(prior.pmf() == expected);
}

/* ************************************************************************* */
TEST(SparseDiscreteDistribution, sample) {
  SparseDiscreteDistribution prior(X % "2/3");
  prior.sample();
}

/* ************************************************************************* */
TEST(SparseDiscreteDistribution, argmax) {
  SparseDiscreteDistribution prior(X % "2/3");
  EXPECT_LONGS_EQUAL(prior.argmax(), 1);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
