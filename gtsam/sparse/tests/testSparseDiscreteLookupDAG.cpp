/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testSparseDiscreteLookupDAG.cpp
 *
 *  @date May, 2023
 *  @author Yoonwoo Kim
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/sparse/SparseDiscreteLookupDAG.h>

using namespace gtsam;

/* ************************************************************************* */
TEST(SparseDiscreteLookupDAG, argmax) {

  // Declare 2 keys
  DiscreteKey A(0, 2), B(1, 2);

  // Create lookup table corresponding to "marginalIsNotMPE" in testDFG.
  SparseDiscreteLookupDAG dag;

  TableFactor tableB(DiscreteKeys{B, A}, std::vector<double>{0.5, 1. / 3, 0.5, 2. / 3});
  dag.add(1, DiscreteKeys{B, A}, tableB);

  TableFactor tableA(A, {0.5 * 10 / 19, (2. / 3) * (9. / 19)});
  dag.add(1, DiscreteKeys{A}, tableA);

  // The expected MPE is A=1, B=1
  DiscreteValues mpe{{0, 1}, {1, 1}};

  // check:
  auto actualMPE = dag.argmax();
  EXPECT(assert_equal(mpe, actualMPE));
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
