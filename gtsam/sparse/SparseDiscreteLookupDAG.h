/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SparseDiscreteLookupDAG.h
 * @date May, 2022
 * @author Yoonwoo Kim
 */

#pragma once

#include <gtsam/sparse/SparseDiscreteDistribution.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

class SparseDiscreteBayesNet;

/**
 * @brief SparseDiscreteLookupTable table for max-product
 * @ingroup discrete
 *
 * Inherits from discrete conditional for convenience, but is not normalized.
 * Is used in the max-product algorithm.
 */
class GTSAM_EXPORT SparseDiscreteLookupTable : public SparseDiscreteConditional {
 public:
  using This = SparseDiscreteLookupTable;
  using shared_ptr = std::shared_ptr<This>;
  using BaseConditional = Conditional<TableFactor, This>;

  /**
   * @brief Construct a new Discrete Lookup Table object
   *
   * @param nFrontals number of frontal variables
   * @param keys a sorted list of gtsam::Keys
   * @param potentials the algebraic decision tree with lookup values
   */
  SparseDiscreteLookupTable(size_t nFrontals, const DiscreteKeys& keys,
                      const TableFactor& potentials)
      : SparseDiscreteConditional(nFrontals, keys, potentials) {}

  /// GTSAM-style print
  void print(
      const std::string& s = "Discrete Lookup Table: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /**
   * @brief return assignment for single frontal variable that maximizes value.
   * @param parentsValues Known assignments for the parents.
   * @return maximizing assignment for the frontal variable.
   */
  size_t argmax(const DiscreteValues& parentsValues) const;

  /**
   * @brief Calculate assignment for frontal variables that maximizes value.
   * @param (in/out) parentsValues Known assignments for the parents.
   */
  void argmaxInPlace(DiscreteValues* parentsValues) const;
};

/** A DAG made from lookup tables, as defined above. */
class GTSAM_EXPORT SparseDiscreteLookupDAG : public BayesNet<SparseDiscreteLookupTable> {
 public:
  using Base = BayesNet<SparseDiscreteLookupTable>;
  using This = SparseDiscreteLookupDAG;
  using shared_ptr = std::shared_ptr<This>;

  /// @name Standard Constructors
  /// @{

  /// Construct empty DAG.
  SparseDiscreteLookupDAG() {}

  /// Create from BayesNet with LookupTables
  static SparseDiscreteLookupDAG FromBayesNet(const SparseDiscreteBayesNet& bayesNet);

  /// @}

  /// @name Testable
  /// @{

  /** Check equality */
  bool equals(const This& bn, double tol = 1e-9) const;

  /// @}

  /// @name Standard Interface
  /// @{

  /** Add a SparseDiscreteLookupTable */
  template <typename... Args>
  void add(Args&&... args) {
    emplace_shared<SparseDiscreteLookupTable>(std::forward<Args>(args)...);
  }

  /**
   * @brief argmax by back-substitution, optionally given certain variables.
   *
   * Assumes the DAG is reverse topologically sorted, i.e. last
   * conditional will be optimized first *and* that the
   * DAG does not contain any conditionals for the given variables. If the DAG
   * resulted from eliminating a factor graph, this is true for the elimination
   * ordering.
   *
   * @return given assignment extended w. optimal assignment for all variables.
   */
  DiscreteValues argmax(DiscreteValues given = DiscreteValues()) const;
  /// @}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

// traits
template <>
struct traits<SparseDiscreteLookupDAG> : public Testable<SparseDiscreteLookupDAG> {};

}  // namespace gtsam
