/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file SparseDiscreteDistribution.h
 *  @date May 2023
 *  @author Yoonwoo Kim
 */

#pragma once

#include <gtsam/sparse/SparseDiscreteConditional.h>

#include <string>
#include <vector>

namespace gtsam {

/**
 * A prior probability on a set of discrete variables.
 * Derives from SparseDiscreteConditional
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT SparseDiscreteDistribution : public SparseDiscreteConditional {
 public:
  using Base = SparseDiscreteConditional;

  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  SparseDiscreteDistribution() {}

  /// Constructor from factor.
  explicit SparseDiscreteDistribution(const TableFactor& f)
      : Base(f.size(), f) {}

  /**
   * Construct from a Signature.
   *
   * Example: SparseDiscreteDistribution P(D % "3/2");
   */
  explicit SparseDiscreteDistribution(const Signature& s) : Base(s) {}

  /**
   * Construct from key and a vector of floats specifying the probability mass
   * function (PMF).
   *
   * Example: SparseDiscreteDistribution P(D, {0.4, 0.6});
   */
  SparseDiscreteDistribution(const DiscreteKey& key, const std::vector<double>& spec)
      : SparseDiscreteDistribution(Signature(key, {}, Signature::Table{spec})) {}

  /**
   * Construct from key and a string specifying the probability mass function
   * (PMF).
   *
   * Example: SparseDiscreteDistribution P(D, "9/1 2/8 3/7 1/9");
   */
  SparseDiscreteDistribution(const DiscreteKey& key, const std::string& spec)
      : SparseDiscreteDistribution(Signature(key, {}, spec)) {}

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(
      const std::string& s = "Discrete Prior: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard interface
  /// @{

  /// Evaluate given a single value.
  double operator()(size_t value) const;

  /// We also want to keep the Base version, taking DiscreteValues:
  // TODO(dellaert): does not play well with wrapper!
  // using Base::operator();

  /// Return entire probability mass function.
  std::vector<double> pmf() const;

  /// @}
};
// SparseDiscreteDistribution

// traits
template <>
struct traits<SparseDiscreteDistribution> : public Testable<SparseDiscreteDistribution> {};

}  // namespace gtsam
