/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SparseDiscreteBayesNet.cpp
 * @date May, 2023
 * @author Yoonwoo Kim
 */

#include <gtsam/sparse/SparseDiscreteBayesNet.h>
#include <gtsam/sparse/SparseDiscreteConditional.h>
#include <gtsam/inference/FactorGraph-inst.h>

namespace gtsam {

// Instantiate base class
template class FactorGraph<SparseDiscreteConditional>;

/* ************************************************************************* */
bool SparseDiscreteBayesNet::equals(const This& bn, double tol) const {
  return Base::equals(bn, tol);
}

/* ************************************************************************* */
double SparseDiscreteBayesNet::logProbability(const DiscreteValues& values) const {
  // evaluate all conditionals and add
  double result = 0.0;
  for (const SparseDiscreteConditional::shared_ptr& conditional : *this)
    result += conditional->logProbability(values);
  return result;
}

/* ************************************************************************* */
double SparseDiscreteBayesNet::evaluate(const DiscreteValues& values) const {
  // evaluate all conditionals and multiply
  double result = 1.0;
  for (const SparseDiscreteConditional::shared_ptr& conditional : *this)
    result *= (*conditional)(values);
  return result;
}

/* ************************************************************************* */
DiscreteValues SparseDiscreteBayesNet::sample() const {
  DiscreteValues result;
  return sample(result);
}

DiscreteValues SparseDiscreteBayesNet::sample(DiscreteValues result) const {
  // sample each node in turn in topological sort order (parents first)
  for (auto it = std::make_reverse_iterator(end()); it != std::make_reverse_iterator(begin()); ++it) {
    (*it)->sampleInPlace(&result);
  }
  return result;
}

/* *********************************************************************** */
std::string SparseDiscreteBayesNet::markdown(
    const KeyFormatter& keyFormatter,
    const DiscreteFactor::Names& names) const {
  using std::endl;
  std::stringstream ss;
  ss << "`SparseDiscreteBayesNet` of size " << size() << endl << endl;
  for (const SparseDiscreteConditional::shared_ptr& conditional : *this)
    ss << conditional->markdown(keyFormatter, names) << endl;
  return ss.str();
}

/* *********************************************************************** */
std::string SparseDiscreteBayesNet::html(const KeyFormatter& keyFormatter,
                                   const DiscreteFactor::Names& names) const {
  using std::endl;
  std::stringstream ss;
  ss << "<div><p><tt>SparseDiscreteBayesNet</tt> of size " << size() << "</p>";
  for (const SparseDiscreteConditional::shared_ptr& conditional : *this)
    ss << conditional->html(keyFormatter, names) << endl;
  return ss.str();
}

/* ************************************************************************* */
}  // namespace gtsam
