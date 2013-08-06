/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactor.h
 * @brief   A factor with a quadratic error function - a Gaussian
 * @brief   GaussianFactor
 * @author  Richard Roberts, Christian Potthast
 */

// \callgraph

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Factor.h>

namespace gtsam {

  // Forward declarations
  class VectorValues;

  /**
   * An abstract virtual base class for JacobianFactor and HessianFactor. A GaussianFactor has a
   * quadratic error function. GaussianFactor is non-mutable (all methods const!). The factor value
   * is exp(-0.5*||Ax-b||^2) */
  class GTSAM_EXPORT GaussianFactor : public Factor
  {
  public:
    typedef GaussianFactor This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class
    typedef Factor Base; ///< Our base class

    /** Default constructor creates empty factor */
    GaussianFactor() {}
    
    /** Construct from container of keys.  This constructor is used internally from derived factor
     *  constructors, either from a container of keys or from a boost::assign::list_of. */
    template<typename CONTAINER>
    GaussianFactor(const CONTAINER& keys) : Base(keys) {}

    // Implementing Testable interface
    virtual void print(const std::string& s = "",
        const KeyFormatter& formatter = DefaultKeyFormatter) const = 0;

    /** Equals for testable */
    virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const = 0;

    /** Print for testable */
    virtual double error(const VectorValues& c) const = 0; /**  0.5*(A*x-b)'*D*(A*x-b) */

    /** Return the dimension of the variable pointed to by the given key iterator */
    virtual size_t getDim(const_iterator variable) const = 0;

    /**
     * Return a dense \f$ [ \;A\;b\; ] \in \mathbb{R}^{m \times n+1} \f$
     * Jacobian matrix, augmented with b with the noise models baked
     * into A and b.  The negative log-likelihood is
     * \f$ \frac{1}{2} \Vert Ax-b \Vert^2 \f$.  See also
     * GaussianFactorGraph::jacobian and GaussianFactorGraph::sparseJacobian.
     */
    virtual Matrix augmentedJacobian() const = 0;

    /**
     * Return the dense Jacobian \f$ A \f$ and right-hand-side \f$ b \f$,
     * with the noise models baked into A and b. The negative log-likelihood
     * is \f$ \frac{1}{2} \Vert Ax-b \Vert^2 \f$.  See also
     * GaussianFactorGraph::augmentedJacobian and
     * GaussianFactorGraph::sparseJacobian.
     */
    virtual std::pair<Matrix,Vector> jacobian() const = 0;

    /** Return the augmented information matrix represented by this GaussianFactor.
     * The augmented information matrix contains the information matrix with an
     * additional column holding the information vector, and an additional row
     * holding the transpose of the information vector.  The lower-right entry
     * contains the constant error term (when \f$ \delta x = 0 \f$).  The
     * augmented information matrix is described in more detail in HessianFactor,
     * which in fact stores an augmented information matrix.
     */
    virtual Matrix augmentedInformation() const = 0;

    /** Return the non-augmented information matrix represented by this
     * GaussianFactor.
     */
    virtual Matrix information() const = 0;

    /** Clone a factor (make a deep copy) */
    virtual GaussianFactor::shared_ptr clone() const = 0;

    /** Test whether the factor is empty */
    virtual bool empty() const = 0;

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    virtual GaussianFactor::shared_ptr negate() const = 0;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }

  }; // GaussianFactor
  
} // namespace gtsam
