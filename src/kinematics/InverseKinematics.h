#include "ForwardKinematics.h"
#include <ObjectiveFunction.h>
#include <NewtonFunctionMinimizer.h>

class InverseKinematics : public ObjectiveFunction
{
public:
    const Linkage *linkage; // pointer to linkae. (const Linkage &linkage_ = *linkage;)
    const Vector2d *target; // the end-effector target position
public:

    InverseKinematics(const Linkage &linkage, const Vector2d &target)
        : linkage(&linkage), target(&target){

    }

    // Return the objective function value f(x) corresponding to the IK problem
    // Given `x` which are the two angles.
    // This will be the scalar function that we want to minimize to solve IK.
    double evaluate(const VectorXd& x) const override {
        double e = 0;
        Vector2d xt = endEffectorPosition(*linkage, x);
        e = 0.5*((xt-*target).transpose())*(xt-*target);
        // 3 - Inverse Kinematics, Task 1
        // put your code in this function

        return e;
    }

    // Compute the gradient of the objective function, df/dx.
    // Given `x` which are the two angles.
    VectorXd gradient(const VectorXd& x) const override {

        // 3 - Inverse Kinematics, Task 2
        // put your code in this function
        VectorXd grad_e = VectorXd::Zero(x.size());
        Vector2d xt = endEffectorPosition(*linkage, x);
        Matrix2d J = dendEffector_dangles(*linkage, x);
        grad_e = J.transpose()*(xt-*target);
        return grad_e;
    }

    // Compute the Hessian of the objective function, d^2f/dx^2
    // Given `x` which are the two angles.
    Matrix2d hessian(const VectorXd &x) const {
        Matrix2d hess = Matrix2d::Zero();

        // 3 - Inverse Kinematics, Task 3
        // put your code in this function
        Vector2d xt = endEffectorPosition(*linkage, x);
        Matrix2d J = dendEffector_dangles(*linkage, x);
        Tensor2x2x2 DDJ = ddendEffector_ddangles(*linkage, x);
        Vector2d delta = xt - *target;
        double a = double(delta[0]);
        double b = double(delta[1]);
        hess = J.transpose()*J; // BUG HERE!!
        hess(0,0) += a*DDJ[0](0,0)+b*DDJ[0](1,0);
        hess(0,1) += a*DDJ[0](0,1)+b*DDJ[0](1,1);
        hess(1,0) += a*DDJ[1](0,0)+b*DDJ[1](1,0);
        hess(1,1) += a*DDJ[1](0,1)+b*DDJ[1](1,1);
        return hess;
    }

    // prepares the dense matrix from `hessian(...)` to be added to a sparse matrix.
    // You can ignore this piece of code.
    void addHessianEntriesTo(const VectorXd& x, std::vector<Triplet<double>>& hessianEntries) const override {
        auto hess = hessian(x);
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                hessianEntries.push_back(Triplet<double>(i, j, hess(i,j)));
            }
        }
    }

};

// Return the angles computed with IK for the end-effector to reach the target.
// Given the `linkage`, the end-effector target position `target`,
//       the current angles `anglesCurrent` and a minimization method `method`.
Vector2d inverseKinematics(const Linkage &linkage, const Vector2d &target,
                           const Vector2d &anglesCurrent, MinimizationMethod *method) {

    InverseKinematics objective(linkage, target);
    VectorXd angles = anglesCurrent;
    method->minimize(&objective, angles);

    return angles;

}
