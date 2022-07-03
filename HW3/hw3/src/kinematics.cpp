#include "kinematics.h"

#include <algorithm>
#include <stack>
#include <map>

#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // Same as HW2, but have some minor change
  // Hint:
  //   1. If you don't use `axis` in this function, you can copy-paste your code
  // Note:
  //   1. bone.axis becomes quaternion instead of vector3f

  std::map<int, bool> visit;
  std::stack<Bone*> q;
  visit[bone->idx] = true;
  bone->startPosition = posture.translations[bone->idx];
  bone->endPosition = posture.translations[bone->idx];
  q.push(bone);

  while (!q.empty()) {
    Bone* t = q.top();
    q.pop();
    if (t->idx != 0) {
      t->startPosition = t->parent->endPosition;
      Eigen::Quaternionf rot = t->rotationParentCurrent * posture.rotations[t->idx];
      for (Bone* itr = t->parent; itr != nullptr; itr = itr->parent) {
        rot = itr->rotationParentCurrent * posture.rotations[itr->idx] * rot;
      }

      Eigen::Vector3f dir = t->direction * t->length;
      t->endPosition = rot.toRotationMatrix() * (dir + posture.translations[t->idx]) + t->startPosition;
      t->rotation = rot;
    }
    for (Bone* itr = t->child; itr != nullptr; itr = itr->sibling) {
      if (visit.find(itr->idx) == visit.end()) {
        visit[itr->idx] = true;
        q.push(itr);
      }
    }
  }
}

Eigen::VectorXf leastSquareSolver(const Eigen::Matrix3Xf& jacobian, const Eigen::Vector3f& target) {
  // TODO (find x which min(| jacobian * x - target |))
  // Hint:
  //   1. Linear algebra - least squares solution
  //   2. https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Construction
  // Note:
  //   1. SVD or other pseudo-inverse method is useful
  //   2. Some of them have some limitation, if you use that method you should check it.

  Eigen::VectorXf solution(jacobian.cols());
  solution.setZero();
  Eigen::MatrixXf jacobianInv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
  solution = jacobianInv * target;
  return solution;
}

void inverseKinematics(const Eigen::Vector3f& target, Bone* start, Bone* end, Posture& posture) {
  constexpr int maxIterations = 10000;
  constexpr float epsilon = 1E-3f;
  constexpr float step = 0.001f;
  // Since bone stores in bones[i] that i == bone->idx, we can use bone - bone->idx to find bones[0] which is root.
  Bone* root = start - start->idx;
  std::vector<Bone*> boneList;
  // TODO
  // Hint:
  //   1. Traverse from end to start is easier than start to end (since there is only 1 parent)
  //   2. If start bone is not reachable from end. Go to root first.
  // Note:
  //   1. Both start and end should be in the list

  for (Bone* itr = end; itr->idx != start->parent->idx && itr != nullptr; itr = itr->parent) {
    boneList.emplace_back(itr);
  }

  // if traversal stop at root
  if (boneList.back()->idx == root->idx) {
    for (Bone* itr = start; itr->idx != root->idx; itr = itr->parent) {
      boneList.emplace_back(itr);
    }
  }

  size_t boneNum = boneList.size();
  Eigen::Matrix3Xf jacobian(3, 3 * boneNum);
  jacobian.setZero();

  for (int i = 0; i < maxIterations; ++i) {
    forwardKinematics(posture, root);
    // TODO (compute jacobian)
    //   1. Compute jacobian columns
    //   2. Compute dTheta
    // Hint:
    //   1. You should not put rotation in jacobian if it doesn't have that DoF.
    //   2. jacobian.col(/* some column index */) = /* jacobian column */
    //   3. Call leastSquareSolver to compute dTheta
    if ((target - end->endPosition).norm() < epsilon) 
        break;

    int column;
    for (int j = 0; j < boneNum; j++) {
      column = j * 3;
      Eigen::Vector3f p = end->endPosition; // end effector position in world space
      Eigen::Vector3f r = boneList[j]->startPosition; // position of joint pivot in world space
      Eigen::Vector3f distance = p - r;
      // Don't need to normalize the axes here since rotation matrix is an orthogonal matrix
      if (boneList[j]->dofrx) {
        jacobian.col(column) =
            posture.rotations[boneList[j]->idx].toRotationMatrix().col(0).cross(distance); 
      }
      if (boneList[j]->dofry) {
        jacobian.col(column + 1) =
            posture.rotations[boneList[j]->idx].toRotationMatrix().col(1).cross(distance);
      }
      if (boneList[j]->dofrz) {
        jacobian.col(column + 2) =
            posture.rotations[boneList[j]->idx].toRotationMatrix().col(2).cross(distance);
      }
    }

    Eigen::Vector3f V = target - end->endPosition;
    Eigen::VectorXf dTheta = leastSquareSolver(jacobian, V);

    for (int j = 0; j < boneNum; j++) {
      const auto& bone = *boneList[j];
      // TODO (update rotation)
      //   1. Update posture's eulerAngle using deltaTheta
      // Hint:
      //   1. Use posture.eulerAngle to get posture's eulerAngle
      //   2. All angles are in radians.
      //   3. You can ignore rotation limit of the bone.
      // Bonus:
      //   1. You cannot ignore rotation limit of the bone.

      column = j * 3;
      posture.eulerAngle[bone.idx][0] += step * dTheta(column);
      posture.eulerAngle[bone.idx][1] += step * dTheta(column + 1);
      posture.eulerAngle[bone.idx][2] += step * dTheta(column + 2);

      posture.rotations[bone.idx] = Eigen::AngleAxisf(posture.eulerAngle[bone.idx][2], Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][1], Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][0], Eigen::Vector3f::UnitX());
    }
  }
}
