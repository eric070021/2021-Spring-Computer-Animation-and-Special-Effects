#pragma once
#include <string>
#include <vector>

#include "posture.h"
#include "skeleton.h"

class Motion {
 public:
  Motion() noexcept = default;
  Motion(const std::string &amc_file, const Skeleton &skeleton) noexcept;
  /**
   * @brief Get the reference of postures
   */
  std::vector<Posture> &posture() { return _posture; }
  /**
   * @brief Get the const reference of postures
   */
  const std::vector<Posture> &posture() const { return _posture; }
  /**
   * @brief Get posture of specific frame
   */
  Posture &posture(int i) { return _posture[i]; }
  /**
   * @brief Get posture of specific frame (const)
   */
  const Posture &posture(int i) const { return _posture[i]; }
  /**
   * @brief Get the number of frames of this motion.
   */
  int size() const { return static_cast<int>(_posture.size()); }
  /**
   * @brief Read motion data from file.
   */
  bool readAMCFile(const std::string &file_name, const Skeleton &skeleton);

 private:
  std::vector<Posture> _posture;
};
