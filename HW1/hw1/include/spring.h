#pragma once

class Spring {
 public:
  enum class Type { STRUCTURAL, SHEAR, BEND };
  Spring(unsigned int start, unsigned int end, float restLength, Type type) :
      _startParticleIndex(start), _endParticleIndex(end), _length(restLength), _springType(type) {}

  unsigned int startParticleIndex() const { return _startParticleIndex; }
  unsigned int endParticleIndex() const { return _endParticleIndex; }
  float length() const { return _length; }
  Type type() const { return _springType; }

 private:
  unsigned int _startParticleIndex;
  unsigned int _endParticleIndex;
  float _length;
  Type _springType;
};
