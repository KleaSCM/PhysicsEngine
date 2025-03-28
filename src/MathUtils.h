#pragma once

#include <cmath>  
#include <algorithm> 

/**
 * @class Vector3
 * @brief Represents a 3D vector for physics calculations.
 */
struct Vector3 {
    float x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vector3& operator+=(const Vector3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vector3& operator-=(const Vector3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vector3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    Vector3& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }

    Vector3 operator+(const Vector3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vector3 operator-(const Vector3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vector3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vector3 operator/(float s) const { return {x / s, y / s, z / s}; }

    float Length() const { return std::sqrt(x*x + y*y + z*z); }
    Vector3 Normalize() const { float len = Length(); return len > 0 ? *this / len : *this; }
    float Dot(const Vector3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vector3 Cross(const Vector3& v) const { return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x}; }
};

inline Vector3 operator*(float scalar, const Vector3& v) {
    return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
}

/**
 * @class Matrix3
 * @brief Represents a 3x3 matrix for rotational inertia calculations.
 */
struct Matrix3 {
    float m[3][3];

    Matrix3() { std::fill(&m[0][0], &m[0][0] + 9, 0.0f); }
    Matrix3(float diag) { std::fill(&m[0][0], &m[0][0] + 9, 0.0f); m[0][0] = m[1][1] = m[2][2] = diag; }
    Matrix3(const Vector3& row1, const Vector3& row2, const Vector3& row3) {
        m[0][0] = row1.x; m[0][1] = row1.y; m[0][2] = row1.z;
        m[1][0] = row2.x; m[1][1] = row2.y; m[1][2] = row2.z;
        m[2][0] = row3.x; m[2][1] = row3.y; m[2][2] = row3.z;
    }

    Vector3 operator*(const Vector3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Matrix3 Transpose() const {
        return {
            {m[0][0], m[1][0], m[2][0]},
            {m[0][1], m[1][1], m[2][1]},
            {m[0][2], m[1][2], m[2][2]}
        };
    }

    Vector3 GetColumn(int index) const {
        return {m[0][index], m[1][index], m[2][index]};
    }

    Matrix3 Abs() const {
        return {
            {std::fabs(m[0][0]), std::fabs(m[0][1]), std::fabs(m[0][2])},
            {std::fabs(m[1][0]), std::fabs(m[1][1]), std::fabs(m[1][2])},
            {std::fabs(m[2][0]), std::fabs(m[2][1]), std::fabs(m[2][2])}
        };
    }
};

/**
 * @class Quaternion
 * @brief Represents a quaternion for 3D rotation.
 */
struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
    Quaternion(const Vector3& v, float scalar) : w(scalar), x(v.x), y(v.y), z(v.z) {}

    static Quaternion Identity() { return {1, 0, 0, 0}; }

    Quaternion operator*(const Quaternion& q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
    }

    Quaternion& operator+=(const Quaternion& q) {
        w += q.w; x += q.x; y += q.y; z += q.z;
        return *this;
    }

    Quaternion Conjugate() const { return {w, -x, -y, -z}; }

    void Normalize() {
        float length = std::sqrt(w * w + x * x + y * y + z * z);
        if (length > 0.0f) {
            float invLen = 1.0f / length;
            w *= invLen; x *= invLen; y *= invLen; z *= invLen;
        }
    }

    Quaternion operator*(float scalar) const {
        return {w * scalar, x * scalar, y * scalar, z * scalar};
    }

    Quaternion& operator*=(float scalar) {
        w *= scalar; x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    // Convert quaternion to rotation matrix
    Matrix3 ToMatrix() const;
};

// Implement Quaternion → Matrix3 conversion
inline Matrix3 Quaternion::ToMatrix() const {
    return Matrix3(
        Vector3(1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y),
        Vector3(2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x),
        Vector3(2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y)
    );
}

namespace MathUtils {

// Basic Math Functions
float Clamp(float value, float min, float max);
float Lerp(float a, float b, float t);
float SmoothStep(float t);
float RandomFloat(float min, float max);
Vector3 RandomVector3(float min, float max);

// Quaternion Functions
Quaternion FromAxisAngle(const Vector3& axis, float angle);
Quaternion FromEulerAngles(float pitch, float yaw, float roll);
Vector3 ToEulerAngles(const Quaternion& q);

// Utility Functions
float DegToRad(float degrees);
float RadToDeg(float radians);
bool IsNearlyEqual(float a, float b, float tolerance = 1e-6f);
bool IsNearlyZero(float value, float tolerance = 1e-6f);

// Physics-specific math functions
float CalculateInertiaTensor(float mass, float radius, bool isHollow);
Matrix3 CalculateInertiaTensorMatrix(const Vector3& halfExtents, float mass);
float CalculateAngularVelocity(float linearVelocity, float radius);
Vector3 CalculateCentripetalForce(float mass, float velocity, float radius);
float CalculateKineticEnergy(float mass, const Vector3& velocity);
float CalculateRotationalKineticEnergy(const Matrix3& inertiaTensor, const Vector3& angularVelocity);
float CalculatePotentialEnergy(float mass, float height, float gravity = 9.81f);
Vector3 CalculateImpulse(const Vector3& normal, float restitution, const Vector3& relativeVelocity, float invMassA, float invMassB);
Vector3 CalculateFrictionImpulse(const Vector3& normal, float friction, const Vector3& relativeVelocity, float invMassA, float invMassB);
float CalculateDragForce(float density, float velocity, float area, float dragCoefficient);
Vector3 CalculateBuoyantForce(float fluidDensity, float volume, const Vector3& gravity);
float CalculateElasticCollisionVelocity(float m1, float m2, float v1, float v2);
float CalculateInelasticCollisionVelocity(float m1, float m2, float v1, float v2, float restitution);
Vector3 CalculateSpringForce(const Vector3& displacement, float springConstant, float restLength);
Vector3 CalculateDampingForce(const Vector3& velocity, float dampingCoefficient);
float CalculatePeriod(float frequency);
float CalculateFrequency(float period);
float CalculateAngularFrequency(float period);
Vector3 CalculateTorque(const Vector3& force, const Vector3& leverArm);
float CalculateMomentOfInertia(float mass, float distance);
Vector3 CalculateAngularMomentum(const Matrix3& inertiaTensor, const Vector3& angularVelocity);
float CalculateWork(const Vector3& force, const Vector3& displacement);
float CalculatePower(const Vector3& force, const Vector3& velocity);

// Fluid Dynamics Functions
Vector3 CalculateFluidForce(const Vector3& velocity, float fluidDensity, float volume, float dragCoefficient);
Vector3 CalculateViscousForce(const Vector3& velocity, float viscosity, float surfaceArea);
Vector3 CalculatePressureForce(float pressure, const Vector3& normal, float area);
float CalculateReynoldsNumber(float density, float velocity, float characteristicLength, float viscosity);

// Soft Body Physics Functions
Vector3 CalculateDeformationForce(const Vector3& displacement, float stiffness, float damping);
Matrix3 CalculateDeformationGradient(const Vector3& position, const Vector3& restPosition);
float CalculateVolumePreservation(const Matrix3& deformationGradient);

// Complex Interaction Functions
Vector3 CalculateCoriolisForce(const Vector3& velocity, const Vector3& angularVelocity);
Vector3 CalculateCentrifugalForce(const Vector3& position, const Vector3& angularVelocity);
Matrix3 CalculateRotationMatrix(const Vector3& axis, float angle);
Vector3 CalculateRelativeVelocity(const Vector3& velocity, const Vector3& angularVelocity, const Vector3& position);
float CalculateImpactForce(float mass, float velocity, float impactTime);
Vector3 CalculateFrictionForce(const Vector3& normal, float frictionCoefficient, const Vector3& relativeVelocity);

// Advanced Collision Functions
float CalculateRestitutionCoefficient(float initialVelocity, float finalVelocity);
Vector3 CalculateCollisionResponse(const Vector3& normal, float restitution, const Vector3& relativeVelocity, float invMassA, float invMassB);

// Energy and Momentum Functions
float CalculateTotalEnergy(float kineticEnergy, float potentialEnergy, float rotationalEnergy);
Vector3 CalculateConservationOfMomentum(const Vector3& momentum1, const Vector3& momentum2);

// Constraint Functions
Vector3 CalculateConstraintForce(const Vector3& position, const Vector3& target, float stiffness);
Vector3 CalculateDistanceConstraint(const Vector3& posA, const Vector3& posB, float targetDistance);

// Advanced Motion Functions
Vector3 CalculatePrecession(const Vector3& angularMomentum, const Vector3& torque, float deltaTime);
Vector3 CalculateGyroscopicForce(const Vector3& angularVelocity, const Matrix3& inertiaTensor);

// Stability Functions
float CalculateStabilityFactor(const Vector3& centerOfMass, const Vector3& supportPoint);
bool IsStable(const Vector3& centerOfMass, const Vector3& supportBase, float threshold);

// Gravity Environment Functions
Vector3 CalculateGravityForce(float mass, const Vector3& gravity);
Vector3 CalculateVariableGravityForce(float mass, const Vector3& position, float planetMass, float planetRadius);

// Magnetic Field Functions
Vector3 CalculateMagneticForce(const Vector3& velocity, const Vector3& magneticField, float charge);
Vector3 CalculateMagneticField(const Vector3& position, const Vector3& magnetPosition, float magnetStrength);

// Advanced Fluid Dynamics
Vector3 CalculateLaminarFlowForce(const Vector3& velocity, float viscosity, float length, float radius);
Vector3 CalculateTurbulentFlowForce(const Vector3& velocity, float density, float area, float roughness);

// Advanced Buoyancy and Floating
Vector3 CalculateFloatingForce(float submergedVolume, float fluidDensity, const Vector3& gravity);
float CalculateSubmergedVolume(const Vector3& dimensions, float waterLevel, float objectHeight);

// Rolling and Friction
Vector3 CalculateRollingResistance(const Vector3& normal, float coefficient, float wheelRadius);
Vector3 CalculateWheelFriction(const Vector3& velocity, const Vector3& normal, float frictionCoefficient);

// Projectile Motion
Vector3 CalculateProjectilePosition(const Vector3& initialPos, const Vector3& initialVel, const Vector3& gravity, float time);
Vector3 CalculateProjectileVelocity(const Vector3& initialVel, const Vector3& gravity, float time);
float CalculateProjectileRange(const Vector3& initialVel, float gravity, float height);

// Advanced Collision with Deformation
Vector3 CalculateDeformableCollisionResponse(const Vector3& normal, float restitution, const Vector3& relativeVelocity, 
                                           float invMassA, float invMassB, float deformationStiffness, float deformationDamping);

// Energy Transfer and Heat
float CalculateHeatTransfer(float temperature1, float temperature2, float conductivity, float area, float thickness);
float CalculateThermalExpansion(float initialLength, float temperatureChange, float expansionCoefficient);

// Advanced Rotational Dynamics
Vector3 CalculatePrecessionTorque(const Vector3& angularMomentum, const Vector3& externalTorque);
Vector3 CalculateNutation(const Vector3& angularMomentum, const Vector3& externalTorque, float deltaTime);

// Complex Environmental Forces
Vector3 CalculateWindForce(const Vector3& velocity, const Vector3& windVelocity, float airDensity, float dragCoefficient, float area);
Vector3 CalculateThermalForce(const Vector3& temperatureGradient, float thermalConductivity, float area);

// Advanced Stability Analysis
float CalculateMetacentricHeight(const Vector3& centerOfMass, const Vector3& centerOfBuoyancy, const Vector3& metacenter);
bool IsStaticallyStable(const Vector3& centerOfMass, const Vector3& supportBase, const Vector3& externalForce, float threshold);

// Advanced Material Properties
float CalculateYoungsModulus(float stress, float strain);
float CalculatePoissonRatio(float lateralStrain, float axialStrain);

// Complex Motion Analysis
Vector3 CalculateHarmonicMotion(const Vector3& amplitude, float frequency, float time, float phase);
Vector3 CalculateDampedHarmonicMotion(const Vector3& amplitude, float frequency, float damping, float time, float phase);

// Advanced Wave Physics
Vector3 CalculateWaveForce(const Vector3& position, float amplitude, float frequency, float phase);
Vector3 CalculateWaveVelocity(const Vector3& position, float amplitude, float frequency, float phase);

// Advanced Particle Systems
Vector3 CalculateParticleForce(const Vector3& position, const Vector3& velocity, float mass, float charge, 
                              const Vector3& electricField, const Vector3& magneticField);
Vector3 CalculateParticleTrajectory(const Vector3& initialPos, const Vector3& initialVel, const Vector3& electricField,
                                  const Vector3& magneticField, float charge, float mass, float time);

// Advanced Rigid Body Dynamics
Vector3 CalculateContactForce(const Vector3& normal, float penetration, float stiffness, float damping, const Vector3& relativeVelocity);
Matrix3 CalculateAngularAcceleration(const Matrix3& inertiaTensor, const Vector3& torque);

// Advanced Fluid-Structure Interaction
Vector3 CalculateFluidStructureForce(const Vector3& velocity, float fluidDensity, float viscosity, 
                                   float characteristicLength, float surfaceArea);
Vector3 CalculateVortexForce(const Vector3& position, const Vector3& vortexCenter, float circulation);

// Advanced Material Deformation
Matrix3 CalculateStressTensor(const Matrix3& strainTensor, float youngsModulus, float poissonRatio);

// Advanced Thermodynamics
Vector3 CalculateThermalConvection(const Vector3& temperatureGradient, float thermalConductivity,
                                 float density, float specificHeat, float viscosity);

// Advanced Electromagnetic Fields
Vector3 CalculateElectricField(const Vector3& position, const Vector3& chargePosition, float charge);
Vector3 CalculateMagneticFieldFromCurrent(const Vector3& position, const Vector3& currentPosition,
                                        const Vector3& currentDirection, float current);

// Advanced Quantum Mechanics (Simplified)
float CalculateWaveFunction(const Vector3& position, float amplitude, float wavelength, float time);

// Advanced Chaos Theory
Vector3 CalculateLorenzAttractor(const Vector3& position, float sigma, float rho, float beta);

// Advanced Acoustics
float CalculateSoundIntensity(float pressure, float density, float soundSpeed);
Vector3 CalculateSoundPressureGradient(const Vector3& position, float frequency, float amplitude);

} // namespace MathUtils
