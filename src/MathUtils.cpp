#include "MathUtils.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace MathUtils {

float Clamp(float value, float min, float max) {
    return std::max(min, std::min(max, value));
}

float Lerp(float a, float b, float t) {
    return a + t * (b - a);
}

float SmoothStep(float t) {
    return t * t * (3.0f - 2.0f * t);
}

float RandomFloat(float min, float max) {
    static unsigned int seed = 0;
    seed = seed * 1664525 + 1013904223;
    float random = static_cast<float>(seed) / 4294967296.0f;
    return min + random * (max - min);
}

Vector3 RandomVector3(float min, float max) {
    return Vector3(
        RandomFloat(min, max),
        RandomFloat(min, max),
        RandomFloat(min, max)
    );
}

Quaternion FromAxisAngle(const Vector3& axis, float angle) {
    float halfAngle = angle * 0.5f;
    float sinHalf = std::sin(halfAngle);
    float cosHalf = std::cos(halfAngle);
    Vector3 normalizedAxis = axis.Normalize();
    return Quaternion(
        cosHalf,
        normalizedAxis.x * sinHalf,
        normalizedAxis.y * sinHalf,
        normalizedAxis.z * sinHalf
    );
}

Quaternion FromEulerAngles(float pitch, float yaw, float roll) {
    // Convert to radians if needed
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);

    return Quaternion(
        cr * cp * cy - sr * sp * sy,  // w
        sr * cp * cy + cr * sp * sy,  // x
        cr * sp * cy + sr * cp * sy,  // y
        cr * cp * sy - sr * sp * cy   // z
    );
}

Vector3 ToEulerAngles(const Quaternion& q) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    float pitch;
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(M_PI / 2.0f, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return Vector3(pitch, yaw, roll);
}

float DegToRad(float degrees) {
    return degrees * (M_PI / 180.0f);
}

float RadToDeg(float radians) {
    return radians * (180.0f / M_PI);
}

bool IsNearlyEqual(float a, float b, float tolerance) {
    return std::abs(a - b) <= tolerance;
}

bool IsNearlyZero(float value, float tolerance) {
    return std::abs(value) <= tolerance;
}

// Physics-specific math functions
float CalculateInertiaTensor(float mass, float radius, bool isHollow) {
    if (isHollow) {
        return (2.0f/3.0f) * mass * radius * radius;  // Hollow sphere
    }
    return (2.0f/5.0f) * mass * radius * radius;      // Solid sphere
}

Matrix3 CalculateInertiaTensorMatrix(const Vector3& halfExtents, float mass) {
    float x = halfExtents.x;
    float y = halfExtents.y;
    float z = halfExtents.z;
    
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;
    
    return Matrix3(
        Vector3((1.0f/12.0f) * mass * (yy + zz), 0, 0),
        Vector3(0, (1.0f/12.0f) * mass * (xx + zz), 0),
        Vector3(0, 0, (1.0f/12.0f) * mass * (xx + yy))
    );
}

float CalculateAngularVelocity(float linearVelocity, float radius) {
    return linearVelocity / radius;
}

Vector3 CalculateCentripetalForce(float mass, float velocity, float radius) {
    return Vector3(-mass * velocity * velocity / radius, 0.0f, 0.0f);
}

float CalculateKineticEnergy(float mass, const Vector3& velocity) {
    return 0.5f * mass * velocity.Dot(velocity);
}

float CalculateRotationalKineticEnergy(const Matrix3& inertiaTensor, const Vector3& angularVelocity) {
    return 0.5f * angularVelocity.Dot(inertiaTensor * angularVelocity);
}

float CalculatePotentialEnergy(float mass, float height, float gravity) {
    return mass * gravity * height;
}

Vector3 CalculateImpulse(const Vector3& normal, float restitution, 
                        const Vector3& relativeVelocity, float invMassA, float invMassB) {
    float j = -(1.0f + restitution) * relativeVelocity.Dot(normal);
    j /= (invMassA + invMassB);
    return normal * j;
}

Vector3 CalculateFrictionImpulse(const Vector3& normal, float friction,
                               const Vector3& relativeVelocity, float invMassA, float invMassB) {
    Vector3 tangent = relativeVelocity - normal * relativeVelocity.Dot(normal);
    float tangentLength = tangent.Length();
    if (tangentLength > 0.0f) {
        tangent = tangent / tangentLength;
    }
    
    float j = -friction * relativeVelocity.Dot(tangent);
    j /= (invMassA + invMassB);
    return tangent * j;
}

float CalculateDragForce(float density, float velocity, float area, float dragCoefficient) {
    return 0.5f * density * velocity * velocity * area * dragCoefficient;
}

Vector3 CalculateBuoyantForce(float fluidDensity, float volume, const Vector3& gravity) {
    return -fluidDensity * volume * gravity;
}

float CalculateElasticCollisionVelocity(float m1, float m2, float v1, float v2) {
    return (m1 * v1 + m2 * v2) / (m1 + m2);
}

float CalculateInelasticCollisionVelocity(float m1, float m2, float v1, float v2, float restitution) {
    return (m1 * v1 + m2 * v2) * restitution / (m1 + m2);
}

Vector3 CalculateSpringForce(const Vector3& displacement, float springConstant, float restLength) {
    float currentLength = displacement.Length();
    if (currentLength > 0.0f) {
        return -springConstant * (currentLength - restLength) * (displacement / currentLength);
    }
    return Vector3(0.0f, 0.0f, 0.0f);
}

Vector3 CalculateDampingForce(const Vector3& velocity, float dampingCoefficient) {
    return -dampingCoefficient * velocity;
}

float CalculatePeriod(float frequency) {
    return 1.0f / frequency;
}

float CalculateFrequency(float period) {
    return 1.0f / period;
}

float CalculateAngularFrequency(float period) {
    return 2.0f * M_PI / period;
}

Vector3 CalculateTorque(const Vector3& force, const Vector3& leverArm) {
    return leverArm.Cross(force);
}

float CalculateMomentOfInertia(float mass, float distance) {
    return mass * distance * distance;
}

Vector3 CalculateAngularMomentum(const Matrix3& inertiaTensor, const Vector3& angularVelocity) {
    return inertiaTensor * angularVelocity;
}

float CalculateWork(const Vector3& force, const Vector3& displacement) {
    return force.Dot(displacement);
}

float CalculatePower(const Vector3& force, const Vector3& velocity) {
    return force.Dot(velocity);
}

// Fluid Dynamics Functions
Vector3 CalculateFluidForce(const Vector3& velocity, float fluidDensity, float volume, float dragCoefficient) {
    return -0.5f * fluidDensity * volume * dragCoefficient * velocity * std::abs(velocity.Length());
}

Vector3 CalculateViscousForce(const Vector3& velocity, float viscosity, float surfaceArea) {
    return -viscosity * surfaceArea * velocity;
}

Vector3 CalculatePressureForce(float pressure, const Vector3& normal, float area) {
    return pressure * normal * area;
}

float CalculateReynoldsNumber(float density, float velocity, float characteristicLength, float viscosity) {
    return (density * velocity * characteristicLength) / viscosity;
}

// Soft Body Physics Functions
Vector3 CalculateDeformationForce(const Vector3& displacement, float stiffness, float damping) {
    return -stiffness * displacement - damping * displacement;
}

Matrix3 CalculateDeformationGradient(const Vector3& position, const Vector3& restPosition) {
    return Matrix3(
        Vector3(position.x - restPosition.x, 0, 0),
        Vector3(0, position.y - restPosition.y, 0),
        Vector3(0, 0, position.z - restPosition.z)
    );
}

float CalculateVolumePreservation(const Matrix3& deformationGradient) {
    return deformationGradient.m[0][0] * deformationGradient.m[1][1] * deformationGradient.m[2][2];
}

// Complex Interaction Functions
Vector3 CalculateCoriolisForce(const Vector3& velocity, const Vector3& angularVelocity) {
    return -2.0f * (angularVelocity.Cross(velocity));
}

Vector3 CalculateCentrifugalForce(const Vector3& position, const Vector3& angularVelocity) {
    return angularVelocity.Cross(angularVelocity.Cross(position));
}

Matrix3 CalculateRotationMatrix(const Vector3& axis, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    float t = 1.0f - c;
    Vector3 n = axis.Normalize();
    
    return Matrix3(
        Vector3(t * n.x * n.x + c, t * n.x * n.y - s * n.z, t * n.x * n.z + s * n.y),
        Vector3(t * n.x * n.y + s * n.z, t * n.y * n.y + c, t * n.y * n.z - s * n.x),
        Vector3(t * n.x * n.z - s * n.y, t * n.y * n.z + s * n.x, t * n.z * n.z + c)
    );
}

Vector3 CalculateRelativeVelocity(const Vector3& velocity, const Vector3& angularVelocity, const Vector3& position) {
    return velocity + angularVelocity.Cross(position);
}

float CalculateImpactForce(float mass, float velocity, float impactTime) {
    return mass * velocity / impactTime;
}

Vector3 CalculateFrictionForce(const Vector3& normal, float frictionCoefficient, const Vector3& relativeVelocity) {
    Vector3 tangent = relativeVelocity - normal * relativeVelocity.Dot(normal);
    float tangentLength = tangent.Length();
    if (tangentLength > 0.0f) {
        tangent = tangent / tangentLength;
    }
    return -frictionCoefficient * tangent;
}

// Advanced Collision Functions
float CalculateRestitutionCoefficient(float initialVelocity, float finalVelocity) {
    return std::abs(finalVelocity / initialVelocity);
}

Vector3 CalculateCollisionResponse(const Vector3& normal, float restitution,
                                 const Vector3& relativeVelocity, float invMassA, float invMassB) {
    float j = -(1.0f + restitution) * relativeVelocity.Dot(normal);
    j /= (invMassA + invMassB);
    return normal * j;
}

// Energy and Momentum Functions
float CalculateTotalEnergy(float kineticEnergy, float potentialEnergy, float rotationalEnergy) {
    return kineticEnergy + potentialEnergy + rotationalEnergy;
}

Vector3 CalculateConservationOfMomentum(const Vector3& momentum1, const Vector3& momentum2) {
    return momentum1 + momentum2;
}

// Constraint Functions
Vector3 CalculateConstraintForce(const Vector3& position, const Vector3& target, float stiffness) {
    return stiffness * (target - position);
}

Vector3 CalculateDistanceConstraint(const Vector3& posA, const Vector3& posB, float targetDistance) {
    Vector3 diff = posB - posA;
    float currentDistance = diff.Length();
    if (currentDistance > 0.0f) {
        return diff * (1.0f - targetDistance / currentDistance);
    }
    return Vector3(0.0f, 0.0f, 0.0f);
}

// Advanced Motion Functions
Vector3 CalculatePrecession(const Vector3& angularMomentum, const Vector3& torque, float deltaTime) {
    return angularMomentum + torque * deltaTime;
}

Vector3 CalculateGyroscopicForce(const Vector3& angularVelocity, const Matrix3& inertiaTensor) {
    return angularVelocity.Cross(inertiaTensor * angularVelocity);
}

// Stability Functions
float CalculateStabilityFactor(const Vector3& centerOfMass, const Vector3& supportPoint) {
    return (centerOfMass - supportPoint).Length();
}

bool IsStable(const Vector3& centerOfMass, const Vector3& supportBase, float threshold) {
    return CalculateStabilityFactor(centerOfMass, supportBase) < threshold;
}

// Gravity Environment Functions
Vector3 CalculateGravityForce(float mass, const Vector3& gravity) {
    return mass * gravity;
}

Vector3 CalculateVariableGravityForce(float mass, const Vector3& position, float planetMass, float planetRadius) {
    float distance = position.Length();
    float gravityMagnitude = (6.67430e-11f * mass * planetMass) / (distance * distance);
    Vector3 normalizedPos = position.Normalize();
    return normalizedPos * (-gravityMagnitude);
}

// Magnetic Field Functions
Vector3 CalculateMagneticForce(const Vector3& velocity, const Vector3& magneticField, float charge) {
    return charge * velocity.Cross(magneticField);
}

Vector3 CalculateMagneticField(const Vector3& position, const Vector3& magnetPosition, float magnetStrength) {
    Vector3 r = position - magnetPosition;
    float distance = r.Length();
    if (distance < 1e-6f) return Vector3(0.0f, 0.0f, 0.0f);
    return magnetStrength * r / (distance * distance * distance);
}

// Advanced Fluid Dynamics
Vector3 CalculateLaminarFlowForce(const Vector3& velocity, float viscosity, float length, float radius) {
    return -8.0f * M_PI * viscosity * length * velocity / (radius * radius);
}

Vector3 CalculateTurbulentFlowForce(const Vector3& velocity, float density, float area, float roughness) {
    float reynolds = CalculateReynoldsNumber(density, velocity.Length(), 2.0f * std::sqrt(area/M_PI), 1.789e-5f);
    float frictionFactor = 0.25f / std::pow(std::log10(roughness/(3.7f * 2.0f * std::sqrt(area/M_PI)) + 5.74f/std::pow(reynolds, 0.9f)), 2.0f);
    return -0.5f * density * velocity * std::abs(velocity.Length()) * frictionFactor;
}

// Advanced Buoyancy and Floating
Vector3 CalculateFloatingForce(float submergedVolume, float fluidDensity, const Vector3& gravity) {
    float forceMagnitude = fluidDensity * submergedVolume;
    return Vector3(
        -forceMagnitude * gravity.x,
        -forceMagnitude * gravity.y,
        -forceMagnitude * gravity.z
    );
}

float CalculateSubmergedVolume(const Vector3& dimensions, float waterLevel, float objectHeight) {
    float submergedHeight = std::max(0.0f, std::min(objectHeight, waterLevel));
    return dimensions.x * dimensions.y * submergedHeight;
}

// Rolling and Friction
Vector3 CalculateRollingResistance(const Vector3& normal, float coefficient, float wheelRadius) {
    return -coefficient * normal / wheelRadius;
}

Vector3 CalculateWheelFriction(const Vector3& velocity, const Vector3& normal, float frictionCoefficient) {
    Vector3 tangent = velocity - normal * velocity.Dot(normal);
    float tangentLength = tangent.Length();
    if (tangentLength > 0.0f) {
        tangent = tangent / tangentLength;
    }
    return -frictionCoefficient * normal.Length() * tangent;
}

// Projectile Motion
Vector3 CalculateProjectilePosition(const Vector3& initialPos, const Vector3& initialVel, 
                                  const Vector3& gravity, float time) {
    return initialPos + initialVel * time + 0.5f * gravity * time * time;
}

Vector3 CalculateProjectileVelocity(const Vector3& initialVel, const Vector3& gravity, float time) {
    return initialVel + gravity * time;
}

float CalculateProjectileRange(const Vector3& initialVel, float gravity, float height) {
    float v0 = initialVel.Length();
    float theta = std::atan2(initialVel.y, initialVel.x);
    return (v0 * v0 * std::sin(2.0f * theta)) / gravity;
}

// Advanced Collision with Deformation
Vector3 CalculateDeformableCollisionResponse(const Vector3& normal, float restitution,
                                           const Vector3& relativeVelocity, float invMassA, float invMassB,
                                           float deformationStiffness, float deformationDamping) {
    Vector3 impulse = CalculateCollisionResponse(normal, restitution, relativeVelocity, invMassA, invMassB);
    Vector3 deformationForce = CalculateDeformationForce(normal, deformationStiffness, deformationDamping);
    return impulse + deformationForce;
}

// Energy Transfer and Heat
float CalculateHeatTransfer(float temperature1, float temperature2, float conductivity, float area, float thickness) {
    return conductivity * area * (temperature2 - temperature1) / thickness;
}

float CalculateThermalExpansion(float initialLength, float temperatureChange, float expansionCoefficient) {
    return initialLength * (1.0f + expansionCoefficient * temperatureChange);
}

// Advanced Rotational Dynamics
Vector3 CalculatePrecessionTorque(const Vector3& angularMomentum, const Vector3& externalTorque) {
    return angularMomentum.Cross(externalTorque);
}

Vector3 CalculateNutation(const Vector3& angularMomentum, const Vector3& externalTorque, float deltaTime) {
    return angularMomentum + externalTorque * deltaTime;
}

// Complex Environmental Forces
Vector3 CalculateWindForce(const Vector3& velocity, const Vector3& windVelocity, 
                          float airDensity, float dragCoefficient, float area) {
    Vector3 relativeVelocity = velocity - windVelocity;
    return -0.5f * airDensity * dragCoefficient * area * relativeVelocity * std::abs(relativeVelocity.Length());
}

Vector3 CalculateThermalForce(const Vector3& temperatureGradient, float thermalConductivity, float area) {
    return -thermalConductivity * area * temperatureGradient;
}

// Advanced Stability Analysis
float CalculateMetacentricHeight(const Vector3& centerOfMass, const Vector3& centerOfBuoyancy,
                                const Vector3& metacenter) {
    return (metacenter - centerOfMass).Length() - (centerOfBuoyancy - centerOfMass).Length();
}

bool IsStaticallyStable(const Vector3& centerOfMass, const Vector3& supportBase, 
                        const Vector3& externalForce, float threshold) {
    Vector3 moment = (centerOfMass - supportBase).Cross(externalForce);
    return moment.Length() < threshold;
}

// Advanced Material Properties
float CalculateYoungsModulus(float stress, float strain) {
    return stress / strain;
}

float CalculatePoissonRatio(float lateralStrain, float axialStrain) {
    return -lateralStrain / axialStrain;
}

// Complex Motion Analysis
Vector3 CalculateHarmonicMotion(const Vector3& amplitude, float frequency, float time, float phase) {
    return amplitude * std::sin(2.0f * M_PI * frequency * time + phase);
}

Vector3 CalculateDampedHarmonicMotion(const Vector3& amplitude, float frequency, float damping,
                                    float time, float phase) {
    return amplitude * std::exp(-damping * time) * std::sin(2.0f * M_PI * frequency * time + phase);
}

// Advanced Wave Physics
Vector3 CalculateWaveForce(const Vector3& position, float amplitude, float frequency, float phase) {
    float waveHeight = amplitude * std::sin(2.0f * M_PI * frequency * position.x + phase);
    return Vector3(0.0f, waveHeight, 0.0f);
}

Vector3 CalculateWaveVelocity(const Vector3& position, float amplitude, float frequency, float phase) {
    float velocity = 2.0f * M_PI * frequency * amplitude * std::cos(2.0f * M_PI * frequency * position.x + phase);
    return Vector3(0.0f, velocity, 0.0f);
}

// Advanced Particle Systems
Vector3 CalculateParticleForce(const Vector3& position, const Vector3& velocity, float mass, 
                              float charge, const Vector3& electricField, const Vector3& magneticField) {
    Vector3 electricForce = charge * electricField;
    Vector3 magneticForce = charge * velocity.Cross(magneticField);
    return electricForce + magneticForce;
}

Vector3 CalculateParticleTrajectory(const Vector3& initialPos, const Vector3& initialVel,
                                  const Vector3& electricField, const Vector3& magneticField,
                                  float charge, float mass, float time) {
    float cyclotronFrequency = std::abs(charge * magneticField.Length() / mass);
    float radius = mass * initialVel.Length() / (std::abs(charge) * magneticField.Length());
    
    Vector3 center = initialPos + radius * initialVel.Cross(magneticField).Normalize();
    float angle = cyclotronFrequency * time;
    
    return center + radius * (std::cos(angle) * initialVel.Normalize() + 
                             std::sin(angle) * initialVel.Cross(magneticField).Normalize());
}

// Advanced Rigid Body Dynamics
Vector3 CalculateContactForce(const Vector3& normal, float penetration, float stiffness, float damping,
                            const Vector3& relativeVelocity) {
    Vector3 springForce = stiffness * penetration * normal;
    Vector3 dampingForce = damping * relativeVelocity.Dot(normal) * normal;
    return springForce + dampingForce;
}

Matrix3 CalculateAngularAcceleration(const Matrix3& inertiaTensor, const Vector3& torque) {
    return Matrix3(
        Vector3(torque.x / inertiaTensor.m[0][0], 0, 0),
        Vector3(0, torque.y / inertiaTensor.m[1][1], 0),
        Vector3(0, 0, torque.z / inertiaTensor.m[2][2])
    );
}

// Advanced Fluid-Structure Interaction
Vector3 CalculateFluidStructureForce(const Vector3& velocity, float fluidDensity, float viscosity,
                                   float characteristicLength, float surfaceArea) {
    float reynolds = CalculateReynoldsNumber(fluidDensity, velocity.Length(), characteristicLength, viscosity);
    float dragCoefficient = reynolds < 1.0f ? 24.0f / reynolds : 0.5f;
    return -0.5f * fluidDensity * dragCoefficient * surfaceArea * velocity * std::abs(velocity.Length());
}

Vector3 CalculateVortexForce(const Vector3& position, const Vector3& vortexCenter, float circulation) {
    Vector3 r = position - vortexCenter;
    float distance = r.Length();
    if (distance < 1e-6f) return Vector3(0.0f, 0.0f, 0.0f);
    return circulation * r.Cross(Vector3(0.0f, 1.0f, 0.0f)) / (2.0f * M_PI * distance * distance);
}

// Advanced Material Deformation
Matrix3 CalculateStressTensor(const Matrix3& strainTensor, float youngsModulus, float poissonRatio) {
    float factor = youngsModulus / ((1.0f + poissonRatio) * (1.0f - 2.0f * poissonRatio));
    return Matrix3(
        Vector3(factor * ((1.0f - poissonRatio) * strainTensor.m[0][0] + poissonRatio * (strainTensor.m[1][1] + strainTensor.m[2][2])), 0, 0),
        Vector3(0, factor * ((1.0f - poissonRatio) * strainTensor.m[1][1] + poissonRatio * (strainTensor.m[0][0] + strainTensor.m[2][2])), 0),
        Vector3(0, 0, factor * ((1.0f - poissonRatio) * strainTensor.m[2][2] + poissonRatio * (strainTensor.m[0][0] + strainTensor.m[1][1])))
    );
}

// Advanced Thermodynamics
Vector3 CalculateThermalConvection(const Vector3& temperatureGradient, float thermalConductivity,
                                 float density, float specificHeat, float viscosity) {
    float prandtlNumber = viscosity * specificHeat / thermalConductivity;
    float rayleighNumber = density * specificHeat * temperatureGradient.Length() / (viscosity * thermalConductivity);
    float heatTransferCoefficient = thermalConductivity * std::pow(rayleighNumber, 0.25f) / 
                                   (density * specificHeat * std::pow(prandtlNumber, 0.5f));
    return -heatTransferCoefficient * temperatureGradient;
}

// Advanced Electromagnetic Fields
Vector3 CalculateElectricField(const Vector3& position, const Vector3& chargePosition, float charge) {
    Vector3 r = position - chargePosition;
    float distance = r.Length();
    if (distance < 1e-6f) return Vector3(0.0f, 0.0f, 0.0f);
    float k = 8.99e9f; // Coulomb's constant
    return k * charge * r / (distance * distance * distance);
}

Vector3 CalculateMagneticFieldFromCurrent(const Vector3& position, const Vector3& currentPosition,
                                        const Vector3& currentDirection, float current) {
    Vector3 r = position - currentPosition;
    float distance = r.Length();
    if (distance < 1e-6f) return Vector3(0.0f, 0.0f, 0.0f);
    float mu0 = 4.0f * M_PI * 1e-7f; // Permeability of free space
    return mu0 * current * currentDirection.Cross(r) / (2.0f * M_PI * distance * distance);
}

// Advanced Quantum Mechanics (Simplified)
float CalculateWaveFunction(const Vector3& position, float amplitude, float wavelength, float time) {
    float k = 2.0f * M_PI / wavelength;
    float omega = 2.0f * M_PI * 3e8f / wavelength; // Speed of light
    return amplitude * std::cos(k * position.x - omega * time);
}

// Advanced Chaos Theory
Vector3 CalculateLorenzAttractor(const Vector3& position, float sigma, float rho, float beta) {
    return Vector3(
        sigma * (position.y - position.x),
        position.x * (rho - position.z) - position.y,
        position.x * position.y - beta * position.z
    );
}

// Advanced Acoustics
float CalculateSoundIntensity(float pressure, float density, float soundSpeed) {
    return pressure * pressure / (2.0f * density * soundSpeed);
}

Vector3 CalculateSoundPressureGradient(const Vector3& position, float frequency, float amplitude) {
    float k = 2.0f * M_PI * frequency / 343.0f; // Speed of sound
    return Vector3(
        -k * amplitude * std::sin(k * position.x),
        -k * amplitude * std::sin(k * position.y),
        -k * amplitude * std::sin(k * position.z)
    );
}

} // namespace MathUtils
