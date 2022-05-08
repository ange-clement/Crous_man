#ifndef ROGIDBODY_COMPONENT_HPP
#define ROGIDBODY_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

#define STATIC_OBJECT_MASS 0.0f

class ContactPoint;

enum RBType {
    //A particles has a mass but no volume (no rotation forces)
    PARTICLES,
    VOLUME,
};

//A rigid body type of element is an element that cant be streachable during time
//got the same shape over time, and also same volume
struct RigidBody {
    bool static_RB = false;
    RBType type = VOLUME;

    glm::vec3 velocity              = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 angularSpeed          = glm::vec3(0.0f, 0.0f, 0.0f);

    float mass = 1.0f;
    float inverseOfMass = 1.0f;

    float staticFriction = 1.0f;
    float cineticFriction = .5f;

    glm::vec3 combinedForces            = glm::vec3(0.0, 0.0, 0.0);
    //Torque of our representation
    glm::vec3 combinedAngularForces     = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 combinedStaticFriction    = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 combinedCineticFriction   = glm::vec3(0.0, 0.0, 0.0);

    //résistance
    float drag = 0.5f;
    float angularDrag = 0.2f;

    //Coefficient of Restitution
    float bounce            = .7f;       //represents how much energy is kept when a smt bounces off a surface
    
    //For more precise integration over time, like Verlet
    glm::vec3 oldPosition;


    void rotational_addImpulseAtPosition(const glm::vec3& point, const glm::vec3& impulse, const glm::vec3& currentPosition, const glm::mat3& inverseTensor);


    void setMass(float mass);
    

    void addForce(glm::vec3 force);
    void addAcceleration(glm::vec3 acc);
    void addVelocity(glm::vec3 vel);
    void addImpulse(glm::vec3 impulse);
    
    void addAngularForce(glm::vec3 force);
    void addAngularAcceleration(glm::vec3 acc);
    void addAngularVelocity(glm::vec3 vel);
    void addAngularImpulse(glm::vec3 impulse);

    void addForceAtPosition(glm::vec3 force, glm::vec3 pos);
    void addAccelerationAtPosition(glm::vec3 acc, glm::vec3 pos);
    void addVelocityAtPosition(glm::vec3 cel, glm::vec3 pos);
    void addImpulseAtPosition(glm::vec3 impulse, glm::vec3 pos);
};

struct TensorMatrix{
    inline TensorMatrix();
    virtual inline ~TensorMatrix();
    bool computed = false;
    glm::mat4 mat;
};


class RigidBodySystem : public virtual ComponentSystem {
public:
    glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
    glm::vec3 gravityDirection = glm::vec3(0.0f, -1.0f, 0.0f);

    std::vector<unsigned short> entityIDsToIndex;
    
    std::vector<TensorMatrix> tensorMatrix;
    std::vector<Collider*> colliderForRotational;
    ColliderSystem* colliderSystem = 0;

    //  The linear projection value indicates how much positional correction to apply. A 
    //  smaller value will allow objects to penetrate more. 
    //      VAL : between 0.2 and 0.8
    float linearProjectionPercent = .45f;
    // determines how much to allow objects to penetrate. This 
    // helps avoid jitter. The larger this number, the less jitter we have in the system. 
    //      VAL : between 0.01 and 0.1
    float penetrationSlack = .01f;

public:
    RigidBodySystem();

    ~RigidBodySystem();

    void setGravity(glm::vec3 gravity);

    virtual void updateOnCollide(unsigned short i, unsigned short entityID, const std::vector<ColliderResult*>& collisionResults);
    virtual void updatePhysics(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    RigidBody* getRigidBody(unsigned short i);
    RigidBody* getRigidBodyFromEntityId(unsigned short entityID);



    Collider* getColliderFromEntityID(unsigned short entityID);
    /*=========================================================================================================================*/
    //matrix tensor list manip (more perf)
    void clearMatrixTensorStructure(unsigned short id);
    glm::mat4 getMatrixInverseTensor(unsigned short entityID, Collider c, RigidBody* rb);
    //init foces on RB
    void initForcesRB(RigidBody* rb);

    //Particles treatment
    glm::vec3 resolveConstraintParticles_Verlet(Collider& collider, RigidBody* rb_particles, const glm::vec3& currentPos);
    glm::vec3 resolveConstraintParticles_Euler(Collider& collider, RigidBody* rb_particles, const glm::vec3& currentPos);

    /*========================================== LINEAR EQUATIONS ===========================================================*/

    //Update rb positions
    glm::vec3 linear_update_EulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime);
    glm::vec3 linear_update_AccurateEulerIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime);
    glm::vec3 linear_update_VerletIntegration(RigidBody* rb, const glm::vec3& currentPos, float deltaTime);

    void linear_resolveConstraintVolumeRB_Euler(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC);
    void linear_resolveConstraintVolumeRB_Euler_dynamicfriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC);

    //Apply a possible position correction, depending on linearProjectionPercent and penetrationSlack
    void positionCorrection(glm::vec3& newPos1, glm::vec3& newPos2, RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res);

    /*========================================== ROTATION EQUATIONS ===========================================================*/
    glm::mat4 inverseTensorComputation(Collider c, RigidBody* rb);
    void rotational_update(RigidBody* rb, float deltaTime, const glm::mat3& inverseTensor);

    void linearAndAngular_resolveConstraintVolumeRB_Euler(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2);
    void linearAndAngular_resolveConstraintVolumeRB_Euler_dynamicfriction(RigidBody* rb_1, RigidBody* rb_2, ContactPoint* p_res, int nbC, const glm::vec3& posrb1, const glm::vec3& posrb2, const glm::mat4& inverseTensorrb1, const glm::mat4& inverseTensorrb2);
    
    
    //NEXT USE QUATERNIONS FROM GLM : quat and common/quaternion_utils.hpp
    // Gimbal lock
    // -> glm::mat4 RotationMatrix = glm::toMat4(myQuat);


    //Apply forces spring connection
    // -> For soft body implementation 
    void applySpringConnection(RigidBody* rb_1, RigidBody* rb_2, const glm::vec3 pos_1, const glm::vec3 pos_2, float deltaTime, float restingLength, float k_springStrenght, float b_springFriction);
    //Apply force joint connection
    //In three dimensions, an object has six degrees of freedom
    //A joint is a type of constraint that limits the degrees of freedom between two objects.
    void resolveConstraintParticles_JointDistance(RigidBody* rb_particles_1, RigidBody* rb_particles_2, glm::vec3& currentPos_1, glm::vec3& currentPos_2, float distancejoint);
};



float computeVelocityFactor(RigidBody* rbA, RigidBody* rbB, glm::vec3 normal);
float computeAngularFactor(RigidBody* rbA, RigidBody* rbB, glm::vec3 normal, glm::vec3 rA, glm::vec3 rB);






#endif